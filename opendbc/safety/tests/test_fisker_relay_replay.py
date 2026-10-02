#!/usr/bin/env python3
"""
End-to-end replay of the Fisker relay: the real C arbiter (via libsafety), the real CarController
and RelayClock, a stock ADAS stream with the recorded counter and SecOC-window behavior, and the
Comma -> panda pipeline timing measured from recorded drives (fisker_relay_sim_data.py).

For every engage/release scenario the vehicle-bus stream of 0x1D0, 0x1C0 and 0x121 (stock frames
panda forwarded plus ours it accepted) is judged by the same rules the drive-log audit uses
(opendbc/car/fisker/relay_audit.py): one source at a time, a strictly +1 alive counter, bounded
steps only at a hand-off, and no long gaps. The scenarios include the one that faulted the car: ACC
main pressed while panda's lateral flag stays set for 2.4 s.
"""
import heapq
import random
import unittest
from dataclasses import dataclass
from types import SimpleNamespace as NS

import opendbc.safety.tests.common as common
from opendbc.car import Bus
from opendbc.car.fisker.carcontroller import CarController
from opendbc.car.fisker.interface import CarInterface
from opendbc.car.fisker.relay_audit import OURS, STOCK, BusFrame, audit_stream
from opendbc.car.fisker.values import CAR, RELAY_CTRL_ADDR
from opendbc.car.structs import CarParams
from opendbc.safety.tests.fisker_relay_sim_data import OURS_TICK_INTERVAL_US, TX_TO_BUS_US
from opendbc.safety.tests.libsafety import libsafety_py

STEER, LAT, ACCEL = 0x1D0, 0x1C0, 0x121
STOCK_PERIOD_US = 10_000
WINDOW_FRAMES = (101, 102)           # frames per SecOC Reset window on the stock module, alternating
RX_LATENCY_US = (2_000, 8_000)       # stock frame on bus 2 -> visible to the Comma
SYNC_LEAD_US = (-6_000, 12_000)      # GW sync reaches the Comma this long before the stock window flips
ENGAGED_STATES = (3, 4, 5, 6, 11)


@dataclass
class Scenario:
  name: str
  seed: int = 0
  engage_s: float = 1.0
  disengage_s: float = 4.0
  end_s: float = 7.0
  cruise: bool = True              # stock ACC engaged (0x313 state 3) between engage and disengage
  mads: bool = False               # MADS lateral engaged between engage and disengage
  long: bool = True                # alpha long
  lat_flag_lag_s: float = 2.4      # panda's controls_allowed_lateral drops only on a heartbeat mismatch
  stall: tuple[float, float] | None = None       # (start_s, duration_s): the Comma stops sending
  tx_spike_prob: float = 0.0
  tx_spike_us: int = 0


class Sim:
  def __init__(self, sc: Scenario):
    self.sc = sc
    self.rng = random.Random(sc.seed)
    self.heap: list = []
    self.seq = 0
    self.bus: dict[int, list[BusFrame]] = {STEER: [], LAT: [], ACCEL: []}
    self.rejected: list[tuple[float, int]] = []

    self.safety = libsafety_py.libsafety
    self.safety.set_timer(0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 1 if sc.long else 0)
    self.safety.init_tests()

    fp = {i: {} for i in range(8)}
    self.CP = CarInterface.get_params(CAR.FISKER_OCEAN, fp, [], sc.long, False, False)
    CP_SP = CarInterface.get_params_sp(self.CP, CAR.FISKER_OCEAN, fp, [], sc.long, False, False)
    self.CP.secOcKeyAvailable = True
    self.cc = CarController({Bus.pt: "fisker_ocean_adas"}, self.CP, CP_SP)
    self.cc.secoc_key = b"\x11" * 16
    self.cc.secoc_key_verified = True

    # stock stream state
    self.alive = self.rng.randrange(15)
    self.pos = self.rng.randrange(1, 90)
    self.reset = 4000 + self.rng.randrange(1000)
    self.window_idx = 0
    # Comma's view
    self.inbox: list[tuple[int, dict]] = []
    self.cs_state = 2
    self.cs_reset = self.reset
    self.last_arrival_us = 0
    self.tx_arrivals: list[int] = []

  # ---- event queue ----
  def at(self, t_us, kind, payload=None):
    self.seq += 1
    heapq.heappush(self.heap, (int(t_us), self.seq, kind, payload))

  def acc_state_at(self, t_s):
    sc = self.sc
    if not sc.cruise:
      return 2
    if t_s < sc.engage_s:
      return 2
    if t_s < sc.disengage_s:
      return 3
    return 2 if t_s > sc.disengage_s + 0.05 else (8 if t_s < sc.disengage_s + 0.02 else 10)

  def mads_at(self, t_s, comma_view=False):
    sc = self.sc
    return sc.mads and sc.engage_s <= t_s < sc.disengage_s + (0.012 if comma_view else 0.0)

  # ---- run ----
  def run(self):
    sc = self.sc
    end_us = int(sc.end_s * 1e6)
    self.at(0, "stock")
    self.at(0, "acc")
    self.at(int(0.5e6), "tick")
    # panda's lateral flag follows the MADS/cruise engagement, but drops late (heartbeat)
    if sc.mads or sc.cruise:
      self.at(int(sc.engage_s * 1e6) + 5_000, "lat_flag", True)
      self.at(int((sc.disengage_s + sc.lat_flag_lag_s) * 1e6), "lat_flag", False)
    while self.heap:
      t, _, kind, payload = heapq.heappop(self.heap)
      if t > end_us:
        break
      getattr(self, "on_" + kind)(t, payload)
    return self

  # ---- stock ADAS module + panda's forward/RX path ----
  def on_stock(self, t, _):
    self.at(t + STOCK_PERIOD_US + self.rng.randint(-150, 150), "stock")
    self.alive = (self.alive + 1) % 15
    self.pos += 1
    window = WINDOW_FRAMES[self.window_idx % 2]
    if self.pos > window:
      self.pos, self.reset, self.window_idx = 1, self.reset + 1, self.window_idx + 1
    # the GW announces the new Reset window around when the stock stream flips to it
    if self.pos == 1:
      self.at(t + self.rng.randint(*SYNC_LEAD_US), "sync", self.reset)
    fresh = ((self.pos & 63) << 2) | (self.reset & 3)
    for addr in (STEER, LAT, ACCEL):
      data = bytearray(8)
      data[1] = self.alive
      data[4] = fresh
      self.safety.set_timer(t)
      fwd = self.safety.safety_fwd_hook(2, addr)           # panda decides before its RX hook runs
      self.safety.safety_rx_hook(common.make_msg(2, addr, dat=bytes(data)))
      if fwd != -1:
        self.bus[addr].append(BusFrame(t * 1e-6, STOCK, self.alive))
    arrive = t + self.rng.randint(*RX_LATENCY_US)
    self.inbox.append((arrive, {"ADAS_1D0_AliveCounter": self.alive, "ADAS_1D0_SSecOC_Fresh_Byte0": fresh}))

  def on_acc(self, t, _):
    self.at(t + 20_000, "acc")
    state = self.acc_state_at(t * 1e-6)
    dat = bytearray(8)
    dat[4] = state
    self.safety.set_timer(t)
    self.safety.safety_rx_hook(common.make_msg(2, 0x313, dat=bytes(dat)))
    self.at(t + self.rng.randint(*RX_LATENCY_US), "comma_acc", state)

  def on_comma_acc(self, t, state):
    self.cs_state = state

  def on_sync(self, t, reset):
    self.at(t + self.rng.randint(*RX_LATENCY_US), "comma_sync", reset)

  def on_comma_sync(self, t, reset):
    self.cs_reset = max(self.cs_reset, reset)

  def on_lat_flag(self, t, value):
    self.safety.set_timer(t)
    self.safety.set_controls_allowed_lateral(value)

  # ---- the Comma ----
  def on_tick(self, t, _):
    sc = self.sc
    nxt = t + self.rng.choice(OURS_TICK_INTERVAL_US)
    self.at(nxt, "tick")
    if sc.stall and sc.stall[0] * 1e6 <= t < (sc.stall[0] + sc.stall[1]) * 1e6:
      return
    frames = [f for arr, f in self.inbox if arr <= t]
    self.inbox = [(arr, f) for arr, f in self.inbox if arr > t]
    ts = t * 1e-6
    engaged_lat = self.cs_state in ENGAGED_STATES or self.mads_at(ts, comma_view=True)
    CS = NS(out=NS(cruiseState=NS(enabled=self.cs_state in ENGAGED_STATES), gasPressed=False, vEgoRaw=10.0, steeringAngleDeg=0.0),
            secoc_sync_seen=True, secoc_trip=7, secoc_reset=self.cs_reset, secoc_sync_mac=b"",
            oem_frames=frames, icc_settings_frames=[], icc_0x35b_frames=[])
    accel = 0.8 * (1 if int(ts * 2) % 2 == 0 else -1)
    CC = NS(actuators=_Act(accel=accel, steeringAngleDeg=0.0), latActive=engaged_lat, longActive=self.cs_state in ENGAGED_STATES)
    CC_SP = NS(mads=NS(enabled=self.mads_at(ts, comma_view=True)))
    _, sends = self.cc.update(CC, CC_SP, CS, int(t * 1e3))
    for addr, data, _bus in sends:
      lat = self.rng.choice(TX_TO_BUS_US)
      if self.rng.random() < sc.tx_spike_prob:
        lat += sc.tx_spike_us
      arrive = max(t + lat, self.last_arrival_us)       # USB + panda queue are FIFO: late, never reordered
      self.last_arrival_us = arrive
      self.at(arrive, "panda_tx", (addr, bytes(data)))

  # ---- panda's TX path ----
  def on_panda_tx(self, t, payload):
    addr, data = payload
    self.safety.set_timer(t)
    ok = self.safety.safety_tx_hook(common.make_msg(0, addr, dat=data))
    if addr in self.bus and ok:
      self.bus[addr].append(BusFrame(t * 1e-6, OURS, data[1] & 0x0F))
    elif not ok:
      self.rejected.append((t * 1e-6, addr))


class _Act(NS):
  def as_builder(self):
    return NS(**vars(self))


def audit_all(sim: Sim, **kw):
  return {addr: audit_stream(frames, **kw) for addr, frames in sim.bus.items()}


class TestFiskerRelayReplay(unittest.TestCase):
  def check_ok(self, sim, addrs=(STEER, LAT, ACCEL), takeovers=1, releases=1, **kw):
    res = audit_all(sim, **kw)
    for addr in addrs:
      r = res[addr]
      with self.subTest(addr=hex(addr)):
        self.assertEqual([(v.kind, round(v.t, 3), v.detail) for v in r.violations], [])
        self.assertEqual((r.takeovers, r.releases), (takeovers, releases))
        self.assertLessEqual(len(r.notes), 2)    # at most the bounded step at each hand-off
    return res

  def test_acc_main_with_stale_lateral_flag(self):
    """The drive that faulted the car: ACC main pressed, panda's lateral flag stays set for 2.4 s."""
    for seed in range(8):
      with self.subTest(seed=seed):
        sim = Sim(Scenario("c0", seed=seed)).run()
        res = self.check_ok(sim)
        for r in res.values():
          self.assertLess(r.max_gap_s, 0.045)
        # nothing of ours is rejected from the takeover until the hand-back (the release frame itself always is)
        self.assertEqual([(round(t, 3), hex(a)) for t, a in sim.rejected if 1.0 <= t < 3.95], [])

  def test_release_phases(self):
    """Disengage at many phases of the stock counter, SecOC window and Comma tick."""
    for seed in range(40):
      with self.subTest(seed=seed):
        sim = Sim(Scenario("phase", seed=100 + seed, disengage_s=3.0 + 0.0137 * seed, end_s=5.5)).run()
        self.check_ok(sim)

  def test_mads_only_lateral(self):
    for seed in range(8):
      with self.subTest(seed=seed):
        sim = Sim(Scenario("mads", seed=200 + seed, cruise=False, mads=True)).run()
        self.check_ok(sim, addrs=(STEER, LAT))
        self.assertFalse([f for f in sim.bus[ACCEL] if f.source == OURS])    # stock accel is never replaced

  def test_no_long_flag_never_touches_accel(self):
    sim = Sim(Scenario("nolong", seed=300, long=False)).run()
    self.check_ok(sim, addrs=(STEER, LAT))
    self.assertFalse([f for f in sim.bus[ACCEL] if f.source == OURS])

  def test_comma_latency_spikes(self):
    for seed in range(8):
      with self.subTest(seed=seed):
        sim = Sim(Scenario("spikes", seed=400 + seed, tx_spike_prob=0.08, tx_spike_us=20_000)).run()
        self.check_ok(sim, max_gap_s=0.055)   # a 20 ms spike on top of the measured jitter, still inside panda's 50 ms fail-safe

  def test_comma_stall_fails_safe_to_stock_and_retakes(self):
    """The Comma goes quiet for 120 ms mid-engagement: panda gives the stock stream back within 50 ms."""
    sim = Sim(Scenario("stall", seed=500, stall=(2.5, 0.12))).run()
    res = audit_all(sim, failsafe_ok=True)
    for addr in (STEER, LAT, ACCEL):
      with self.subTest(addr=hex(addr)):
        kinds = {v.kind for v in res[addr].violations}
        # only the counter step of the fail-safe hand-back and the 120 ms gap it bridges are allowed
        self.assertFalse(kinds - {"gap"}, kinds)
        stock_during = [f for f in sim.bus[addr] if f.source == STOCK and 2.5 < f.t < 2.62]
        self.assertTrue(stock_during, "stock stream never came back during the stall")
        self.assertLess(min(f.t for f in stock_during) - 2.5, 0.075)
        self.assertTrue([f for f in sim.bus[addr] if f.source == OURS and f.t > 2.62], "never retook after the stall")

  def test_release_frame_sent_once_per_relay_end(self):
    sim = Sim(Scenario("ctrl", seed=600)).run()
    ctrl = [t for t, a in sim.rejected if a == RELAY_CTRL_ADDR]
    self.assertTrue(1 <= len(ctrl) <= 4)
    self.assertTrue(all(3.9 < t < 4.2 for t in ctrl))


if __name__ == "__main__":
  unittest.main()
