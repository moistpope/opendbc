import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.carlog import carlog
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

from opendbc.car.fisker.fiskercan import FiskerCAN
from opendbc.car.fisker.secoc import stamp_secoc, sync_mac
from opendbc.car.fisker.values import CarControllerParams, ICC_0x35B_OVERRIDES


VisualAlert = structs.CarControl.HUDControl.VisualAlert

# CAN IDs of the SecOC-protected actuator messages we transmit.
STEER_CAN_ID = 0x1D0
ACCEL_CAN_ID = 0x121


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.params = CarControllerParams
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.fcan = FiskerCAN(CP, self.packer)

    self.apply_angle_last = 0.0

    # SecOC message counter. The counter is a per-Reset-window frame index, NOT a free
    # monotonic counter. It restarts at 1 on the first 0x1D0/0x121 frame after the GW
    # Reset counter (0x20) increments, then +1 per 100 Hz frame (reaching ~101 before the
    # next Reset ~1 s later). Only the low 6 bits appear on the wire (SSecOC_Fresh_Byte0);
    # the MAC authenticates the full 64-bit freshness. The EPS reconstructs this window
    # index for anti-replay, so we must reproduce it exactly — a free monotonic counter
    # diverges from the window rule and
    # the EPS rejects every frame (no actuation + ADAS fault). 0x1D0 and 0x121 are both
    # 100 Hz and restart on the same boundary, so they share the index value.
    self.secoc_window_ctr = 0
    self.secoc_prev_reset = None

    self.secoc_key_verified = False
    self.secoc_warn_logged = False

    # "We ARE OEM" AliveCounter substitution: every OEM 0x1D0/0x1C0 tick that panda
    # blocks, we transmit a frame with the SAME AliveCounter that OEM would have sent —
    # our stream literally continues OEM's numbering during the blocked window. At
    # disengage panda unblocks OEM, and OEM's next tick lands at our_last + 1 which is
    # exactly what EPS was expecting next — seamless handoff both directions, no
    # momentary BSM error on disengage (see user's diagram). Only transmit when OEM's
    # bus-2 counter has advanced since our previous send, else we'd emit a duplicate
    # (EPS rejects duplicates in the E2E monotonic check).
    self.alive_1d0 = 0
    self.alive_1c0 = 0
    self.last_sent_alive_1d0 = -1   # invalid sentinel — force initialization at engage
    self.last_sent_alive_1c0 = -1
    self.was_engaged_prev = False

  def _maybe_verify_key(self, CS) -> None:
    """Verify the stored SecOC key against the GW sync MAC once at startup."""
    if self.secoc_key_verified or not CS.secoc_sync_seen or not self.secoc_key:
      return
    if sync_mac(self.secoc_key, CS.secoc_trip, CS.secoc_reset) == CS.secoc_sync_mac:
      self.secoc_key_verified = True
      carlog.info("Fisker SecOC key verified against GW sync MAC")
    elif not self.secoc_warn_logged:
      carlog.error("Fisker SecOC key mismatch — GW sync MAC does not match stored SecOCKey")
      self.secoc_warn_logged = True

  def _stamp(self, msg, can_id, trip, reset, msg_counter):
    """Fill the SecOC tail (fresh byte + MAC) of a packed frame with the per-window
    message counter (see self.secoc_window_ctr)."""
    addr, data, bus = msg
    stamped = stamp_secoc(self.secoc_key, can_id, data, trip, reset, msg_counter)
    return addr, stamped, bus

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    self._maybe_verify_key(CS)
    secoc_ok = self.CP.secOcKeyAvailable and self.secoc_key_verified
    trip, reset = CS.secoc_trip, CS.secoc_reset

    # Maintain the per-Reset-window SecOC frame index (see __init__). It free-runs at the
    # 100 Hz control rate and restarts on every GW Reset change, so at engagement it
    # already matches the OEM/EPS window position: the first injected 0x1D0 is accepted
    # and every subsequent window stays in lockstep (both restart at 1 on each boundary).
    if reset != self.secoc_prev_reset:
      self.secoc_window_ctr = 0
      self.secoc_prev_reset = reset
    self.secoc_window_ctr += 1

    # E2E AliveCounter (byte1 low nibble): 0..14, +1 per frame, never 15 (the E2E invalid sentinel).
    # It is tracked per tick (alive_1d0/alive_1c0 below), seeded from the stock module's counter at
    # every engage transition so our first frame is what the receivers expect next.

    # ---- Lateral (steering angle 0x1D0 + activation 0x1C0 @ 100 Hz) ----
    # Send for the WHOLE engaged window so the cluster/EPS never see the frame disappear.
    # Engagement can come from EITHER regular cruise (cruiseState.enabled) OR sunnypilot
    # MADS (CC_SP.mads.enabled). The panda's fisker_fwd_hook mirrors this: it blocks the
    # OEM's 0x1D0/0x1C0 on bus 2 whenever (controls_allowed || controls_allowed_lateral),
    # which is the exact same window. If openpilot stops transmitting during that window
    # (e.g. MADS engaged without cruise), the EPS receives NEITHER openpilot's frames
    # nor the OEM's (panda blocks the OEM's) → LKA fault + wheel doesn't move.
    #
    # DISABLED: driver-torque override (release EPS on steeringPressed). The Ocean's
    # EPS_DrvrSteerTq reads torque on the whole steering column — including reaction
    # torque from the ADAS assist itself while it's actively steering. That meant
    # `steeringPressed` fired even when the driver wasn't touching the wheel, dropping
    # Req=0 and cancelling engagement. Until we can decouple driver torque from motor
    # reaction (either a smarter threshold, a longer debounce, or a different sensor
    # input), just gate Req on CC.latActive directly and let openpilot's stock nudge
    # behaviour handle overrides. `driver_override` on 0x1C0 is Gateway-only (EPS doesn't
    # read it) so we send False to keep the wire quiet.
    lat_active = CC.latActive and secoc_ok
    mads_engaged = bool(CC_SP.mads.enabled) if CC_SP is not None else False
    engaged = (CS.out.cruiseState.enabled or mads_engaged) and secoc_ok
    self.apply_angle_last = apply_std_steer_angle_limits(
      actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
      CS.out.steeringAngleDeg, lat_active, self.params.ANGLE_LIMITS,
    )

    # "We ARE OEM" AliveCounter substitution — seed once from OEM at engage, then
    # increment 1 per tick to stay in perfect lockstep with OEM (both at 100 Hz).
    #
    # Illustrating (0x1D0 example):
    #   OEM on bus 2 (all ticks):        1  2  3  4  5  6  7  8  9  10 11 12
    #   Panda forwards to bus 0:         1  2  3  4                    11 12
    #                                    └── ENGAGE ──┘     └─ DISENGAGE
    #   Our TX (self-incrementing):               5  6  7  8  9  10
    #
    # Previous version used `alive = OEM_bus2_current` per-tick. Problem: if OEM ticked
    # twice between our two carcontroller reads (tick jitter), we'd read a value that
    # skipped one from our last-sent — creating a +2 jump on the wire that the E2E
    # validator rejects. After a few of these, LKA faults mid-engagement.
    #
    # Seed-and-increment eliminates that: our counter marches at exactly our tick rate
    # (which is nominally the same as OEM's), so we produce every value once, in order,
    # regardless of when OEM's frames land in carstate.
    #
    # SecOC msg counter uses the same seed-and-increment idea — the per-Reset-window
    # free-running counter (self.secoc_window_ctr) already increments +1 per tick and
    # resets on GW Reset boundary. At engage rising edge we snap it to OEM's observed
    # counter so we start matching OEM's actual counter value, then let the free-running
    # increment carry it forward.
    if engaged and not self.was_engaged_prev:
      # Rising edge: seed our alive to (OEM_current - 1) so that AFTER the standard
      # per-tick increment below, our first-TX alive equals OEM_current — matching the
      # value OEM's blocked frame would have carried. This is the "we ARE OEM" model:
      # our stream literally continues OEM's numbering during the blocked window.
      #
      # At disengage, our last alive was OEM_current+N. OEM has been ticking at 100 Hz
      # in lockstep with our carcontroller (also 100 Hz), so OEM's counter is also at
      # OEM_current+N when we stop. Panda unblocks; OEM's next tick lands on bus 0 at
      # OEM_current+N+1 — which is exactly our_last + 1, i.e. what EPS expects next.
      # Clean handoff both directions, no +1 jump at engage, no duplicate at disengage.
      self.alive_1d0 = (int(CS.oem_1d0_alive) - 1) % 15
      self.alive_1c0 = (int(CS.oem_1c0_alive) - 1) % 15
      # SecOC msg counter: snap low 6 bits to (OEM_wire - 1) mod 64. The free-running
      # secoc_window_ctr already got its +1 earlier in this update, so after this snap
      # + no further change, secoc_window_ctr's low bits = OEM_wire - 1. Then... wait,
      # we WANT low bits = OEM_wire on first TX. So snap to OEM_wire directly (no -1),
      # since secoc_window_ctr is NOT going to increment again in this tick.
      self.secoc_window_ctr = (self.secoc_window_ctr & ~0x3F) | (int(CS.oem_1d0_secoc_wire_ctr) & 0x3F)
    self.was_engaged_prev = engaged

    if engaged:
      # Per-tick increment: alive matches OEM's tick rate exactly (both 100 Hz) so we
      # produce every value once in order, immune to read-jitter that would otherwise
      # cause skips (see prior "alive = oem_now" implementation which occasionally
      # jumped +2 and tripped E2E validation).
      self.alive_1d0 = (self.alive_1d0 + 1) % 15
      self.alive_1c0 = (self.alive_1c0 + 1) % 15
      # secoc_window_ctr's per-tick increment already happened earlier in this update
      # (free-running block above). On rising edge tick it was set to OEM_wire and no
      # further change is applied this tick, so our first TX carries msg_counter
      # matching OEM_current exactly. From next tick on, the free-running increment
      # carries it forward in lockstep with OEM.
      steer_msg = self.fcan.create_steering_control(self.apply_angle_last, self.alive_1d0)
      can_sends.append(self._stamp(steer_msg, STEER_CAN_ID, trip, reset, self.secoc_window_ctr))
      can_sends.append(self.fcan.create_lat_control(lat_active, self.alive_1c0, driver_override=False))

    # ---- Longitudinal (accel 0x121 + status 0x117 + ESP handshake 0x118 @ 100 Hz) ----
    # openpilot's own longitudinal control (planner/e2e model -> accel), not the stock ACC's.
    # Same "we ARE OEM" takeover as lateral: while cruise is engaged and openpilot drives
    # longitudinal, panda blocks the stock module's 0x121/0x117/0x118 on bus 2 and we send ours in
    # their place on every tick, so the VCU/ESP see exactly one stream. The stock ACC state (0x313)
    # still gates engagement (panda's controls_allowed follows it), and its frames take over again
    # the moment we stop. All five ADAS messages share one AliveCounter on the stock module, so
    # these use the same lockstep value as 0x1D0/0x1C0 (seeded from the stock counter at engage).
    # We send only inside the window panda blocks (cruise engaged, not MADS-only); outside it the
    # stock frames flow and any frame of ours would be a duplicate with its own counter.
    #   long_active, no gas:  AccelReq = openpilot's accel
    #   engaged, otherwise:   AccelReq = 0 (Vld stays Valid, as on the stock frame)
    if self.CP.openpilotLongitudinalControl and engaged and CS.out.cruiseState.enabled:
      long_active = CC.longActive and secoc_ok
      gas_override = CS.out.gasPressed         # driver commanding accel via pedal
      accel = 0.0
      if long_active and not gas_override:
        accel = float(np.clip(actuators.accel, self.params.ACCEL_MIN, self.params.ACCEL_MAX))

      accel_msg = self.fcan.create_accel_command(accel, self.alive_1d0)
      can_sends.append(self._stamp(accel_msg, ACCEL_CAN_ID, trip, reset, self.secoc_window_ctr))
      can_sends.append(self.fcan.create_long_status(gas_override, CS.oem_isa_spd_lmt, self.alive_1d0))
      can_sends.append(self.fcan.create_long_esp_handshake(self.alive_1d0))

    # ---- ICC feature settings (0x52A, ICC on bus 0 -> ADAS module on bus 2) ----
    # Always on, independent of engagement: panda blocks the ICC's own 0x52A from reaching the
    # ADAS module for as long as we keep sending, and we re-send each ICC frame with
    # ICC_SETTINGS_OVERRIDES applied (this is what enables ACC). The ICC's AliveCounter is
    # carried through unchanged, so the ADAS module sees one continuous counter sequence — and
    # if we stop, panda's fallback forwards the ICC's frames without a counter jump.
    for icc_values in CS.icc_settings_frames:
      can_sends.append(self.fcan.create_icc_settings(icc_values))

    # ---- ICC SVS / BSD / APA settings (0x35B, ICC on bus 0 -> ADAS module on bus 2) ----
    # Optional: only relayed when ICC_0x35B_OVERRIDES has entries. Panda blocks the ICC's own
    # 0x35B only while our copies are arriving, so with no overrides it's forwarded untouched.
    if ICC_0x35B_OVERRIDES:
      for icc_values in CS.icc_0x35b_frames:
        can_sends.append(self.fcan.create_icc_0x35b(icc_values))

    # ---- HUD ----
    # Forwarding intercept: the OEM ADAS module stays alive on bus 2 and the panda
    # forwards its status/HUD frames (ACC HUD 0x31C, warning HUD 0x317) to the cluster,
    # so openpilot must NOT also transmit them — that would collide with the OEM's. It
    # only injects the steering command; the OEM keeps driving the cluster/HUD.

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
