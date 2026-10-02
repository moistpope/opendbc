import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.carlog import carlog
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

from opendbc.car.fisker.fiskercan import FiskerCAN
from opendbc.car.fisker.relay import RelayClock
from opendbc.car.fisker.secoc import stamp_secoc, sync_mac
from opendbc.car.fisker.values import CarControllerParams, ICC_0x35B_OVERRIDES, RELAY_RELEASE_ACCEL, RELAY_RELEASE_STEER_LAT


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

    # Counters for the messages we replace (steering 0x1D0, lateral activation 0x1C0, accel 0x121)
    # come from the stock stream, not from our own tick: one relay set per stock 0x1D0 frame, with the
    # AliveCounter and SecOC counter continuing the stock ones (see relay.py). The stock module's five
    # 100 Hz commands share one counter in lockstep, so one clock serves all three.
    self.relay = RelayClock()
    self.lat_relaying = False
    self.long_relaying = False
    self.release_mask = 0          # relays handed back and not yet confirmed to panda
    self.release_sends_left = 0

    self.secoc_key_verified = False
    self.secoc_warn_logged = False
    self.secoc_unsynced_logged = False

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
    """Fill the SecOC tail (fresh byte + MAC) of a packed frame with the relay's SecOC counter."""
    addr, data, bus = msg
    stamped = stamp_secoc(self.secoc_key, can_id, data, trip, reset, msg_counter)
    return addr, stamped, bus

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    self._maybe_verify_key(CS)
    secoc_ok = self.CP.secOcKeyAvailable and self.secoc_key_verified
    trip = CS.secoc_trip

    # ---- Which relays are on ----
    # Lateral (0x1D0 steering angle + 0x1C0 activation): for the WHOLE engaged window, so the EPS never
    # sees the stream change hands mid-engagement. Engagement can come from regular cruise
    # (cruiseState.enabled) OR sunnypilot MADS (CC_SP.mads.enabled).
    #
    # DISABLED: driver-torque override (release the EPS on steeringPressed). The Ocean's EPS_DrvrSteerTq
    # reads torque on the whole steering column, including reaction torque from the ADAS assist itself,
    # so steeringPressed fired with the hands off the wheel and cancelled engagement. Req follows
    # CC.latActive only; `driver_override` on 0x1C0 is Gateway-only (the EPS does not read it).
    #
    # Longitudinal (0x121 accel, alpha long only): while cruise is engaged. Only the accel is replaced;
    # the stock 0x117/0x118 already say "ACC Active" while the stock ACC is engaged (engagement follows
    # 0x313), so openpilot never sends them.
    lat_active = CC.latActive and secoc_ok
    mads_engaged = bool(CC_SP.mads.enabled) if CC_SP is not None else False
    lat_relay = (CS.out.cruiseState.enabled or mads_engaged) and secoc_ok
    long_relay = self.CP.openpilotLongitudinalControl and CS.out.cruiseState.enabled and secoc_ok

    self.apply_angle_last = apply_std_steer_angle_limits(
      actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
      CS.out.steeringAngleDeg, lat_active, self.params.ANGLE_LIMITS,
    )

    accel = 0.0
    gas_override = CS.out.gasPressed         # driver commanding accel via pedal
    if long_relay and CC.longActive and not gas_override:
      accel = float(np.clip(actuators.accel, self.params.ACCEL_MIN, self.params.ACCEL_MAX))

    # ---- Relay: one set of frames per stock 0x1D0 frame ----
    # The stock stream is always observed so the Reset-window position is known before we need it.
    stock = self.relay.observe(CS.oem_frames, CS.secoc_reset)
    relaying = lat_relay or long_relay
    if relaying:
      if not self.relay.synced and not self.secoc_unsynced_logged:
        carlog.warning("Fisker relay waiting for a SecOC Reset-window boundary")
        self.secoc_unsynced_logged = True
      # When the relay starts, only the newest stock frame is relayed: the older ones in this batch are
      # already behind the stock counter, and panda would reject them (they must continue it).
      to_relay = stock if self.relay.relaying else stock[-1:]
      for alive_stock, _, _, _ in to_relay:
        tick = self.relay.tick(alive_stock, CS.secoc_reset)
        if tick is None:
          continue
        if lat_relay:
          steer_msg = self.fcan.create_steering_control(self.apply_angle_last, tick.alive)
          can_sends.append(self._stamp(steer_msg, STEER_CAN_ID, trip, tick.secoc_reset, tick.secoc_ctr))
          can_sends.append(self.fcan.create_lat_control(lat_active, tick.alive, driver_override=False))
        if long_relay:
          accel_msg = self.fcan.create_accel_command(accel, tick.alive)
          can_sends.append(self._stamp(accel_msg, ACCEL_CAN_ID, trip, tick.secoc_reset, tick.secoc_ctr))
    else:
      self.relay.stop()

    # ---- Hand back ----
    # When a relay ends, tell panda so it resumes the stock stream at exactly our last counter + 1
    # (the release frame is consumed and rejected there, it never reaches the car). Repeated a few
    # times in case one is lost; dropped if the relay restarts first.
    if self.lat_relaying and not lat_relay:
      self.release_mask |= RELAY_RELEASE_STEER_LAT
      self.release_sends_left = 3
    if self.long_relaying and not long_relay:
      self.release_mask |= RELAY_RELEASE_ACCEL
      self.release_sends_left = 3
    if lat_relay:
      self.release_mask &= ~RELAY_RELEASE_STEER_LAT
    if long_relay:
      self.release_mask &= ~RELAY_RELEASE_ACCEL
    if self.release_mask and self.release_sends_left > 0:
      can_sends.append(self.fcan.create_relay_release(self.release_mask))
      self.release_sends_left -= 1
    if self.release_sends_left == 0:
      self.release_mask = 0
    self.lat_relaying, self.long_relaying = lat_relay, long_relay

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
