from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.carlog import carlog
from opendbc.car import fisker_secoc
from numpy import clip
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.fisker import fiskercan
from opendbc.car.fisker.values import CarControllerParams

LongCtrlState = structs.CarControl.Actuators.LongControlState

# NOTE: this is an early bring-up controller. The car currently runs under SafetyModel.noOutput
# (read-only), so any control messages built here are blocked by panda and never reach the car.
# The message-building path is kept real so the CRC8-J1850 + SecOC plumbing is exercised and tested
# (see opendbc/car/fisker/tests/test_secoc_crc.py). Steering/accel scaling are still provisional.
# TODO: implement a real SAFETY_FISKER panda mode before enabling TX.


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(self.CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.steer_msg_counter = 0
    self.accel_msg_counter = 0
    self.prev_lat_active = False
    self.prev_long_active = False
    self.secoc_prev_reset_cnt = None
    self.secoc_sync_valid = False

  def _has_valid_secoc_key(self) -> bool:
    return len(self.secoc_key) == 16

  @staticmethod
  def _next_counter_from_stock_state(stock_state, reset_cnt: int) -> int | None:
    if stock_state is None or stock_state.get("reset_counter") != reset_cnt:
      return None

    message_counter_lower = stock_state.get("message_counter_lower")
    counter_a = stock_state.get("counter_a")
    if message_counter_lower is None or counter_a is None:
      return None

    if (message_counter_lower & 0x0F) == counter_a:
      return (message_counter_lower + 1) & 0x3F

    return (counter_a + 1) & 0x0F

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []
    stock_control_state = getattr(CS, "stock_adas_control_state", {})

    carlog.warning(f"fisker.controller frame={self.frame} enabled={CC.enabled} latActive={CC.latActive} longActive={CC.longActive} "
                   f"secoc_key_valid={self._has_valid_secoc_key()} sync_valid={self.secoc_sync_valid} "
                   f"stock_1d0={stock_control_state.get(0x1D0)} stock_121={stock_control_state.get(0x121)}")

    # Never send SecOC-protected frames without a valid recovered key.
    if not self._has_valid_secoc_key():
      carlog.warning("fisker.controller suppress reason=invalid_secoc_key")
      new_actuators = actuators.as_builder()
      if CC.latActive:
        new_actuators.torque = actuators.torque
      self.frame += 1
      return new_actuators, can_sends

    trip_cnt = int(CS.secoc_synchronization["TRIP_CNT"])
    reset_cnt = int(CS.secoc_synchronization["RESET_CNT"])

    # *** handle SecOC reset counter increase ***
    if reset_cnt != self.secoc_prev_reset_cnt:
      self.steer_msg_counter = 0
      self.accel_msg_counter = 0
      self.secoc_prev_reset_cnt = reset_cnt

      expected_mac = int.from_bytes(fisker_secoc.sync_mac(self.secoc_key, trip_cnt, reset_cnt), "big")
      self.secoc_sync_valid = int(CS.secoc_synchronization["AUTHENTICATOR"]) == expected_mac
      carlog.warning(f"fisker.controller sync_update trip={trip_cnt} reset={reset_cnt} expected_mac=0x{expected_mac:06X} "
                     f"rx_mac=0x{int(CS.secoc_synchronization['AUTHENTICATOR']):06X} sync_valid={self.secoc_sync_valid}")
      if not self.secoc_sync_valid:
        carlog.error("SecOC synchronization MAC mismatch, wrong key?")

    if not self.secoc_sync_valid:
      carlog.warning("fisker.controller suppress reason=invalid_sync_mac")
      new_actuators = actuators.as_builder()
      if CC.latActive:
        new_actuators.torque = actuators.torque
      self.frame += 1
      return new_actuators, can_sends

    # *** lateral (ADAS_STEER_CONTROL) ***
    if CC.latActive:
      if not self.prev_lat_active:
        next_counter = self._next_counter_from_stock_state(stock_control_state.get(0x1D0), reset_cnt)
        if next_counter is not None:
          self.steer_msg_counter = next_counter
          carlog.warning(f"fisker.controller seed steer_msg_counter={self.steer_msg_counter} from stock={stock_control_state.get(0x1D0)}")
        else:
          carlog.warning(f"fisker.controller seed steer_msg_counter unavailable stock={stock_control_state.get(0x1D0)} reset={reset_cnt}")
      cs_out = getattr(CS, "out", None)
      steering_angle_deg = getattr(cs_out, "steeringAngleDeg", getattr(CS, "steeringAngleDeg", float("nan")))
      v_ego = getattr(cs_out, "vEgo", getattr(CS, "vEgo", float("nan")))
      desired_curvature = getattr(actuators, "curvature", float("nan"))
      actual_curvature = getattr(CC, "currentCurvature", float("nan"))
      carlog.warning(f"fisker.controller lateral_source torque_in={float(actuators.torque):.6f} latActive={CC.latActive} "
                     f"steeringAngleDeg={float(steering_angle_deg):.6f} vEgo={float(v_ego):.6f} "
                     f"desiredCurvature={float(desired_curvature):.8f} actualCurvature={float(actual_curvature):.8f}")
      apply_torque_unclamped = int(round(actuators.torque * self.params.STEER_MAX))
      apply_torque = int(clip(apply_torque_unclamped, self.params.STEER_MIN, self.params.STEER_MAX))
      if apply_torque != apply_torque_unclamped:
        carlog.warning(f"fisker.controller steer_saturated unclamped={apply_torque_unclamped} clamped={apply_torque} "
                       f"min={self.params.STEER_MIN} max={self.params.STEER_MAX}")
      counter_a = self.steer_msg_counter & 0x0F
      addr, dat, bus = fiskercan.create_steer_command(self.packer, apply_torque, True, counter_a)
      dat = fisker_secoc.stamp_secoc(self.secoc_key, addr, dat, trip_cnt, reset_cnt, self.steer_msg_counter)
      can_sends.append((addr, dat, bus))
      raw12 = apply_torque & 0x0FFF
      carlog.warning(f"fisker.controller emit steer addr=0x{addr:03X} bus={bus} apply_torque={apply_torque} raw12={raw12} msg_counter={self.steer_msg_counter} dat={dat.hex()}")
      self.steer_msg_counter += 1
    else:
      carlog.warning("fisker.controller suppress steer reason=latInactive")

    # *** longitudinal (ADAS_ACCEL_CONTROL) ***
    if CC.longActive:
      if not self.prev_long_active:
        next_counter = self._next_counter_from_stock_state(stock_control_state.get(0x121), reset_cnt)
        if next_counter is not None:
          self.accel_msg_counter = next_counter
          carlog.warning(f"fisker.controller seed accel_msg_counter={self.accel_msg_counter} from stock={stock_control_state.get(0x121)}")
        else:
          carlog.warning(f"fisker.controller seed accel_msg_counter unavailable stock={stock_control_state.get(0x121)} reset={reset_cnt}")
      accel = clip(actuators.accel, self.params.ACCEL_MIN, self.params.ACCEL_MAX)
      accel_payload = int(round(self.params.ACCEL_PAYLOAD_NEUTRAL + accel * self.params.ACCEL_PAYLOAD_PER_MPS2))
      accel_payload = int(clip(accel_payload, self.params.ACCEL_PAYLOAD_MIN, self.params.ACCEL_PAYLOAD_MAX))
      counter_a = self.accel_msg_counter & 0x0F
      counter_b = self.accel_msg_counter & 0x0F
      addr, dat, bus = fiskercan.create_accel_command(self.packer, accel_payload, counter_a, counter_b)
      dat = fisker_secoc.stamp_secoc(self.secoc_key, addr, dat, trip_cnt, reset_cnt, self.accel_msg_counter)
      can_sends.append((addr, dat, bus))
      carlog.warning(f"fisker.controller emit accel addr=0x{addr:03X} bus={bus} accel={float(accel):.3f} payload={accel_payload} msg_counter={self.accel_msg_counter} dat={dat.hex()}")
      self.accel_msg_counter += 1
    else:
      carlog.warning("fisker.controller suppress accel reason=longInactive")

    new_actuators = actuators.as_builder()
    if CC.latActive:
      new_actuators.torque = actuators.torque

    self.prev_lat_active = bool(CC.latActive)
    self.prev_long_active = bool(CC.longActive)
    self.frame += 1
    return new_actuators, can_sends
