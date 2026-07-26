from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.carlog import carlog
from opendbc.car import fisker_secoc
from opendbc.car.common.numpy_fast import clip
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
    self.secoc_prev_reset_cnt = None

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    trip_cnt = int(CS.secoc_synchronization["TRIP_CNT"])
    reset_cnt = int(CS.secoc_synchronization["RESET_CNT"])

    # *** handle SecOC reset counter increase ***
    if reset_cnt != self.secoc_prev_reset_cnt:
      self.steer_msg_counter = 0
      self.accel_msg_counter = 0
      self.secoc_prev_reset_cnt = reset_cnt

      expected_mac = int.from_bytes(fisker_secoc.sync_mac(self.secoc_key, trip_cnt, reset_cnt), "big")
      if int(CS.secoc_synchronization["AUTHENTICATOR"]) != expected_mac:
        carlog.error("SecOC synchronization MAC mismatch, wrong key?")

    # *** lateral (ADAS_STEER_CONTROL) ***
    if CC.latActive:
      apply_torque = int(round(actuators.torque * self.params.STEER_MAX))
      counter_a = self.steer_msg_counter & 0xFF
      addr, dat, bus = fiskercan.create_steer_command(self.packer, apply_torque, True, counter_a)
      dat = fisker_secoc.stamp_secoc(self.secoc_key, addr, dat, trip_cnt, reset_cnt, self.steer_msg_counter)
      can_sends.append((addr, dat, bus))
      self.steer_msg_counter += 1

    # *** longitudinal (ADAS_ACCEL_CONTROL) ***
    if CC.longActive:
      accel = clip(actuators.accel, self.params.ACCEL_MIN, self.params.ACCEL_MAX)
      accel_payload = int(round(self.params.ACCEL_PAYLOAD_NEUTRAL + accel * self.params.ACCEL_PAYLOAD_PER_MPS2))
      accel_payload = int(clip(accel_payload, self.params.ACCEL_PAYLOAD_MIN, self.params.ACCEL_PAYLOAD_MAX))
      counter_a = self.accel_msg_counter & 0xFF
      counter_b = self.accel_msg_counter & 0x0F
      addr, dat, bus = fiskercan.create_accel_command(self.packer, accel_payload, counter_a, counter_b)
      dat = fisker_secoc.stamp_secoc(self.secoc_key, addr, dat, trip_cnt, reset_cnt, self.accel_msg_counter)
      can_sends.append((addr, dat, bus))
      self.accel_msg_counter += 1

    new_actuators = actuators.as_builder()
    if CC.latActive:
      new_actuators.torque = actuators.torque

    self.frame += 1
    return new_actuators, can_sends
