from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.fisker import fiskercan
from opendbc.car.fisker.values import CarControllerParams

LongCtrlState = structs.CarControl.Actuators.LongControlState

# NOTE: this is an early bring-up controller. The car currently runs under SafetyModel.noOutput
# (read-only), so any control messages built here are blocked by panda and never reach the car.
# The message-building path is kept real so the CRC8-J1850 + SecOC plumbing is exercised and tested
# (see opendbc/car/fisker/tests/test_secoc_crc.py). Steering/accel scaling and the SecOC freshness
# decoding are still provisional. TODO: implement a real SAFETY_FISKER panda mode before enabling TX.


def _freshness(secoc_sync):
  """Trip/reset counters from GW_SECOC_SYNC (0x20): confirmed against a real capture to carry
  TRIP_CNT (bytes 0-1), an incrementing RESET_CNT (bytes 2-4), and a 24-bit AUTHENTICATOR."""
  if secoc_sync is None:
    return 0, 0
  return int(secoc_sync.get("TRIP_CNT", 0)) & 0xFFFF, int(secoc_sync.get("RESET_CNT", 0)) & 0xFFFFFF


def _freshness_value(trip_cnt, reset_cnt, msg_cnt):
  """Provisional 64-bit AUTOSAR SecOC freshness value fed to build_secoc_mac.
  WARNING: the exact composition of trip/reset/message counters into the 64-bit freshness is NOT
  yet reverse engineered -- attempts to reproduce real ADAS MACs with the recovered key and this
  layout did not match. Only the CMAC construction itself (id|payload|freshness -> top 3 bytes) is
  confirmed, via the known-answer vector in tests/secoc_crc_simulator.py."""
  return ((trip_cnt & 0xFFFF) << 32) | ((reset_cnt & 0xFFFFFF) << 8) | (msg_cnt & 0xFF)


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(self.CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.steer_msg_cnt = 0
    self.accel_msg_cnt = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    trip_cnt, reset_cnt = _freshness(CS.secoc_synchronization)

    # *** lateral (ADAS_STEER_CONTROL) ***
    if CC.latActive:
      apply_torque = int(round(actuators.torque * self.params.STEER_MAX))
      counter_a = self.steer_msg_cnt & 0xFF
      counter_b = self.steer_msg_cnt & 0x03
      payload = bytes([counter_a, apply_torque & 0xFF, (apply_torque >> 8) & 0x0F, counter_b])
      mac = fiskercan.build_secoc_mac(self.secoc_key, 0x1D0, payload, _freshness_value(trip_cnt, reset_cnt, self.steer_msg_cnt))
      can_sends.append(fiskercan.create_steer_command(self.packer, apply_torque, True, counter_a, counter_b, mac))
      self.steer_msg_cnt += 1

    # *** longitudinal (ADAS_ACCEL_CONTROL) ***
    if CC.longActive:
      # TODO: map actuators.accel (m/s^2) to the raw ADAS_ACCEL_CONTROL PAYLOAD field
      accel_payload = 0
      counter_a = self.accel_msg_cnt & 0xFF
      counter_b = self.accel_msg_cnt & 0x0F
      payload = bytes([counter_a, (accel_payload >> 4) & 0xFF, ((accel_payload & 0x0F) << 4) | counter_b, 0])
      mac = fiskercan.build_secoc_mac(self.secoc_key, 0x121, payload, _freshness_value(trip_cnt, reset_cnt, self.accel_msg_cnt))
      can_sends.append(fiskercan.create_accel_command(self.packer, accel_payload, counter_a, counter_b, mac))
      self.accel_msg_cnt += 1

    new_actuators = actuators.as_builder()
    if CC.latActive:
      new_actuators.torque = actuators.torque

    self.frame += 1
    return new_actuators, can_sends
