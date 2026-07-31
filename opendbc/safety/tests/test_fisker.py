#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, make_msg


class TestFiskerSteeringSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  """openpilot impersonates the ADAS/Hydra module on bus 0: it transmits ADAS_STEER_CONTROL (0x1D0,
  the SecOC torque command) and LKAS_STEER_AUTHORITY (0x1C0, the validity frame) while the stock
  module is isolated on bus 2."""

  TX_MSGS = [[0x1D0, 0], [0x1C0, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x1C0, 0x1D0)}
  FWD_BLACKLISTED_ADDRS = {2: [0x1C0, 0x1D0]}

  MAX_TORQUE_LOOKUP = [0], [192]
  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 25
  MAX_RT_DELTA = 150

  DRIVER_TORQUE_ALLOWANCE = 100
  DRIVER_TORQUE_FACTOR = 2

  def setUp(self):
    self.packer = CANPackerSafety("fisker_ocean_pt_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.safety.init_tests()

  def _torque_cmd_msg(self, torque, steer_req=1):
    # UNKNOWN_CONSTANT is 3 while requesting steer, 1 otherwise (the safety's steer-req signal)
    values = {"LKAS_STEERING_TORQUE": torque, "UNKNOWN_CONSTANT": 3 if steer_req else 1}
    return self.packer.make_can_msg_safety("ADAS_STEER_CONTROL", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"STEERING_WHEEL_INPUT_TORQUE": torque}
    return self.packer.make_can_msg_safety("STEERING_WHEEL1", 0, values)

  def _speed_msg(self, speed):
    values = {"WHEEL_SPEED_1": speed * 3.6, "WHEEL_SPEED_2": speed * 3.6}
    return self.packer.make_can_msg_safety("ESP_WHEEL_SPEED", 0, values)

  def _speed_msg_2(self, speed):
    # single speed source; no second source to cross-check
    return None

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PRESSED_1": 1 if brake else 0}
    return self.packer.make_can_msg_safety("BCM", 0, values)

  def _user_gas_msg(self, gas):
    values = {"ACCELERATOR_PEDAL_POSITION_ABSOLUTE": gas}
    return self.packer.make_can_msg_safety("ACCELERATOR_PEDAL", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_CONTROL_STATUS": 2 if enable else 0}
    return self.packer.make_can_msg_safety("CRUISE_CONTROL_STATUS", 0, values)

  def test_authority_frame_allowed(self):
    # LKAS_STEER_AUTHORITY carries no actuation and is always allowed
    self.assertTrue(self._tx(self.packer.make_can_msg_safety("LKAS_STEER_AUTHORITY", 0,
                                                             {"ADAS_LatCtrl_StsVld": 1})))


class TestFiskerIgnition(unittest.TestCase):
  """Ignition is detected from CAN by the brand-agnostic ignition_can_hook using MAYBE_READY (0x333),
  independent of the selected safety mode."""

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.safety.init_tests()

  def _ready_msg(self, val):
    # MAYBE_READY is a 2-bit field at (data[2] >> 4) & 0x3
    return make_msg(0, 0x333, dat=bytes([0, 0, (val & 0x3) << 4]) + b"\x00" * 5)

  def test_ignition_on(self):
    for val in (1, 2, 3):
      self.safety.set_ignition_can(False)
      self.safety.ignition_can_hook(self._ready_msg(val))
      self.assertTrue(self.safety.get_ignition_can(), f"MAYBE_READY={val} should be ignition on")

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._ready_msg(1))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._ready_msg(0))
    self.assertFalse(self.safety.get_ignition_can())

  def test_ignition_ignores_wrong_bus_and_length(self):
    self.safety.set_ignition_can(False)
    self.safety.ignition_can_hook(make_msg(1, 0x333, dat=bytes([0, 0, 0x10]) + b"\x00" * 5))
    self.assertFalse(self.safety.get_ignition_can(), "0x333 on bus != 0 must be ignored")
    self.safety.ignition_can_hook(make_msg(0, 0x333, length=4))
    self.assertFalse(self.safety.get_ignition_can(), "0x333 with len != 8 must be ignored")


if __name__ == "__main__":
  unittest.main()
