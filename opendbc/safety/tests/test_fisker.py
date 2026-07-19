#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import make_msg


class TestFiskerIgnition(unittest.TestCase):
  """The Fisker Ocean has no panda safety mode yet; ignition is detected from CAN by the
  brand-agnostic ignition_can_hook (opendbc/safety/ignition.h) using MAYBE_READY (0x333)."""

  def setUp(self):
    self.safety = libsafety_py.libsafety
    # ignition_can_hook is independent of the selected safety mode; init with noOutput
    self.safety.set_safety_hooks(CarParams.SafetyModel.noOutput, 0)
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
