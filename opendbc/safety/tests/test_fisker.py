#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.libsafety import libsafety_py

ICC_SETTINGS = 0x52A
ICC_FRAME = bytes.fromhex("BE2026A043008220")
TIMEOUT_US = 500_000


class TestFiskerIccSettings(unittest.TestCase):
  """ICC_0x52A: openpilot re-sends the ICC's frame on bus 2; panda drops the ICC's original
  (bus 0 -> 2) only while openpilot's copies keep arriving."""

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_timer(0)  # init_tests() also resets the timer to 0; keep init's timestamp consistent
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.safety.init_tests()

  def _tx(self, bus):
    return self.safety.safety_tx_hook(common.make_msg(bus, ICC_SETTINGS, dat=ICC_FRAME))

  def _fwd(self, bus):
    return self.safety.safety_fwd_hook(bus, ICC_SETTINGS)

  def test_tx_allowed_on_adas_bus_only(self):
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertTrue(self._tx(2))
      self.assertFalse(self._tx(0))
      self.assertFalse(self._tx(1))

  def test_tx_allowed_with_long_flag(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 1)
    self.assertTrue(self._tx(2))

  def test_blocked_at_init(self):
    # openpilot is already sending when the safety mode starts: block from the first ICC frame
    self.assertEqual(self._fwd(0), -1)
    self.safety.set_timer(TIMEOUT_US - 1)
    self.assertEqual(self._fwd(0), -1)

  def test_forwarded_if_openpilot_never_sends(self):
    self.safety.set_timer(TIMEOUT_US)
    self.assertEqual(self._fwd(0), 2)

  def test_blocked_while_openpilot_sends(self):
    # each of our frames extends the window past the init window
    for t in range(0, 5_000_000, 200_000):
      self.safety.set_timer(t)
      self.assertTrue(self._tx(2))
      self.assertEqual(self._fwd(0), -1)
    self.safety.set_timer(t + TIMEOUT_US - 1)
    self.assertEqual(self._fwd(0), -1)
    self.safety.set_timer(t + TIMEOUT_US)
    self.assertEqual(self._fwd(0), 2)

  def test_fallback_to_icc_when_openpilot_stops(self):
    self.assertTrue(self._tx(2))
    self.safety.set_timer(TIMEOUT_US)
    self.assertEqual(self._fwd(0), 2)

    # resumes blocking as soon as openpilot sends again
    self.assertTrue(self._tx(2))
    self.assertEqual(self._fwd(0), -1)

  def test_window_restarts_on_safety_mode_init(self):
    self.safety.set_timer(2 * TIMEOUT_US)
    self.assertEqual(self._fwd(0), 2)
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.assertEqual(self._fwd(0), -1)

  def test_adas_side_not_blocked(self):
    # only the ICC -> ADAS direction is intercepted
    self.assertTrue(self._tx(2))
    self.assertEqual(self._fwd(2), 0)

  def test_no_relay_malfunction(self):
    # 0x52A on bus 2 must not trip relay malfunction (that would stop all forwarding)
    self.safety.safety_rx_hook(common.make_msg(2, ICC_SETTINGS, dat=ICC_FRAME))
    self.assertFalse(self.safety.get_relay_malfunction())


if __name__ == "__main__":
  unittest.main()
