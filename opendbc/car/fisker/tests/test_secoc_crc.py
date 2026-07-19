from opendbc.car.fisker.tests import secoc_crc_simulator as sim


class TestFiskerSecOCCRC:
  def test_crc8_j1850(self):
    assert sim.validate_crc() == []

  def test_secoc_known_answer(self):
    # fiskercan.build_secoc_mac must reproduce the captured ground-truth MAC (0x498330)
    assert sim.validate_secoc_known_answer() == []

  def test_fisker_secoc_matches_reference(self):
    assert sim.validate_fisker_secoc() == []

  def test_openpilot_secoc_self_consistent(self):
    assert sim.validate_openpilot_secoc() == []

  def test_end_to_end_signed_frames(self):
    assert sim.validate_end_to_end() == []
