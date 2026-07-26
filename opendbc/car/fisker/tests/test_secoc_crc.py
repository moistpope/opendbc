from opendbc.car.fisker.tests import secoc_crc_simulator as sim


class TestFiskerSecOCCRC:
  def test_crc8_j1850(self):
    assert sim.validate_crc() == []

  def test_fisker_secoc_matches_reference(self):
    assert sim.validate_fisker_secoc() == []

  def test_sync_helpers(self):
    assert sim.validate_sync_helpers() == []

  def test_openpilot_secoc_is_a_distinct_layout(self):
    assert sim.validate_openpilot_secoc() == []

  def test_end_to_end_signed_frames(self):
    assert sim.validate_end_to_end() == []
