from opendbc.car import Bus
from opendbc.can import CANParser
from opendbc.car.fisker.values import CAR, DBC, SECOC_CAR
from opendbc.car.fisker.carstate import CarState
from opendbc.car.fisker.fingerprints import FINGERPRINTS

# messages carstate/carcontroller depend on; the DBC must define all of them
REQUIRED_MESSAGES = [
  "GW_SECOC_SYNC", "ESP_WHEEL_SPEED", "BCM", "STEERING", "STEERING_WHEEL1",
  "ACCELERATOR_PEDAL", "BCM_DOORS", "BCM_LIGHTS", "PARKING_BRAKES", "MAYBE_READY",
  "ADAS_STEER_CONTROL", "ADAS_ACCEL_CONTROL",
]


class TestFiskerInterface:
  def test_single_ocean_platform(self):
    assert set(CAR) == {CAR.FISKER_OCEAN}

  def test_ocean_is_secoc(self):
    assert CAR.FISKER_OCEAN in SECOC_CAR

  def test_dbc_has_required_messages(self):
    # building a CANParser for every message we use asserts they exist in the DBC
    dbc = DBC[CAR.FISKER_OCEAN][Bus.pt]
    CANParser(dbc, [(m, 0) for m in REQUIRED_MESSAGES], 0)

  def test_carstate_parsers_build(self):
    CarState.get_can_parsers(_ocean_cp())

  def test_fingerprint_present_and_addressable(self):
    fp = FINGERPRINTS[CAR.FISKER_OCEAN][0]
    assert len(fp) > 50
    # ignition (MAYBE_READY 0x333) and both ADAS control messages must be in the fingerprint
    assert 0x333 in fp and 0x1D0 in fp and 0x121 in fp
    # all fingerprint addresses are 11-bit (matter for fingerprinting)
    assert all(addr < 0x800 for addr in fp)


def _ocean_cp():
  from opendbc.car.structs import CarParams
  cp = CarParams()
  cp.carFingerprint = CAR.FISKER_OCEAN
  return cp
