from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, PlatformConfig, Platforms
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs, CarParts, CarHarness, SupportType
from opendbc.car.fw_query_definitions import FwQueryConfig

Ecu = CarParams.Ecu

# The Fisker Ocean is a fully electric SUV. This port is an early bring-up:
# the CAN/DBC has been largely reverse engineered (see opendbc/dbc/generator/fisker/), and the
# ADAS_STEER_CONTROL/ADAS_ACCEL_CONTROL frame checksum (CRC8-J1850) and SecOC MAC/freshness
# construction are now confirmed (see opendbc/car/fisker_secoc.py and fiskercan.py). There is still
# no panda safety mode (modes/fisker.h) though: the ACC/cruise-engagement state (ADAS_ACC 0x313)
# isn't decoded yet and the steering/accel actuation limits are still placeholders, so the car
# runs read-only (SafetyModel.noOutput) until those are nailed down.


class CarControllerParams:
  # TODO: all steering limits are placeholders pending RE of ADAS_STEER_CONTROL scaling
  STEER_STEP = 1
  STEER_MAX = 1000           # LKAS_STEERING_TORQUE is a 12-bit field, real max unknown
  STEER_DELTA_UP = 10
  STEER_DELTA_DOWN = 25
  STEER_ERROR_MAX = 350

  def __init__(self, CP):
    self.ACCEL_MAX = 2.0      # m/s^2
    self.ACCEL_MIN = -3.5     # m/s^2
    # ADAS_ACCEL_CONTROL.PAYLOAD is an unsigned 12-bit value with unknown OEM scaling.
    # Use a centered placeholder map so 0 m/s^2 commands the midpoint.
    self.ACCEL_PAYLOAD_MIN = 0
    self.ACCEL_PAYLOAD_MAX = 4095
    self.ACCEL_PAYLOAD_NEUTRAL = 2048
    self.ACCEL_PAYLOAD_PER_MPS2 = 300.0


class FiskerSafetyFlags(IntFlag):
  # Reserved for the future SAFETY_FISKER panda mode. The Ocean signs ADAS
  # control messages with SecOC, so it will need a SecOC-aware safety mode.
  SECOC = 1


class FiskerFlags(IntFlag):
  # ADAS control messages are protected with AUTOSAR SecOC (CRC8-J1850 + truncated CMAC)
  SECOC = 1


@dataclass
class FiskerCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))
  # SecOC cars need the per-ECU key recovered to send authenticated messages
  support_type: SupportType = SupportType.CUSTOM


class CAR(Platforms):
  FISKER_OCEAN = PlatformConfig(
    [FiskerCarDocs("Fisker Ocean 2023-24")],
    CarSpecs(mass=2300., wheelbase=2.92, steerRatio=15.0, tireStiffnessFactor=0.5),
    {Bus.pt: 'fisker_ocean_pt_generated'},
    flags=FiskerFlags.SECOC,
  )


# No FW-version fingerprinting yet: FW_VERSIONS is empty and the Ocean is identified by its CAN
# message set (see fingerprints.py). The requests list MUST stay empty until FW_VERSIONS is
# populated -- a brand with requests but no offline FW versions makes get_brand_ecu_matches return
# an empty match list and get_fw_versions_ordered() crashes with ZeroDivisionError. The config is
# still declared (not omitted) because other code indexes FW_QUERY_CONFIGS['fisker'].
#
# TODO: once ECU FW is captured on the DIAG bus, add FW_VERSIONS entries AND the matching requests:
#   0xF187 spare-part number, 0xF188 software number, 0xF191 hardware number (see _reference/ecus.json),
#   e.g. StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST / MANUFACTURER_ECU_HARDWARE_NUMBER_REQUEST.
FW_QUERY_CONFIG = FwQueryConfig(requests=[])

# steering torque (STEERING_WHEEL1->STEERING_WHEEL_INPUT_TORQUE) above which the driver
# is considered to be overriding. TODO: calibrate against real data
STEER_THRESHOLD = 100

# Placeholder ADAS_ACC states while cruise-state reverse engineering is incomplete.
# 0 appears inactive on captures; any non-zero state is treated as available.
ACC_UNAVAILABLE_STATES = (0,)
ACC_ENABLED_STATES = (2, 3)

# LEFT_STALK placeholder mapping to OP cruise buttons.
# 1/2 map to down/up full presses in the DBC comments and are treated as set/resume.
LEFT_STALK_SET = 1
LEFT_STALK_RESUME = 2
LEFT_STALK_CANCEL = 8

SECOC_CAR = CAR.with_flags(FiskerFlags.SECOC)

DBC = CAR.create_dbc_map()
