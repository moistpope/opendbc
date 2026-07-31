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
# construction are now confirmed (see opendbc/car/fisker_secoc.py and fiskercan.py). The car runs
# under SAFETY_FISKER (opendbc/safety/modes/fisker.h), which torque-limits the lateral takeover;
# the steering/accel actuation limits it enforces are still provisional placeholders.


class CarControllerParams:
  # TODO: all steering limits are placeholders pending RE of ADAS_STEER_CONTROL scaling
  STEER_STEP = 1
  # LKAS_STEERING_TORQUE DBC physical range is [-192, 192].
  STEER_MAX = 192
  STEER_MIN = -192
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
  # Flags for the SAFETY_FISKER panda mode (opendbc/safety/modes/fisker.h).
  SECOC = 1         # reserved: the Ocean signs ADAS control frames with SecOC
  LONGITUDINAL = 2  # allow openpilot longitudinal (ADAS_ACCEL_CONTROL) actuation


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

# steering torque above which OP should disengage immediately (user disable).
# Kept higher than STEER_THRESHOLD so light hand torque only triggers lateral override.
STEER_DISENGAGE_THRESHOLD = 250

# 0x358 CRUISE_CONTROL_STATUS values from DBC labels.
# INACTIVE: cruise main off, SET: main on/armed, ACTIVE: speed-control engaged.
CRUISE_STATUS_INACTIVE = 0
CRUISE_STATUS_SET = 1
CRUISE_STATUS_ACTIVE = 2

# Steering wheel right-bottom button mapping.
# Use one physical button with short press to engage and long press to cancel/disengage.
STEERING_BTN_PRESSED = 1

SECOC_CAR = CAR.with_flags(FiskerFlags.SECOC)

DBC = CAR.create_dbc_map()
