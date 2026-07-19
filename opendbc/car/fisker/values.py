from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, PlatformConfig, Platforms
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs, CarParts, CarHarness, SupportType
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

# The Fisker Ocean is a fully electric SUV. This port is an early bring-up:
# the CAN/DBC has been largely reverse engineered (see opendbc/dbc/generator/fisker/)
# but there is no panda safety mode yet, so the car runs read-only (SafetyModel.noOutput).


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


# Fisker ECUs answer standard UDS ReadDataByIdentifier on the DIAG bus:
#   0xF187 spare-part number, 0xF188 software number, 0xF191 hardware number
# (see _reference/ecus.json). The DIAG bus is physically the comma's can3/OBD-II
# transceiver; queries are sent on panda bus 0 here as a starting point.
FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_ECU_HARDWARE_NUMBER_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_ECU_HARDWARE_NUMBER_RESPONSE],
      bus=0,
    ),
  ],
)

# steering torque (STEERING_WHEEL1->STEERING_WHEEL_INPUT_TORQUE) above which the driver
# is considered to be overriding. TODO: calibrate against real data
STEER_THRESHOLD = 100

SECOC_CAR = CAR.with_flags(FiskerFlags.SECOC)

DBC = CAR.create_dbc_map()
