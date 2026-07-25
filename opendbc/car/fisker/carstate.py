import copy

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.fisker.values import DBC, STEER_THRESHOLD

# Fisker Ocean gear comes from BCM->DRIVE_MODE_1 (1=PARK, 2=NEUTRAL, 3=REVERSE, 4=DRIVE)


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["BCM"]["DRIVE_MODE_1"]

    # raw SecOC synchronization / freshness message, forwarded to the controller
    self.secoc_synchronization = None

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    ret = structs.CarState()

    # SecOC freshness (GW_SECOC_SYNC). Signal decoding is still being reverse engineered;
    # forward the whole message so the controller/validator can consume it.
    self.secoc_synchronization = copy.copy(cp.vl["GW_SECOC_SYNC"])

    # speed. ESP_WHEEL_SPEED reports two wheels scaled to km/h (0.075 factor applied by the parser).
    # TODO: confirm which wheels these are and whether a 4-wheel source exists (WHEEL_SPEED, 0x125)
    ws1 = cp.vl["ESP_WHEEL_SPEED"]["WHEEL_SPEED_1"]
    ws2 = cp.vl["ESP_WHEEL_SPEED"]["WHEEL_SPEED_2"]
    self.parse_wheel_speeds(ret, ws1, ws1, ws2, ws2, unit=CV.KPH_TO_MS)
    ret.standstill = abs(ret.vEgoRaw) < 1e-3

    # gear
    can_gear = int(cp.vl["BCM"]["DRIVE_MODE_1"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    # steering. TODO: STEERING_ANGLE_ABSOLUTE is a raw 15-bit count (a real capture showed values
    # like 12442); a scale factor and center offset still need to be reverse engineered.
    ret.steeringAngleDeg = cp.vl["STEERING"]["STEERING_ANGLE_ABSOLUTE"]
    ret.steeringRateDeg = cp.vl["STEERING"]["STEERING_ANGLE_RATE_OF_CHANGE"]
    ret.steeringTorque = cp.vl["STEERING_WHEEL1"]["STEERING_WHEEL_INPUT_TORQUE"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # pedals / brake
    ret.brakePressed = cp.vl["BCM"]["BRAKE_PRESSED_1"] != 0
    ret.gasPressed = cp.vl["ACCELERATOR_PEDAL"]["ACCELERATOR_PEDAL_POSITION_ABSOLUTE"] > 0

    # doors / seatbelt / parking brake
    ret.doorOpen = cp.vl["BCM_DOORS"]["MAYBE_DOOR_OPEN"] != 0
    ret.parkingBrake = cp.vl["PARKING_BRAKES"]["PARKING_BRAKE_ACTUATOR_1"] != 0

    # blinkers
    ret.leftBlinker = cp.vl["BCM_LIGHTS"]["LEFT_TURN_SIGNAL_ACTIVE"] != 0
    ret.rightBlinker = cp.vl["BCM_LIGHTS"]["RIGHT_TURN_SIGNAL_ACTIVE"] != 0
    ret.genericToggle = cp.vl["BCM_LIGHTS"]["HIGH_BEAMS_ACTIVE"] != 0

    # TODO: decode the Ocean ACC state (ADAS_ACC 0x313 / ADAS_ACC_HUD 0x31C). Until then the car
    # runs read-only (SafetyModel.noOutput) and cruise is reported unavailable.
    ret.cruiseState.available = False
    ret.cruiseState.enabled = False

    return ret

  @staticmethod
  def get_can_parsers(CP):
    pt_messages = [
      ("GW_SECOC_SYNC", float('nan')),
      ("ESP_WHEEL_SPEED", float('nan')),
      ("BCM", float('nan')),
      ("STEERING", float('nan')),
      ("STEERING_WHEEL1", float('nan')),
      ("ACCELERATOR_PEDAL", float('nan')),
      ("BCM_DOORS", float('nan')),
      ("BCM_LIGHTS", float('nan')),
      ("PARKING_BRAKES", float('nan')),
      ("MAYBE_READY", float('nan')),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
    }
