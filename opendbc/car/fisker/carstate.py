import copy

from opendbc.can import CANDefine, CANParser
from opendbc.car import create_button_events
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.fisker.values import DBC, STEER_THRESHOLD, STEER_DISENGAGE_THRESHOLD, CRUISE_STATUS_INACTIVE, CRUISE_STATUS_SET, CRUISE_STATUS_ACTIVE, STEERING_BTN_PRESSED

ButtonType = structs.CarState.ButtonEvent.Type

# Fisker Ocean gear comes from BCM->DRIVE_MODE_1 (1=PARK, 2=NEUTRAL, 3=REVERSE, 4=DRIVE)


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["BCM"]["DRIVE_MODE_1"]
    self.right_bottom_btn = 0
    self.right_bottom_btn_long = 0

    # raw SecOC synchronization / freshness message, forwarded to the controller
    self.secoc_synchronization = None

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    ret = structs.CarState()

    # SecOC freshness (GW_SECOC_SYNC). TRIP_CNT/RESET_CNT/AUTHENTICATOR layout is confirmed (see
    # opendbc/car/fisker_secoc.py); forward the whole message so the controller can track resets
    # and verify the sync MAC.
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

    # steering
    ret.steeringAngleDeg = cp.vl["STEERING"]["STEERING_ANGLE_ABSOLUTE"]
    ret.steeringRateDeg = cp.vl["STEERING"]["STEERING_ANGLE_RATE_OF_CHANGE"]
    ret.steeringTorque = cp.vl["STEERING_WHEEL1"]["STEERING_WHEEL_INPUT_TORQUE"]
    steering_torque_abs = abs(ret.steeringTorque)
    ret.steeringPressed = steering_torque_abs > STEER_THRESHOLD
    ret.steeringDisengage = steering_torque_abs > STEER_DISENGAGE_THRESHOLD

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

    # Cruise state from 0x358 CRUISE_CONTROL_STATUS.
    cruise_status = int(cp.vl["CRUISE_CONTROL_STATUS"]["CRUISE_CONTROL_STATUS"])
    ret.cruiseState.available = cruise_status in (CRUISE_STATUS_SET, CRUISE_STATUS_ACTIVE)
    ret.cruiseState.enabled = cruise_status == CRUISE_STATUS_ACTIVE

    # One-button controls: short press right-bottom to engage, long press to cancel/disengage.
    prev_right_bottom_btn = self.right_bottom_btn
    prev_right_bottom_btn_long = self.right_bottom_btn_long
    self.right_bottom_btn = int(cp.vl["STEERING_WHEEL"]["STEERING_BUTTONS_RIGHT_BOTTOM"])
    self.right_bottom_btn_long = int(cp.vl["STEERING_WHEEL"]["STEERING_BUTTONS_RIGHT_BUTTOM_LONG"])
    ret.buttonEvents = [
      *create_button_events(self.right_bottom_btn, prev_right_bottom_btn,
                            {STEERING_BTN_PRESSED: ButtonType.decelCruise}),
      *create_button_events(self.right_bottom_btn_long, prev_right_bottom_btn_long,
                            {STEERING_BTN_PRESSED: ButtonType.cancel}),
    ]

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
      ("CRUISE_CONTROL_STATUS", float('nan')),
      ("STEERING_WHEEL", float('nan')),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
    }
