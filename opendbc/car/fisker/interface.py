from opendbc.car import get_safety_config, structs
from opendbc.car.fisker.values import FiskerFlags, FiskerSafetyFlags
from opendbc.car.fisker.carstate import CarState
from opendbc.car.fisker.carcontroller import CarController
from opendbc.car.fisker.radar_interface import RadarInterface
from opendbc.car.interfaces import CarInterfaceBase

SafetyModel = structs.CarParams.SafetyModel


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "fisker"
    ret.dashcamOnly = False

    # Lateral is torque-based. Use the shared torque-tune path so commanded curvature
    # can produce non-zero actuator torque during bring-up.
    ret.steerActuatorDelay = 0.15
    ret.steerLimitTimer = 0.4
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    ret.steerControlType = structs.CarParams.SteerControlType.torque

    if ret.flags & FiskerFlags.SECOC:
      ret.secOcRequired = True

    # radar (MRR 0x33B) is not decoded yet
    ret.radarUnavailable = True

    # Longitudinal takeover is not mapped well enough yet; keep stock ACC longitudinal active
    # and limit bring-up to lateral/SecOC steering takeover.
    ret.openpilotLongitudinalControl = False
    ret.pcmCruise = True

    # SAFETY_FISKER (opendbc/safety/modes/fisker.h): torque-limited lateral takeover with the
    # authority frame allowlisted. Longitudinal actuation is gated behind the LONGITUDINAL flag and
    # left off here (stock ACC longitudinal stays active). Steer/accel limits are still provisional
    # (see values.py / the mode).
    safety_param = 0
    if ret.openpilotLongitudinalControl:
      safety_param |= FiskerSafetyFlags.LONGITUDINAL
    ret.safetyConfigs = [get_safety_config(SafetyModel.fisker, safety_param)]
    ret.minEnableSpeed = -1.0
    ret.stopAccel = -2.0
    ret.longitudinalTuning.kpBP = [0.0, 5.0, 15.0, 30.0]
    ret.longitudinalTuning.kpV = [1.2, 1.0, 0.8, 0.6]
    ret.longitudinalTuning.kiBP = [0.0, 5.0, 15.0, 30.0]
    ret.longitudinalTuning.kiV = [0.25, 0.22, 0.18, 0.12]

    return ret
