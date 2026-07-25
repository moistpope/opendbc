from opendbc.car import get_safety_config, structs
from opendbc.car.fisker.values import FiskerFlags
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

    # Early bring-up: there is no SAFETY_FISKER panda mode yet, so run read-only.
    # TODO: implement modes/fisker.h (SecOC-aware) and switch to SafetyModel.fisker.
    ret.safetyConfigs = [get_safety_config(SafetyModel.noOutput)]
    ret.dashcamOnly = True

    # lateral is torque-based (ADAS_STEER_CONTROL->LKAS_STEERING_TORQUE); tune is inert while read-only
    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.lateralTuning.pid.kpBP = [0.]
    ret.lateralTuning.pid.kpV = [0.]
    ret.lateralTuning.pid.kiBP = [0.]
    ret.lateralTuning.pid.kiV = [0.]

    if ret.flags & FiskerFlags.SECOC:
      ret.secOcRequired = True

    # radar (MRR 0x33B) is not decoded yet
    ret.radarUnavailable = True

    # no ACC decoded yet -> openpilot cannot do longitudinal
    ret.openpilotLongitudinalControl = False

    return ret
