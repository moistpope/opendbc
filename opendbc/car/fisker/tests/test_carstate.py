"""
Cruise state comes from the ADAS module's ACC (ADAS_0x313 / ADAS_0x31C on bus 2).
"""

import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.fisker.carstate import CarState
from opendbc.car.fisker.interface import CarInterface
from opendbc.car.fisker.values import CANBUS, CAR, DBC

# ADAS_Sts_ACC_ICC -> (enabled, available, standstill)
ACC_STATES = {
  0: (False, False, False),   # ACC_Off
  1: (False, False, False),   # Initialization
  2: (False, True, False),    # Standby
  3: (True, True, False),     # Active
  4: (True, True, False),     # Override
  5: (True, True, True),      # Standstill_active
  6: (True, True, True),      # Standstill_wait
  7: (False, True, False),    # Deactivation_brake
  8: (False, True, False),    # Deactivation_other
  9: (False, False, False),   # Failure_reversible
  10: (False, False, False),  # Failure_irreversible
  11: (True, True, True),     # Standstill_GoNotification
}


def run_carstate(acc_state: int, set_speed: int, mph: bool):
  CP = CarInterface.get_non_essential_params(CAR.FISKER_OCEAN)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.FISKER_OCEAN)
  CS = CarState(CP, CP_SP)
  parsers = CS.get_can_parsers(CP, CP_SP)
  packer = CANPacker(DBC[CAR.FISKER_OCEAN][Bus.pt])

  frames = [
    packer.make_can_msg("ADAS_0x313", CANBUS.cam, {"ADAS_Sts_ACC_ICC": acc_state}),
    packer.make_can_msg("ADAS_0x31C", CANBUS.cam, {"ADAS_AccTrgSpdDisp": set_speed, "ADAS_DispSpdUnit_ACC": int(mph)}),
  ]
  for cp in parsers.values():
    cp.update([(0, frames)])
  ret, _ = CS.update(parsers)
  return ret


@pytest.mark.parametrize("acc_state", ACC_STATES)
def test_cruise_state_from_adas_acc(acc_state):
  ret = run_carstate(acc_state, 100, mph=False)
  enabled, available, standstill = ACC_STATES[acc_state]
  assert ret.cruiseState.enabled == enabled
  assert ret.cruiseState.available == available
  assert ret.cruiseState.standstill == standstill


@pytest.mark.parametrize("set_speed,mph,expected", [
  (100, False, 100 * CV.KPH_TO_MS),
  (65, True, 65 * CV.MPH_TO_MS),
  (255, False, 0.0),  # no_display
])
def test_cruise_set_speed(set_speed, mph, expected):
  ret = run_carstate(3, set_speed, mph)
  assert ret.cruiseState.speed == pytest.approx(expected)
  assert ret.cruiseState.speedCluster == pytest.approx(expected)
