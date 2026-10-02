"""
CAN packing helpers for Fisker Ocean.

Plain 8-bit `*_CheckSum` algorithm:

    CRC-8 (poly=0x1D / SAE-J1850, init=0x00, xorout=0x00)
    input = [DataID_byte] || bytes[1 : DataLength/8]

  - DataID_byte is the per-message data identifier (see E2E_PARAMS).
  - DataLength (bits) is the covered payload length — 64 for normal messages,
    32 for SecOC-protected messages (which excludes the SecOC tail from the CRC).

Ocean's E2E flavour: SAE-J1850 polynomial with init/xorout stripped to 0 —
NOT the standard AUTOSAR E2E Profile 1 which uses 0xFF/0xFF.

SecOC stamping is performed in carcontroller.py after CAN packing — see
opendbc.car.fisker.secoc.stamp_secoc.
"""

from opendbc.can import CANPacker
from opendbc.car.fisker.values import CANBUS, ICC_0x35B_OVERRIDES, ICC_SETTINGS_OVERRIDES, RELAY_CTRL_ADDR

# Per-address (data_id, length_bits) pair used by the plain-CRC checksum below.
E2E_PARAMS: dict[int, tuple[int, int]] = {
  0x115: (138, 64),   # ESP wheel speeds F
  0x116: (159, 64),   # ESP wheel speeds R
  0x117: (72, 64),    # ADAS long-control status
  0x118: (81, 64),    # ADAS long/ESP handshake
  0x119: (164, 64),   # iBooster
  0x121: (23, 32),    # ADAS accel command (SecOC)
  0x1C0: (195, 64),   # ADAS lateral-control activation/status
  0x1D0: (16, 32),    # ADAS steering command (SecOC)
  0x317: (205, 64),   # ADAS chime / takeover
  0x318: (121, 64),   # ESP vehicle speed
  0x31A: (55, 64),    # ADAS AEB / telltale
  0x52A: (0xF5, 64),  # ICC feature settings (verified against logged ICC frames)
}


def fisker_plain_checksum(addr: int, data: bytes) -> int:
  """
  Fisker Ocean plain 8-bit CheckSum.
  data must be the 8-byte frame with byte[0] (the checksum position) already
  zero. Returns the 8-bit CRC to write into byte[0].
  """
  data_id, data_len_bits = E2E_PARAMS[addr]
  n_bytes = data_len_bits // 8
  crc_input = bytes([data_id]) + data[1:n_bytes]
  crc = 0
  for b in crc_input:
    crc ^= b
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc


class FiskerCAN:
  def __init__(self, CP, packer: CANPacker):
    self.CP = CP
    self.packer = packer

  # ---- ADASBUS (Bus.pt) — actuator commands and status -------------------

  def create_steering_control(self, angle_deg: float, counter: int):
    """ADAS_0x1D0 — steering angle request. 0x1D0 has NO active/enable bit (the EPS gates
    actuation entirely on 0x1C0's Req/Sts). `angle_deg` should be the commanded angle
    while lat_active, or the mirrored measured angle when the driver is overriding —
    apply_std_steer_angle_limits already produces the right value. E2E checksum on byte0;
    SecOC tail stamped later."""
    values = {
      "ADAS_LatCtrl_SteerAnReq": angle_deg,
      "ADAS_1D0_AliveCounter": counter & 0xF,
      "ADAS_1D0_CheckSum": 0,   # filled below (SecOC tail zeroed until stamped)
    }
    addr, data, bus = self.packer.make_can_msg("ADAS_0x1D0", CANBUS.pt, values)
    chk = fisker_plain_checksum(addr, data)
    return addr, bytes([chk]) + data[1:], bus

  def create_lat_control(self, lat_active: bool, counter: int, driver_override: bool = False):
    """ADAS_0x1C0 — lateral-control activation/status, 10 ms cycle (plain E2E checksum,
    no SecOC).

    Sent for the full engaged window (not only while lat_active) so the cluster never sees
    the frame disappear. The content mirrors the OEM's semantics:
      * lat_active True  -> Sts=1, Req=1, Typ=1 (active, requesting the 0x1D0 angle)
      * lat_active False -> Sts=0, Req=0, Typ=0 (inactive, i.e. the OEM's default frame)
    This way, when the driver overrides and lat_active drops, the EPS gets a fresh Req=0
    on the very next 10 ms cycle and releases -- unlike the previous design that stopped
    sending 0x1C0 altogether, which left the EPS servoing the last commanded angle for a
    beat and made the wheel very hard to move.

    ADAS_LatCtrl_DrvrOvrd is a diagnostic bit the ADAS sends to the Gateway only (DBC
    receiver list = GW). The EPS does NOT read it; it uses its own driver-torque sensor
    (EPS_DrvrSteerTq/EPS_DrvrIntvSteerWhlDetd on 0x1C4). Setting DrvrOvrd here is purely
    informational for the GW log."""
    values = {
      "ADAS_LatCtrl_Sts": 1 if lat_active else 0,        # 0=Inactive 1=Active
      "ADAS_LatCtrl_StsVld": 1,                           # 1=Valid
      "ADAS_LatCtrl_DrvrOvrd": 1 if driver_override else 0,  # GW-only diagnostic
      "ADAS_LatCtrl_DrvrOvrdVld": 1,                      # 1=Valid
      "ADAS_LatCtrl_Typ": 1 if lat_active else 0,         # 1=LKA_angle_request 0=Not_active
      "ADAS_LatCtrl_ReqVld": 1 if lat_active else 0,      # 1=Steering_angle_request_valid
      "ADAS_LatCtrl_Req": 1 if lat_active else 0,         # 1=Angle_request_active -> EPS servoes
      "ADAS_1C0_AliveCounter": counter & 0xF,
      "ADAS_1C0_CheckSum": 0,   # filled below
    }
    addr, data, bus = self.packer.make_can_msg("ADAS_0x1C0", CANBUS.pt, values)
    chk = fisker_plain_checksum(addr, data)
    return addr, bytes([chk]) + data[1:], bus

  # 0x121 is built to match the stock ADAS module's frame byte for byte while its ACC is Active (checked
  # against recorded drives, see tests/test_long_frames.py). openpilot replaces it while it drives and panda
  # blocks the stock one. The all-ones fill in bits the DBC doesn't define is what the stock module sends;
  # the receivers' E2E CRC covers it. The stock 0x117/0x118 are never replaced: they already say
  # "ACC Active" while the stock ACC is engaged, and engagement follows 0x313.

  def create_accel_command(self, accel: float, counter: int):
    """ADAS_0x121 — longitudinal accel request, 10 ms cycle. SecOC-protected (tail stamped by
    the caller). AccelVld is Valid on every stock frame, including with zero accel in standby, so
    "not commanding" is accel=0, never Vld=0."""
    values = {
      "ADAS_LgtCtrl_AccelReq": accel,
      "ADAS_LgtCtrl_AccelVld": 1,
      "ADAS_121_Rsv_B1": 3,
      "ADAS_121_AliveCounter": counter & 0xF,
      "ADAS_121_CheckSum": 0,   # filled in below
    }
    addr, data, bus = self.packer.make_can_msg("ADAS_0x121", CANBUS.pt, values)
    chk = fisker_plain_checksum(addr, data)
    data = bytes([chk]) + data[1:]
    return addr, data, bus

  def create_relay_release(self, mask: int):
    """Release request for panda's relay arbiter (see fisker.h): hands the relays in `mask` back to the
    stock ADAS stream at exactly our last counter + 1. A pseudo address with no car message; panda
    consumes it and rejects it, so it never reaches the vehicle bus."""
    return RELAY_CTRL_ADDR, bytes([mask & 0xFF]) + bytes(7), CANBUS.pt

  # ---- ADAS module side (Bus.cam) --------------------------------------------

  def create_icc_settings(self, icc_values: dict[str, float]):
    """ICC_0x52A — the ICC's feature-settings frame, re-sent to the ADAS module (bus 2) with
    ICC_SETTINGS_OVERRIDES applied. `icc_values` is one decoded ICC frame from bus 0; every
    signal not overridden — including ICC_0x52A_AliveCounter — is copied from it, so our stream
    carries the ICC's own counter sequence (one output per ICC frame). E2E checksum on byte0."""
    values = {**icc_values, **ICC_SETTINGS_OVERRIDES, "ICC_0x52A_CheckSum": 0}
    addr, data, bus = self.packer.make_can_msg("ICC_0x52A", CANBUS.cam, values)
    chk = fisker_plain_checksum(addr, data)
    return addr, bytes([chk]) + data[1:], bus

  def create_icc_0x35b(self, icc_values: dict[str, float]):
    """ICC_0x35B — SVS view requests + BSD/DOW/APA/park-assist settings, re-sent to the ADAS
    module (bus 2) with ICC_0x35B_OVERRIDES applied. `icc_values` is one decoded ICC frame from
    bus 0; every other signal (incl. the ICC_0x35B_PASSTHROUGH camera-view requests) is copied
    from it. No E2E on this message."""
    values = {**icc_values, **ICC_0x35B_OVERRIDES}
    return self.packer.make_can_msg("ICC_0x35B", CANBUS.cam, values)

  def create_acc_hud(self, set_speed_kph: float, gap_setting: int, icon: int, counter: int):
    """ADAS_0x31C — ACC display injection (target speed, gap, icon)."""
    values = {
      "ADAS_AccTrgSpdDisp": float(set_speed_kph),
      "ADAS_TiGapSet_ACC": gap_setting & 0x7,
      "ADAS_ACCIconDisp": icon & 0x7,
      "ADAS_DispSpdUnit_ACC": 0,    # 0 = km/h
    }
    return self.packer.make_can_msg("ADAS_0x31C", CANBUS.pt, values)

  def create_warning_hud(self, chime: int, takeover: int, counter: int):
    """ADAS_0x317 — chimes / driver-takeover requests."""
    values = {
      "ADAS_ChimeReq": chime & 0x7,
      "ADAS_DrvrTakeOvrReq": takeover & 0x3,
    }
    return self.packer.make_can_msg("ADAS_0x317", CANBUS.pt, values)
