# Fisker Ocean ADAS control messages (ADAS_STEER_CONTROL 0x1D0, ADAS_ACCEL_CONTROL 0x121) are
# protected by two mechanisms, both reverse engineered from the ADAS bus and now CONFIRMED against
# real captures:
#   - byte 0:      an 8-bit checksum (CRC8_J1850_b1_b3 in the DBC) over bytes 1-3
#   - bytes 4..7:  a truncated AUTOSAR SecOC freshness byte + 24-bit CMAC (AES-128), built by
#                  opendbc/car/fisker_secoc.py from the TRIP_CNT/RESET_CNT/message-counter state
#                  synchronized via GW_SECOC_SYNC (0x20)
#
# CRC: CRC-8 with the SAE-J1850 polynomial (0x1D), init 0x00 (NOT the canonical 0xFF), computed
# over bytes 1-3, then XORed with a per-message constant (0x25 for ADAS_STEER_CONTROL, 0xD1 for
# ADAS_ACCEL_CONTROL). Confirmed to reproduce 100% of a real capture: 6159/6159 frames parked,
# 5911/5911 frames with LKAS engaged, for both messages.
#
# SecOC: see opendbc/car/fisker_secoc.py for the freshness/MAC construction, confirmed against the
# AUTOSAR worked example. CarController supplies the trip/reset counters (from GW_SECOC_SYNC) and
# owns the per-message-type message counter; fisker_secoc.stamp_secoc() fills in FRESHNESS/SECOC.

ADAS_STEER_CONTROL_ADDR = 0x1D0
ADAS_ACCEL_CONTROL_ADDR = 0x121
LKAS_STEER_AUTHORITY_ADDR = 0x1C0

_CRC8_J1850_POLY = 0x1D
# per-message XOR-out constant applied on top of the CRC-8/SAE-J1850 shift register
_CRC_XOROUT = {
  ADAS_STEER_CONTROL_ADDR: 0x25,
  ADAS_ACCEL_CONTROL_ADDR: 0xD1,
  LKAS_STEER_AUTHORITY_ADDR: 0x03,
}
# CRC coverage (start, end) as a byte slice: the two SecOC control frames checksum only bytes 1-3
# (the command portion, ahead of the SecOC freshness/MAC tail), while non-SecOC COUNTER_CRC frames
# like LKAS_STEER_AUTHORITY checksum the full payload, bytes 1-7. (knowledge base section 6)
_CRC_COVERAGE = {
  ADAS_STEER_CONTROL_ADDR: (1, 4),
  ADAS_ACCEL_CONTROL_ADDR: (1, 4),
  LKAS_STEER_AUTHORITY_ADDR: (1, 8),
}


def crc8_j1850(data: bytes, init: int = 0xFF, xorout: int = 0xFF) -> int:
  """Generic CRC-8/SAE-J1850. Defaults are the canonical parameters:
  crc8_j1850(b"123456789") == 0x4B"""
  crc = init
  for byte in data:
    crc ^= byte
    for _ in range(8):
      if crc & 0x80:
        crc = ((crc << 1) ^ _CRC8_J1850_POLY) & 0xFF
      else:
        crc = (crc << 1) & 0xFF
  return crc ^ xorout


def _checksum(addr: int, dat: bytes) -> int:
  """The real frame checksum (byte 0): CRC-8/SAE-J1850 (init 0x00) over this message's covered
  bytes, XORed with a per-message constant."""
  lo, hi = _CRC_COVERAGE[addr]
  return crc8_j1850(bytes(dat[lo:hi]), init=0x00, xorout=_CRC_XOROUT[addr])


def _finalize(msg):
  addr, dat, bus = msg
  dat = bytearray(dat)
  dat[0] = _checksum(addr, bytes(dat))
  return addr, bytes(dat), bus


def create_steer_command(packer, torque, steer_req, counter_a):
  """ADAS_STEER_CONTROL (0x1D0) — LKAS torque command.

  FRESHNESS/SECOC are left at 0 here; the caller (CarController) fills them in via
  fisker_secoc.stamp_secoc() once the SecOC trip/reset/message counters are known.
  """
  values = {
    "COUNTER_A": counter_a,
    "UNKNOWN_CONSTANT_1": 0,
    "LKAS_STEERING_TORQUE": torque if steer_req else 0,
    "UNKNOWN_CONSTANT": 3 if steer_req else 1,
  }
  return _finalize(packer.make_can_msg("ADAS_STEER_CONTROL", 0, values))


def create_accel_command(packer, accel_payload, counter_a, counter_b):
  """ADAS_ACCEL_CONTROL (0x121) — longitudinal command."""
  values = {
    "COUNTER_A": counter_a,
    "UNKNOWN_CONSTANT_1": 1,
    "PAYLOAD": accel_payload,
    "UNKNOWN_CONSTANT": 8,
  }
  return _finalize(packer.make_can_msg("ADAS_ACCEL_CONTROL", 0, values))


# LKAS_STEER_AUTHORITY (0x1C0) payload, confirmed from capture. Byte 1's low nibble is the ARC and
# byte 0 is the CRC; the rest of the payload is constant except three lateral-control validity bits
# that flip to "valid" while LKAS is active:
#   idle:   b2=0x48 b3=0x10 b4..b7=0x00
#   active: b2=0x49 b3=0x51   (b2 |= 0x01 StsVld, b3 |= 0x01 DrvrOvrdVld, b3 |= 0x40 ReqVld)
_AUTH_B2_CONST = 0x48
_AUTH_B3_CONST = 0x10
_AUTH_B2_STSVLD = 0x01
_AUTH_B3_DRVROVRDVLD = 0x01
_AUTH_B3_REQVLD = 0x40


def create_authority_command(counter_a, lat_active=True):
  """LKAS_STEER_AUTHORITY (0x1C0) — the lateral-control validity frame the EPS requires alongside
  ADAS_STEER_CONTROL. Non-SecOC: byte 0 is a CRC over the full payload (bytes 1-7, xor-out 0x03) and
  byte 1's low nibble is a standalone ARC. While lat_active the three validity fields read valid; if
  the frame is absent the EPS raises U12F787 (lost communication) and if the fields are not valid it
  raises U12F786 (implausible) — either way it ignores our steering command. Byte layout confirmed
  from capture (see the constants above).
  """
  dat = bytearray(8)
  dat[1] = counter_a & 0x0F
  dat[2] = _AUTH_B2_CONST | (_AUTH_B2_STSVLD if lat_active else 0)
  dat[3] = _AUTH_B3_CONST | ((_AUTH_B3_DRVROVRDVLD | _AUTH_B3_REQVLD) if lat_active else 0)
  dat[0] = _checksum(LKAS_STEER_AUTHORITY_ADDR, bytes(dat))
  return LKAS_STEER_AUTHORITY_ADDR, bytes(dat), 0
