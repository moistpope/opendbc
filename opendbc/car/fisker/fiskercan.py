import struct

from Crypto.Hash import CMAC
from Crypto.Cipher import AES

# Fisker Ocean ADAS control messages (ADAS_STEER_CONTROL 0x1D0, ADAS_ACCEL_CONTROL 0x121)
# are protected by two mechanisms, both reverse engineered from the ADAS bus:
#   - byte 0:      an 8-bit checksum (labelled CRC8_J1850 in the DBC)
#   - bytes 5..7:  a 24-bit truncated AUTOSAR SecOC CMAC (AES-128)
# plus COUNTER_A (byte 1) and COUNTER_B rolling counters.
#
# WARNING: validated against a real 1-minute capture, byte 0 does NOT match CRC8-J1850 (nor any
# other CRC8) over contiguous bytes 1..3 (or 1..7), even with a 16-bit data-ID seed. The exact
# byte-0 checksum is therefore still UNRESOLVED; crc8_j1850() below is a correct CRC-8/SAE-J1850
# (canonical check passes) kept for when the real coverage/algorithm is found. Likewise, the real
# per-message SecOC MAC could not yet be reproduced from the capture with the recovered key, so the
# freshness/payload composition is still open. Only the SecOC CMAC construction itself is confirmed
# (via the known-answer vector in tests/secoc_crc_simulator.py). See that simulator for details.

# CRC-8/SAE-J1850: poly 0x1D, init 0xFF, reflected in/out = false, xorout 0xFF
# canonical check value: crc8_j1850(b"123456789") == 0x4B
_CRC8_J1850_POLY = 0x1D


def crc8_j1850(data: bytes) -> int:
  crc = 0xFF
  for byte in data:
    crc ^= byte
    for _ in range(8):
      if crc & 0x80:
        crc = ((crc << 1) ^ _CRC8_J1850_POLY) & 0xFF
      else:
        crc = (crc << 1) & 0xFF
  return crc ^ 0xFF


def build_secoc_mac(key: bytes, addr: int, payload: bytes, freshness: int) -> int:
  """24-bit truncated AUTOSAR SecOC CMAC for a Fisker Ocean ADAS message.

  Authenticated data is [Data ID (16b) | payload (64b, the full 8-byte authentic message) |
  freshness (64b)], and the MAC is the top 3 bytes of the AES-128 CMAC. This layout is confirmed
  against the worked example in _reference/SecOC:
    key=0102..10, id=0x0021, payload=E8030000000000FF, freshness=0x100000405 -> MAC 0x498330

  NOTE: which 8 message bytes form the authenticated payload, and how the trip/reset/message
  counters compose the 64-bit freshness (only its low byte is transmitted on the wire), are still
  being reverse engineered. This function pins down the confirmed CMAC construction; the caller
  supplies the (still provisional) payload and freshness.
  """
  to_auth = struct.pack('>H', addr & 0xFFFF) + payload[:8].ljust(8, b'\x00') + \
            struct.pack('>Q', freshness & 0xFFFFFFFFFFFFFFFF)

  cmac = CMAC.new(key, ciphermod=AES)
  cmac.update(to_auth)
  return int.from_bytes(cmac.digest()[:3], 'big')  # truncate to top 24 bits


# Frame layout confirmed from a real capture: byte0=checksum, byte1=COUNTER_A, byte2..3=payload,
# byte4=FRESHNESS (a truncated freshness byte that is GLOBAL across ADAS messages at a given instant),
# bytes5..7=24-bit SecOC MAC. The `counter_b` argument below is written to byte4 (FRESHNESS).
def _finalize_crc(msg):
  # PLACEHOLDER: byte 0 is a checksum, but its algorithm/coverage is NOT yet reverse engineered
  # (real capture does not match CRC8-J1850 over any contiguous range). crc8_j1850 is a correct
  # CRC-8/SAE-J1850 kept for when the real coverage is found; the value set here is not yet valid.
  addr, dat, bus = msg
  dat = bytearray(dat)
  dat[0] = crc8_j1850(bytes(dat[1:4]))
  return addr, bytes(dat), bus


def create_steer_command(packer, torque, steer_req, counter_a, counter_b, secoc_mac):
  """ADAS_STEER_CONTROL (0x1D0) — LKAS torque command."""
  values = {
    "COUNTER_A": counter_a,
    "LKAS_STEERING_TORQUE": torque if steer_req else 0,
    "UNKNOWN_CONSTANT": 1,
    "FRESHNESS": counter_b,
    "SECOC": secoc_mac,
  }
  return _finalize_crc(packer.make_can_msg("ADAS_STEER_CONTROL", 0, values))


def create_accel_command(packer, accel_payload, counter_a, counter_b, secoc_mac):
  """ADAS_ACCEL_CONTROL (0x121) — longitudinal command."""
  values = {
    "COUNTER_A": counter_a,
    "PAYLOAD": accel_payload,
    "COUNTER_B": counter_b,
    "SECOC": secoc_mac,
  }
  return _finalize_crc(packer.make_can_msg("ADAS_ACCEL_CONTROL", 0, values))
