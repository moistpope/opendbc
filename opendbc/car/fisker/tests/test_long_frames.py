"""
Our longitudinal frames must match the stock ADAS module's, byte for byte, while openpilot is
the only sender of 0x121/0x117/0x118. The golden frames are the stock module's with ACC Active
(0x313 state 3) in drives 000000af--28a9f95752--1 and 000000b4--d0f733ebb2--5 (checksum and alive
counter masked; they differ per frame).
"""
from opendbc.can import CANPacker
from opendbc.car.fisker.fiskercan import AEB_DECEL_IDLE, FiskerCAN, fisker_plain_checksum
from opendbc.car.fisker.values import CAR, DBC
from opendbc.car import Bus

STOCK_ACTIVE_0X117 = bytes.fromhex("00004b4050a1ff28")   # ISA_SpdLmt_VCU 40 kph
STOCK_0X118 = bytes.fromhex("00f0484844d080d2")          # same in standby and ACC Active
STOCK_ACCEL_0X121 = bytes.fromhex("00d08005")            # bytes 0-3 with zero accel (raw 0x8005)


def make():
  return FiskerCAN(None, CANPacker(DBC[CAR.FISKER_OCEAN][Bus.pt]))


def mask(data: bytes) -> bytes:
  m = bytearray(data)
  m[0] = 0
  m[1] &= 0xF0   # AliveCounter is the low nibble of byte 1
  return bytes(m)


def test_0x117_matches_stock_active():
  _, data, _ = make().create_long_status(False, 40, 7)
  assert mask(data) == STOCK_ACTIVE_0X117
  assert data[1] & 0xF == 7


def test_0x117_relays_speed_limit():
  for lim in (40, 48, 104):
    assert make().create_long_status(False, lim, 0)[1][7] == lim


def test_0x118_matches_stock():
  _, data, _ = make().create_long_esp_handshake(7)
  assert mask(data) == STOCK_0X118
  assert abs(AEB_DECEL_IDLE - 0.0999) < 0.001   # not raw 0 = -16 m/s2


def test_0x121_matches_stock_idle_and_scales():
  _, data, _ = make().create_accel_command(0.0, 7)
  assert mask(data)[:4] == STOCK_ACCEL_0X121
  for accel in (-3.5, -0.85, 0.5, 2.0):
    raw = (make().create_accel_command(accel, 0)[1][2] << 8) | make().create_accel_command(accel, 0)[1][3]
    assert abs(raw * 0.0004882 - 16 - accel) < 0.001


def test_checksums_valid():
  f = make()
  for addr, data, _ in (f.create_long_status(True, 40, 3), f.create_long_esp_handshake(3), f.create_accel_command(1.0, 3)):
    assert fisker_plain_checksum(addr, bytes([0]) + data[1:]) == data[0]
