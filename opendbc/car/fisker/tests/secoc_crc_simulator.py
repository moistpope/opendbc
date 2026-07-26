#!/usr/bin/env python3
"""Fisker Ocean SecOC / CRC simulator and validator.

Confirmed cracked constructions:
  * CRC-8/SAE-J1850 (poly 0x1D, init 0x00) over ADAS_STEER_CONTROL (0x1D0) / ADAS_ACCEL_CONTROL
    (0x121) bytes 1-3, XORed with a per-message constant (0x25 / 0xD1 respectively). See
    opendbc/car/fisker/fiskercan.py.
  * AUTOSAR SecOC freshness (trip/reset/message counters) + truncated CMAC, synchronized via
    GW_SECOC_SYNC (0x20). See opendbc/car/fisker_secoc.py.

This module cross-checks openpilot's implementation against independent reference implementations
written directly from the confirmed byte layouts (rather than reusing openpilot's own code), and
simulates a ground-truth ADAS ECU signing a trip's worth of frames end to end.
"""
import random
import struct

from Crypto.Hash import CMAC
from Crypto.Cipher import AES

from opendbc.car import secoc
from opendbc.car import fisker_secoc
from opendbc.car.fisker import fiskercan

TEST_KEY = bytes.fromhex("0123456789abcdef0123456789abcdef")

ADDR_STEER = fiskercan.ADAS_STEER_CONTROL_ADDR  # 0x1D0
ADDR_ACCEL = fiskercan.ADAS_ACCEL_CONTROL_ADDR  # 0x121
_CRC_XOROUT = {ADDR_STEER: 0x25, ADDR_ACCEL: 0xD1}


# ---------------------------------------------------------------------------
# Independent reference implementations (written from the confirmed byte layouts)
# ---------------------------------------------------------------------------

def crc8_j1850_reference(data: bytes, init: int = 0xFF, xorout: int = 0xFF) -> int:
  """Table-driven CRC-8/SAE-J1850, independent of fiskercan's bitwise implementation."""
  table = []
  for i in range(256):
    crc = i
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    table.append(crc)
  crc = init
  for byte in data:
    crc = table[(crc ^ byte) & 0xFF]
  return crc ^ xorout


def reference_checksum(addr: int, dat: bytes) -> int:
  return crc8_j1850_reference(bytes(dat[1:4]), init=0x00, xorout=_CRC_XOROUT[addr])


def _cmac_top_bytes(key: bytes, to_auth: bytes, n: int) -> int:
  c = CMAC.new(key, ciphermod=AES)
  c.update(to_auth)
  return int.from_bytes(c.digest()[:n], 'big')


def reference_freshness(trip: int, reset: int, msg_counter: int) -> bytes:
  return (struct.pack(">H", trip & 0xFFFF)
          + (reset & 0xFFFFFF).to_bytes(3, "big")
          + struct.pack(">H", msg_counter & 0xFFFF)
          + bytes([(reset & 0x3) << 6]))


def reference_secoc_mac(key: bytes, can_id: int, pdu4: bytes, trip: int, reset: int, msg_counter: int) -> int:
  to_auth = struct.pack(">H", can_id & 0xFFFF) + bytes(pdu4[:4]) + reference_freshness(trip, reset, msg_counter)
  return _cmac_top_bytes(key, to_auth, 3)


def reference_wire_freshness_byte(msg_counter: int, reset: int) -> int:
  return (((msg_counter & 0x3F) << 2) | (reset & 0x3)) & 0xFF


def reference_sync_mac(key: bytes, trip: int, reset: int) -> int:
  to_auth = struct.pack(">H", fisker_secoc.SYNC_CAN_ID) + struct.pack(">H", trip & 0xFFFF) + (reset & 0xFFFFFF).to_bytes(3, "big")
  return _cmac_top_bytes(key, to_auth, 3)


# ---------------------------------------------------------------------------
# Validation sections
# ---------------------------------------------------------------------------

def validate_crc(verbose=False):
  failures = []
  check = fiskercan.crc8_j1850(b"123456789")
  if verbose:
    print(f"  crc8_j1850('123456789') = {check:#04x} (expect 0x4b) {'OK' if check == 0x4B else 'FAIL'}")
  if check != 0x4B:
    failures.append(f"CRC8-J1850 canonical check failed: {check:#04x} != 0x4b")

  rng = random.Random(0xF15E)
  for _ in range(500):
    data = bytes(rng.randrange(256) for _ in range(rng.randrange(1, 9)))
    if fiskercan.crc8_j1850(data) != crc8_j1850_reference(data):
      failures.append(f"CRC8-J1850 mismatch on {data.hex()}")

  # real ADAS frame checksum (init 0x00, per-message xorout), both message IDs
  for addr in (ADDR_STEER, ADDR_ACCEL):
    for _ in range(200):
      dat = bytes([0] + [rng.randrange(256) for _ in range(7)])
      op = fiskercan._checksum(addr, dat)
      ref = reference_checksum(addr, dat)
      if op != ref:
        failures.append(f"frame checksum mismatch addr={addr:#x} dat={dat.hex()}: {op:#04x} != {ref:#04x}")

  if verbose and not failures:
    print("  crc8_j1850 + frame checksum vs independent reference: OK")
  return failures


def validate_fisker_secoc(verbose=False):
  """fisker_secoc.secoc_mac/wire_freshness_byte/sync_mac must match independent references."""
  failures = []
  rng = random.Random(0x0CEA)

  for _ in range(200):
    addr = rng.choice([ADDR_STEER, ADDR_ACCEL])
    pdu = bytes(rng.randrange(256) for _ in range(4))
    trip = rng.randrange(1 << 16)
    reset = rng.randrange(1 << 24)
    msg_counter = rng.randrange(1 << 16)
    op = int.from_bytes(fisker_secoc.secoc_mac(TEST_KEY, addr, pdu, trip, reset, msg_counter), 'big')
    ref = reference_secoc_mac(TEST_KEY, addr, pdu, trip, reset, msg_counter)
    if op != ref:
      failures.append(f"secoc_mac mismatch addr={addr:#x} pdu={pdu.hex()} trip={trip} reset={reset} cnt={msg_counter}")

  for _ in range(200):
    msg_counter = rng.randrange(1 << 16)
    reset = rng.randrange(1 << 24)
    if fisker_secoc.wire_freshness_byte(msg_counter, reset) != reference_wire_freshness_byte(msg_counter, reset):
      failures.append(f"wire_freshness_byte mismatch cnt={msg_counter} reset={reset}")

  for _ in range(100):
    trip = rng.randrange(1 << 16)
    reset = rng.randrange(1 << 24)
    op = int.from_bytes(fisker_secoc.sync_mac(TEST_KEY, trip, reset), 'big')
    ref = reference_sync_mac(TEST_KEY, trip, reset)
    if op != ref:
      failures.append(f"sync_mac mismatch trip={trip} reset={reset}")

  if verbose and not failures:
    print("  fisker_secoc.py vs independent reference: OK")
  return failures


def validate_sync_helpers(verbose=False):
  """fisker_secoc.verify_sync/parse_sync round-trip a raw GW_SECOC_SYNC payload and detect tamper."""
  failures = []
  rng = random.Random(0x5717)
  for _ in range(50):
    trip = rng.randrange(1 << 16)
    reset = rng.randrange(1 << 24)
    mac = fisker_secoc.sync_mac(TEST_KEY, trip, reset)
    payload = struct.pack(">H", trip) + reset.to_bytes(3, "big") + mac
    if not fisker_secoc.verify_sync(TEST_KEY, payload):
      failures.append(f"verify_sync failed for trip={trip} reset={reset}")
    if fisker_secoc.parse_sync(payload) != (trip, reset):
      failures.append(f"parse_sync mismatch for trip={trip} reset={reset}")
    tampered = bytearray(payload)
    tampered[0] ^= 0xFF
    if fisker_secoc.verify_sync(TEST_KEY, bytes(tampered)):
      failures.append("verify_sync did not detect a tampered payload")
  if verbose and not failures:
    print("  verify_sync/parse_sync round-trip + tamper detection: OK")
  return failures


def validate_openpilot_secoc(verbose=False):
  """Confirm Toyota's generic SecOC (opendbc/car/secoc.py) is a DIFFERENT layout from Fisker's:
  it authenticates payload[:4] with a 48-bit freshness and a 28-bit MAC, while Fisker uses an
  8-byte freshness and a 24-bit MAC -- the two are not interchangeable."""
  failures = []
  pdu = bytes.fromhex("00010203")
  fisker_mac = int.from_bytes(fisker_secoc.secoc_mac(TEST_KEY, ADDR_STEER, pdu, 1, 1, 1), 'big')
  _, out, _ = secoc.add_mac(TEST_KEY, 1, 1, 1, (ADDR_STEER, pdu, 0))
  toyota_mac = int.from_bytes(out[-4:], 'big') & 0xFFFFFFF  # 28-bit MAC packed in the last nibble+3 bytes
  if fisker_mac == toyota_mac:
    failures.append("Fisker and Toyota SecOC MACs unexpectedly matched (layouts should differ)")
  if verbose and not failures:
    print("  secoc.py (Toyota layout) confirmed distinct from fisker_secoc.py: OK")
  return failures


def validate_end_to_end(verbose=False):
  """Simulate a trip: a ground-truth ECU signs frames (reference implementation), openpilot's
  fiskercan + fisker_secoc reproduce the on-wire bytes, and a tamper is confirmed detected."""
  from opendbc.can import CANPacker
  from opendbc.car.fisker.values import CAR, DBC
  from opendbc.car import Bus

  failures = []
  packer = CANPacker(DBC[CAR.FISKER_OCEAN][Bus.pt])
  trip = 7
  reset = 0
  msg_cnt = {ADDR_STEER: 0, ADDR_ACCEL: 0}
  frames = 0

  for i in range(40):
    if i == 20:
      reset += 1  # simulate a SecOC reset partway through the trip
      msg_cnt = {ADDR_STEER: 0, ADDR_ACCEL: 0}

    raw_msgs = [
      fiskercan.create_steer_command(packer, i % 4096, True, i & 0xFF),
      fiskercan.create_accel_command(packer, i % 4096, i & 0xFF, i & 0x0F),
    ]
    for addr, raw, bus in raw_msgs:
      cnt = msg_cnt[addr]
      stamped = fisker_secoc.stamp_secoc(TEST_KEY, addr, raw, trip, reset, cnt)
      ref_mac = reference_secoc_mac(TEST_KEY, addr, stamped[0:4], trip, reset, cnt)
      ref_fresh = reference_wire_freshness_byte(cnt, reset)
      xorout = _CRC_XOROUT[addr]

      if stamped[0] != crc8_j1850_reference(bytes(stamped[1:4]), init=0x00, xorout=xorout):
        failures.append(f"frame {i} addr={addr:#x}: CRC wrong")
      if stamped[4] != ref_fresh:
        failures.append(f"frame {i} addr={addr:#x}: freshness byte {stamped[4]:#04x} != {ref_fresh:#04x}")
      if int.from_bytes(stamped[5:8], 'big') != ref_mac:
        failures.append(f"frame {i} addr={addr:#x}: MAC {stamped[5:8].hex()} != {ref_mac:06x}")

      # tamper inside CRC coverage must break the CRC
      tampered = bytearray(stamped)
      tampered[1] ^= 0x01
      if tampered[0] == crc8_j1850_reference(bytes(tampered[1:4]), init=0x00, xorout=xorout):
        failures.append(f"frame {i} addr={addr:#x}: tamper not detected")

      msg_cnt[addr] = (cnt + 1) & 0xFFFF
      frames += 1

  if verbose:
    print(f"  end-to-end: {frames} signed frames checked (trip={trip} final reset={reset}) "
          f"{'OK' if not failures else 'FAIL'}")
  return failures


def run(verbose=True):
  sections = [
    ("CRC8-J1850 + frame checksum", validate_crc),
    ("Fisker SecOC (fisker_secoc.py)", validate_fisker_secoc),
    ("GW_SECOC_SYNC helpers", validate_sync_helpers),
    ("openpilot generic SecOC (secoc.py) is a distinct layout", validate_openpilot_secoc),
    ("end-to-end signed-frame simulation", validate_end_to_end),
  ]
  all_failures = []
  for name, fn in sections:
    if verbose:
      print(f"[{name}]")
    fails = fn(verbose=verbose)
    all_failures += fails
    if verbose:
      print(f"  -> {'PASS' if not fails else 'FAIL (' + str(len(fails)) + ')'}")
      for f in fails:
        print(f"     * {f}")
      print()
  ok = not all_failures
  if verbose:
    print("=" * 64)
    print(f"RESULT: {'ALL VALIDATIONS PASSED' if ok else str(len(all_failures)) + ' FAILURE(S)'}")
  return ok


if __name__ == "__main__":
  import sys
  sys.exit(0 if run(verbose=True) else 1)

