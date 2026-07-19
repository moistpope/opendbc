#!/usr/bin/env python3
"""Fisker Ocean SecOC / CRC simulator and validator.

This models a ground-truth Fisker ADAS ECU and uses it to validate openpilot's SecOC and CRC
implementations. The simulated ECU manages the AUTOSAR SecOC freshness (trip / reset / per-message
counters), emits signed ADAS_STEER_CONTROL / ADAS_ACCEL_CONTROL frames using an INDEPENDENT
reference implementation of CRC8-J1850 and the AUTOSAR SecOC CMAC, and cross-checks every frame
against openpilot's code paths:
    * opendbc/car/fisker/fiskercan.py -> crc8_j1850(), build_secoc_mac(), message builders
    * opendbc/car/secoc.py            -> add_mac(), build_sync_mac()   (openpilot's generic SecOC)

The Fisker SecOC construction is CONFIRMED against the worked example captured in _reference/SecOC:

    to_auth = DataID(16 bit) | payload(64 bit, full 8-byte message) | freshness(64 bit)
    MAC     = top 3 bytes (24 bit) of AES-128 CMAC(key, to_auth)

    key=0102030405060708090A0B0C0D0E0F10, id=0x0021,
    payload=E8030000000000FF, freshness=0x100000405  ->  MAC 0x498330

This layout differs from Toyota's opendbc/car/secoc.py (which authenticates only payload[:4] with a
48-bit freshness and a 28-bit MAC), which is exactly why the Ocean needs its own fiskercan path.

The example above uses the public AUTOSAR example key, so it is embedded here as a true known-answer
test. The real per-vehicle key (recovered via UDS DID 0xEFF5) lives only in _reference/SecOC and is
deliberately NOT committed. To turn the end-to-end section into a full known-answer test on your car,
paste the real key + a captured signed frame into REAL_KEY / KNOWN_ANSWER_VECTORS locally.
"""
import random

from Crypto.Hash import CMAC
from Crypto.Cipher import AES

from opendbc.car import secoc
from opendbc.car.fisker import fiskercan

TEST_KEY = bytes.fromhex("0123456789abcdef0123456789abcdef")

ADDR_STEER = 0x1D0  # ADAS_STEER_CONTROL
ADDR_ACCEL = 0x121  # ADAS_ACCEL_CONTROL

# (name, key, data_id, payload8, freshness, expected_truncated_mac) captured ground truth
KNOWN_ANSWER_VECTORS = [
  ("_reference/SecOC AUTOSAR example",
   bytes.fromhex("0102030405060708090A0B0C0D0E0F10"), 0x0021,
   bytes.fromhex("E8030000000000FF"), 0x100000405, 0x498330),
]


# ---------------------------------------------------------------------------
# Independent reference implementations (written from explicit byte construction)
# ---------------------------------------------------------------------------

def crc8_j1850_reference(data: bytes) -> int:
  """Table-driven CRC-8/SAE-J1850, independent of fiskercan's bitwise implementation."""
  table = []
  for i in range(256):
    crc = i
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    table.append(crc)
  crc = 0xFF
  for byte in data:
    crc = table[(crc ^ byte) & 0xFF]
  return crc ^ 0xFF


def _cmac_top_bytes(key: bytes, to_auth: bytes, n: int) -> int:
  c = CMAC.new(key, ciphermod=AES)
  c.update(to_auth)
  return int.from_bytes(c.digest()[:n], 'big')


def reference_mac_fisker(key: bytes, addr: int, payload: bytes, freshness: int) -> int:
  """Independent reference for the Fisker SecOC MAC: id16 | payload64 | freshness64 -> top 24 bits."""
  to_auth = bytes([(addr >> 8) & 0xFF, addr & 0xFF]) + payload[:8].ljust(8, b'\x00') + \
            freshness.to_bytes(8, 'big')
  return _cmac_top_bytes(key, to_auth, 3)


def reference_mac_toyota(key: bytes, addr: int, payload4: bytes, trip: int, reset: int, msg_cnt: int) -> int:
  """Independent reference for opendbc/car/secoc.py add_mac: id16 | payload32 | freshness48 -> top 28 bits."""
  reset_flag = reset & 0b11
  fv = bytearray(6)
  fv[0] = (trip >> 8) & 0xFF
  fv[1] = trip & 0xFF
  packed = ((reset & 0xFFFFF) << 12) | ((msg_cnt & 0xFF) << 4) | ((reset_flag & 0x3) << 2)
  fv[2] = (packed >> 24) & 0xFF
  fv[3] = (packed >> 16) & 0xFF
  fv[4] = (packed >> 8) & 0xFF
  fv[5] = packed & 0xFF
  to_auth = bytes([(addr >> 8) & 0xFF, addr & 0xFF]) + payload4[:4] + bytes(fv)
  return _cmac_top_bytes(key, to_auth, 4) >> 4  # top 28 bits


def reference_sync_mac(key: bytes, trip: int, reset: int, id_: int = 0xF) -> int:
  to_auth = bytes([(id_ >> 8) & 0xFF, id_ & 0xFF, (trip >> 8) & 0xFF, trip & 0xFF]) + \
            ((reset & 0xFFFFF) << 4).to_bytes(3, 'big')
  return _cmac_top_bytes(key, to_auth, 4) >> 4  # top 28 bits


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
  if verbose and not failures:
    print("  crc8_j1850 vs independent table CRC: 500/500 OK")
  return failures


def validate_secoc_known_answer(verbose=False):
  """The definitive test: reproduce the captured ground-truth MAC(s)."""
  failures = []
  for name, key, data_id, payload, freshness, expected in KNOWN_ANSWER_VECTORS:
    op = fiskercan.build_secoc_mac(key, data_id, payload, freshness)
    ref = reference_mac_fisker(key, data_id, payload, freshness)
    if verbose:
      print(f"  {name}:")
      print(f"    fiskercan.build_secoc_mac = {op:06x} (expect {expected:06x}) {'OK' if op == expected else 'FAIL'}")
      print(f"    independent reference     = {ref:06x} (expect {expected:06x}) {'OK' if ref == expected else 'FAIL'}")
    if op != expected:
      failures.append(f"{name}: build_secoc_mac {op:06x} != expected {expected:06x}")
    if ref != expected:
      failures.append(f"{name}: reference {ref:06x} != expected {expected:06x}")
  return failures


def validate_fisker_secoc(verbose=False):
  """fiskercan.build_secoc_mac must match the independent reference over randomized inputs."""
  failures = []
  rng = random.Random(0x0CEA)
  for _ in range(200):
    addr = rng.choice([ADDR_STEER, ADDR_ACCEL])
    payload = bytes(rng.randrange(256) for _ in range(8))
    freshness = rng.getrandbits(64)
    if fiskercan.build_secoc_mac(TEST_KEY, addr, payload, freshness) != \
       reference_mac_fisker(TEST_KEY, addr, payload, freshness):
      failures.append(f"build_secoc_mac mismatch addr={addr:#x} payload={payload.hex()} fresh={freshness:#x}")
  if verbose:
    print(f"  fiskercan.build_secoc_mac vs reference: {200 - len(failures)}/200 OK")
  return failures


def validate_openpilot_secoc(verbose=False):
  """Validate openpilot's generic SecOC (opendbc/car/secoc.py) against an independent reference of
  ITS Toyota layout, and confirm it does NOT (and should not) reproduce the Fisker layout."""
  failures = []
  for payload, trip, reset, msg_cnt in [(b"\x00\x00\x00\x00", 0, 0, 0),
                                        (b"\x12\x34\x56\x78", 1, 1, 1),
                                        (b"\xff\xff\xff\xff", 0x1234, 0xABCDE, 0xFF),
                                        (b"\xde\xad\xbe\xef", 42, 7, 200)]:
    _, out, _ = secoc.add_mac(TEST_KEY, trip, reset, msg_cnt, (ADDR_STEER, payload, 0))
    op_mac = int(out.hex()[-7:], 16)  # 28-bit MAC in the last 7 nibbles
    ref = reference_mac_toyota(TEST_KEY, ADDR_STEER, payload, trip, reset, msg_cnt)
    if op_mac != ref:
      failures.append(f"secoc.add_mac mismatch trip={trip} reset={reset} cnt={msg_cnt}: {op_mac:07x} != {ref:07x}")
    if out[:4] != payload:
      failures.append(f"secoc.add_mac corrupted payload: {out[:4].hex()} != {payload.hex()}")
  for trip, reset in [(0, 0), (1, 1), (0x1234, 0xABCDE), (99, 0x100)]:
    if secoc.build_sync_mac(TEST_KEY, trip, reset) != reference_sync_mac(TEST_KEY, trip, reset):
      failures.append(f"secoc.build_sync_mac mismatch trip={trip} reset={reset}")
  if verbose:
    print(f"  secoc.py (Toyota layout) vs reference: {'OK' if not failures else 'FAIL'}")
    print("    note: secoc.py authenticates payload[:4] with a 28-bit MAC, so it cannot reproduce")
    print("          the Fisker 8-byte-payload / 24-bit MAC (0x498330) -> Fisker uses fiskercan (expected)")
  return failures


class FiskerSecOCECU:
  """Ground-truth ADAS ECU: manages freshness and signs frames with the reference implementation."""

  def __init__(self, key, trip_cnt=1):
    self.key = key
    self.trip_cnt = trip_cnt
    self.reset_cnt = 0
    self.msg_cnt = {ADDR_STEER: 0, ADDR_ACCEL: 0}

  def on_reset(self):
    self.reset_cnt += 1
    self.msg_cnt = {ADDR_STEER: 0, ADDR_ACCEL: 0}

  def _freshness(self, cnt):
    return ((self.trip_cnt & 0xFFFF) << 32) | ((self.reset_cnt & 0xFFFFF) << 8) | (cnt & 0xFF)

  def sign_steer(self, torque):
    cnt = self.msg_cnt[ADDR_STEER]
    counter_a, counter_b = cnt & 0xFF, cnt & 0x03
    payload = bytes([counter_a, torque & 0xFF, (torque >> 8) & 0x0F, counter_b])
    mac = reference_mac_fisker(self.key, ADDR_STEER, payload, self._freshness(cnt))
    self.msg_cnt[ADDR_STEER] = cnt + 1
    return counter_a, counter_b, mac, payload

  def sign_accel(self, accel_payload):
    cnt = self.msg_cnt[ADDR_ACCEL]
    counter_a, counter_b = cnt & 0xFF, cnt & 0x0F
    payload = bytes([counter_a, (accel_payload >> 4) & 0xFF, ((accel_payload & 0x0F) << 4) | counter_b, 0])
    mac = reference_mac_fisker(self.key, ADDR_ACCEL, payload, self._freshness(cnt))
    self.msg_cnt[ADDR_ACCEL] = cnt + 1
    return counter_a, counter_b, mac, payload


def validate_end_to_end(verbose=False):
  """Simulate a trip: the ECU signs frames, openpilot's fiskercan builders reproduce them, a verifier
  confirms CRC + MAC, and a tamper is confirmed to be detected."""
  from opendbc.can import CANPacker
  from opendbc.car.fisker.values import CAR, DBC
  from opendbc.car import Bus

  failures = []
  packer = CANPacker(DBC[CAR.FISKER_OCEAN][Bus.pt])
  ecu = FiskerSecOCECU(TEST_KEY)
  frames = 0

  for i in range(40):
    if i == 20:
      ecu.on_reset()  # simulate a SecOC reset partway through the trip

    counter_a, counter_b, mac, payload = ecu.sign_steer(i % 4096)
    a, dat, _ = fiskercan.create_steer_command(packer, payload[1] | (payload[2] << 8), True, counter_a, counter_b, mac)
    _check_frame(failures, i, ADDR_STEER, dat, mac); frames += 1

    counter_a, counter_b, mac, payload = ecu.sign_accel(i % 4096)
    accel_payload = (payload[1] << 4) | (payload[2] >> 4)
    a, dat, _ = fiskercan.create_accel_command(packer, accel_payload, counter_a, counter_b, mac)
    _check_frame(failures, i, ADDR_ACCEL, dat, mac); frames += 1

  if verbose:
    print(f"  end-to-end: {frames} signed frames checked (trip={ecu.trip_cnt} final reset={ecu.reset_cnt}) "
          f"{'OK' if not failures else 'FAIL'}")
  return failures


def _check_frame(failures, i, addr, dat, mac):
  # CRC byte 0 == CRC8-J1850 over bytes 1..3, per both implementations
  if dat[0] != fiskercan.crc8_j1850(dat[1:4]) or dat[0] != crc8_j1850_reference(dat[1:4]):
    failures.append(f"frame {i} addr={addr:#x}: CRC wrong")
  # SECOC field (bytes 5..7, big-endian) == ECU MAC
  if int.from_bytes(dat[5:8], 'big') != mac:
    failures.append(f"frame {i} addr={addr:#x}: wire MAC {dat[5:8].hex()} != {mac:06x}")
  # tamper inside CRC coverage must break the CRC
  tampered = bytearray(dat)
  tampered[1] ^= 0x01
  if tampered[0] == crc8_j1850_reference(tampered[1:4]):
    failures.append(f"frame {i} addr={addr:#x}: tamper not detected")


def run(verbose=True):
  sections = [
    ("CRC8-J1850", validate_crc),
    ("SecOC known-answer (ground truth)", validate_secoc_known_answer),
    ("Fisker SecOC MAC (fiskercan)", validate_fisker_secoc),
    ("openpilot generic SecOC (secoc.py)", validate_openpilot_secoc),
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
