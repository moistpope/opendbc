import struct
from Crypto.Cipher import AES
from Crypto.Hash import CMAC


def _cmac24(key: bytes, data: bytes) -> bytes:
  c = CMAC.new(key, ciphermod=AES)
  c.update(data)
  return c.digest()[:3]   # 24 most-significant bits


def build_freshness(trip: int, reset: int, msg_counter: int) -> bytes:
  """Assemble the 8-byte full freshness value used as MAC input"""
  reset_flag = reset & 0x3
  return (struct.pack(">H", trip & 0xFFFF)
          + (reset & 0xFFFFFF).to_bytes(3, "big")
          + struct.pack(">H", msg_counter & 0xFFFF)
          + bytes([(reset_flag << 6) & 0xFF]))


def secoc_mac(key: bytes, can_id: int, pdu: bytes, trip: int, reset: int, msg_counter: int) -> bytes:
  """
  Compute the 3-byte Short-SecOC MAC for a data message.
  `pdu` is the first 4 bytes of the frame (byte 0 = E2E checksum already filled).
  """
  fresh = build_freshness(trip, reset, msg_counter)
  to_auth = struct.pack(">H", can_id & 0xFFFF) + pdu[:4] + fresh
  return _cmac24(key, to_auth)


def wire_freshness_byte(msg_counter: int, reset: int) -> int:
  """The SSecOC_Fresh_Byte0 value transmitted on the wire."""
  return (((msg_counter & 0x3F) << 2) | (reset & 0x3)) & 0xFF


def stamp_secoc(key: bytes, can_id: int, frame8: bytes, trip: int, reset: int, msg_counter: int) -> bytes:
  """
  Given an 8-byte frame with bytes 0..3 populated (incl. E2E checksum), write the
  SecOC tail (byte4 = wire freshness, bytes5..7 = MAC) and return the full frame.
  """
  buf = bytearray(frame8[:8].ljust(8, b"\x00"))
  buf[4] = wire_freshness_byte(msg_counter, reset)
  buf[5:8] = secoc_mac(key, can_id, bytes(buf[0:4]), trip, reset, msg_counter)
  return bytes(buf)


# ---- GW freshness-sync message (0x20) -------------------------------------

SYNC_CAN_ID = 0x20


def sync_mac(key: bytes, trip: int, reset: int) -> bytes:

  to_auth = struct.pack(">H", SYNC_CAN_ID) + struct.pack(">H", trip & 0xFFFF) + (reset & 0xFFFFFF).to_bytes(3, "big")
  return _cmac24(key, to_auth)


def verify_sync(key: bytes, sync_payload: bytes) -> bool:

  trip = struct.unpack(">H", sync_payload[0:2])[0]
  reset = int.from_bytes(sync_payload[2:5], "big")
  return sync_mac(key, trip, reset) == sync_payload[5:8]


def parse_sync(sync_payload: bytes) -> tuple[int, int]:

  trip = struct.unpack(">H", sync_payload[0:2])[0]
  reset = int.from_bytes(sync_payload[2:5], "big")
  return trip, reset
