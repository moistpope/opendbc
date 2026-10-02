"""The release frame's address and bits are shared with panda's relay arbiter in fisker.h."""
import os
import re

from opendbc.car.fisker.values import RELAY_CTRL_ADDR, RELAY_RELEASE_ACCEL, RELAY_RELEASE_STEER_LAT

FISKER_H = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../safety/modes/fisker.h")


def _header():
  with open(FISKER_H) as f:
    return f.read()


def test_ctrl_addr_matches_panda():
  m = re.search(r"#define FISKER_RELAY_CTRL_ADDR\s+0x([0-9A-Fa-f]+)U", _header())
  assert m and int(m.group(1), 16) == RELAY_CTRL_ADDR


def test_release_bits_match_panda():
  bits = {int(a, 16): int(b, 16) for a, b in re.findall(r"\.addr = 0x([0-9A-Fa-f]+)U, \.release_bit = 0x([0-9A-Fa-f]+)U", _header())}
  assert bits[0x1D0] | bits[0x1C0] == RELAY_RELEASE_STEER_LAT
  assert bits[0x121] == RELAY_RELEASE_ACCEL
