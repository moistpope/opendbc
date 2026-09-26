"""
ICC_0x52A feature-settings rewrite: the ICC's frame from bus 0 is re-sent on bus 2 with
ICC_SETTINGS_OVERRIDES applied, keeping its AliveCounter and every other bit.
"""

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car.fisker.fiskercan import FiskerCAN, fisker_plain_checksum
from opendbc.car.fisker.values import CANBUS, ICC_SETTINGS_OVERRIDES

DBC_NAME = "fisker_ocean_adas"

# Captured from the ICC on bus 0 (one per AliveCounter value, 0..14).
ICC_FRAMES = [bytes.fromhex(h) for h in (
  "1C2024A000008220", "412124A000008220", "A62224A000008220", "FB2324A000008220",
  "752424A000008220", "282524A000008220", "CF2624A000008220", "922724A000008220",
  "CE2824A000008220", "932924A000008220", "742A24A000008220", "292B24A000008220",
  "A72C24A000008220", "FA2D24A000008220", "1D2E24A000008220",
)]


def decode(frames: list[bytes], bus: int = CANBUS.pt) -> list[dict[str, float]]:
  cp = CANParser(DBC_NAME, [("ICC_0x52A", float('nan'))], bus)
  cp.update([(i * 200_000_000, [(0x52A, f, bus)]) for i, f in enumerate(frames)])
  msgs = cp.vl_all["ICC_0x52A"]
  return [{sig: vals[i] for sig, vals in msgs.items()} for i in range(len(frames))]


def test_dbc_round_trips_captured_frames():
  packer = CANPacker(DBC_NAME)
  for frame, values in zip(ICC_FRAMES, decode(ICC_FRAMES), strict=True):
    assert packer.make_can_msg("ICC_0x52A", CANBUS.pt, values)[1] == frame


@pytest.mark.parametrize("frame", ICC_FRAMES)
def test_create_icc_settings(frame):
  fcan = FiskerCAN(None, CANPacker(DBC_NAME))
  src = decode([frame])[0]
  addr, data, bus = fcan.create_icc_settings(src)

  assert addr == 0x52A
  assert bus == CANBUS.cam
  assert len(data) == 8
  assert data[0] == fisker_plain_checksum(0x52A, bytes([0]) + data[1:])

  out = decode([data], bus=CANBUS.cam)[0]
  for sig, val in src.items():
    if sig == "ICC_0x52A_CheckSum":
      continue
    assert out[sig] == ICC_SETTINGS_OVERRIDES.get(sig, val), sig

  # ICC's counter is carried through (freshness stays in sync with the ICC)
  assert out["ICC_0x52A_AliveCounter"] == src["ICC_0x52A_AliveCounter"]
  # undocumented bits set by the ICC survive the repack
  assert out["ICC_0x52A_Undoc_Bit49"] == 1 and out["ICC_0x52A_Undoc_Bit55"] == 1
