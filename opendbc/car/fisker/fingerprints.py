""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.fisker.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.OCEAN: {
    (Ecu.abs, 0x7b0, None): [
      b'F152650290\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.dsu, 0x791, None): [
      b'881515003400\x00\x00\x00\x00',
    ],
    (Ecu.eps, 0x7a1, None): [
      b'8965B50022\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.engine, 0x700, None): [
      b'\x028966350K7200\x00\x00\x00\x00896655066200\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x750, 0xf): [
      b'8821F4702300\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x750, 0x6d): [
      b'8646F5001200\x00\x00\x00\x00',
    ],
  },
}
