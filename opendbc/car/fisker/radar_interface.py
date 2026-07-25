from opendbc.car.interfaces import RadarInterfaceBase

# The Fisker Ocean forward radar (MRR, 0x33B) has not been decoded yet, so radar is
# unavailable (see interface.py: ret.radarUnavailable = True) and this is a no-op stub.
# TODO: parse MRR / MAYBE_CORNER_RADAR tracks.


class RadarInterface(RadarInterfaceBase):
  pass
