"""Continuously mirror the stock ADAS/Hydra module's per-message freshness counters.

For a seamless takeover, openpilot must continue the exact counter sequences the receiving ECUs
(EPS/EPS2) already expect from the Hydra. Two independent counters ride on each ADAS control frame:

  * The SecOC running message counter (0x1D0/0x121). Only its low 6 bits appear on the wire
    (MESSAGE_COUNTER_LOWER), but the full running value -- which climbs well past 63 within one
    reset epoch (observed ~102) -- is what feeds the AES-CMAC. The receiver reconstructs the full
    value by predicting prev+1 and jumping forward to the nearest value whose low 6 bits match the
    wire, and it authenticates against that reconstructed value. If we seed our counter from the
    raw 6-bit wire value (or from zero) our MAC is computed over a different freshness than the EPS
    expects and every frame is rejected. So we reconstruct the full running counter the same way the
    receiver does, from the Hydra's own frames, and hand off running+1 at takeover.

  * The Alive-Rolling-Counter (COUNTER_A, byte-1 low nibble). It advances by one per frame and is
    checked independently (DTC U12F782/U12F882); on 0x1C0 it is a 15-state counter that skips 0x0F.
    Seeding it from the Hydra's last value and advancing by one keeps the receiver's delta check
    happy regardless of how it relates to the SecOC counter.

This module is pure (no CAN/hardware dependencies) so it can be unit tested. CarState feeds it every
observed stock frame; CarController reads a seed from it on the latActive/longActive rising edge.
"""

WIRE_COUNTER_MASK = 0x3F  # MESSAGE_COUNTER_LOWER is 6 bits
ARC_STATES = 15           # ARC cycles 0..0x0E then wraps; 0x0F is skipped


def next_arc(arc: int) -> int:
  """Advance a 15-state Alive-Rolling-Counter, skipping the invalid 0x0F state."""
  return (arc + 1) % ARC_STATES


def reconstruct_running(prev_running: int, wire_counter6: int) -> int:
  """Given the previous full running counter and a freshly observed 6-bit wire counter, return the
  smallest full value >= prev_running+1 whose low 6 bits equal the wire value (self-healing against
  dropped frames), matching how the AUTOSAR receiver recovers the counter."""
  base = prev_running + 1
  return base + ((wire_counter6 - base) & WIRE_COUNTER_MASK)


class SecOCCounterMirror:
  def __init__(self, addrs: list[int]):
    self.running: dict[int, int | None] = {a: None for a in addrs}
    self.arc: dict[int, int | None] = {a: None for a in addrs}
    self.epoch: dict[int, int | None] = {a: None for a in addrs}

  def observe(self, addr: int, reset_counter: int, wire_counter6: int, arc: int) -> None:
    """Update the mirror from one stock frame of message `addr`."""
    if addr not in self.running:
      return

    if self.epoch[addr] != reset_counter or self.running[addr] is None:
      # New reset epoch (the counter restarts at 1) or the very first frame we have seen: the best
      # estimate of the full running counter is the observed wire value itself.
      self.epoch[addr] = reset_counter
      self.running[addr] = wire_counter6 & WIRE_COUNTER_MASK
    else:
      self.running[addr] = reconstruct_running(self.running[addr], wire_counter6 & WIRE_COUNTER_MASK)

    self.arc[addr] = arc & 0x0F

  def observe_arc(self, addr: int, arc: int) -> None:
    """Update just the ARC for a non-SecOC frame (e.g. LKAS_STEER_AUTHORITY 0x1C0), which carries
    no SecOC running counter."""
    if addr in self.arc:
      self.arc[addr] = arc & 0x0F

  def seed_secoc(self, addr: int, reset_counter: int) -> int | None:
    """Full SecOC message counter to use for our next frame of `addr`, or None if we have no
    in-epoch observation of the stock module to continue from."""
    if self.epoch.get(addr) != reset_counter or self.running.get(addr) is None:
      return None
    return self.running[addr] + 1

  def seed_arc(self, addr: int) -> int | None:
    """ARC (byte-1 low nibble) to use for our next frame of `addr`, or None if unseen."""
    if self.arc.get(addr) is None:
      return None
    return next_arc(self.arc[addr])
