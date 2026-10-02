"""
Counter clock for the messages openpilot replaces on the stock ADAS module's behalf: 0x1D0 (steering
angle), 0x1C0 (lateral activation) and 0x121 (accel).

The stock module sends all five of its 100 Hz commands (0x117/0x118/0x121/0x1C0/0x1D0) with one shared
AliveCounter in lockstep (offset 0 in 5978 of 5997 recorded frames), and 0x1D0/0x121 carry the same
SecOC counter. Rather than free-running a counter on our own 100 Hz tick, which drifts (it ended two
ticks behind the stock one in the drive that faulted the car), the relay is driven by the stock stream
itself: one relay set per stock 0x1D0 frame, with the counters one step ahead of it.

    AliveCounter  = stock + 1 (mod 15)
    SecOC counter = position in the current Reset window (1, 2, ...; the low 6 bits go on the wire)

Panda checks that our counters continue the stock ones when it takes over and that they only move
forward afterwards (fisker.h, relay arbiter), and hands the stock stream back at exactly our last
counter + 1 on release, so the receivers see a strictly +1 sequence at both transitions.

The stock wire byte SSecOC_Fresh_Byte0 = (counter[5:0] << 2) | (Reset counter & 3). A Reset window runs
about 101 frames and the stock counter restarts at 1 on its first frame, so the flag change marks the
window boundary frame-accurately and the counter wraps at 64 inside a window.
"""
from dataclasses import dataclass

ALIVE_MOD = 15
LEAD = 2          # our counter is this many frames ahead of the stock frame that triggered it
WINDOW_MAX = 128  # a Reset window is ~101 frames; anything past this means we lost track


@dataclass
class RelayTick:
  alive: int         # AliveCounter for every relayed message of this tick
  secoc_ctr: int     # SecOC message counter (full value, low 6 bits on the wire)
  secoc_reset: int   # Reset counter (24 bit) the MAC is computed with


def _reset_for_flag(flag: int, cs_reset: int) -> int | None:
  """The full Reset counter whose low 2 bits are `flag`, taken from the three values around the one
  the GW sync last reported (the sync can be a frame or two ahead of or behind the stock stream)."""
  for r in (cs_reset, cs_reset - 1, cs_reset + 1):
    if (r & 3) == flag:
      return r
  return None


class RelayClock:
  def __init__(self):
    self.reset()

  def reset(self):
    # stock stream position, tracked continuously whether or not we are relaying
    self.stock_flag: int | None = None
    self.stock_window_reset: int | None = None   # full Reset counter of the stock window
    self.stock_full = 0                          # position of the last stock frame in its window (1-based)
    self.synced = False                          # have we seen a window boundary (so the window position is known)
    # our own position
    self.ours_reset: int | None = None
    self.ours_full = 0
    self.relaying = False

  def observe(self, stock_frames: list[dict[str, float]], cs_reset: int) -> list[tuple[int, int, int, int]]:
    """Feed every stock 0x1D0 frame since the last call (oldest first). Returns, per frame,
    (alive, wire_counter, window_reset, window_position) for tracking; call tick() per frame to relay."""
    out = []
    for fr in stock_frames:
      alive = int(fr["ADAS_1D0_AliveCounter"])
      fresh = int(fr["ADAS_1D0_SSecOC_Fresh_Byte0"])
      flag, wire = fresh & 3, fresh >> 2
      if self.stock_flag is not None and flag != self.stock_flag:
        # window boundary: the stock counter restarts at 1
        self.stock_full = 1
        r = _reset_for_flag(flag, cs_reset)
        self.stock_window_reset = r if r is not None else self.stock_window_reset
        self.synced = r is not None
      elif self.synced:
        self.stock_full += 1
        if (self.stock_full & 63) != wire:
          # we missed stock frames: re-sync to the nearest position with this wire counter
          cands = [c for c in range(wire, WINDOW_MAX + 1, 64) if c >= 1]
          self.stock_full = min(cands, key=lambda c: abs(c - self.stock_full)) if cands else 0
          self.synced = bool(cands)
      self.stock_flag = flag
      out.append((alive, wire, self.stock_window_reset, self.stock_full))
    return out

  def tick(self, alive: int, cs_reset: int) -> RelayTick | None:
    """The relay set for one stock frame (with AliveCounter `alive`), or None while the window
    position is unknown (until the first window boundary after start)."""
    if not self.synced or self.stock_window_reset is None:
      return None
    announced = self.ours_reset is not None and cs_reset > self.ours_reset
    if not self.relaying:
      # takeover: continue the stock stream
      self.relaying = True
      if cs_reset > self.stock_window_reset:
        self.ours_reset, self.ours_full = cs_reset, 1
      else:
        self.ours_reset, self.ours_full = self.stock_window_reset, self.stock_full + LEAD
    elif announced:
      # the GW announced a new Reset window that the stock stream has not restarted yet: the receivers
      # expect 1 next, then +1 per frame; stay self-consistent from here
      self.ours_reset, self.ours_full = cs_reset, 1
    else:
      # +1 per frame from here, with no re-sync to the stock numbering: if we ever differ from it, being
      # behind (stock resumes a step or two ahead of us) is acceptable to the receivers, being ahead
      # (a stale counter) is not
      self.ours_full += 1
    return RelayTick(alive=(alive + LEAD) % ALIVE_MOD, secoc_ctr=self.ours_full, secoc_reset=self.ours_reset)

  def stop(self):
    """The relay ended (released); the next tick() is a fresh takeover."""
    self.relaying = False
    self.ours_reset = None
    self.ours_full = 0
