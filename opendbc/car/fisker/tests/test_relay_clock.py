from opendbc.car.fisker.relay import LEAD, RelayClock

WINDOW = 101


class Stock:
  """The stock ADAS module's 0x1D0 stream: AliveCounter 0..14 and a SecOC wire byte of
  (position & 63) << 2 | (Reset & 3), position restarting at 1 each Reset window."""

  def __init__(self, reset=1000, pos=0, alive=0):
    self.reset, self.pos, self.alive = reset, pos, alive

  def frame(self):
    self.pos += 1
    if self.pos > WINDOW:
      self.pos, self.reset = 1, self.reset + 1
    self.alive = (self.alive + 1) % 15
    return {"ADAS_1D0_AliveCounter": self.alive, "ADAS_1D0_SSecOC_Fresh_Byte0": ((self.pos & 63) << 2) | (self.reset & 3)}


def run_to_sync(clk, stock):
  while not clk.synced:
    clk.observe([stock.frame()], stock.reset)


def test_not_ready_before_a_window_boundary():
  clk, stock = RelayClock(), Stock(pos=30)
  for _ in range(20):
    f = stock.frame()
    clk.observe([f], stock.reset)
    assert clk.tick(f["ADAS_1D0_AliveCounter"], stock.reset) is None
  run_to_sync(clk, stock)
  assert clk.synced and clk.stock_window_reset == stock.reset


def test_alive_leads_stock_by_one_and_wraps():
  clk, stock = RelayClock(), Stock(pos=30)
  run_to_sync(clk, stock)
  for _ in range(40):
    f = stock.frame()
    clk.observe([f], stock.reset)
    t = clk.tick(f["ADAS_1D0_AliveCounter"], stock.reset)
    assert t.alive == (f["ADAS_1D0_AliveCounter"] + LEAD) % 15
    assert 0 <= t.alive <= 14


def test_takeover_continues_stock_secoc_position():
  clk, stock = RelayClock(), Stock(pos=40)
  run_to_sync(clk, stock)
  for _ in range(10):
    clk.observe([stock.frame()], stock.reset)
  f = stock.frame()
  clk.observe([f], stock.reset)
  t = clk.tick(f["ADAS_1D0_AliveCounter"], stock.reset)
  assert (t.secoc_reset, t.secoc_ctr) == (stock.reset, stock.pos + LEAD)


def test_secoc_consecutive_within_and_across_windows():
  clk, stock = RelayClock(), Stock(pos=60)
  run_to_sync(clk, stock)
  prev = None
  for _ in range(3 * WINDOW):
    f = stock.frame()
    clk.observe([f], stock.reset)
    t = clk.tick(f["ADAS_1D0_AliveCounter"], stock.reset)
    if prev is not None:
      same = (t.secoc_reset, t.secoc_ctr) == (prev.secoc_reset, prev.secoc_ctr + 1)
      new_window = t.secoc_reset == prev.secoc_reset + 1 and t.secoc_ctr == 1
      assert same or new_window, (prev, t)
    prev = t


def test_gw_announce_ahead_of_stock_restarts_our_window():
  clk, stock = RelayClock(), Stock(pos=95)
  run_to_sync(clk, stock)
  while stock.pos < WINDOW - 2:
    clk.observe([stock.frame()], stock.reset)
  # the GW announces the next Reset window before the stock stream restarts
  f = stock.frame()
  clk.observe([f], stock.reset)
  t = clk.tick(f["ADAS_1D0_AliveCounter"], stock.reset + 1)
  assert (t.secoc_reset, t.secoc_ctr) == (stock.reset + 1, 1)
  f = stock.frame()
  clk.observe([f], stock.reset)
  t2 = clk.tick(f["ADAS_1D0_AliveCounter"], stock.reset + 1)
  assert (t2.secoc_reset, t2.secoc_ctr) == (stock.reset + 1, 2)


def test_multiple_stock_frames_in_one_batch_send_consecutive_counters():
  clk, stock = RelayClock(), Stock(pos=30)
  run_to_sync(clk, stock)
  clk.observe([stock.frame()], stock.reset)
  frames = [stock.frame() for _ in range(3)]
  ticks = [clk.tick(a, stock.reset) for a, _, _, _ in clk.observe(frames, stock.reset)]
  assert [t.alive for t in ticks] == [(f["ADAS_1D0_AliveCounter"] + LEAD) % 15 for f in frames]
  assert [t.secoc_ctr for t in ticks] == [ticks[0].secoc_ctr + i for i in range(3)]


def test_missed_stock_frames_resync_without_going_backwards():
  clk, stock = RelayClock(), Stock(pos=30)
  run_to_sync(clk, stock)
  f = stock.frame()
  clk.observe([f], stock.reset)
  t1 = clk.tick(f["ADAS_1D0_AliveCounter"], stock.reset)
  for _ in range(5):          # five stock frames we never saw
    stock.frame()
  f = stock.frame()
  clk.observe([f], stock.reset)
  assert clk.stock_full == stock.pos
  t2 = clk.tick(f["ADAS_1D0_AliveCounter"], stock.reset)
  assert t2.secoc_ctr == t1.secoc_ctr + 1       # ours stays +1 per frame, never ahead of the stock numbering


def test_stop_then_fresh_takeover():
  clk, stock = RelayClock(), Stock(pos=30)
  run_to_sync(clk, stock)
  for _ in range(5):
    f = stock.frame()
    clk.observe([f], stock.reset)
    clk.tick(f["ADAS_1D0_AliveCounter"], stock.reset)
  clk.stop()
  for _ in range(7):
    clk.observe([stock.frame()], stock.reset)
  f = stock.frame()
  clk.observe([f], stock.reset)
  t = clk.tick(f["ADAS_1D0_AliveCounter"], stock.reset)
  assert t.secoc_ctr == stock.pos + LEAD        # continues the stock position, not our old count
