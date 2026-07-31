from opendbc.car.fisker import fiskercan
from opendbc.car.fisker import secoc_mirror as sm


class TestFiskerAuthorityCRC:
  def test_authority_crc_covers_full_payload(self):
    # LKAS_STEER_AUTHORITY (0x1C0) checksums bytes 1-7 with xor-out 0x03; the SecOC control frames
    # checksum only bytes 1-3.
    assert fiskercan._CRC_COVERAGE[fiskercan.LKAS_STEER_AUTHORITY_ADDR] == (1, 8)
    assert fiskercan._CRC_XOROUT[fiskercan.LKAS_STEER_AUTHORITY_ADDR] == 0x03
    assert fiskercan._CRC_COVERAGE[fiskercan.ADAS_STEER_CONTROL_ADDR] == (1, 4)
    assert fiskercan._CRC_COVERAGE[fiskercan.ADAS_ACCEL_CONTROL_ADDR] == (1, 4)

    frame = bytes([0x00, 0x31, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00])
    expected = fiskercan.crc8_j1850(frame[1:8], init=0x00, xorout=0x03)
    assert fiskercan._checksum(fiskercan.LKAS_STEER_AUTHORITY_ADDR, frame) == expected

  def test_steer_crc_unchanged(self):
    frame = bytes([0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77])
    expected = fiskercan.crc8_j1850(frame[1:4], init=0x00, xorout=0x25)
    assert fiskercan._checksum(fiskercan.ADAS_STEER_CONTROL_ADDR, frame) == expected

  def test_authority_frame_bytes_match_capture(self):
    # Active LKAS: b2=0x49 b3=0x51 (validity fields valid); idle: b2=0x48 b3=0x10. b4..b7 constant 0.
    addr, active, bus = fiskercan.create_authority_command(0x0A, lat_active=True)
    assert addr == 0x1C0 and bus == 0
    assert active[1] & 0x0F == 0x0A          # ARC
    assert active[2] == 0x49 and active[3] == 0x51
    assert active[4:] == bytes(4)
    assert active[0] == fiskercan.crc8_j1850(active[1:8], init=0x00, xorout=0x03)

    _, idle, _ = fiskercan.create_authority_command(0x00, lat_active=False)
    assert idle[2] == 0x48 and idle[3] == 0x10
    assert idle[0] == fiskercan.crc8_j1850(idle[1:8], init=0x00, xorout=0x03)


class TestArc:
  def test_next_arc_skips_0xf(self):
    seq, a = [], 0
    for _ in range(16):
      seq.append(a)
      a = sm.next_arc(a)
    assert 0x0F not in seq
    assert max(seq) == 0x0E
    assert seq[15] == 0x00  # wraps 0x0E -> 0x00


class TestReconstructRunning:
  def test_tracks_past_6bit_wrap(self):
    running = 60
    for true in range(61, 200):
      running = sm.reconstruct_running(running, true & 0x3F)
      assert running == true

  def test_self_heals_dropped_frames(self):
    # smallest value > prev whose low 6 bits match the wire (dropped 11,12,13)
    assert sm.reconstruct_running(10, 14 & 0x3F) == 14


class TestSecOCCounterMirror:
  def test_seed_continues_full_running_counter(self):
    m = sm.SecOCCounterMirror([0x1D0, 0x121, 0x1C0])
    epoch = 2
    for cnt in range(1, 100):  # Hydra counter climbs to 99 (past the 6-bit wire range)
      m.observe(0x1D0, epoch, cnt & 0x3F, cnt & 0x0F)
      m.observe_arc(0x1C0, cnt % 15)
    assert m.running[0x1D0] == 99
    assert m.seed_secoc(0x1D0, epoch) == 100
    assert m.seed_arc(0x1C0) == sm.next_arc(99 % 15)

  def test_no_seed_across_epoch(self):
    m = sm.SecOCCounterMirror([0x1D0])
    m.observe(0x1D0, 2, 5, 5)
    assert m.seed_secoc(0x1D0, 3) is None

  def test_no_seed_without_observation(self):
    m = sm.SecOCCounterMirror([0x1D0])
    assert m.seed_secoc(0x1D0, 1) is None
    assert m.seed_arc(0x1D0) is None

  def test_epoch_change_resets_running(self):
    m = sm.SecOCCounterMirror([0x1D0])
    m.observe(0x1D0, 1, 40, 8)
    m.observe(0x1D0, 2, 3, 3)  # new epoch: running restarts from the observed wire value
    assert m.running[0x1D0] == 3
    assert m.epoch[0x1D0] == 2
