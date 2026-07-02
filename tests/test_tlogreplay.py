#!/usr/bin/env python3
"""test_tlogreplay.py -- .tlog replay robustness against a corrupt / truncated recording (realistic
after an interrupted record). Two layers:
  * read_tlog() must never crash and must drop trailing garbage / truncated frames (the byte parser
    is fuzzed elsewhere; here it's the reader's framing + bounds).
  * ReplayLink scheduling must not STALL on a bad timestamp. Recorded microsecond timestamps can be
    non-monotonic or absurd (a corrupt 1e18 us ~= 31000 years); scheduling straight off `t_us - t0`
    let such a frame never come 'due', freezing playback forever (regression guard for the iter102
    fix -- replay_schedule() clamps each inter-frame gap to [0, max_gap]).
No link to a real vehicle, no arming. Drives _pump() directly (no QTimer/event loop -> no hang)."""
import os
import sys
import struct
import time
import tempfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mavlink
import tlog
from link import ReplayLink

app = QApplication.instance() or QApplication([])


def hb():
    return mavlink.frame(mavlink.HEARTBEAT, b"\x00" * mavlink._WIRE[mavlink.HEARTBEAT][2], 0, 1, 1)


def rec(t, fr):
    return struct.pack(">Q", t & 0xFFFFFFFFFFFFFFFF) + fr


def write(data):
    fd, p = tempfile.mkstemp(suffix=".tlog")
    with os.fdopen(fd, "wb") as f:
        f.write(data)
    return p


# 1) read_tlog must not crash on any adversarial input, and drops garbage/truncated tails ----------
GOOD3 = rec(1000, hb()) + rec(2000, hb()) + rec(3000, hb())
CASES = {
    "normal": (GOOD3, 3),
    "empty": (b"", 0),
    "garbage": (bytes(range(256)) * 3, 0),
    "ts_then_junk": (struct.pack(">Q", 1000) + b"\x11\x22\x33\x44", 0),
    "truncated_tail": (rec(1000, hb()) + struct.pack(">Q", 2000) + hb()[:3], 1),  # good one kept
    "ts_no_frame": (struct.pack(">Q", 1000), 0),
    "huge_len_claim": (struct.pack(">Q", 1000) + b"\xFD\xFF" + b"\x00" * 8, 0),    # 255B not present
    "good_then_noise": (GOOD3 + bytes(range(200)), 3),
}
for name, (data, expect) in CASES.items():
    p = write(data)
    r = tlog.read_tlog(p)                # must not raise
    os.unlink(p)
    assert len(r) == expect, f"{name}: read {len(r)} records, expected {expect}"

# 2) replay_schedule: monotonic, gaps clamped to [0, max_gap]; normal cadence preserved ------------
assert tlog.replay_schedule([(1000, b""), (2000, b""), (3000, b"")]) == [0, 1000, 2000]
assert tlog.replay_schedule([(5000, b""), (1000, b""), (3000, b"")]) == [0, 0, 2000]  # backwards->0
hg = tlog.replay_schedule([(1000, b""), (10 ** 18, b""), (2000, b"")])
assert hg == [0, 10_000_000, 10_000_000], hg                                          # huge->capped
assert tlog.replay_schedule([]) == []
sched = tlog.replay_schedule([(1000, b"")] + [(1000 + 10 ** 18 * k, b"") for k in range(1, 5)])
assert all(sched[i] <= sched[i + 1] for i in range(len(sched) - 1)), "schedule not monotonic"

# 3) ReplayLink must not STALL: after a large elapsed, EVERY frame plays and the timer stops --------
for name, data in [("huge_gap", rec(1000, hb()) + rec(10 ** 18, hb()) + rec(2000, hb())),
                   ("backwards", rec(5000, hb()) + rec(1000, hb()) + rec(3000, hb())),
                   ("normal", GOOD3)]:
    p = write(data)
    rl = ReplayLink()
    assert rl.open(path=p, speed=1000.0)
    rl._wall0 = time.monotonic() - 3600          # pretend an hour elapsed -> all frames long due
    fed = []
    rl.messages.connect(lambda b: fed.extend(m.msgid for m in b))
    for _ in range(6):
        rl._pump()
    assert rl._i == len(rl._records), f"{name}: STALLED at {rl._i}/{len(rl._records)}"
    assert not rl._tick.isActive(), f"{name}: replay timer still running after completion"
    assert mavlink.HEARTBEAT in fed, f"{name}: no frames decoded on replay"
    rl.close()
    os.unlink(p)

# 4) record -> read round-trip: TlogWriter frames read back intact -------------------------------
p = write(b"")
w = tlog.TlogWriter(p)
w.write(hb() + hb(), 111)
w.write(hb(), 222)
w.close()
r = tlog.read_tlog(p)
os.unlink(p)
assert len(r) == 3 and all(fr[0] in (0xFE, 0xFD) for _t, fr in r), r

# -- audit batch 12: big-file cap -- a huge log must not be slurped whole (OOM/GUI freeze) ---------
many = b"".join(rec(1000 * i, hb()) for i in range(200))
p = write(many)
full = tlog.read_tlog(p)
assert len(full) == 200 and tlog.read_tlog.truncated is False, "unbounded read regressed"
one = len(rec(0, hb()))
capped = tlog.read_tlog(p, max_bytes=one * 20 + 3)          # cap mid-record
os.unlink(p)
assert tlog.read_tlog.truncated is True, "over-cap file not flagged truncated"
assert 0 < len(capped) <= 20, f"cap not honoured: {len(capped)} records"
assert all(fr[0] in (0xFE, 0xFD) for _t, fr in capped), "capped read emitted a partial frame"

print("TLOGREPLAY PASSED (corrupt/truncated .tlog: no crash, no replay stall, round-trip intact; "
      "big-file cap honoured + flagged)")
