#!/usr/bin/env python3
"""test_scalemission.py -- scale + count-field robustness for missions/params:
  * enc_param_request_read clamps param_index to the int16 field (a vehicle/spoof reporting >32767
    params would otherwise crash struct.pack -- iter107 fix).
  * enc_mission_count / enc_mission_item_int mask count/seq to their uint16 fields (no overflow).
  * survey_grid caps the generated sweep count (a huge area / tiny spacing = millions of waypoints
    would OOM the app and exceed MAVLink's 65535-item mission -- iter107 fix), while still covering
    the whole area and leaving small surveys unchanged.
  * MissionProtocol uploads a 10k-item mission through the whole request/ack handshake with no crash
    and bounded state.
Pure logic + mock link; no socket, no arming."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtCore import QCoreApplication
import mavlink
from mission import MissionProtocol, MissionItem, survey_grid, MAX_SURVEY_LINES

app = QCoreApplication.instance() or QCoreApplication([])


class Msg:
    __slots__ = ("msgid", "sysid", "compid", "seq", "fields")
    def __init__(self, msgid, **fields):
        self.msgid, self.sysid, self.compid, self.seq, self.fields = msgid, 1, 1, 0, fields


# 1) count / index field packing never overflows -------------------------------------------------
for idx in (-1, 0, 32767, 32768, 40000, 65535, 10 ** 9):
    assert len(mavlink.enc_param_request_read("P", idx)) == 20
for count in (0, 3, 65535, 70000, 10 ** 9):
    assert len(mavlink.enc_mission_count(count)) == 5
for seq in (0, 65535, 70000, 10 ** 9):
    assert len(mavlink.enc_mission_item_int(seq, 47.0, 8.0, 50.0)) == 38

# 2) survey_grid is bounded for any area/spacing, still covers the area, small surveys unchanged ----
poly = [(40.0, 8.0), (50.0, 8.0), (50.0, 9.0), (40.0, 9.0)]      # ~1100 km N-S bounding box
for sp in (35.0, 5.0, 0.5, 0.01, 0.0, -5.0):                     # incl. degenerate zero/negative
    wps = survey_grid(poly, spacing_m=sp, alt=50.0)
    assert len(wps) <= 2 * MAX_SURVEY_LINES + 4, f"survey unbounded at {sp}m: {len(wps)}"
    assert wps and abs(wps[0].lat - 40.0) < 0.5                  # starts at the south edge
    assert abs(wps[-1].lat - 50.0) < (50.0 - 40.0) / MAX_SURVEY_LINES * 4   # reaches the north edge
assert survey_grid([(47.0, 8.0)]) == []                         # <2 points -> empty, no crash
assert survey_grid([]) == []
# a normal, small survey is unaffected by the cap
small = survey_grid([(47.40, 8.54), (47.41, 8.55)], spacing_m=35.0)
assert 2 <= len(small) <= 200, len(small)

# 3) MissionProtocol uploads a 10k-item mission end to end, no crash, bounded state ----------------
class MockLink:
    def __init__(self):
        self.is_open = True
        self.remote = ("127.0.0.1", 14550)
        self.sent_count = 0
        self.sent_items = 0
        self.acked = False
    def send_mission_count(self, tgt, n, mt): self.sent_count = n
    def send_mission_item(self, tgt, item, mt): self.sent_items += 1
    def send_mission_request_int(self, tgt, seq, mt): pass
    def send_mission_request_list(self, tgt, mt): pass
    def send_mission_ack(self, tgt, res, mt): self.acked = True
    def send_mission_clear(self, tgt, mt): pass


N = 10000
link = MockLink()
mp = MissionProtocol(lambda: link, lambda: 1)
items = [MissionItem(i, 47.0 + i * 1e-6, 8.0 + i * 1e-6, 50.0) for i in range(N)]
mp.upload(items, 0)
assert link.sent_count == N, f"MISSION_COUNT sent {link.sent_count}, expected {N}"
# the autopilot walks every seq, then ACKs -- must not crash and must end idle
for s in range(N):
    mp.handle(Msg(mavlink.MISSION_REQUEST_INT, seq=s))
mp.handle(Msg(mavlink.MISSION_ACK, type=0))
assert link.sent_items == N, f"sent {link.sent_items} items, expected {N}"
assert mp.state == "idle" and not mp.busy, (mp.state, mp.busy)
# a re-request of an in-range seq still re-sends (no corruption at scale)
mp.upload(items, 0)
mp.handle(Msg(mavlink.MISSION_REQUEST_INT, seq=9999))
mp.handle(Msg(mavlink.MISSION_REQUEST_INT, seq=9999))   # duplicate -> re-sent, no crash

print(f"SCALEMISSION PASSED (count/index fields clamped, survey<=~{2 * MAX_SURVEY_LINES}, 10k upload ok)")
