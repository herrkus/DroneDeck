#!/usr/bin/env python3
"""test_missionrobust.py -- MissionProtocol robustness against adversarial / buggy autopilot
behaviour (state machines are as bug-prone as the parser was). Drives the protocol through a headless
mock link: out-of-order items, duplicate items, retransmitted requests, an ACK arriving mid-transfer,
an empty mission, a rejected upload, unexpected messages in the wrong state, and timeout+retry. Every
case must never hang/crash/corrupt and must complete or fail cleanly. No real link, no arming."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtCore import QCoreApplication
import mavlink
from mission import MissionProtocol, MissionItem, MAX_RETRIES

app = QCoreApplication.instance() or QCoreApplication([])


class Msg:
    __slots__ = ("msgid", "sysid", "compid", "seq", "fields")
    def __init__(self, msgid, **fields):
        self.msgid, self.sysid, self.compid, self.seq, self.fields = msgid, 1, 1, 0, fields


class MockLink:
    """Records every send_* call; is_open/remote satisfy MissionProtocol._ready()."""
    def __init__(self):
        self.is_open = True
        self.remote = ("127.0.0.1", 14550)
        self.sent = []                       # list of (method, *args)
    def _rec(self, name, *a): self.sent.append((name, *a))
    def send_mission_count(self, tgt, n, mt): self._rec("count", n, mt)
    def send_mission_item(self, tgt, item, mt): self._rec("item", item.seq, mt)
    def send_mission_request_list(self, tgt, mt): self._rec("req_list", mt)
    def send_mission_request_int(self, tgt, seq, mt): self._rec("req", seq, mt)
    def send_mission_ack(self, tgt, res, mt): self._rec("ack", res, mt)
    def send_mission_clear(self, tgt, mt): self._rec("clear", mt)


def new_proto():
    link = MockLink()
    mp = MissionProtocol(lambda: link, lambda: 1)
    fin = []
    dl = []
    mp.finished.connect(lambda ok, msg: fin.append((ok, msg)))
    mp.downloaded.connect(lambda items: dl.append(items))
    return mp, link, fin, dl


def item_msg(seq, lat=47.0 + 0.0, lon=8.0, alt=50.0, command=16, frame=3):
    return Msg(mavlink.MISSION_ITEM_INT, seq=seq, x=int((47.0 + seq * 0.001) * 1e7),
               y=int(lon * 1e7), z=alt, command=command, frame=frame, autocontinue=1,
               param1=float(seq), param2=0.0, param3=0.0, param4=0.0)


def reqs(link):                              # the seqs the protocol requested, in order
    return [a[1] for a in link.sent if a[0] == "req"]


# 1) normal download baseline ----------------------------------------------------------
mp, link, fin, dl = new_proto()
mp.download(0)
mp.handle(Msg(mavlink.MISSION_COUNT, count=3))
for s in (0, 1, 2):
    mp.handle(item_msg(s))
assert fin == [(True, "downloaded 3 items")], fin
assert len(dl[-1]) == 3 and [it.seq for it in dl[-1]] == [0, 1, 2]
assert mp.state == "idle" and not mp.busy

# 2) OUT-OF-ORDER download: vehicle sends seq 1 first -> must re-request 0 and recover ---
mp, link, fin, dl = new_proto()
mp.download(0)
mp.handle(Msg(mavlink.MISSION_COUNT, count=3))
mp.handle(item_msg(1))                        # wrong (expected 0)
assert reqs(link)[-1] == 0, "did not re-request the missing seq 0"
mp.handle(item_msg(0)); mp.handle(item_msg(1)); mp.handle(item_msg(2))
assert fin[-1] == (True, "downloaded 3 items"), fin
assert [it.seq for it in dl[-1]] == [0, 1, 2]

# 3) DUPLICATE item: seq 0 twice must not double-append or desync ----------------------
mp, link, fin, dl = new_proto()
mp.download(0)
mp.handle(Msg(mavlink.MISSION_COUNT, count=2))
mp.handle(item_msg(0)); mp.handle(item_msg(0))   # duplicate
assert mp.next_seq == 1, f"duplicate advanced next_seq wrongly: {mp.next_seq}"
mp.handle(item_msg(1))
assert [it.seq for it in dl[-1]] == [0, 1], "duplicate corrupted the list"
assert len(dl[-1]) == 2

# 4) empty mission: COUNT 0 -> ack + clean 'no mission' --------------------------------
mp, link, fin, dl = new_proto()
mp.download(0)
mp.handle(Msg(mavlink.MISSION_COUNT, count=0))
assert dl[-1] == [] and fin[-1][0] is True and mp.state == "idle"
assert any(a[0] == "ack" for a in link.sent)

# 5) upload + RETRANSMIT: re-requested seq must be re-sent, no corruption ---------------
items = [MissionItem(i, 47.0 + i * 1e-3, 8.0, 50.0, command=16) for i in range(3)]
mp, link, fin, dl = new_proto()
mp.upload(items, 0)
assert link.sent[0][0] == "count" and link.sent[0][1] == 3
mp.handle(Msg(mavlink.MISSION_REQUEST_INT, seq=0))
mp.handle(Msg(mavlink.MISSION_REQUEST_INT, seq=1))
mp.handle(Msg(mavlink.MISSION_REQUEST_INT, seq=1))   # retransmit request for 1
sent_items = [a[1] for a in link.sent if a[0] == "item"]
assert sent_items == [0, 1, 1], f"retransmit not honoured: {sent_items}"
mp.handle(Msg(mavlink.MISSION_REQUEST_INT, seq=2))
mp.handle(Msg(mavlink.MISSION_ACK, type=0))
assert fin[-1] == (True, "upload complete") and mp.state == "idle"

# 6) upload with OUT-OF-ORDER requests (autopilot picks the order) ---------------------
mp, link, fin, dl = new_proto()
mp.upload(items, 0)
for s in (2, 0, 1):
    mp.handle(Msg(mavlink.MISSION_REQUEST_INT, seq=s))
assert [a[1] for a in link.sent if a[0] == "item"] == [2, 0, 1]
# an out-of-RANGE request must be ignored, not crash
mp.handle(Msg(mavlink.MISSION_REQUEST_INT, seq=99))
mp.handle(Msg(mavlink.MISSION_REQUEST_INT, seq=-1))
mp.handle(Msg(mavlink.MISSION_ACK, type=0))
assert fin[-1][0] is True

# 7) ACK arriving MID-upload -> completes cleanly (trusts the autopilot) ---------------
mp, link, fin, dl = new_proto()
mp.upload(items, 0)
mp.handle(Msg(mavlink.MISSION_REQUEST_INT, seq=0))
mp.handle(Msg(mavlink.MISSION_ACK, type=0))      # early ack, before items 1/2 requested
assert fin[-1] == (True, "upload complete") and mp.state == "idle"

# 8) REJECTED upload: non-zero ACK -> clean failure with a reason ----------------------
mp, link, fin, dl = new_proto()
mp.upload(items, 0)
mp.handle(Msg(mavlink.MISSION_REQUEST_INT, seq=0))
mp.handle(Msg(mavlink.MISSION_ACK, type=1))      # MAV_MISSION_ERROR
assert fin[-1][0] is False and "reject" in fin[-1][1].lower() and mp.state == "idle"

# 9) unexpected messages in the wrong state must not crash / must be ignored -----------
mp, link, fin, dl = new_proto()
mp.download(0)                                    # state dl_count
mp.handle(item_msg(0))                            # ITEM before COUNT -> ignored
mp.handle(Msg(mavlink.MISSION_ACK, type=0))       # ACK in dl_count -> ignored
assert mp.state == "dl_count" and not fin
mp.handle(Msg(mavlink.MISSION_COUNT, count=1))
mp.handle(item_msg(0))
assert fin[-1][0] is True
# handle_messages while idle is a no-op
mp.handle_messages([item_msg(5), Msg(mavlink.MISSION_ACK, type=0)])
assert mp.state == "idle"

# 10) TIMEOUT + retry then clean failure ----------------------------------------------
mp, link, fin, dl = new_proto()
mp.download(0)
base = len(link.sent)
for _ in range(MAX_RETRIES):
    mp._on_timeout()
    assert mp.state == "dl_count", "gave up too early"
mp._on_timeout()                                  # one past MAX_RETRIES -> fail
assert fin[-1] == (False, "mission protocol timed out") and mp.state == "idle"
# it re-sent the request-list on each retry (not silently stuck)
assert sum(1 for a in link.sent[base:] if a[0] == "req_list") == MAX_RETRIES

# 11) link lost mid-timeout -> clean failure ------------------------------------------
mp, link, fin, dl = new_proto()
mp.upload(items, 0)
link.is_open = False
mp._on_timeout()
assert fin[-1] == (False, "link lost") and mp.state == "idle"

# 12) cross-transfer contamination (audit batch 7): a stale ACK / foreign vehicle item is dropped ---
# a fence upload must NOT be completed by a leftover mission (type 0) ACK from a prior transfer
mp, link, fin, dl = new_proto()
mp.upload(items, 1)                                   # mission_type 1 = FENCE
mp.handle(Msg(mavlink.MISSION_ACK, type=0, mission_type=0))   # stale ACK for the mission transfer
assert mp.state == "upload" and not fin, "stale mission-type ACK wrongly completed the fence upload"
mp.handle(Msg(mavlink.MISSION_ACK, type=0, mission_type=1))   # the correct fence ACK
assert fin[-1][0] is True and mp.state == "idle"
# a download must ignore a MISSION_ITEM_INT from a DIFFERENT vehicle (sysid 2 while target is 1)
mp, link, fin, dl = new_proto()
mp.download(0)
mp.handle(Msg(mavlink.MISSION_COUNT, count=2, mission_type=0))
foreign = item_msg(0); foreign.sysid = 2              # another vehicle on the shared link
mp.handle(foreign)
assert mp.next_seq == 0, "accepted a mission item from the wrong vehicle"
mp.handle(item_msg(0)); mp.handle(item_msg(1))        # correct vehicle (sysid 1)
assert dl[-1] and len(dl[-1]) == 2 and mp.state == "idle"

print("MISSIONROBUST PASSED (13 adversarial scenarios)")
