#!/usr/bin/env python3
"""test_missionack.py -- human-readable mission rejection. A MISSION_ACK carrying a non-zero
MAV_MISSION_RESULT is reported with a plain-language reason instead of a bare code, so a user whose
mission is rejected (e.g. PX4 rejecting a Return-To-Launch item with code 3) sees why. No link."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mavlink
from mission import MissionProtocol

app = QApplication([])

# the result -> text map
assert mavlink.mission_result_text(0) == "accepted"
assert mavlink.mission_result_text(3) == "a command is not supported by the vehicle"
assert mavlink.mission_result_text(4) == "not enough storage space"
assert mavlink.mission_result_text(13) == "item received out of sequence"
assert mavlink.mission_result_text(99) == "unknown reason"


class FakeLink:
    is_open = True
    remote = ("127.0.0.1", 1)
    def send_mission_count(self, *a, **k): pass
    def send_mission_item(self, *a, **k): pass
    def send_mission_clear(self, *a, **k): pass


class Msg:
    def __init__(self, mid, fields):
        self.msgid = mid
        self.fields = fields


# an upload rejected with code 3 -> readable reason + the code
mp = MissionProtocol(lambda: FakeLink(), lambda: 1)
got = []
mp.finished.connect(lambda ok, msg: got.append((ok, msg)))
mp.state = "upload"
mp.handle(Msg(mavlink.MISSION_ACK, {"type": 3, "mission_type": 0}))
assert got and got[-1][0] is False, got
assert "not supported" in got[-1][1] and "code 3" in got[-1][1], got[-1]

# an accepted upload still reads cleanly
got.clear()
mp.state = "upload"
mp.handle(Msg(mavlink.MISSION_ACK, {"type": 0, "mission_type": 0}))
assert got and got[-1][0] is True and "complete" in got[-1][1], got[-1]

# a rejected clear is also readable
got.clear()
mp.state = "clear"
mp.handle(Msg(mavlink.MISSION_ACK, {"type": 14, "mission_type": 0}))
assert got and got[-1][0] is False and "not accepting missions" in got[-1][1], got[-1]

print("MISSIONACK PASSED")
