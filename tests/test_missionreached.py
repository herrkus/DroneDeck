#!/usr/bin/env python3
"""test_missionreached.py -- mission progress feedback. MISSION_ITEM_REACHED was parsed but had no
handler; now the vehicle emits mission_reached and DroneDeck logs 'reached waypoint N' (QGC-style).
Also confirms the wp-progress plumbing: MISSION_CURRENT -> vehicle.current_wp -> map target ring.
No link."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import main as m
import mavlink


class Msg:
    __slots__ = ("msgid", "sysid", "compid", "seq", "fields")
    def __init__(self, msgid, fields, sysid=1, compid=1, seq=0):
        self.msgid = msgid
        self.sysid = sysid
        self.compid = compid
        self.seq = seq
        self.fields = fields


app = QApplication([])
win = m.DroneDeck(14599)
ve = win.vehicle

# MISSION_ITEM_REACHED -> reached_wp set, signal fires, and a console note is logged
fired = []
ve.mission_reached.connect(fired.append)
ve.consume([Msg(mavlink.MISSION_ITEM_REACHED, {"seq": 2})])
assert ve.reached_wp == 2 and fired == [2], (ve.reached_wp, fired)
notes = [win.console.item(i).text() for i in range(win.console.count())]
assert any("reached waypoint 2" in t for t in notes), notes

# MISSION_CURRENT -> current_wp, and _refresh pushes it to the map's live target ring
ve.consume([Msg(mavlink.MISSION_CURRENT, {"seq": 3})])
assert ve.current_wp == 3
ve.last_heartbeat = __import__("time").monotonic()      # so _refresh treats the link as alive
win._refresh()
assert win.map.current_wp == 3, win.map.current_wp

# a later waypoint reached logs again
ve.consume([Msg(mavlink.MISSION_ITEM_REACHED, {"seq": 3})])
assert ve.reached_wp == 3 and fired == [2, 3]

print("MISSIONREACHED PASSED")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
