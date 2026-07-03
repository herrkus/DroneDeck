#!/usr/bin/env python3
"""test_wpprogress.py -- active-waypoint progress in the NAVIGATION group. When a mission is
loaded and the autopilot reports a current waypoint (MISSION_CURRENT -> ve.current_wp), the
panel shows 'Waypoint n / total', distance and ETA to that active waypoint; otherwise it
falls back to the nearest-waypoint distance. Isolated UDP port, drives _refresh directly."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import main as m
from mission import MissionItem

app = QApplication([])
win = m.DroneDeck(14599)
tp = win.panel

win.mission_items = [MissionItem(seq=0, lat=47.0, lon=8.0, alt=0, command=22),
                     MissionItem(seq=1, lat=47.01, lon=8.0, alt=50, command=16),
                     MissionItem(seq=2, lat=47.02, lon=8.0, alt=50, command=16)]
ve = win.vehicle
ve.have_position = True
ve.lat, ve.lon = 47.0, 8.0
ve.groundspeed = 10.0

# actively flying to waypoint index 1 -> "2 / 3", distance ~1.1 km, ETA ~1:51
ve.current_wp = 1
win._refresh()
assert tp.v["wp_num"].text() == "2 / 3", tp.v["wp_num"].text()
assert "m" in tp.v["wp_dist"].text() and ":" in tp.v["wp_eta"].text()

# stationary -> ETA unavailable
ve.groundspeed = 0.0
win._refresh()
assert tp.v["wp_eta"].text() == "--"

# no active waypoint -> number blank, distance falls back to the nearest waypoint
ve.current_wp = -1
ve.groundspeed = 10.0
win._refresh()
assert tp.v["wp_num"].text() == "--" and "m" in tp.v["wp_dist"].text()

# no mission at all -> everything blank
win.mission_items = []
win._refresh()
assert tp.v["wp_num"].text() == "--" and tp.v["wp_dist"].text() == "--" and tp.v["wp_eta"].text() == "--"

print("WPPROGRESS PASSED")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
