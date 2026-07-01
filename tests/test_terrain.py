#!/usr/bin/env python3
"""test_terrain.py -- terrain-relative (AGL) waypoint altitude mode. The WaypointEditor
altmode combo offers Terrain (MAV_FRAME_GLOBAL_TERRAIN_ALT_INT=11); apply_to writes that
frame, the editor preselects it for a frame-11 item, and it survives a .plan round-trip.
Port-independent (WaypointEditor + planfile, no link)."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import main as m
import mavlink
from mission import MissionItem
from planfile import mission_to_plan, plan_to_mission, save_plan, load_plan

app = QApplication([])
TERR = mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
assert TERR == 11

# Selecting Terrain writes frame 11 through apply_to.
it = MissionItem(seq=1, lat=47.1, lon=8.2, alt=40.0, command=16)
ed = m.WaypointEditor(it)
idx = ed.altmode.findData(TERR)
assert idx >= 0, "Terrain option missing from altmode combo"
ed.altmode.setCurrentIndex(idx)
ed.apply_to(it)
assert it.frame == 11

# Opening a frame-11 item preselects Terrain (not silently reset to Relative).
ed2 = m.WaypointEditor(MissionItem(seq=2, lat=47.1, lon=8.2, alt=40.0, command=16, frame=11))
assert ed2.altmode.currentData() == 11

# The three modes stay distinct through the combo.
for frame in (mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mavlink.MAV_FRAME_GLOBAL_INT, TERR):
    e = m.WaypointEditor(MissionItem(seq=3, lat=47.1, lon=8.2, alt=10.0, command=16, frame=frame))
    assert e.altmode.currentData() == frame

# .plan round-trip (in-memory + on-disk) preserves the terrain frame.
mission = [MissionItem(seq=0, lat=47.1, lon=8.2, alt=20, command=22),
           MissionItem(seq=1, lat=47.11, lon=8.21, alt=50, command=16, frame=11),
           MissionItem(seq=2, lat=47.12, lon=8.22, alt=0, command=21)]
back = plan_to_mission(mission_to_plan(mission))
assert len([x for x in back if x.frame == 11]) == 1, [x.frame for x in back]

path = os.path.join(os.path.dirname(__file__), "_terrain_tmp.plan")
try:
    save_plan(path, mission)
    loaded = load_plan(path)
    items = loaded[0] if isinstance(loaded, tuple) else loaded
    assert any(x.frame == 11 for x in items), [x.frame for x in items]
finally:
    if os.path.exists(path):
        os.remove(path)

print("TERRAIN PASSED")
