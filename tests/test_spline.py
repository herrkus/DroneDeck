#!/usr/bin/env python3
"""test_spline.py -- SPLINE_WP (82) in the WaypointEditor. ArduPilot-only smooth waypoint;
PX4 rejects command 82, so this is proven headless + via .plan round-trip only (no link)."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import main as m
import mavlink
from mission import MissionItem, validate_mission
from planfile import mission_to_plan, plan_to_mission, save_plan, load_plan

app = QApplication([])

# The editor offers Spline waypoint and treats it as a georeferenced position item.
it = MissionItem(seq=1, lat=47.1, lon=8.2, alt=30.0, command=16)
ed = m.WaypointEditor(it)
idx = next(i for i in range(ed.cmd.count()) if ed.cmd.itemData(i) == 82)
ed.cmd.setCurrentIndex(idx)
ed._sync_fields()
assert ed.alt.isEnabled() and ed.altmode.isEnabled() and ed.p4.isEnabled()
assert not (ed.p1.isEnabled() or ed.p3.isEnabled() or ed.spd.isEnabled() or ed.jump_to.isEnabled())

# Apply with an AMSL altitude + yaw -> command 82, frame 5, values written through.
ed.alt.setValue(55.0)
ed.p4.setValue(90.0)
amsl = next(i for i in range(ed.altmode.count()) if ed.altmode.itemData(i) == mavlink.MAV_FRAME_GLOBAL_INT)
ed.altmode.setCurrentIndex(amsl)
ed.apply_to(it)
assert it.command == 82 and abs(it.alt - 55.0) < 1e-6 and abs(it.param4 - 90.0) < 1e-6
assert it.frame == mavlink.MAV_FRAME_GLOBAL_INT
assert it.cmd_name == "SPLINE_WP"

# A well-formed mission containing a spline waypoint draws no spline-specific complaint.
mission = [MissionItem(seq=0, lat=47.1, lon=8.2, alt=20, command=22),      # takeoff
           MissionItem(seq=1, lat=47.11, lon=8.21, alt=50, command=82),    # spline
           MissionItem(seq=2, lat=47.12, lon=8.22, alt=0, command=21)]     # land
assert not any("SPLINE" in w.upper() for w in validate_mission(mission))

# .plan round-trip (in-memory + on-disk) preserves command 82 + frame + alt.
back = plan_to_mission(mission_to_plan(mission))
sp = [x for x in back if x.command == 82]
assert len(sp) == 1 and sp[0].frame == mission[1].frame

path = os.path.join(os.path.dirname(__file__), "_spline_tmp.plan")
try:
    save_plan(path, mission)
    loaded = load_plan(path)
    items = loaded[0] if isinstance(loaded, tuple) else loaded
    sp2 = [x for x in items if x.command == 82]
    assert len(sp2) == 1 and abs(sp2[0].alt - 50) < 1e-6
finally:
    if os.path.exists(path):
        os.remove(path)

print("SPLINE PASSED")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
