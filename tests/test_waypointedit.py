#!/usr/bin/env python3
"""test_waypointedit.py -- WaypointEditor per-command editing for the QGC-parity mission items added
across the loop: Loiter(turns) + Delay (iter116), Set-servo (iter119), Condition-Yaw (iter123) and
Camera-trigger-distance (iter124). Verifies the per-command field enable/relabel logic and that
apply_to writes the correct command / params / frame for each type. Builds the dialog but never
exec()s it (modal exec blocks offscreen). No link, no arming."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import main as m
import mavlink
from mission import MissionItem

app = QApplication.instance() or QApplication([])
fail = []


def editor_for(cmd):
    it = MissionItem(3, 47.0, 8.0, 50.0, command=cmd)
    return it, m.WaypointEditor(it)


def select(ed, cmd):
    idx = ed.cmd.findData(cmd)
    assert idx >= 0, f"command {cmd} missing from the palette"
    ed.cmd.setCurrentIndex(idx)                 # fires currentIndexChanged -> _sync_fields


# 1) Loiter (turns) 18: georeferenced; p1(turns)+p3(radius)+alt enabled; p1 relabelled ------------
it, ed = editor_for(16)
select(ed, 18)
if ed._p1_label.text() != "Loiter turns":
    fail.append(f"turns p1 label = {ed._p1_label.text()!r}")
if not (ed.p1.isEnabled() and ed.p3.isEnabled() and ed.alt.isEnabled()):
    fail.append("turns: alt/p1/p3 should be enabled")
ed.alt.setValue(80); ed.p1.setValue(3); ed.p3.setValue(40)
ed.apply_to(it)
if not (it.command == 18 and it.param1 == 3 and it.param3 == 40 and it.alt == 80):
    fail.append(f"turns apply: cmd={it.command} p1={it.param1} p3={it.param3} alt={it.alt}")
if it.frame == 2:
    fail.append("turns should be georeferenced (frame != 2)")

# 2) Delay 93: no position; p1(delay) enabled + relabelled; alt/p3 disabled; frame = mission (2) ---
it, ed = editor_for(16)
select(ed, 93)
if ed._p1_label.text() != "Delay (s)":
    fail.append(f"delay p1 label = {ed._p1_label.text()!r}")
if not ed.p1.isEnabled():
    fail.append("delay: p1 should be enabled")
if ed.alt.isEnabled() or ed.p3.isEnabled():
    fail.append("delay: alt/p3 should be disabled (no position)")
ed.p1.setValue(12)
ed.apply_to(it)
if not (it.command == 93 and it.param1 == 12 and it.frame == 2 and it.alt == 0.0):
    fail.append(f"delay apply: cmd={it.command} p1={it.param1} frame={it.frame} alt={it.alt}")

# 3) friendly names in the mission list (not CMD18 / CMD93) ----------------------------------------
if MissionItem(0, 47, 8, 50, command=18).cmd_name != "LOITER_TURNS":
    fail.append("cmd 18 name")
if MissionItem(0, 47, 8, 50, command=93).cmd_name != "DELAY":
    fail.append("cmd 93 name")

# 4) Set servo 183 (iter119): no position; servo channel + PWM enabled; params -> p1/p2, frame 2 ----
it, ed = editor_for(16)
select(ed, 183)
if not (ed.servo_ch.isEnabled() and ed.servo_pwm.isEnabled()):
    fail.append("set-servo: servo channel/PWM should be enabled")
if ed.alt.isEnabled() or ed.p1.isEnabled() or ed.p3.isEnabled():
    fail.append("set-servo: position/loiter fields should be disabled")
ed.servo_ch.setValue(9); ed.servo_pwm.setValue(1800)
ed.apply_to(it)
if not (it.command == 183 and it.param1 == 9 and it.param2 == 1800 and it.frame == 2 and it.alt == 0.0):
    fail.append(f"set-servo apply: cmd={it.command} p1={it.param1} p2={it.param2} "
                f"frame={it.frame} alt={it.alt}")
if MissionItem(0, 47, 8, 50, command=183).cmd_name != "SET_SERVO":
    fail.append("cmd 183 name")

# 5) Condition: Yaw 115 (iter123): no position; Yaw field reused as heading -> param1, frame 2 ------
it, ed = editor_for(16)
select(ed, 115)
if not ed.p4.isEnabled():
    fail.append("condition-yaw: the Yaw field should stay enabled as the heading input")
if ed.alt.isEnabled() or ed.p1.isEnabled() or ed.p3.isEnabled():
    fail.append("condition-yaw: position/loiter fields should be disabled")
ed.p4.setValue(270)
ed.apply_to(it)
if not (it.command == 115 and it.param1 == 270 and it.param2 == 0 and it.frame == 2 and it.alt == 0.0):
    fail.append(f"condition-yaw apply: cmd={it.command} p1={it.param1} p2={it.param2} "
                f"frame={it.frame} alt={it.alt}")
if MissionItem(0, 47, 8, 50, command=115).cmd_name != "YAW":
    fail.append("cmd 115 name")

# 6) Camera trig dist 206 (iter124): no position; p1 reused as trigger distance -> param1, frame 2 --
it, ed = editor_for(16)
select(ed, 206)
if ed._p1_label.text() != "Trigger dist (m)":
    fail.append(f"cam-trigg p1 label = {ed._p1_label.text()!r}")
if not ed.p1.isEnabled():
    fail.append("cam-trigg: the trigger-distance field should be enabled")
if ed.alt.isEnabled() or ed.p3.isEnabled() or ed.p4.isEnabled():
    fail.append("cam-trigg: position/loiter fields should be disabled")
ed.p1.setValue(25)
ed.apply_to(it)
if not (it.command == 206 and it.param1 == 25 and it.param2 == 0 and it.frame == 2 and it.alt == 0.0):
    fail.append(f"cam-trigg apply: cmd={it.command} p1={it.param1} p2={it.param2} "
                f"frame={it.frame} alt={it.alt}")
if MissionItem(0, 47, 8, 50, command=206).cmd_name != "CAM_TRIGG_DIST":
    fail.append("cmd 206 name")

# 6b) Loiter to alt 31 (iter129, PX4-verified): georeferenced; alt + radius(p3->param2); no p1/p4 -----
it, ed = editor_for(16)
select(ed, 31)
if not (ed.alt.isEnabled() and ed.p3.isEnabled()):
    fail.append("loiter-to-alt: alt + radius should be enabled")
if ed.p1.isEnabled() or ed.p4.isEnabled():
    fail.append("loiter-to-alt: loiter-time (p1) and yaw (p4) should be disabled")
ed.alt.setValue(120); ed.p3.setValue(90)
ed.apply_to(it)
if not (it.command == 31 and it.alt == 120 and it.param2 == 90 and it.param1 == 0 and it.frame != 2):
    fail.append(f"loiter-to-alt apply: cmd={it.command} alt={it.alt} p2={it.param2} "
                f"p1={it.param1} frame={it.frame}")
if MissionItem(0, 47, 8, 50, command=31).cmd_name != "LOITER_TO_ALT":
    fail.append("cmd 31 name")

# 7) no regression: a plain waypoint stays georeferenced with the chosen altitude ------------------
it, ed = editor_for(16)
select(ed, 16)
ed.alt.setValue(60)
ed.apply_to(it)
if not (it.command == 16 and it.alt == 60 and it.frame != 2):
    fail.append(f"waypoint regressed: cmd={it.command} alt={it.alt} frame={it.frame}")

# 8) autopilot-aware palette (iter128): PX4 hides the items it rejects, ArduPilot shows all ---------
def combo_cmds(ed):
    return [ed.cmd.itemData(i) for i in range(ed.cmd.count())]


plain = MissionItem(0, 47, 8, 50, command=16)
cmds_px4 = combo_cmds(m.WaypointEditor(plain, autopilot=mavlink.MAV_AUTOPILOT_PX4))
for bad in (20, 18, 82, 183, 115):                  # RTL, loiter-turns, spline, set-servo, cond-yaw
    if bad in cmds_px4:
        fail.append(f"PX4 palette should hide unsupported cmd {bad}")
for good in (16, 22, 21, 19, 93, 206, 195, 177):    # standard + live-verified PX4-accepted
    if good not in cmds_px4:
        fail.append(f"PX4 palette dropped supported cmd {good}")
cmds_ardu = combo_cmds(m.WaypointEditor(plain, autopilot=mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA))
for c in (20, 18, 82, 183, 115):                    # ArduPilot supports them -> all offered
    if c not in cmds_ardu:
        fail.append(f"ArduPilot palette should offer cmd {c}")
# editing an EXISTING spline item on a PX4 link must still show spline (so it stays editable)
if 82 not in combo_cmds(m.WaypointEditor(MissionItem(0, 47, 8, 50, command=82),
                                         autopilot=mavlink.MAV_AUTOPILOT_PX4)):
    fail.append("PX4: editing an existing spline item must keep spline in the palette")

print("WAYPOINTEDIT FAILED: " + "; ".join(fail) if fail else
      "WAYPOINTEDIT PASSED (Loiter-turns + Loiter-to-alt + Delay + Set-servo + Condition-Yaw + Cam-trigg: fields/labels/params/"
      "frame correct, no regression)")
sys.exit(1 if fail else 0)
