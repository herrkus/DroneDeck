#!/usr/bin/env python3
"""test_changeheading.py -- Change Heading guided action (iter120), QGC-parity.

Points the vehicle's nose to a compass bearing while holding position + altitude, via DO_REPOSITION
with the yaw in param4 (the same primitive change_altitude uses with yaw=NaN). Verifies link.change_
heading builds the right COMMAND_INT (yaw wrapped mod 360, CHANGE_MODE flag, relative-alt frame) and
that the GUI action guards a missing vehicle and passes the current position/altitude + chosen bearing."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication, QInputDialog
import mavlink
from link import UdpLink

app = QApplication.instance() or QApplication([])
fail = []

# 1) link.change_heading -> DO_REPOSITION, yaw in param4, pos in x/y, relative-alt frame -----------
sent = []
lk = UdpLink()
lk.send_command_int = lambda tgt, cmd, params, x, y, z, frame=0: sent.append(
    (tgt, cmd, list(params), x, y, z, frame))
lk.change_heading(2, 47.5, 8.5, 30.0, 270.0)
tgt, cmd, p, x, y, z, frame = sent[-1]
if cmd != mavlink.MAV_CMD_DO_REPOSITION or mavlink.MAV_CMD_DO_REPOSITION != 192:
    fail.append(f"cmd={cmd} (expect DO_REPOSITION 192)")
if p[3] != 270.0:
    fail.append(f"yaw param4={p[3]} (expect 270)")
if p[1] != 1.0:
    fail.append("param2 should be 1 (CHANGE_MODE, so PX4 enters guided reposition)")
if x != int(47.5 * 1e7) or y != int(8.5 * 1e7) or z != 30.0:
    fail.append(f"position wrong: x={x} y={y} z={z}")
if frame != 6:
    fail.append(f"frame={frame} (expect 6, relative-alt)")
lk.change_heading(2, 47.5, 8.5, 30.0, 450.0)          # heading must wrap mod 360
if sent[-1][2][3] != 90.0:
    fail.append(f"heading not wrapped mod 360: {sent[-1][2][3]}")

# 2) GUI action guards no-vehicle, else sends current pos/alt + the chosen bearing -----------------
import main as m
QInputDialog.getInt = staticmethod(lambda *a, **k: (135, True))
win = m.DroneDeck(14710)
win._persist = False
calls = []
win.link.change_heading = lambda *a: calls.append(a)
win._has_vehicle = lambda: False
win._change_heading()
if calls:
    fail.append("sent change_heading with no vehicle connected")
win._has_vehicle = lambda: True
win._sysid = lambda: 4
win.vehicle.lat, win.vehicle.lon = 51.0, -1.0
win.vehicle.alt_rel, win.vehicle.heading = 42.0, 10.0
win._change_heading()
if calls != [(4, 51.0, -1.0, 42.0, 135.0)]:
    fail.append(f"wrong change_heading args: {calls}")

print("CHANGEHEADING FAILED: " + "; ".join(fail) if fail else
      "CHANGEHEADING PASSED (DO_REPOSITION yaw=param4, mod-360 wrap, CHANGE_MODE, frame 6; GUI guards "
      "+ passes current pos/alt + bearing)", flush=True)
os._exit(1 if fail else 0)
