#!/usr/bin/env python3
"""test_fence_shapes.py -- mixed geofence (inclusion/exclusion polygons + circles).

Uploads one FENCE containing an inclusion polygon, an exclusion polygon, an
inclusion circle and an exclusion circle, downloads it, and checks every shape's
command id and the circle radii (param1) survive the round-trip.
"""
import os
import sys
import subprocess

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
SIM = os.path.join(ROOT, "sim", "simulator.py")

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer

import mavlink
from link import UdpLink
from mission import MissionProtocol, MissionItem

app = QApplication([])
link = UdpLink()
link.open(port=14550)
sim = subprocess.Popen([sys.executable, SIM, "--target", "127.0.0.1:14550"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

INC = mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
EXC = mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
CIN = mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
CEX = mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION

items = []
seq = 0
for la, lo in [(54.690, 25.270), (54.694, 25.270), (54.692, 25.280)]:
    items.append(MissionItem(seq, la, lo, 0, command=INC, param1=3.0)); seq += 1
for la, lo in [(54.691, 25.272), (54.693, 25.272), (54.692, 25.276)]:
    items.append(MissionItem(seq, la, lo, 0, command=EXC, param1=3.0)); seq += 1
items.append(MissionItem(seq, 54.6950, 25.2850, 0, command=CIN, param1=80.0)); seq += 1
items.append(MissionItem(seq, 54.6960, 25.2820, 0, command=CEX, param1=40.0)); seq += 1

mp = MissionProtocol(lambda: link, lambda: 1)
link.messages.connect(mp.handle_messages)
got = {}
mp.downloaded.connect(lambda its: got.__setitem__("items", list(its)))

QTimer.singleShot(1000, lambda: mp.upload(items, mavlink.MAV_MISSION_TYPE_FENCE))
QTimer.singleShot(2400, lambda: mp.download(mavlink.MAV_MISSION_TYPE_FENCE))
QTimer.singleShot(3800, app.quit)
app.exec()

sim.terminate()
try:
    sim.wait(timeout=2)
except Exception:
    sim.kill()
link.close()

back = got.get("items", [])
by_cmd = {}
for it in back:
    by_cmd.setdefault(it.command, []).append(it)
print("downloaded by command:", {c: len(v) for c, v in by_cmd.items()})

fail = []
if len(back) != len(items):
    fail.append(f"got {len(back)} items, expected {len(items)}")
if len(by_cmd.get(INC, [])) != 3:
    fail.append("inclusion polygon vertices missing")
if len(by_cmd.get(EXC, [])) != 3:
    fail.append("exclusion polygon vertices missing")
cin = by_cmd.get(CIN, [])
cex = by_cmd.get(CEX, [])
if len(cin) != 1 or abs(cin[0].param1 - 80.0) > 0.5:
    fail.append(f"inclusion circle wrong: {[(c.param1) for c in cin]}")
if len(cex) != 1 or abs(cex[0].param1 - 40.0) > 0.5:
    fail.append(f"exclusion circle wrong: {[(c.param1) for c in cex]}")

print("FENCE SHAPES FAILED: " + "; ".join(fail) if fail else "FENCE SHAPES PASSED")
sys.exit(1 if fail else 0)
