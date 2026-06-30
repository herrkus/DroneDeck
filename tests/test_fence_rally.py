#!/usr/bin/env python3
"""test_fence_rally.py -- geofence + rally upload/download via mission_type.

Uploads a 4-vertex inclusion fence (MAV_MISSION_TYPE_FENCE) and 2 rally points
(MAV_MISSION_TYPE_RALLY) to the simulator, downloads each back, and checks the
round-trip (including command id and the fence vertex-count in param1).
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

FENCE = [MissionItem(i, la, lo, 0.0, command=mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                     param1=4.0)
         for i, (la, lo) in enumerate([(54.69, 25.27), (54.70, 25.27), (54.70, 25.29), (54.69, 25.29)])]
RALLY = [MissionItem(i, la, lo, 60.0, command=mavlink.MAV_CMD_NAV_RALLY_POINT)
         for i, (la, lo) in enumerate([(54.695, 25.280), (54.698, 25.285)])]

mp = MissionProtocol(lambda: link, lambda: 1)
link.messages.connect(mp.handle_messages)
dl = {}
steps = []
mp.downloaded.connect(lambda items: dl.__setitem__(mp.mtype, list(items)))
mp.finished.connect(lambda ok, msg: steps.append((mp.mtype, ok, msg)))

QTimer.singleShot(1000, lambda: mp.upload(FENCE, mavlink.MAV_MISSION_TYPE_FENCE))
QTimer.singleShot(2200, lambda: mp.download(mavlink.MAV_MISSION_TYPE_FENCE))
QTimer.singleShot(3400, lambda: mp.upload(RALLY, mavlink.MAV_MISSION_TYPE_RALLY))
QTimer.singleShot(4600, lambda: mp.download(mavlink.MAV_MISSION_TYPE_RALLY))
QTimer.singleShot(5800, app.quit)
app.exec()

sim.terminate()
try:
    sim.wait(timeout=2)
except Exception:
    sim.kill()
link.close()

fence = dl.get(mavlink.MAV_MISSION_TYPE_FENCE, [])
rally = dl.get(mavlink.MAV_MISSION_TYPE_RALLY, [])
print("steps:", [(t, ok, msg) for t, ok, msg in steps])
print(f"fence back: {len(fence)} verts (cmd {fence[0].command if fence else '-'}, "
      f"p1 {fence[0].param1 if fence else '-'})")
print(f"rally back: {len(rally)} points (cmd {rally[0].command if rally else '-'})")

fail = []
if len(fence) != len(FENCE):
    fail.append(f"fence count {len(fence)} != {len(FENCE)}")
elif fence[0].command != 5001 or abs(fence[0].param1 - 4.0) > 0.01:
    fail.append(f"fence item wrong (cmd {fence[0].command}, p1 {fence[0].param1})")
elif abs(fence[2].lat - FENCE[2].lat) > 1e-5 or abs(fence[2].lon - FENCE[2].lon) > 1e-5:
    fail.append("fence vertex position mismatch")
if len(rally) != len(RALLY):
    fail.append(f"rally count {len(rally)} != {len(RALLY)}")
elif rally[0].command != 5100 or abs(rally[0].alt - 60.0) > 0.5:
    fail.append(f"rally item wrong (cmd {rally[0].command}, alt {rally[0].alt})")
if len([s for s in steps if s[1]]) < 4:
    fail.append("not all 4 transfers succeeded")

print("FENCE/RALLY FAILED: " + "; ".join(fail) if fail else "FENCE/RALLY PASSED")
sys.exit(1 if fail else 0)
