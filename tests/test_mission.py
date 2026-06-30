#!/usr/bin/env python3
"""test_mission.py -- mission protocol end to end.

Uploads a 4-waypoint mission to the test vehicle, downloads it back, and checks
the round-trip matches. Exercises MissionProtocol (GCS side) against the
simulator's vehicle-side handler, both decoding through the native core.
"""
import os
import sys
import subprocess

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APP = os.path.join(ROOT, "app")
SIM = os.path.join(ROOT, "sim", "simulator.py")
sys.path.insert(0, APP)

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer

from link import UdpLink
from mission import MissionProtocol, MissionItem

app = QApplication([])
link = UdpLink()
link.open(port=14550)

sim = subprocess.Popen([sys.executable, SIM, "--target", "127.0.0.1:14550"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

WPS = [
    MissionItem(0, 54.6872, 25.2797, 0.0, command=16),
    MissionItem(1, 54.6900, 25.2800, 50.0),
    MissionItem(2, 54.6920, 25.2850, 60.0),
    MissionItem(3, 54.6890, 25.2900, 55.0),
]

mp = MissionProtocol(lambda: link, lambda: 1)
link.messages.connect(mp.handle_messages)

steps = []
dl = {}
mp.finished.connect(lambda ok, msg: steps.append((ok, msg)))
mp.downloaded.connect(lambda items: dl.__setitem__("items", items))

QTimer.singleShot(1000, lambda: mp.upload(WPS) if link.remote is not None else None)
QTimer.singleShot(2500, lambda: mp.download())
QTimer.singleShot(4500, app.quit)
app.exec()

sim.terminate()
try:
    sim.wait(timeout=2)
except Exception:
    sim.kill()
link.close()

items = dl.get("items", [])
print("protocol steps:")
for ok, msg in steps:
    print(f"  {'ok ' if ok else 'ERR'} {msg}")
print(f"uploaded {len(WPS)} waypoints, downloaded {len(items)}")

fail = []
if len(steps) < 2 or not all(ok for ok, _ in steps):
    fail.append("a protocol step failed")
if len(items) != len(WPS):
    fail.append(f"count mismatch: {len(items)} != {len(WPS)}")
else:
    for i, (a, b) in enumerate(zip(WPS, items)):
        if abs(a.lat - b.lat) > 1e-5 or abs(a.lon - b.lon) > 1e-5 or abs(a.alt - b.alt) > 0.5:
            fail.append(f"wp{i} mismatch: ({b.lat:.5f},{b.lon:.5f},{b.alt:.1f}) "
                        f"vs ({a.lat:.5f},{a.lon:.5f},{a.alt:.1f})")

print("MISSION FAILED: " + "; ".join(fail) if fail else "MISSION PASSED")
sys.exit(1 if fail else 0)
