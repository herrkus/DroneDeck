#!/usr/bin/env python3
"""test_adsb.py -- ADSB_VEHICLE traffic reaches the GCS and is tracked.

Runs the simulator (which broadcasts two ADSB targets) and checks the real
window collects them with their callsigns and plausible positions.
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
import main as appmain

app = QApplication([])
win = appmain.DroneDeck(14550)
sim = subprocess.Popen([sys.executable, SIM, "--target", "127.0.0.1:14550"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

snap = {}
def grab():
    snap["traffic"] = {k: dict(v) for k, v in win.traffic.items()}
    snap["map_traffic"] = list(win.map.traffic)
    app.quit()

QTimer.singleShot(3500, grab)
app.exec()

sim.terminate()
try:
    sim.wait(timeout=2)
except Exception:
    sim.kill()

traffic = snap.get("traffic", {})
callsigns = sorted(v["callsign"] for v in traffic.values())
print("traffic:", {hex(k): (v["callsign"], round(v["lat"], 4), round(v["lon"], 4))
                   for k, v in traffic.items()})

fail = []
if len(traffic) < 2:
    fail.append(f"only {len(traffic)} ADSB targets tracked")
if callsigns != ["DRN001", "HEL022"]:
    fail.append(f"callsigns {callsigns} != ['DRN001', 'HEL022']")
for v in traffic.values():
    if not (54.0 < v["lat"] < 55.0 and 24.0 < v["lon"] < 26.0):
        fail.append(f"target position implausible: {v['lat']},{v['lon']}")
        break
if len(snap.get("map_traffic", [])) < 2:
    fail.append("map did not receive the traffic")

print("ADSB FAILED: " + "; ".join(fail) if fail else "ADSB PASSED")
sys.exit(1 if fail else 0)
