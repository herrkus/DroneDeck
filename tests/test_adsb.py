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
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # for _ports
from _ports import free_udp_port
PORT = free_udp_port()
SIM = os.path.join(ROOT, "sim", "simulator.py")

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer
import main as appmain

app = QApplication([])
win = appmain.DroneDeck(PORT)
sim = subprocess.Popen([sys.executable, SIM, "--target", f"127.0.0.1:{PORT}"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

snap = {}
def grab():
    snap["traffic"] = {k: dict(v) for k, v in win.traffic.items()}
    snap["map_traffic"] = list(win.map.traffic)
    tp = win.traffic_panel.table
    snap["panel_rows"] = tp.rowCount()
    snap["panel_callsigns"] = sorted(tp.item(r, 0).text() for r in range(tp.rowCount()))
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
if snap.get("panel_rows", 0) < 2:
    fail.append(f"traffic panel listed {snap.get('panel_rows')} rows, expected >= 2")
if snap.get("panel_callsigns") != ["DRN001", "HEL022"]:
    fail.append(f"panel callsigns {snap.get('panel_callsigns')} != ['DRN001', 'HEL022']")

print("ADSB FAILED: " + "; ".join(fail) if fail else "ADSB PASSED")
sys.stdout.flush()
os._exit(1 if fail else 0)
