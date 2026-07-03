#!/usr/bin/env python3
"""test_multivehicle.py -- two vehicles (sysid 1 and 2) are tracked separately.

Runs two simulators with different system IDs into the same GCS UDP port and
checks both are tracked, both have positions, and the Vehicle selector switches
the active vehicle.
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
sims = [subprocess.Popen([sys.executable, SIM, "--target", f"127.0.0.1:{PORT}", "--sysid", str(s)],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) for s in (1, 2)]

snap = {}
def grab():
    snap["ids"] = sorted(win.vehicles.keys())
    snap["have_pos"] = {s: v.have_position for s, v in win.vehicles.items()}
    snap["combo"] = win.vehicle_combo.count()
    # switch to vehicle #2 via the selector
    idx = win.vehicle_combo.findData(2)
    if idx >= 0:
        win.vehicle_combo.setCurrentIndex(idx)
    snap["active_after_switch"] = win.vehicle.sysid
    app.quit()

QTimer.singleShot(3500, grab)
app.exec()

for sim in sims:
    sim.terminate()
for sim in sims:
    try:
        sim.wait(timeout=2)
    except Exception:
        sim.kill()

print("snapshot:", snap)

fail = []
if snap.get("ids") != [1, 2]:
    fail.append(f"tracked vehicles {snap.get('ids')} != [1, 2]")
if not all(snap.get("have_pos", {}).values()) or len(snap.get("have_pos", {})) < 2:
    fail.append(f"not all vehicles have positions: {snap.get('have_pos')}")
if snap.get("combo", 0) < 2:
    fail.append(f"selector has {snap.get('combo')} entries")
if snap.get("active_after_switch") != 2:
    fail.append(f"switching selected sysid {snap.get('active_after_switch')}, expected 2")

print("MULTIVEHICLE FAILED: " + "; ".join(fail) if fail else "MULTIVEHICLE PASSED")
sys.stdout.flush()
os._exit(1 if fail else 0)
