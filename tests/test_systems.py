#!/usr/bin/env python3
"""test_systems.py -- detailed battery / vibration / altitude telemetry.

Runs the simulator (which now emits BATTERY_STATUS, VIBRATION and ALTITUDE) and
checks the vehicle captures per-cell voltages, consumed mAh, temperature,
vibration and terrain clearance, and that the Systems panel reflects them.
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
    ve = win.vehicle
    snap["cells"] = list(ve.cells)
    snap["voltage"] = ve.voltage
    snap["consumed"] = ve.battery_consumed
    snap["temp"] = ve.battery_temp
    snap["vibration"] = ve.vibration
    snap["alt_terrain"] = ve.alt_terrain
    snap["radio_rssi"] = ve.radio_rssi
    snap["panel_volt"] = win.systems.b_volt.text()
    snap["panel_cells"] = win.systems.b_cells.text()
    app.quit()

QTimer.singleShot(3200, grab)
app.exec()

sim.terminate()
try:
    sim.wait(timeout=2)
except Exception:
    sim.kill()

print("snapshot:", {k: v for k, v in snap.items()})

fail = []
if len(snap.get("cells", [])) < 1:
    fail.append("no battery cells captured")
if not (9.0 < snap.get("voltage", 0) < 13.5):
    fail.append(f"pack voltage implausible ({snap.get('voltage')})")
if snap.get("consumed", -1) < 0:
    fail.append("battery consumed mAh not captured")
if snap.get("temp") is None:
    fail.append("battery temperature not captured")
if all(v == 0 for v in snap.get("vibration", (0, 0, 0))):
    fail.append("vibration not captured")
if snap.get("alt_terrain") is None:
    fail.append("terrain clearance not captured")
if snap.get("radio_rssi") is None:
    fail.append("radio link RSSI not captured")
if "V" not in snap.get("panel_volt", ""):
    fail.append("Systems panel voltage not updated")

print("SYSTEMS FAILED: " + "; ".join(fail) if fail else "SYSTEMS PASSED")
sys.exit(1 if fail else 0)
