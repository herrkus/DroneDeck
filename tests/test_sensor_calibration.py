#!/usr/bin/env python3
"""test_sensor_calibration.py -- sensor calibration workflow round-trip.

Requests an accelerometer calibration and checks the vehicle's guided prompts
(STATUSTEXT) arrive and are shown in the calibration widget, ending with the
success line. This exercises the GCS side of the workflow -- the command out,
the prompts back -- which is exactly the part that is testable without hardware.
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
from calibration import CalibrationDialog

app = QApplication([])
win = appmain.DroneDeck(PORT)
dlg = CalibrationDialog(lambda: win.link, parent=win)
dlg.sensor.calRequested.connect(lambda kind: win.link.calibrate(win._sysid(), kind))
sim = subprocess.Popen([sys.executable, SIM, "--target", f"127.0.0.1:{PORT}"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

snap = {}
def request():
    dlg.sensor._request("accel")

def grab():
    snap["lines"] = [dlg.sensor.log.item(i).text() for i in range(dlg.sensor.log.count())]
    app.quit()

QTimer.singleShot(1200, request)      # wait until the vehicle is connected
QTimer.singleShot(2600, grab)
app.exec()

sim.terminate()
try:
    sim.wait(timeout=2)
except Exception:
    sim.kill()

lines = snap.get("lines", [])
blob = "\n".join(lines)
print("prompts received:\n  " + "\n  ".join(lines))

fail = []
if not any("Place vehicle level" in ln for ln in lines):
    fail.append("missing 'Place vehicle level' prompt")
if not any("LEFT side" in ln for ln in lines):
    fail.append("missing orientation prompts")
if not any("Calibration successful" in ln for ln in lines):
    fail.append("missing success line")

print("SENSOR CAL FAILED: " + "; ".join(fail) if fail else "SENSOR CAL PASSED")
sys.exit(1 if fail else 0)
