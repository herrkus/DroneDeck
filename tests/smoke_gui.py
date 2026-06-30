#!/usr/bin/env python3
"""smoke_gui.py -- headless end-to-end check.

Launches the real GUI under Qt's offscreen platform, starts the test telemetry
source as a subprocess, pumps the event loop, and verifies live telemetry
arrived. Saves a rendered screenshot for visual confirmation. Exits non-zero on
failure so it can gate CI / build scripts.
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

import main as appmain

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "build", "dronedeck.png")
os.makedirs(os.path.dirname(OUT), exist_ok=True)

app = QApplication([])
app.setStyleSheet(appmain.DARK_QSS)
win = appmain.DroneDeck(14550)
win.setFixedSize(1240, 770)   # offscreen screen would otherwise clamp the window
win.show()

sim = subprocess.Popen([sys.executable, SIM, "--target", "127.0.0.1:14550"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

state = {}


def finish():
    ve = win.vehicle
    ok, drop = (win.link.parser.stats if win.link.parser else (0, 0))
    state.update(msgs=ve.msg_count, have_pos=ve.have_position,
                 lat=ve.lat, lon=ve.lon, roll=ve.roll, hdg=ve.heading,
                 volt=ve.voltage, sats=ve.satellites, ok=ok, drop=drop,
                 trail=len(ve.trail))
    try:
        win.grab().save(OUT)
    except Exception as e:
        state["grab_error"] = repr(e)
    app.quit()


QTimer.singleShot(3500, finish)
app.exec()
sim.terminate()
try:
    sim.wait(timeout=2)
except Exception:
    sim.kill()

print("smoke results:")
for k, v in state.items():
    print(f"  {k:10}: {v}")

fail = []
if state.get("msgs", 0) < 50:
    fail.append("too few messages")
if not state.get("have_pos"):
    fail.append("no position")
if state.get("drop", 1) != 0:
    fail.append(f"{state.get('drop')} dropped frames")
if not os.path.exists(OUT):
    fail.append("no screenshot")

print(f"screenshot: {OUT}")
print("SMOKE FAILED: " + "; ".join(fail) if fail else "SMOKE PASSED")
sys.exit(1 if fail else 0)
