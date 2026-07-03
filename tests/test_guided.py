#!/usr/bin/env python3
"""test_guided.py -- guided actions (Orbit / ROI / Set Home) over COMMAND_INT.

Drives the map context-menu handlers, which send COMMAND_INT to the simulator,
and checks the vehicle acks each command and relays the expected feedback.
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
import mavlink

app = QApplication([])
win = appmain.DroneDeck(PORT)
sim = subprocess.Popen([sys.executable, SIM, "--target", f"127.0.0.1:{PORT}"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

snap = {}
def send_actions():
    win._on_map_context("sethome", 54.7000, 25.3000)
    win._on_map_context("orbit", 54.6900, 25.2800)
    win._on_map_context("roi", 54.6800, 25.2700)

def grab():
    ve = win.vehicle
    snap["texts"] = [t for (s, t) in ve.messages]
    snap["last_ack"] = ve.last_ack
    app.quit()

QTimer.singleShot(1300, send_actions)
QTimer.singleShot(2800, grab)
app.exec()

sim.terminate()
try:
    sim.wait(timeout=2)
except Exception:
    sim.kill()

texts = snap.get("texts", [])
print("feedback:", [t for t in texts if any(w in t for w in ("Home", "Orbit", "ROI"))])
print("last_ack:", snap.get("last_ack"))

fail = []
if not any("Home position set" in t for t in texts):
    fail.append("no Set-Home feedback")
if not any("Orbit radius" in t for t in texts):
    fail.append("no Orbit feedback")
if not any("ROI set" in t for t in texts):
    fail.append("no ROI feedback")
if snap.get("last_ack") is None:
    fail.append("no COMMAND_ACK received")

print("GUIDED FAILED: " + "; ".join(fail) if fail else "GUIDED PASSED")
sys.stdout.flush()
os._exit(1 if fail else 0)
