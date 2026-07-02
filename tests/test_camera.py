#!/usr/bin/env python3
"""test_camera.py -- camera + gimbal COMMAND_LONG round-trip vs the simulator.

Sends photo / trigger-distance / video / gimbal commands and checks the vehicle
returns the matching STATUSTEXT feedback (and that COMMAND_ACKs arrive).
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

from link import UdpLink
from vehicle import Vehicle

app = QApplication([])
link = UdpLink()
link.open(port=PORT)
ve = Vehicle()
link.messages.connect(ve.consume)
sim = subprocess.Popen([sys.executable, SIM, "--target", f"127.0.0.1:{PORT}"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

sysid = [1]
QTimer.singleShot(1400, lambda: link.trigger_camera(ve.sysid or 1))
QTimer.singleShot(1800, lambda: link.set_trigger_distance(ve.sysid or 1, 25))
QTimer.singleShot(2200, lambda: link.video_capture(ve.sysid or 1, True))
QTimer.singleShot(2600, lambda: link.video_capture(ve.sysid or 1, False))
QTimer.singleShot(3000, lambda: link.set_gimbal(ve.sysid or 1, -35, 90))
QTimer.singleShot(4200, app.quit)
app.exec()

sim.terminate()
try:
    sim.wait(timeout=2)
except Exception:
    sim.kill()
link.close()

texts = [t for _, t in ve.messages]
print("messages:", texts[-8:])

fail = []
def seen(sub):
    return any(sub in t for t in texts)

if not seen("Photo captured"):
    fail.append("no photo feedback")
if not seen("trigger dist 25"):
    fail.append("no trigger-distance feedback")
if not seen("Video recording"):
    fail.append("no video-start feedback")
if not seen("Video stopped"):
    fail.append("no video-stop feedback")
if not (seen("Gimbal pitch -35") and seen("yaw 90")):
    fail.append("no gimbal feedback")
if ve.last_ack is None:
    fail.append("no COMMAND_ACK received")

print("CAMERA FAILED: " + "; ".join(fail) if fail else "CAMERA PASSED")
sys.exit(1 if fail else 0)
