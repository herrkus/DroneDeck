#!/usr/bin/env python3
"""test_manual.py -- MANUAL_CONTROL end to end against the simulator.

Arms the vehicle, streams forward-pitch then yaw stick commands, and verifies
the simulated vehicle switches to ALT_HOLD, translates north, and changes
heading -- proving the joystick -> MANUAL_CONTROL -> vehicle path works.
"""
import os
import sys
import math
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

import mavlink
from link import UdpLink
from vehicle import Vehicle

app = QApplication([])
link = UdpLink()
link.open(port=PORT)
ve = Vehicle()
link.messages.connect(ve.consume)
sim = subprocess.Popen([sys.executable, SIM, "--target", f"127.0.0.1:{PORT}"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

snap = {}
stick = {"x": 0, "r": 0}


def arm():
    link.arm(ve.sysid or 1, True)


def push_forward():
    snap["lat0"], snap["lon0"] = ve.lat, ve.lon   # start of forward phase
    stick["x"], stick["r"] = 900, 0               # full forward pitch


def grab_after_forward():
    snap["mode"] = ve.mode
    snap["lat1"], snap["lon1"] = ve.lat, ve.lon   # end of forward phase
    snap["hdg0"] = ve.heading
    stick["x"], stick["r"] = 0, 900               # full yaw right


def finish():
    snap["hdg1"] = ve.heading
    app.quit()


def tick():
    # 25 Hz manual stream once armed
    if ve.armed:
        link.send_manual_control(ve.sysid or 1, stick["x"], 0, 0, stick["r"])


send_timer = QTimer()
send_timer.timeout.connect(tick)
send_timer.start(40)

QTimer.singleShot(1500, arm)
QTimer.singleShot(2200, push_forward)
QTimer.singleShot(4200, grab_after_forward)
QTimer.singleShot(6200, finish)
app.exec()

sim.terminate()
try:
    sim.wait(timeout=2)
except Exception:
    sim.kill()
link.close()

# ground distance covered during the forward-pitch phase (moves along heading)
dlat = (snap.get("lat1", 0) - snap.get("lat0", 0)) * 111320.0
dlon = (snap.get("lon1", 0) - snap.get("lon0", 0)) * 111320.0 * math.cos(math.radians(snap.get("lat0", 0)))
move_m = math.hypot(dlat, dlon)
raw = (snap.get("hdg1", 0) - snap.get("hdg0", 0)) % 360.0
turn = min(raw, 360.0 - raw)            # smallest angle, wrap-aware
print("snapshot:", {k: (round(v, 6) if isinstance(v, float) else v) for k, v in snap.items()})
print(f"ground movement: {move_m:.1f} m   heading change: {turn:.1f} deg")

fail = []
if snap.get("mode") != "ALT_HOLD":
    fail.append(f"manual did not select ALT_HOLD (mode={snap.get('mode')})")
if move_m < 10.0:
    fail.append(f"forward pitch moved only {move_m:.1f} m")
if turn < 20.0:
    fail.append(f"yaw command changed heading by only {turn:.1f} deg")

print("MANUAL FAILED: " + "; ".join(fail) if fail else "MANUAL PASSED")
sys.exit(1 if fail else 0)
