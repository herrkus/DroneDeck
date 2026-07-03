#!/usr/bin/env python3
"""test_accel_cal.py -- accel 6-position calibration handshake (1:1 QGC parity).

DroneDeck sent PREFLIGHT_CALIBRATION(accel) and showed the vehicle's STATUSTEXT prompts, but had no
way to advance ArduPilot's interactive 6-position dance (it needs MAV_CMD_ACCELCAL_VEHICLE_POS per
orientation). Added link.accel_cal_position + a position-button row in the sensor-cal widget (enabled
only during accel cal). Verifies the command encoding, the widget enable/disable + emit behaviour, and
the main-window routing/guard."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import mavlink
import core
from link import Link

fail = []


class CaptureLink(Link):
    def __init__(self):
        super().__init__()
        self.sent = []
        self._open = True
        self.remote = True

    def _write(self, data):
        self.sent.append(data)


# 1) the command encodes ACCELCAL_VEHICLE_POS with param1 = position ---------------------------------
lk = CaptureLink()
lk.accel_cal_position(1, mavlink.ACCELCAL_POS_NOSEDOWN)
msgs = core.Parser().feed(lk.sent[-1])
f = msgs[0].fields if msgs else None
if not f or int(f.get("command", -1)) != mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS or abs(f["param1"] - 4) > 1e-6:
    fail.append(f"accel_cal_position should send 42429 param1=4, got cmd={f and f.get('command')} p1={f and f.get('param1')}")

# 2) the sensor-cal widget: position buttons enable only during accel cal + emit the position --------
from PySide6.QtWidgets import QApplication
app = QApplication.instance() or QApplication([])
from calibration import SensorCalibrationWidget
w = SensorCalibrationWidget()
if any(b.isEnabled() for b in w._pos_btns):
    fail.append("position buttons should start disabled")
w._request("accel")
if not all(b.isEnabled() for b in w._pos_btns):
    fail.append("accel cal should enable the position buttons")
got = []
w.accelPosRequested.connect(lambda p: got.append(p))
w._pos_btns[0].click()      # Level = 1
w._pos_btns[3].click()      # Nose Down = 4
if got != [mavlink.ACCELCAL_POS_LEVEL, mavlink.ACCELCAL_POS_NOSEDOWN]:
    fail.append(f"position buttons should emit their position, got {got}")
w._request("gyro")
if any(b.isEnabled() for b in w._pos_btns):
    fail.append("a non-accel calibration should disable the position buttons")

# 3) main window routes the position + guards no-vehicle ---------------------------------------------
import main as m
win = m.DroneDeck(16410)
win._persist = False
calls = []
win.link.accel_cal_position = lambda s, p: calls.append(p)
win._sysid = lambda: 1
win._has_vehicle = lambda: False
win._cal_accel_pos(2)
if calls:
    fail.append("no-vehicle: accel position must not send")
win._has_vehicle = lambda: True
win._cal_accel_pos(2)
win._cal_accel_pos(6)
if calls != [2, 6]:
    fail.append(f"handler should route positions, got {calls}")

print("ACCEL_CAL FAILED: " + "; ".join(fail) if fail else
      "ACCEL_CAL PASSED (ACCELCAL_VEHICLE_POS 42429 param1=position; widget enables position buttons "
      "only during accel cal + emits them; main routes + guards no-vehicle)")
sys.exit(1 if fail else 0)
