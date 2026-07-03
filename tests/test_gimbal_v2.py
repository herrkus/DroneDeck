#!/usr/bin/env python3
"""test_gimbal_v2.py -- gimbal manager v2 support (1:1 QGC parity).

DroneDeck only spoke gimbal v1 (DO_MOUNT_CONTROL 205). Modern gimbals use the gimbal-manager
protocol; QGC sends MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW (1000). Added link.set_gimbal_v2 + a protocol
selector in the camera panel (default v1 for widest support, v2 opt-in). Verifies: v2 encodes the
right command + params (pitch, yaw, NaN rates so it's an ANGLE command, gimbal id); v1 is unchanged;
the panel selector defaults to v1; and the main window routes gimbal commands to v1/v2 per the selector."""
import os
import sys
import math

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


def decode_last(lk):
    msgs = core.Parser().feed(lk.sent[-1])
    return msgs[0].fields if msgs else None


# 1) v2 encodes DO_GIMBAL_MANAGER_PITCHYAW with an angle command -------------------------------------
lk = CaptureLink()
lk.set_gimbal_v2(1, -30.0, 45.0, gimbal_id=2)
f = decode_last(lk)
if not f or int(f.get("command", -1)) != mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
    fail.append(f"v2 should send DO_GIMBAL_MANAGER_PITCHYAW (1000), got {f and f.get('command')}")
else:
    if abs(f["param1"] + 30.0) > 1e-3 or abs(f["param2"] - 45.0) > 1e-3:
        fail.append(f"v2 pitch/yaw wrong: p1={f['param1']} p2={f['param2']}")
    if not (math.isnan(f["param3"]) and math.isnan(f["param4"])):
        fail.append("v2 rate params must be NaN (angle command, not a rate)")
    if abs(f["param7"] - 2.0) > 1e-3:
        fail.append(f"v2 gimbal id should be 2, got {f['param7']}")

# 2) v1 is unchanged (DO_MOUNT_CONTROL) --------------------------------------------------------------
lk2 = CaptureLink()
lk2.set_gimbal(1, -30.0, 45.0)
f1 = decode_last(lk2)
if not f1 or int(f1.get("command", -1)) != mavlink.MAV_CMD_DO_MOUNT_CONTROL:
    fail.append(f"v1 should still send DO_MOUNT_CONTROL (205), got {f1 and f1.get('command')}")

# 3) camera panel has the protocol selector, defaulting to v1 ----------------------------------------
from PySide6.QtWidgets import QApplication
app = QApplication.instance() or QApplication([])
from panels import CameraPanel
cp = CameraPanel()
if cp.gimbal_proto.count() != 2 or cp.gimbal_proto.currentIndex() != 0:
    fail.append("gimbal protocol selector should default to v1 (index 0) with 2 options")

# 4) the main window routes gimbal commands to v1/v2 per the selector --------------------------------
import main as m
win = m.DroneDeck(16010)
win._persist = False
calls = []
win.link.set_gimbal = lambda *a: calls.append(("v1", a))
win.link.set_gimbal_v2 = lambda *a: calls.append(("v2", a))
win._has_vehicle = lambda: True
win._sysid = lambda: 1
win.camera.gimbal_proto.setCurrentIndex(0)
win._cam_gimbal(-10.0, 20.0)
win.camera.gimbal_proto.setCurrentIndex(1)
win._cam_gimbal(-10.0, 20.0)
if [c[0] for c in calls] != ["v1", "v2"]:
    fail.append(f"gimbal routing wrong: expected [v1, v2], got {[c[0] for c in calls]}")

print("GIMBAL_V2 FAILED: " + "; ".join(fail) if fail else
      "GIMBAL_V2 PASSED (DO_GIMBAL_MANAGER_PITCHYAW angle command w/ NaN rates + gimbal id; v1 "
      "DO_MOUNT_CONTROL unchanged; panel selector defaults v1; main routes v1/v2 per selector)")
sys.exit(1 if fail else 0)
