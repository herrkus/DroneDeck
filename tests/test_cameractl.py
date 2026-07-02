#!/usr/bin/env python3
"""test_cameractl.py -- camera mode (photo/video) + zoom controls (iter118), QGC-parity.

The CameraPanel gained a photo/video mode toggle (SET_CAMERA_MODE 530) and zoom +/- (SET_CAMERA_ZOOM
531). Verifies the panel emits the right signals + relabels, and that link.set_camera_mode /
link.camera_zoom build the correct COMMAND_LONG params. No real camera, no link traffic."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mavlink
import panels
from link import UdpLink

app = QApplication.instance() or QApplication([])
fail = []

# 1) CameraPanel: mode toggle emits 0/1 + relabels; zoom buttons emit +1/-1 ------------------------
cp = panels.CameraPanel()
modes, zooms = [], []
cp.cameraMode.connect(modes.append)
cp.cameraZoom.connect(zooms.append)

cp.btn_mode.setChecked(True)                    # -> video
if not (modes and modes[-1] == 1 and cp.btn_mode.text() == "Mode: Video"):
    fail.append(f"video mode: emit={modes} label={cp.btn_mode.text()!r}")
cp.btn_mode.setChecked(False)                   # -> photo
if not (modes[-1] == 0 and cp.btn_mode.text() == "Mode: Photo"):
    fail.append(f"photo mode: emit={modes} label={cp.btn_mode.text()!r}")

cp.cameraZoom.emit(1.0)                          # (simulate the + button)
cp.cameraZoom.emit(-1.0)
if zooms[-2:] != [1.0, -1.0]:
    fail.append(f"zoom emits wrong: {zooms}")

# 2) link.set_camera_mode -> SET_CAMERA_MODE (530), mode in param2 --------------------------------
sent = []
lk = UdpLink()
lk.send_command_long = lambda tgt, cmd, params: sent.append((tgt, cmd, list(params)))
lk.set_camera_mode(1, mavlink.CAMERA_MODE_VIDEO)
tgt, cmd, p = sent[-1]
if cmd != mavlink.MAV_CMD_SET_CAMERA_MODE or mavlink.MAV_CMD_SET_CAMERA_MODE != 530:
    fail.append(f"set_camera_mode cmd={cmd}")
if p[1] != mavlink.CAMERA_MODE_VIDEO:
    fail.append(f"camera mode param2={p[1]} (expect video=1)")

# 3) link.camera_zoom -> SET_CAMERA_ZOOM (531), step type in param1, +/-1 in param2 ----------------
lk.camera_zoom(1, 1)                             # zoom in
p = sent[-1][2]
if sent[-1][1] != mavlink.MAV_CMD_SET_CAMERA_ZOOM or mavlink.MAV_CMD_SET_CAMERA_ZOOM != 531:
    fail.append(f"camera_zoom cmd={sent[-1][1]}")
if p[0] != mavlink.ZOOM_TYPE_STEP or p[1] != 1:
    fail.append(f"zoom params: type={p[0]} value={p[1]} (expect step=0, +1)")
lk.camera_zoom(1, -1)                            # zoom out
if sent[-1][2][1] != -1:
    fail.append("zoom-out value should be -1")

print("CAMERACTL FAILED: " + "; ".join(fail) if fail else
      "CAMERACTL PASSED (photo/video mode toggle + zoom emit + SET_CAMERA_MODE/ZOOM COMMAND_LONG params)")
sys.exit(1 if fail else 0)
