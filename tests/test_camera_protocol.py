#!/usr/bin/env python3
"""test_camera_protocol.py -- CAMERA_SETTINGS(260) + CAMERA_IMAGE_CAPTURED(263) (iter150, QGC parity).

Rounds out the camera protocol (iter143 did storage+capture status). QGC shows the camera's current
mode and counts captured images; DroneDeck had neither. Added both to BOTH parsers (native C++/asm core
+ pure-Python; CRC 146/133). CAMERA_IMAGE_CAPTURED decodes only its 50-byte prefix (index + result +
position + attitude quaternion), skipping the 205-byte file_url; the q[4] array is unpacked as q0..q3.
Both msgids exceed 255 so they REQUIRE MAVLink v2 framing. Verifies: both backends decode identically
with exact values; file_url never leaks; the vehicle stores mode/index/result; the camera panel shows
the mode label + image count."""
import os
import sys
import struct

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import mavlink
import core
from vehicle import Vehicle

fail = []

set_payload = struct.pack("<IB", 1000, 2)                       # mode 2 = image survey
set_frame = mavlink.frame_v2(mavlink.CAMERA_SETTINGS, set_payload, 7, 1, 1, crc_fn=core.crc_extra)
set_exp = {"time_boot_ms": 1000, "mode_id": 2}

cap_prefix = struct.pack("<QIiiiiffffiBb", 111, 2000, 473980000, 85460000, 500000, 100000,
                         1.0, 0.0, 0.0, 0.0, 7, 1, 1)          # image_index 7, capture_result 1 (ok)
cap_payload = cap_prefix + b"/DCIM/IMG_0007.jpg".ljust(205, b"\x00")   # file_url must be skipped
cap_frame = mavlink.frame_v2(mavlink.CAMERA_IMAGE_CAPTURED, cap_payload, 7, 1, 1, crc_fn=core.crc_extra)
cap_exp = {"time_utc": 111, "time_boot_ms": 2000, "lat": 473980000, "lon": 85460000,
           "alt": 500000, "relative_alt": 100000, "q0": 1.0, "image_index": 7,
           "camera_id": 1, "capture_result": 1}

# 1) both backends decode identically with exact values; file_url is skipped -------------------------
print("native backend live:", core.NATIVE)
for frame, exp, name in ((set_frame, set_exp, "CAMERA_SETTINGS"),
                         (cap_frame, cap_exp, "CAMERA_IMAGE_CAPTURED")):
    nat, pyp = core.Parser().feed(frame), mavlink.PyParser().feed(frame)
    if len(nat) != 1 or len(pyp) != 1:
        fail.append(f"{name}: decode count native={len(nat)} py={len(pyp)}")
        continue
    for k, v in exp.items():
        a, b = nat[0].fields.get(k), pyp[0].fields.get(k)
        if a is None or b is None or abs(a - v) > 1e-3 or abs(b - v) > 1e-3:
            fail.append(f"{name}.{k}: native={a} py={b} expected={v}")
    if "file_url" in nat[0].fields or "file_url" in pyp[0].fields:
        fail.append(f"{name}: file_url should be skipped, not decoded")

# 2) vehicle handlers store mode + capture index/result ---------------------------------------------
ve = Vehicle()
ve.consume(core.Parser().feed(set_frame))
ve.consume(core.Parser().feed(cap_frame))
if ve.cam_mode != 2:
    fail.append(f"cam_mode {ve.cam_mode}, want 2 (survey)")
if ve.cam_images_captured != 7 or ve.cam_last_capture_ok is not True:
    fail.append(f"capture state: images={ve.cam_images_captured} ok={ve.cam_last_capture_ok}")

# a failed capture (result != 1) must record ok=False -----------------------------------------------
bad_prefix = struct.pack("<QIiiiiffffiBb", 111, 2100, 0, 0, 0, 0, 1.0, 0.0, 0.0, 0.0, 8, 1, 0)
bad_frame = mavlink.frame_v2(mavlink.CAMERA_IMAGE_CAPTURED, bad_prefix + b"\x00" * 205, 7, 1, 1,
                             crc_fn=core.crc_extra)
ve.consume(core.Parser().feed(bad_frame))
if ve.cam_images_captured != 8 or ve.cam_last_capture_ok is not False:
    fail.append(f"failed capture: images={ve.cam_images_captured} ok={ve.cam_last_capture_ok}")

fresh = Vehicle()
if fresh.cam_mode is not None or fresh.cam_images_captured is not None:
    fail.append("fresh vehicle should have None camera mode/index until telemetry arrives")

# 3) camera panel shows the mode label + image count ------------------------------------------------
from PySide6.QtWidgets import QApplication
from panels import CameraPanel
app = QApplication.instance() or QApplication([])
cp = CameraPanel()
cp.update_status(ve)
txt = cp.cam_status.text()
if "SURVEY" not in txt or "img 8" not in txt:
    fail.append(f"camera panel shows '{txt}', expected SURVEY + img 8")
cp.update_status(fresh)
if cp.cam_status.text() != "":
    fail.append(f"camera panel should be blank with no telemetry, got '{cp.cam_status.text()}'")

print("CAMERA_PROTOCOL FAILED: " + "; ".join(fail) if fail else
      "CAMERA_PROTOCOL PASSED (CAMERA_SETTINGS/CAMERA_IMAGE_CAPTURED decode native==python with exact "
      "values over v2, file_url skipped; vehicle stores mode + capture index/result; panel shows mode "
      "label + image count)")
sys.exit(1 if fail else 0)
