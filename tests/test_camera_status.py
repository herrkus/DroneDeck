#!/usr/bin/env python3
"""test_camera_status.py -- STORAGE_INFORMATION + CAMERA_CAPTURE_STATUS parsing (1:1 QGC parity).

QGC's camera adapts to the vehicle's real camera -- storage remaining, recording state, capture status.
DroneDeck's camera panel was a fixed button set. Added parsing of STORAGE_INFORMATION (261) and
CAMERA_CAPTURE_STATUS (262) on both parser backends (CRC seeds 179/12 verified), vehicle state, and a
live storage/REC readout in the camera panel. Verifies decode parity + exact values, handler storage
(MB) + recording bool + recording seconds, and the panel readout text."""
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
CASES = {
    mavlink.STORAGE_INFORMATION: ("<IfffffBBB", {
        "time_boot_ms": 5000, "total_capacity": 32000.0, "used_capacity": 8000.0,
        "available_capacity": 24000.0, "read_speed": 10.0, "write_speed": 8.0,
        "storage_id": 1, "storage_count": 1, "status": 2}),
    mavlink.CAMERA_CAPTURE_STATUS: ("<IfIfBB", {
        "time_boot_ms": 5000, "image_interval": 2.0, "recording_time_ms": 30000,
        "available_capacity": 1500.0, "image_status": 0, "video_status": 1}),
}


def frame_for(mid):
    fmt, vals = CASES[mid]
    names = mavlink._WIRE[mid][1]
    payload = struct.pack(fmt, *[vals[n] for n in names])
    # msgid > 255 exists only in MAVLink v2 (24-bit id) -- the only way these arrive on a real link
    fn = mavlink.frame_v2 if mid > 255 else mavlink.frame
    return fn(mid, payload, 7, 1, 1, crc_fn=core.crc_extra)


# 1) both backends decode with matching + exact values ----------------------------------------------
for mid, (fmt, vals) in CASES.items():
    fr = frame_for(mid)
    nat, pyp = core.Parser().feed(fr), mavlink.PyParser().feed(fr)
    nm = mavlink.MSG_NAME[mid]
    if len(nat) != 1 or len(pyp) != 1:
        fail.append(f"{nm}: decode count native={len(nat)} py={len(pyp)}")
        continue
    for n, exp in vals.items():
        a, b = nat[0].fields.get(n), pyp[0].fields.get(n)
        if a is None or b is None or abs(a - exp) > 1e-3 or abs(b - exp) > 1e-3:
            fail.append(f"{nm}.{n}: native={a} py={b} expected={exp}")

# 2) vehicle handlers store storage + recording -----------------------------------------------------
ve = Vehicle()
for mid in CASES:
    ve.consume(core.Parser().feed(frame_for(mid)))
checks = [
    (abs((ve.storage_total_mb or 0) - 32000.0) < 1e-3, f"storage_total_mb {ve.storage_total_mb}"),
    (abs((ve.storage_available_mb or 0) - 24000.0) < 1e-3, f"storage_available_mb {ve.storage_available_mb}"),
    (ve.storage_status == 2, f"storage_status {ve.storage_status}"),
    (ve.cam_recording is True, f"cam_recording {ve.cam_recording} (video_status=1 -> True)"),
    (abs((ve.cam_recording_time_s or 0) - 30.0) < 1e-3, f"cam_recording_time_s {ve.cam_recording_time_s}"),
    (ve.cam_image_status == 0, f"cam_image_status {ve.cam_image_status}"),
]
for ok, msg in checks:
    if not ok:
        fail.append("vehicle: " + msg)

# 3) camera panel shows storage + REC ---------------------------------------------------------------
from PySide6.QtWidgets import QApplication
from panels import CameraPanel
app = QApplication.instance() or QApplication([])
cp = CameraPanel()
cp.update_status(ve)
txt = cp.cam_status.text()
if "GB" not in txt or "REC" not in txt or "23.4" not in txt:   # 24000 MB / 1024 = 23.4 GB
    fail.append(f"camera panel status wrong: '{txt}' (want SD ~23.4 GB + REC)")
cp.update_status(Vehicle())        # no camera telemetry -> blank
if cp.cam_status.text() != "":
    fail.append(f"panel status should be blank with no camera telemetry, got '{cp.cam_status.text()}'")

print("CAMERA_STATUS FAILED: " + "; ".join(fail) if fail else
      "CAMERA_STATUS PASSED (STORAGE_INFORMATION/CAMERA_CAPTURE_STATUS decode native==python w/ exact "
      "values; vehicle stores storage MB + recording state/time; panel shows SD GB + REC)")
sys.exit(1 if fail else 0)
