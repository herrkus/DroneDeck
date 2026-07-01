#!/usr/bin/env python3
"""test_gimbal.py -- MOUNT_ORIENTATION (265) gimbal attitude. msgid > 255 so it must ride a
MAVLink v2 frame. yaw_absolute is an extension field (excluded from CRC_EXTRA, which is 26,
NOT 77); a v2 sender may also truncate it, so the parser must zero-pad. Port-independent."""
import os
import struct
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mavlink
import core
import panels
from vehicle import Vehicle

app = QApplication([])

# Full 20-byte payload over a v2 frame -> both parsers agree.
payload = struct.pack("<I4f", 12345, -10.5, 30.2, 90.0, 275.0)
frame = mavlink.frame_v2(mavlink.MOUNT_ORIENTATION, payload, seq=7, sysid=3, compid=1,
                         crc_fn=mavlink.crc16_mcrf4xx)
cf = core.Parser().feed(frame)
pf = mavlink.PyParser().feed(frame)
assert len(cf) == 1 and len(pf) == 1, (len(cf), len(pf))
assert cf[0].fields == pf[0].fields, (cf[0].fields, pf[0].fields)      # C++/Python parity
d = cf[0].fields
assert abs(d["roll"] + 10.5) < 1e-4 and abs(d["pitch"] - 30.2) < 1e-4
assert abs(d["yaw"] - 90.0) < 1e-4 and abs(d["yaw_absolute"] - 275.0) < 1e-4

# v2 truncation: yaw_absolute (the extension) trimmed -> zero-padded to 0, parity holds.
trunc = struct.pack("<I3f", 12345, -10.5, 30.2, 90.0)                  # 16 bytes
tf = mavlink.frame_v2(mavlink.MOUNT_ORIENTATION, trunc, seq=8, sysid=3, compid=1,
                      crc_fn=mavlink.crc16_mcrf4xx)
ct = core.Parser().feed(tf)[0].fields
pt = mavlink.PyParser().feed(tf)[0].fields
assert ct == pt and abs(ct["yaw"] - 90.0) < 1e-4 and ct["yaw_absolute"] == 0.0, ct

# Vehicle maps roll/pitch/yaw; panel renders, and shows '--' with no report.
v = Vehicle()
v._on_mount_orientation(d)
assert v.have_gimbal and abs(v.gimbal_pitch - 30.2) < 1e-4 and abs(v.gimbal_yaw - 90.0) < 1e-4
sp = panels.SystemsPanel()
sp.update_from(v)
assert sp.g_pitch.text() == "+30.2 deg" and sp.g_yaw.text() == "+90.0 deg"
sp.update_from(Vehicle())
assert sp.g_pitch.text() == "--"

print("GIMBAL PASSED")
