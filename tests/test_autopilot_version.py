#!/usr/bin/env python3
"""test_autopilot_version.py -- AUTOPILOT_VERSION (148): firmware version + capability flags.
The uint64 uid loses precision as a double so it is skipped (not decoded); CRC_EXTRA (178) is
still computed over the full 60-byte payload. flight_sw_version decodes to major.minor.patch;
flight_custom_version[8] -> git-hash hex; capabilities -> named protocol flags. v1 frame
(148 <= 255). Port-independent (no link)."""
import os
import struct
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mavlink
import core
from vehicle import Vehicle

app = QApplication([])

caps = 4 | 8 | 2 | 8192 | 32                       # MISSION_INT|COMMAND_INT|PARAM_FLOAT|MAVLINK2|FTP
flight_sw = (1 << 24) | (15 << 16)                # 1.15.0
uid = 0xDEADBEEFCAFEBABE                          # large -> must be skipped, not decoded
gh = [0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0]
payload = struct.pack("<2Q4I2H24B", caps, uid, flight_sw, 0x01020304, 0x05060708, 999,
                      12345, 54321, *(gh + [0] * 8 + [0] * 8))
frame = mavlink.frame(mavlink.AUTOPILOT_VERSION, payload, seq=2, sysid=1, compid=1,
                      crc_fn=mavlink.crc16_mcrf4xx)
cf = core.Parser().feed(frame)
pf = mavlink.PyParser().feed(frame)
assert len(cf) == 1 and len(pf) == 1, (len(cf), len(pf))
assert cf[0].fields == pf[0].fields, (cf[0].fields, pf[0].fields)     # C++/Python parity
d = cf[0].fields
assert "uid" not in d                             # skipped
assert d["capabilities"] == 8238 and d["flight_sw_version"] == flight_sw
assert d["board_version"] == 999 and d["vendor_id"] == 12345 and d["product_id"] == 54321
assert d["fcv0"] == 0x12 and d["fcv7"] == 0xf0

v = Vehicle()
v._on_autopilot_version(d)
assert v.have_autopilot_version and v.fw_version == "1.15.0"
assert v.fw_git == "123456789abcdef0"
names = mavlink.capability_names(v.capabilities)
assert "Mission int" in names and "MAVLink2" in names and "FTP" in names, names

# helper corner: version 0 -> "0.0.0"; empty caps -> []
assert mavlink.fw_version_str(0) == "0.0.0"
assert mavlink.capability_names(0) == []

print("AUTOPILOT_VERSION PASSED")
