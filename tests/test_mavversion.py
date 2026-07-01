#!/usr/bin/env python3
"""test_mavversion.py -- negotiated MAVLink wire-version detection. PX4 does NOT answer
REQUEST_MESSAGE(300)/PROTOCOL_VERSION, so the Vehicle Info dialog derives the version from the
framing byte of received frames (0xFD=v2, 0xFE=v1) via Link._note_framing -- reusing the proven
frame_total scanner, no parser change. Pure logic, no socket."""
import os
import struct
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mavlink
from link import Link

app = QApplication([])

hb = struct.pack("<IBBBBB", 0, 2, 12, 0, 4, 3)     # a HEARTBEAT payload (9 bytes)
v2 = mavlink.frame_v2(mavlink.HEARTBEAT, hb, seq=0, sysid=1, compid=1,
                      crc_fn=mavlink.crc16_mcrf4xx)
v1 = mavlink.frame(mavlink.HEARTBEAT, hb, seq=0, sysid=1, compid=1,
                   crc_fn=mavlink.crc16_mcrf4xx)
assert v2[0] == 0xFD and v1[0] == 0xFE

# unknown until a frame is seen
lk = Link()
assert lk.mavlink_version_str is None

# a v2 frame -> "2.0"
lk._note_framing(v2)
assert lk.rx_mav_v2 and not lk.rx_mav_v1
assert lk.mavlink_version_str == "2.0", lk.mavlink_version_str

# then a v1 frame -> both seen
lk._note_framing(v1)
assert lk.rx_mav_v1 and lk.mavlink_version_str == "2.0 (1.0 also seen)"

# a fresh link seeing only v1 -> "1.0"
lk2 = Link()
lk2._note_framing(v1)
assert lk2.mavlink_version_str == "1.0"

# detection also works when the frame isn't at index 0 (leading junk)
lk3 = Link()
lk3._note_framing(b"\x11\x22" + v2)
assert lk3.mavlink_version_str == "2.0"

# junk / incomplete frame must NOT set a version
lk4 = Link()
lk4._note_framing(b"\x00\x01\x02 not a frame")
assert lk4.mavlink_version_str is None
lk4._note_framing(bytes([0xFD, 0x09]))     # a v2 start but the frame isn't complete
assert lk4.mavlink_version_str is None

print("MAVVERSION PASSED")
