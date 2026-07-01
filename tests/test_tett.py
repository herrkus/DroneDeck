#!/usr/bin/env python3
"""test_tett.py -- TIME_ESTIMATE_TO_TARGET (380): the autopilot's own time estimates for RTL
(safe_return), land, and mission completion (mission_end), in seconds (-1 = n/a). msgid > 255
so it MUST be v2-framed; CRC_EXTRA 232, no extensions. Confirms C++/Python parity, the vehicle
mapping, and that the NAVIGATION panel shows 'RTL time'/'Mission ETA' from the nav dict.
Port-independent (no link)."""
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

payload = struct.pack("<5i", 125, 40, 30, 310, -1)     # rtl=125s land=40 next=30 mission=310 cmd=n/a
frame = mavlink.frame_v2(mavlink.TIME_ESTIMATE_TO_TARGET, payload, seq=6, sysid=1, compid=1,
                         crc_fn=mavlink.crc16_mcrf4xx)
cf = core.Parser().feed(frame)
pf = mavlink.PyParser().feed(frame)
assert len(cf) == 1 and len(pf) == 1, (len(cf), len(pf))
assert cf[0].fields == pf[0].fields, (cf[0].fields, pf[0].fields)     # C++/Python parity
d = cf[0].fields
assert d["safe_return"] == 125 and d["mission_end"] == 310 and d["commanded_action"] == -1

# msgid 380 > 255: a v1 frame truncates the id and must not decode as 380
v1 = mavlink.frame(mavlink.TIME_ESTIMATE_TO_TARGET, payload, seq=6, sysid=1, compid=1,
                   crc_fn=mavlink.crc16_mcrf4xx)
v1_out = core.Parser().feed(v1)
assert not v1_out or v1_out[0].msgid != 380, v1_out

v = Vehicle()
v._on_time_estimate(d)
assert v.have_time_estimate and v.eta_safe_return == 125 and v.eta_mission_end == 310

# NAVIGATION panel shows the two readouts (mm:ss) from the nav dict
tp = panels.TelemetryPanel()
tp.update_all(v, "connected", 5.0, 100, 0, nav={"rtl_time": "02:05", "mission_eta": "05:10"})
assert tp.v["rtl_time"].text() == "02:05" and tp.v["mission_eta"].text() == "05:10"

print("TETT PASSED")
