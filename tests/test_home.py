#!/usr/bin/env python3
"""test_home.py -- HOME_POSITION (242): the authoritative RTL home + altitude. q is a
float[4] array (its length byte goes into CRC_EXTRA = 104); time_usec is an extension
excluded from the CRC. Confirms C++/Python parity and that HOME_POSITION overrides the
first-fix home guess and supplies home altitude. Port-independent (no link)."""
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

payload = struct.pack("<3i10f", int(47.5 * 1e7), int(8.5 * 1e7), int(488.2 * 1000),
                      1.0, 2.0, -3.0, 1.0, 0.0, 0.0, 0.0, 4.0, 5.0, 6.0)
frame = mavlink.frame(mavlink.HOME_POSITION, payload, seq=7, sysid=3, compid=1,
                      crc_fn=mavlink.crc16_mcrf4xx)
cf = core.Parser().feed(frame)
pf = mavlink.PyParser().feed(frame)
assert len(cf) == 1 and len(pf) == 1, (len(cf), len(pf))
assert cf[0].fields == pf[0].fields, (cf[0].fields, pf[0].fields)      # C++/Python parity
d = cf[0].fields
assert d["latitude"] == 475000000 and d["longitude"] == 85000000 and d["altitude"] == 488200
assert abs(d["q0"] - 1.0) < 1e-4 and abs(d["approach_z"] - 6.0) < 1e-4

v = Vehicle()
# a rough first-fix home lands first
v.have_position = True
v.lat, v.lon = 47.4, 8.4
v._on_global_position({"lat": int(47.4 * 1e7), "lon": int(8.4 * 1e7), "alt": 0,
                       "relative_alt": 0, "hdg": 0, "vx": 0, "vy": 0, "vz": 0})
assert v.home == (47.4, 8.4) and v.home_alt is None

# HOME_POSITION is authoritative: it replaces the guess and provides altitude
v._on_home_position(d)
assert v.have_home_position
assert abs(v.home[0] - 47.5) < 1e-6 and abs(v.home[1] - 8.5) < 1e-6
assert abs(v.home_alt - 488.2) < 1e-3

print("HOME PASSED")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
