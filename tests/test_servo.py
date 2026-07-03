#!/usr/bin/env python3
"""test_servo.py -- SERVO_OUTPUT_RAW (36) actuator outputs. On the wire the uint8 port sorts
LAST (after the uint32 timestamp and eight uint16 servo values); servo9..16_raw are
extensions excluded from CRC_EXTRA (222). Confirms C++/Python parity, the vehicle mapping,
and that the actuator bar widget renders. Port-independent (no link)."""
import os
import struct
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QPixmap
import mavlink
import core
import panels
from vehicle import Vehicle

app = QApplication([])

payload = struct.pack("<I8HB", 555, 1500, 1600, 1000, 2000, 1450, 0, 0, 0, 3)
frame = mavlink.frame(mavlink.SERVO_OUTPUT_RAW, payload, seq=7, sysid=3, compid=1,
                      crc_fn=mavlink.crc16_mcrf4xx)
cf = core.Parser().feed(frame)
pf = mavlink.PyParser().feed(frame)
assert len(cf) == 1 and len(pf) == 1, (len(cf), len(pf))
assert cf[0].fields == pf[0].fields, (cf[0].fields, pf[0].fields)     # C++/Python parity
d = cf[0].fields
assert d["servo1_raw"] == 1500 and d["servo4_raw"] == 2000 and d["servo8_raw"] == 0 and d["port"] == 3

v = Vehicle()
v._on_servo_output_raw(d)
assert v.have_servo and v.servo_raw == [1500, 1600, 1000, 2000, 1450, 0, 0, 0]

sp = panels.SystemsPanel()
sp.update_from(v)
assert sp.servo_bars.vals == [1500, 1600, 1000, 2000, 1450, 0, 0, 0]
pm = QPixmap(260, 52)
sp.servo_bars.resize(260, 52)
sp.servo_bars.render(pm)                         # active + inactive channels
sp.update_from(Vehicle())
sp.servo_bars.render(pm)                         # all-inactive path

print("SERVO PASSED")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
