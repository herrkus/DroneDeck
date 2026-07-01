#!/usr/bin/env python3
"""test_esc.py -- ESC_STATUS (291): per-ESC rpm / voltage / current. msgid > 255 so it MUST
be v2-framed. rpm/voltage/current are arrays of 4 (their length byte enters CRC_EXTRA = 10);
the uint8 index sorts last after the uint64 timestamp. 'index' selects the bank of four ESCs,
so index=4 addresses motors 5-8. Confirms C++/Python parity, the vehicle mapping (incl. banks)
and the Systems ESC rows. Port-independent (no link)."""
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

# bank 0 (motors 1-4); motor 4 idle
payload = struct.pack("<Q4i4f4fB", 999, 5200, 5300, 5100, 0,
                      15.1, 15.2, 15.0, 0.0, 3.1, 3.2, 3.0, 0.0, 0)
frame = mavlink.frame_v2(mavlink.ESC_STATUS, payload, seq=4, sysid=1, compid=1,
                         crc_fn=mavlink.crc16_mcrf4xx)
cf = core.Parser().feed(frame)
pf = mavlink.PyParser().feed(frame)
assert len(cf) == 1 and len(pf) == 1, (len(cf), len(pf))
assert cf[0].fields == pf[0].fields, (cf[0].fields, pf[0].fields)      # C++/Python parity
d = cf[0].fields
assert d["rpm1"] == 5200 and d["rpm3"] == 5100 and abs(d["voltage2"] - 15.2) < 1e-4
assert abs(d["current1"] - 3.1) < 1e-4 and d["index"] == 0

# msgid 291 > 255: a v1 frame truncates the id to one byte, so it can never decode as ESC_STATUS
v1 = mavlink.frame(mavlink.ESC_STATUS, payload, seq=4, sysid=1, compid=1,
                   crc_fn=mavlink.crc16_mcrf4xx)
v1_out = core.Parser().feed(v1)
assert not v1_out or v1_out[0].msgid != 291, v1_out

v = Vehicle()
v._on_esc_status(d)
assert v.have_esc and v.esc_rpm[:4] == [5200, 5300, 5100, 0]
assert abs(v.esc_voltage[0] - 15.1) < 1e-4 and abs(v.esc_current[1] - 3.2) < 1e-4

# bank index=4 addresses motors 5-8
p2 = struct.pack("<Q4i4f4fB", 1000, 4000, 0, 0, 0, 14.0, 0, 0, 0, 2.0, 0, 0, 0, 4)
f2 = mavlink.frame_v2(mavlink.ESC_STATUS, p2, seq=5, sysid=1, compid=1,
                      crc_fn=mavlink.crc16_mcrf4xx)
v._on_esc_status(core.Parser().feed(f2)[0].fields)
assert v.esc_rpm[4] == 4000, v.esc_rpm

sp = panels.SystemsPanel()
sp.update_from(v)
assert "5200 rpm" in sp._esc_rows[0][1].text() and "4000 rpm" in sp._esc_rows[4][1].text()
assert not sp._esc_group.isHidden()                    # active motors -> group visible

# at rest (ESC_STATUS received but every motor idle) the group hides -- no empty box
idle = Vehicle()
idle.have_esc = True                                   # message seen, all values zero
sp.update_from(idle)
assert sp._esc_group.isHidden()

print("ESC PASSED")
