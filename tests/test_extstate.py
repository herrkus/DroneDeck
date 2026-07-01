#!/usr/bin/env python3
"""test_extstate.py -- EXTENDED_SYS_STATE (245): landed_state + vtol_state (two uint8, v1,
CRC_EXTRA 130, no extensions). Confirms C++/Python parity, the vehicle mapping, and the
Systems 'Flight state' rows (in-air/on-ground text; the VTOL row hides for non-VTOL craft).
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

payload = struct.pack("<2B", mavlink.MAV_VTOL_STATE_FW, 2)          # fixed-wing + in air
frame = mavlink.frame(mavlink.EXTENDED_SYS_STATE, payload, seq=3, sysid=1, compid=1,
                      crc_fn=mavlink.crc16_mcrf4xx)
cf = core.Parser().feed(frame)
pf = mavlink.PyParser().feed(frame)
assert len(cf) == 1 and len(pf) == 1, (len(cf), len(pf))
assert cf[0].fields == pf[0].fields, (cf[0].fields, pf[0].fields)   # C++/Python parity
d = cf[0].fields
assert d["vtol_state"] == 4 and d["landed_state"] == 2

v = Vehicle()
v._on_extended_sys_state(d)
assert v.have_ext_state and v.landed_state == 2 and v.vtol_state == 4

sp = panels.SystemsPanel()
sp.update_from(v)
assert sp.f_landed.text() == "In air", sp.f_landed.text()
assert sp.f_vtol.text() == "Fixed-wing", sp.f_vtol.text()

# non-VTOL on the ground: state text + VTOL row hidden
v2 = Vehicle()
v2._on_extended_sys_state({"vtol_state": 0, "landed_state": 1})
sp.update_from(v2)
assert sp.f_landed.text() == "On ground" and sp.f_vtol.isHidden()

print("EXTSTATE PASSED")
