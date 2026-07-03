#!/usr/bin/env python3
"""test_gnss.py -- GNSS_INTEGRITY (441): GPS jamming / spoofing / RAIM integrity monitoring
(development dialect, but PX4 emits it). msgid > 255 -> v2-framed; CRC_EXTRA 169, no extensions;
wire order uint32 + 2 uint16 + 9 uint8 (17 bytes). Confirms C++/Python parity, the vehicle
mapping, and the 'GPS integrity' Systems group (hidden until received; DETECTED shown red).
Port-independent (no link).

isHidden() is used rather than isVisible() -- an offscreen widget with no shown ancestor
reports isVisible()==False regardless of its own flag, but isHidden() reflects the explicit
setVisible() we drive."""
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

# system_errors=0, hfom=150, vfom=200, id=0, auth=3, jam=1(OK), spoof=3(DETECTED),
# raim=2(OK), corrections=8, summary=9, signal=7, post_proc=0
payload = struct.pack("<I2H9B", 0, 150, 200, 0, 3, 1, 3, 2, 8, 9, 7, 0)
frame = mavlink.frame_v2(mavlink.GNSS_INTEGRITY, payload, seq=3, sysid=1, compid=1,
                         crc_fn=mavlink.crc16_mcrf4xx)
cf = core.Parser().feed(frame)
pf = mavlink.PyParser().feed(frame)
assert len(cf) == 1 and len(pf) == 1, (len(cf), len(pf))
assert cf[0].fields == pf[0].fields, (cf[0].fields, pf[0].fields)     # C++/Python parity
d = cf[0].fields
assert d["jamming_state"] == 1 and d["spoofing_state"] == 3
assert d["raim_state"] == 2 and d["gnss_signal_quality"] == 7

# msgid 441 > 255: a v1 frame truncates the id and must not decode as 441
v1 = mavlink.frame(mavlink.GNSS_INTEGRITY, payload, seq=3, sysid=1, compid=1,
                   crc_fn=mavlink.crc16_mcrf4xx)
v1_out = core.Parser().feed(v1)
assert not v1_out or v1_out[0].msgid != 441, v1_out

v = Vehicle()
v._on_gnss_integrity(d)
assert v.have_gnss_integrity and v.gps_jamming == 1 and v.gps_spoofing == 3 and v.gps_raim == 2

sp = panels.SystemsPanel()
assert sp._gi_group.isHidden(), "group must start hidden"
sp.update_from(v)
assert not sp._gi_group.isHidden(), "group shows after GNSS_INTEGRITY"
assert sp.gi_spoof.text() == "DETECTED" and "e05050" in sp.gi_spoof.styleSheet()   # red alarm
assert sp.gi_jam.text() == "OK" and "37d67a" in sp.gi_jam.styleSheet()             # green
assert sp.gi_raim.text() == "OK" and sp.gi_sig.text() == "7/10"

# no GNSS_INTEGRITY received -> the group stays hidden (no clutter for non-supporting vehicles)
sp2 = panels.SystemsPanel()
sp2.update_from(Vehicle())
assert sp2._gi_group.isHidden()

print("GNSS PASSED")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
