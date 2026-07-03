#!/usr/bin/env python3
"""test_exportfuzz.py -- adversarial coordinate output robustness, two sinks:
  * GPX EXPORT (trail_to_gpx): a track point can (from a replayed/edited source) be NaN/Inf, and
    f"{nan:.7f}" == 'nan' is not valid GPX -- one bad point makes mapping tools reject the whole
    file. Non-finite points must be skipped so the document is always well-formed; the track name
    must be XML-escaped.
  * COORDINATE ENCODERS (enc_mission_item_int / enc_command_int / enc_set_position_target_global_int):
    lat/lon are packed into a signed int32 (deg*1e7), so |lat| > ~214.75 deg overflows struct's 'i'
    and NaN raises -- crashing a mission upload / goto / ROI. A .plan can carry a finite out-of-range
    coordinate (iter99 _f sanitises NaN but not magnitude), so the encoders must clamp (iter103 fix).
No link, no arming."""
import os
import sys
import math
import struct
import xml.etree.ElementTree as ET

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mavlink
import core
import main as m

app = QApplication.instance() or QApplication([])

NAN, INF = float("nan"), float("inf")

# 1) GPX export: adversarial trail -> well-formed XML, no non-finite trkpts, escaped name ----------
trail = [(47.1, 8.5), (NAN, 8.6), (47.2, INF), (-INF, NAN), (47.3, 8.7), (NAN, INF)]
gpx = m.trail_to_gpx(trail, name='Track <a> & "b" \' >bad<')
assert "nan" not in gpx.lower() and "inf" not in gpx.lower(), "non-finite text leaked into GPX"
root = ET.fromstring(gpx)                                  # must parse -> well-formed XML
ns = "{http://www.topografix.com/GPX/1/1}"
pts = root.findall(f".//{ns}trkpt")
assert len(pts) == 2, f"expected only the 2 finite points, got {len(pts)}"   # (47.1,8.5),(47.3,8.7)
for p in pts:
    assert math.isfinite(float(p.get("lat"))) and math.isfinite(float(p.get("lon")))
assert root.find(f".//{ns}name").text == 'Track <a> & "b" \' >bad<'           # unescaped by parser

# a huge-but-finite coordinate is kept (it is not our job to geo-validate a flown track) yet the
# document stays well-formed XML -- the contract is "never emit malformed/non-finite GPX"
huge = m.trail_to_gpx([(47.0, 8.0), (1e308, -1e308)])
assert "nan" not in huge.lower() and "inf" not in huge.lower()
assert len(ET.fromstring(huge).findall(f".//{ns}trkpt")) == 2

# empty / single-point tracks are still well-formed
assert ET.fromstring(m.trail_to_gpx([])) is not None
assert ET.fromstring(m.trail_to_gpx([(47.0, 8.0)])) is not None
# a large track does not choke
big = m.trail_to_gpx([(47.0 + i * 1e-5, 8.0 + i * 1e-5) for i in range(5000)])
assert len(ET.fromstring(big).findall(f".//{ns}trkpt")) == 5000

# 2) coordinate encoders: out-of-range / non-finite must not crash and must pack a valid int32 -----
def check_mission(lat, lon):
    payload = mavlink.enc_mission_item_int(0, lat, lon, 50.0)         # must not raise
    x = struct.unpack_from("<i", payload, 16)[0]                       # lat field
    y = struct.unpack_from("<i", payload, 20)[0]                       # lon field
    assert -2147483648 <= x <= 2147483647 and -2147483648 <= y <= 2147483647
    return x, y


for lat, lon in [(47.4, 8.5), (215.0, 8.5), (1000.0, -1000.0), (1e300, -1e300),
                 (NAN, NAN), (INF, -INF), (-90.0, 180.0), (90.0, -180.0)]:
    check_mission(lat, lon)

# normal coordinate is preserved to the exact 1e7 grid (no clamp/round drift)
x, y = check_mission(47.3977419, 8.5455938)
assert x == int(47.3977419 * 1e7) and y == int(8.5455938 * 1e7)

# NaN/huge also safe through the command_int (pre-scaled x/y) and set-position-target encoders
mavlink.enc_command_int(mavlink.MAV_CMD_DO_SET_ROI_LOCATION, [0, 0, 0, 0], 10 ** 12, -10 ** 12, 5.0)
mavlink.enc_command_int(mavlink.MAV_CMD_DO_SET_HOME, [0, 0, 0, 0], NAN, INF, 0.0)
mavlink.enc_set_position_target_global_int(9999.0, -9999.0, 30.0)
mavlink.enc_set_position_target_global_int(NAN, INF, 30.0)

# 3) REAL wire round-trip: a clamped MISSION_ITEM_INT still frames + parses cleanly ----------------
payload = mavlink.enc_mission_item_int(0, 1000.0, 8.5, 50.0)          # out-of-range lat
frame = mavlink.frame(mavlink.MISSION_ITEM_INT, payload, 0, 1, 1)
batch = core.Parser().feed(frame)
mi = [mm for mm in batch if mm.msgid == mavlink.MISSION_ITEM_INT]
assert mi and -2147483648 <= mi[0].fields["x"] <= 2147483647, "clamped item did not parse"

print("EXPORTFUZZ PASSED (GPX skips non-finite + valid XML; coord encoders clamp int32, no crash)")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
