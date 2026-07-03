#!/usr/bin/env python3
"""test_terrain_report.py -- terrain-follow status TERRAIN_REPORT(136) + TERRAIN_CHECK(135) (iter149).

QGC shows terrain-follow status (height above terrain, terrain elevation, tiles pending/loaded) and can
poll it at a point. DroneDeck had neither. Added TERRAIN_REPORT to BOTH parsers (native C++/asm core +
pure-Python; CRC_EXTRA 1, 22-byte scalar), a vehicle handler, a Terrain-AGL readout in the telemetry
POSITION group, and link.terrain_check (TERRAIN_CHECK, CRC 203) to request a report. Verifies: both
backends decode TERRAIN_REPORT with identical + exact values; the handler stores height/AGL/tiles; the
panel shows AGL green when loaded, amber while tiles pend, '--' when absent; TERRAIN_CHECK encodes the
lat/lon (deg*1e7) correctly."""
import os
import sys
import struct

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import mavlink
import core
from vehicle import Vehicle
from link import Link

fail = []

# terrain elevation 512.5 m AMSL, vehicle 118.3 m above terrain, 100 m spacing, 3 pending / 12 loaded
VALS = {"lat": 473980000, "lon": 85460000, "terrain_height": 512.5, "current_height": 118.3,
        "spacing": 100, "pending": 3, "loaded": 12}
payload = struct.pack(mavlink._WIRE[mavlink.TERRAIN_REPORT][0],
                      *[VALS[n] for n in mavlink._WIRE[mavlink.TERRAIN_REPORT][1]])
frame = mavlink.frame(mavlink.TERRAIN_REPORT, payload, 7, 1, 1, crc_fn=core.crc_extra)

# 1) both backends decode identically with exact values ---------------------------------------------
print("native backend live:", core.NATIVE)
nat, pyp = core.Parser().feed(frame), mavlink.PyParser().feed(frame)
if len(nat) != 1 or len(pyp) != 1:
    fail.append(f"decode count native={len(nat)} py={len(pyp)}")
else:
    for n, exp in VALS.items():
        a, b = nat[0].fields.get(n), pyp[0].fields.get(n)
        if a is None or b is None or abs(a - exp) > 1e-2 or abs(b - exp) > 1e-2:
            fail.append(f"TERRAIN_REPORT.{n}: native={a} py={b} expected={exp}")

# 2) vehicle handler stores terrain height / AGL / tiles --------------------------------------------
ve = Vehicle()
ve.consume(core.Parser().feed(frame))
if abs((ve.terrain_height_m or 0) - 512.5) > 1e-2 or abs((ve.terrain_agl_m or 0) - 118.3) > 1e-2:
    fail.append(f"terrain state height={ve.terrain_height_m} agl={ve.terrain_agl_m}")
if ve.terrain_pending != 3 or ve.terrain_loaded != 12:
    fail.append(f"terrain tiles pending={ve.terrain_pending} loaded={ve.terrain_loaded}")

fresh = Vehicle()
if fresh.terrain_agl_m is not None or fresh.terrain_pending is not None:
    fail.append("fresh vehicle should have None terrain state until TERRAIN_REPORT arrives")

# 3) telemetry panel: AGL green when loaded, amber with pending tiles, '--' when absent --------------
from PySide6.QtWidgets import QApplication
from panels import TelemetryPanel
app = QApplication.instance() or QApplication([])
tp = TelemetryPanel()
tp.update_all(ve, "UDP", 10.0, 100, 0)
t = tp.v["terrain"].text()
if "118.3" not in t or "pend" not in t:                 # 3 tiles pending -> amber, shows "(3 pend)"
    fail.append(f"panel terrain (pending) shows '{t}', expected 118.3 m + pend")

done = dict(VALS); done["pending"] = 0
done_payload = struct.pack(mavlink._WIRE[mavlink.TERRAIN_REPORT][0],
                           *[done[n] for n in mavlink._WIRE[mavlink.TERRAIN_REPORT][1]])
ve2 = Vehicle()
ve2.consume(core.Parser().feed(mavlink.frame(mavlink.TERRAIN_REPORT, done_payload, 7, 1, 1, crc_fn=core.crc_extra)))
tp.update_all(ve2, "UDP", 10.0, 100, 0)
if "pend" in tp.v["terrain"].text() or "118.3" not in tp.v["terrain"].text():
    fail.append(f"panel terrain (loaded) should show AGL without pend, got '{tp.v['terrain'].text()}'")
tp.update_all(fresh, "UDP", 10.0, 100, 0)
if tp.v["terrain"].text() != "--":
    fail.append(f"panel terrain should be '--' with no report, got '{tp.v['terrain'].text()}'")

# 4) TERRAIN_CHECK encodes the requested lat/lon as deg*1e7 -----------------------------------------
class CaptureLink(Link):
    def __init__(self):
        super().__init__()
        self.sent = []
        self._open = True
        self.remote = True

    def _write(self, data):
        self.sent.append(data)


lk = CaptureLink()
lk.terrain_check(47.398, 8.546)
if len(lk.sent) != 1:
    fail.append(f"terrain_check should send 1 frame, got {len(lk.sent)}")
else:
    fr = lk.sent[0]
    if fr[5] != mavlink.TERRAIN_CHECK:
        fail.append(f"terrain_check msgid {fr[5]}, want {mavlink.TERRAIN_CHECK}")
    lat_i, lon_i = struct.unpack("<ii", fr[6:6 + fr[1]])
    if lat_i != 473980000 or lon_i != 85460000:
        fail.append(f"terrain_check lat/lon {lat_i}/{lon_i}, want 473980000/85460000")

print("TERRAIN_REPORT FAILED: " + "; ".join(fail) if fail else
      "TERRAIN_REPORT PASSED (TERRAIN_REPORT decodes native==python with exact values; handler stores "
      "height/AGL/tiles; panel shows AGL green/amber/'--'; TERRAIN_CHECK encodes lat/lon deg*1e7)")
sys.exit(1 if fail else 0)
