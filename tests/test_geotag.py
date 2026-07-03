#!/usr/bin/env python3
"""test_geotag.py -- GeoTag survey photos from a flight log (iter157, QGC parity).

Completes the mapping pipeline: correlate CAMERA_FEEDBACK(180) events from a .tlog with a folder of
JPEGs (capture order) and write GPS EXIF into each. Verifies CAMERA_FEEDBACK parses; extract_feedback
pulls the right lat/lon/alt in order; geotag writes real EXIF whose GPS coords (incl. S/W hemispheres)
round-trip through Pillow to the source coordinates; and an event/photo count mismatch is reported (only
min(n) tagged), not silently truncated."""
import os
import sys
import struct
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import mavlink
import core
import geotag
from tlog import TlogWriter

fail = []

# three shots across hemispheres: Zurich (N,E), Sydney (S,E), New York (N,W)
EVENTS = [(47.39800, 8.54600, 500.0), (-33.85000, 151.21000, 100.0), (40.71000, -74.00000, 50.0)]


def feedback_frame(i, lat, lon, alt):
    pl = struct.pack("<QiiffffffHBBB", 1000 + i, int(lat * 1e7), int(lon * 1e7), alt, alt - 5,
                     0.0, 0.0, 1.0, 24.0, i, 1, 0, 1)
    return mavlink.frame(mavlink.CAMERA_FEEDBACK, pl, i, 1, 1, crc_fn=core.crc_extra)


tmp = tempfile.mkdtemp(prefix="geotag_")
tlog_path = os.path.join(tmp, "flight.tlog")
tw = TlogWriter(tlog_path)
for i, (lat, lon, alt) in enumerate(EVENTS):
    tw.write(feedback_frame(i, lat, lon, alt), 1000 + i)
tw.close()

# 1) extract_feedback pulls the events in order with correct coords ---------------------------------
ev = geotag.extract_feedback(tlog_path)
if len(ev) != 3:
    fail.append(f"extract_feedback should find 3 events, got {len(ev)}")
else:
    for (glat, glon, galt), (elat, elon, ealt) in zip(ev, EVENTS):
        if abs(glat - elat) > 1e-5 or abs(glon - elon) > 1e-5 or abs(galt - ealt) > 1e-3:
            fail.append(f"event mismatch: {(glat, glon, galt)} vs {(elat, elon, ealt)}")

# 2) geotag writes GPS EXIF that round-trips through Pillow to the source coords ---------------------
try:
    from PIL import Image
except ImportError:
    print("GEOTAG SKIPPED (Pillow not available)")
    sys.exit(0)

photo_dir = os.path.join(tmp, "photos")
out_dir = os.path.join(tmp, "tagged")
os.makedirs(photo_dir)
for i in range(3):
    Image.new("RGB", (8, 8), (30 * i, 60, 90)).save(os.path.join(photo_dir, f"IMG_{i:03d}.jpg"), "JPEG")

res = geotag.geotag(tlog_path, photo_dir, out_dir)
if res["tagged"] != 3 or res["events"] != 3 or res["photos"] != 3 or res["errors"]:
    fail.append(f"geotag summary wrong: {res}")


def read_gps(path):
    g = Image.open(path).getexif().get_ifd(0x8825)
    la = float(g[2][0]) + float(g[2][1]) / 60 + float(g[2][2]) / 3600
    lo = float(g[4][0]) + float(g[4][1]) / 60 + float(g[4][2]) / 3600
    return (la * (1 if g[1] == "N" else -1), lo * (1 if g[3] == "E" else -1), float(g[6]))


for i, (elat, elon, ealt) in enumerate(EVENTS):
    p = os.path.join(out_dir, f"IMG_{i:03d}.jpg")
    if not os.path.exists(p):
        fail.append(f"tagged photo {p} missing")
        continue
    glat, glon, galt = read_gps(p)
    if abs(glat - elat) > 1e-4 or abs(glon - elon) > 1e-4 or abs(galt - ealt) > 0.5:
        fail.append(f"EXIF GPS wrong for photo {i}: {(glat, glon, galt)} vs {(elat, elon, ealt)}")

# 3) event/photo count mismatch is reported, only min(n) tagged -------------------------------------
few_dir = os.path.join(tmp, "few")
os.makedirs(few_dir)
for i in range(2):                                 # 2 photos vs 3 events
    Image.new("RGB", (8, 8), (10, 10, 10)).save(os.path.join(few_dir, f"P{i}.jpg"), "JPEG")
res2 = geotag.geotag(tlog_path, few_dir, os.path.join(tmp, "few_out"))
if res2["tagged"] != 2 or res2["events"] != 3 or res2["photos"] != 2 or res2["unmatched"] != 1:
    fail.append(f"count-mismatch summary wrong: {res2}")

print("GEOTAG FAILED: " + "; ".join(fail) if fail else
      "GEOTAG PASSED (CAMERA_FEEDBACK parsed; extract_feedback pulls ordered lat/lon/alt; geotag writes "
      "GPS EXIF that round-trips through Pillow incl. S/W hemispheres; count mismatch reported, min(n) "
      "tagged, no silent truncation)")
sys.stdout.flush()
os._exit(1 if fail else 0)
