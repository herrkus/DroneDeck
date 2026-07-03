#!/usr/bin/env python3
"""test_geotagdialog.py -- GeoTag Images dialog (iter158, QGC parity).

Step 2/2 of GeoTag: the dialog wrapping geotag.geotag(). Verifies _run tags real photos from a synthetic
tlog and reports the summary, and that the input-validation paths (missing fields / missing files)
surface a clear message instead of crashing."""
import os
import sys
import struct
import tempfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import mavlink
import core
from tlog import TlogWriter

fail = []
from PySide6.QtWidgets import QApplication
app = QApplication.instance() or QApplication([])

try:
    from PIL import Image
except ImportError:
    print("GEOTAGDIALOG SKIPPED (Pillow not available)")
    sys.exit(0)
from geotagdialog import GeotagDialog

EVENTS = [(47.398, 8.546, 500.0), (40.71, -74.0, 50.0)]
tmp = tempfile.mkdtemp(prefix="geotagdlg_")
tlog_path = os.path.join(tmp, "f.tlog")
tw = TlogWriter(tlog_path)
for i, (lat, lon, alt) in enumerate(EVENTS):
    pl = struct.pack("<QiiffffffHBBB", 1000 + i, int(lat * 1e7), int(lon * 1e7), alt, alt,
                     0.0, 0.0, 0.0, 24.0, i, 1, 0, 1)
    tw.write(mavlink.frame(mavlink.CAMERA_FEEDBACK, pl, i, 1, 1, crc_fn=core.crc_extra), 1000 + i)
tw.close()

photo_dir = os.path.join(tmp, "ph")
out_dir = os.path.join(tmp, "out")
os.makedirs(photo_dir)
for i in range(2):
    Image.new("RGB", (8, 8), (20, 40, 60)).save(os.path.join(photo_dir, f"IMG_{i}.jpg"), "JPEG")

dlg = GeotagDialog(tmp)

# 1) a full run tags the photos and reports the summary ---------------------------------------------
dlg._run(tlog_path, photo_dir, out_dir)
st = dlg.status.text()
if "Tagged 2 of 2" not in st or "2 camera event" not in st:
    fail.append(f"run summary wrong: {st!r}")
for i in range(2):
    if not os.path.exists(os.path.join(out_dir, f"IMG_{i}.jpg")):
        fail.append(f"output photo IMG_{i}.jpg missing")
# the written EXIF actually carries GPS
g = Image.open(os.path.join(out_dir, "IMG_0.jpg")).getexif().get_ifd(0x8825)
if 2 not in g or 4 not in g:
    fail.append("output photo has no GPS EXIF")

# 2) missing fields / files are reported, not crashed ----------------------------------------------
dlg._run("", "", "")
if "Please set" not in dlg.status.text():
    fail.append(f"empty paths should prompt, got {dlg.status.text()!r}")
dlg._run(os.path.join(tmp, "nope.tlog"), photo_dir, out_dir)
if "not found" not in dlg.status.text():
    fail.append(f"missing log should report not-found, got {dlg.status.text()!r}")

print("GEOTAGDIALOG FAILED: " + "; ".join(fail) if fail else
      "GEOTAGDIALOG PASSED (dialog _run tags photos from a synthetic tlog + reports the summary + writes "
      "GPS EXIF; missing fields/files reported cleanly)")
sys.stdout.flush()
os._exit(1 if fail else 0)
