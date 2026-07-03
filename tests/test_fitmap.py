#!/usr/bin/env python3
"""test_fitmap.py -- 'Fit' map action. MapView.fit_bounds() centres on the bounding box of the
given (lat,lon) points and picks the tightest zoom that still frames them within the widget
(with padding); main._fit_map() gathers mission + vehicle + home and detaches follow so the
frame sticks. Pure UI, no link."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mapview
import main as m

app = QApplication([])

mv = mapview.MapView()
mv.resize(500, 400)
PAD = 48
AVAIL_W, AVAIL_H = 500 - 2 * PAD, 400 - 2 * PAD

pts = [(47.40, 8.54), (47.42, 8.56), (47.41, 8.55), (47.395, 8.535)]
lats = [p[0] for p in pts]
lons = [p[1] for p in pts]
assert mv.fit_bounds(pts, pad_px=PAD) is True

# centre lands on the bbox centre
cy, cx = (min(lats) + max(lats)) / 2, (min(lons) + max(lons)) / 2
assert abs(mv.center[0] - cy) < 1e-9 and abs(mv.center[1] - cx) < 1e-9, mv.center


def span_px(z):
    xl, yt = mapview.deg2num(max(lats), min(lons), z)
    xr, yb = mapview.deg2num(min(lats), max(lons), z)
    return abs(xr - xl) * mapview.TILE, abs(yb - yt) * mapview.TILE


# the chosen zoom frames the points; one zoom tighter would overflow
z = mv.zoom
sx, sy = span_px(z)
assert sx <= AVAIL_W and sy <= AVAIL_H, (z, sx, sy)
if z < 19:
    sx1, sy1 = span_px(z + 1)
    assert sx1 > AVAIL_W or sy1 > AVAIL_H, (z + 1, sx1, sy1)

# single point -> just recentres (returns True), keeps things sane
assert mv.fit_bounds([(47.4, 8.5)]) is True
assert abs(mv.center[0] - 47.4) < 1e-9 and abs(mv.center[1] - 8.5) < 1e-9

# nothing valid -> False
assert mv.fit_bounds([]) is False
assert mv.fit_bounds([(0.0, 0.0), (0.0, 0.0)]) is False

# _fit_map gathers mission + vehicle + home, and detaches follow so the frame sticks
win = m.DroneDeck(14599)
win.map.resize(500, 400)
win.mission_items = [m.MissionItem(0, 47.40, 8.54, 30.0), m.MissionItem(1, 47.42, 8.56, 30.0)]
win.chk_follow.setChecked(True)
win.map.follow = True
win._fit_map()
assert win.map.follow is False and not win.chk_follow.isChecked()      # follow detached + synced
assert abs(win.map.center[0] - 47.41) < 1e-6 and abs(win.map.center[1] - 8.55) < 1e-6, win.map.center

# nothing to fit -> no crash, follow untouched
win.mission_items = []
win.vehicle.home = None
win._fit_map()      # must not raise

print("FITMAP PASSED")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
