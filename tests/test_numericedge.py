#!/usr/bin/env python3
"""test_numericedge.py -- numeric-edge / domain-error robustness for the values computed EVERY
refresh (a math-domain crash there would kill the app mid-flight). Guards the safe formulations:
  * haversine uses asin(min(1, sqrt(a))) -- NOT acos(dot), whose float rounding raises ValueError at
    identical/near points. This test locks that in (a refactor to acos would fail identical-point).
  * bearing uses atan2 (defined at identical points).
  * scale_nice guards mpp <= 0 before log10.
  * the full _refresh + instrument/map render survive pole / dateline / identical-to-home / antipodal
    vehicle positions and extreme zoom.
No link, no arming. Verification-only guard (no bug was found; this keeps it that way)."""
import os
import sys
import math
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mavlink
import main as m
from mapview import scale_nice

app = QApplication.instance() or QApplication([])

HALF_CIRC = 20015086.0        # metres, pole-to-pole great-circle (~half Earth circumference)

# 1) haversine: no domain error anywhere; identical -> 0; antipodal/pole-to-pole ~ half circumference
EDGES = [(0, 0), (90, 0), (-90, 0), (47.4, 8.5), (0, 180), (0, -180), (85.05, 179.999), (90, 180)]
for lat, lon in EDGES:
    assert m.haversine(lat, lon, lat, lon) == 0.0, f"identical dist != 0 at {lat},{lon}"
    # a hair away (float-rounding neighbourhood) must not raise a domain error
    d = m.haversine(lat, lon, lat + 1e-9, lon + 1e-9)
    assert math.isfinite(d) and d >= 0.0
    # symmetry
    a = m.haversine(lat, lon, 10.0, 20.0)
    b = m.haversine(10.0, 20.0, lat, lon)
    assert abs(a - b) < 1.0, (a, b)
assert abs(m.haversine(90, 0, -90, 0) - HALF_CIRC) < 5000.0        # pole to pole
assert abs(m.haversine(0, 0, 0, 180) - HALF_CIRC) < 5000.0        # antipodal on equator

# 2) bearing: finite 0..360 for every pair incl. identical (atan2, not a domain op) -----------------
for lat, lon in EDGES:
    for tgt in [(lat, lon), (0, 0), (90, 0), (-90, 0), (lat, lon + 180)]:
        br = m.bearing(lat, lon, tgt[0], tgt[1])
        assert math.isfinite(br) and 0.0 <= br < 360.0, (lat, lon, tgt, br)

# 3) scale_nice: mpp <= 0 -> empty (no log10 domain error); tiny/normal mpp -> finite --------------
assert scale_nice(0.0) == (0.0, 0.0, "")
assert scale_nice(-5.0) == (0.0, 0.0, "")
for mpp in (1e-12, 1e-6, 0.5, 5.0, 156543.0):
    metres, bar_px, label = scale_nice(mpp)
    assert math.isfinite(metres) and math.isfinite(bar_px) and isinstance(label, str)

# 4) full _refresh + render at edge vehicle positions (with home set) -> no crash ------------------
win = m.DroneDeck(14741)
win._persist = False
ve = win.vehicle


class Msg:
    __slots__ = ("msgid", "sysid", "compid", "seq", "fields")
    def __init__(self, mid, **f):
        self.msgid, self.sysid, self.compid, self.seq, self.fields = mid, 1, 1, 0, f


def gp(lat, lon):
    return Msg(mavlink.GLOBAL_POSITION_INT, lat=int(lat * 1e7), lon=int(lon * 1e7), alt=100000,
               relative_alt=50000, vx=0, vy=0, vz=0, hdg=0)


for home, veh in [((47.3977, 8.5456, 500.0), (90.0, 0.0)),      # far pole
                  ((90.0, 0.0, 0.0), (90.0, 0.0)),               # vehicle == home at the pole
                  ((0.0, 0.0, 0.0), (0.0, 180.0)),               # antipodal on the equator
                  ((0.0, 179.9999, 0.0), (0.0, -179.9999))]:     # straddling the dateline
    ve.home = home
    ve.consume([gp(*veh)])
    ve.last_heartbeat = time.monotonic()
    win._refresh()                    # haversine(home)/bearing(home)/compass/map/scale all run here
    win.adi.grab(); win.compass.grab(); win.map.grab()

# 5) map render at extreme zoom centred at the pole -> no crash ------------------------------------
for z in (0, 1, 19, 20):
    win.map.zoom = z
    win.map.center = (89.9, 0.0)
    win.map.grab()

print("NUMERICEDGE PASSED (haversine/bearing/scale + full refresh survive pole/dateline/antipodal)")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
