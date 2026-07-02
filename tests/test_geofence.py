#!/usr/bin/env python3
"""test_geofence.py -- geofence breach warning. Fires a one-shot console note + toast when
the vehicle leaves an inclusion fence or enters an exclusion fence (polygon or circle),
edge-detected so it warns once per breach and re-arms on return. Isolated UDP port; the
test drives positions directly through _check_geofence (no link traffic)."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import main as m

app = QApplication([])

# ray-casting sanity on a unit square (lat, lon)
sq = [(0, 0), (0, 1), (1, 1), (1, 0)]
assert m._point_in_poly((0.5, 0.5), sq)
assert not m._point_in_poly((1.5, 0.5), sq)
assert not m._point_in_poly((0.5, 1.5), sq)

win = m.DroneDeck(14599)
notes = []
win.console.add_note = lambda t, c=None: notes.append(t) if "GEOFENCE" in t else None
win._notify = lambda t, c="#e0a030", ms=7000: notes.append("TOAST") if "GEOFENCE" in t else None
ve = win.vehicle
ve.have_position = True


def at(lat, lon):
    ve.lat, ve.lon = lat, lon
    win._check_geofence(ve)


def console_hits():
    return [n for n in notes if n != "TOAST"]


# inclusion polygon: leaving fires once, staying out does not re-fire, return re-arms
win.fence_inc = [(46.99, 7.99), (46.99, 8.01), (47.01, 8.01), (47.01, 7.99)]
win.fence_exc, win.fence_circles = [], []
at(47.0, 8.0)
assert not notes
at(47.05, 8.0)
assert len(console_hits()) == 1 and "inclusion" in console_hits()[0]
assert "TOAST" in notes                                   # toast fired too
at(47.06, 8.0)
assert len(console_hits()) == 1                            # no re-fire while breached
at(47.0, 8.0)
at(47.05, 8.0)
assert len(console_hits()) == 2                            # re-armed and fired again

# exclusion circle: entering the 100 m circle breaches
win.fence_inc = []
win.fence_circles = [{"lat": 47.0, "lon": 8.0, "radius": 100.0, "incl": False}]
win._fence_breached = {}
notes.clear()
at(47.02, 8.0)                                             # ~2.2 km away -> clear
assert not notes
at(47.0, 8.0)                                             # inside -> breach
assert any("exclusion circle" in n for n in console_hits())

# inclusion circle: leaving the 100 m circle breaches
win.fence_circles = [{"lat": 47.0, "lon": 8.0, "radius": 100.0, "incl": True}]
win._fence_breached = {}
notes.clear()
at(47.0, 8.0)
assert not notes
at(47.02, 8.0)
assert any("inclusion circle" in n for n in console_hits())

# no fence defined -> never warns
win.fence_inc, win.fence_exc, win.fence_circles = [], [], []
win._fence_breached = {}
notes.clear()
at(48.0, 9.0)
assert not notes

# -- adversarial robustness (iter111): _check_geofence runs EVERY _refresh, so a crash here kills the
# app mid-flight. The block above proves correctness; this proves it never crashes on junk input. -----
import math
import time

# degenerate inclusion polygons (0 / 1 / 2 vertices) -> _point_in_poly returns False (n<3), no div0
for f in ([], [(47.0, 8.0)], [(47.0, 8.0), (47.01, 8.01)]):
    win.fence_inc, win.fence_exc, win.fence_circles = f, [], []
    win._fence_breached = {}
    at(47.0, 8.0)                                             # must not raise

# NaN/inf fence vertices and NaN/inf circle radius -> comparisons just evaluate False, no crash
NAN, INF = float("nan"), float("inf")
win.fence_inc = [(NAN, 8.0), (47.0, INF), (47.01, 8.01), (NAN, NAN)]
win.fence_exc = []
win.fence_circles = [{"lat": NAN, "lon": 8.0, "radius": INF, "incl": True},
                     {"lat": 47.0, "lon": 8.0, "radius": NAN, "incl": False}]
at(47.0, 8.0)                                                 # must not raise

# NaN/inf vehicle position with a fence set -> no crash, no false breach
win.fence_inc = [(46.99, 7.99), (46.99, 8.01), (47.01, 8.01), (47.01, 7.99)]
win.fence_circles = []
for bad in ((NAN, NAN), (INF, -INF)):
    at(*bad)                                                  # must not raise

# a 20k-vertex fence stays fast (linear) -- no pathological per-refresh cost
big = [(47.0 + 0.01 * math.sin(t / 1000.0), 8.0 + 0.01 * math.cos(t / 1000.0)) for t in range(20000)]
win.fence_inc, win.fence_circles = big, []
t0 = time.monotonic()
at(47.0, 8.0)
assert time.monotonic() - t0 < 0.5, "20k-vertex fence check too slow"

print("GEOFENCE PASSED (+ iter111: degenerate/NaN fences + NaN position safe, 20k-vertex fast)")
