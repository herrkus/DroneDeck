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
win._fence_breached = False
notes.clear()
at(47.02, 8.0)                                             # ~2.2 km away -> clear
assert not notes
at(47.0, 8.0)                                             # inside -> breach
assert any("exclusion circle" in n for n in console_hits())

# inclusion circle: leaving the 100 m circle breaches
win.fence_circles = [{"lat": 47.0, "lon": 8.0, "radius": 100.0, "incl": True}]
win._fence_breached = False
notes.clear()
at(47.0, 8.0)
assert not notes
at(47.02, 8.0)
assert any("inclusion circle" in n for n in console_hits())

# no fence defined -> never warns
win.fence_inc, win.fence_exc, win.fence_circles = [], [], []
win._fence_breached = False
notes.clear()
at(48.0, 9.0)
assert not notes

print("GEOFENCE PASSED")
