#!/usr/bin/env python3
"""test_odometer.py -- cumulative distance-flown odometer. Feeds GLOBAL_POSITION_INT fixes
and checks the vehicle accumulates ground track, rejects GPS jitter, and resyncs (without
counting) on a teleport-sized jump. Port-independent (no link)."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import panels
from vehicle import Vehicle, _haversine

app = QApplication([])
v = Vehicle()


def pos(lat, lon):
    v._on_global_position({"lat": int(lat * 1e7), "lon": int(lon * 1e7), "alt": 0,
                           "relative_alt": 0, "hdg": 0, "vx": 0, "vy": 0, "vz": 0})


# first fix sets the reference, distance stays 0
pos(47.0, 8.0)
assert v.distance_traveled == 0.0

# two ~111 m northward steps accumulate to their great-circle sum
pos(47.001, 8.0)
pos(47.002, 8.0)
expect = _haversine(47.0, 8.0, 47.001, 8.0) + _haversine(47.001, 8.0, 47.002, 8.0)
assert abs(v.distance_traveled - expect) < 1.0, (v.distance_traveled, expect)

# sub-0.5 m jitter does not inflate the total
base = v.distance_traveled
for _ in range(50):
    pos(47.002 + 1e-7, 8.0)
assert abs(v.distance_traveled - base) < 0.5

# a >=1 km jump resyncs without counting; movement from the new point resumes counting
pos(48.5, 9.5)
assert abs(v.distance_traveled - base) < 1.0
pos(48.501, 9.5)
assert v.distance_traveled > base + 100

# the NAVIGATION panel row reflects the nav-dict odometer string (or '--' when absent)
tp = panels.TelemetryPanel()
tp.update_all(v, "UDP", 20.0, 100, 0, {"odometer": "1.2 km"})
assert tp.v["odometer"].text() == "1.2 km"
tp.update_all(v, "UDP", 20.0, 100, 0, {})
assert tp.v["odometer"].text() == "--"

print("ODOMETER PASSED")
