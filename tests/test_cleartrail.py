#!/usr/bin/env python3
"""test_cleartrail.py -- map 'Clear trail' right-click action. The breadcrumb lives in
vehicle.trail (appended in place) and MapView holds the SAME list object, so clearing must empty
that shared source -- clearing a copy would just refill on the next fix. Pure UI, no link."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import main as m

app = QApplication([])
win = m.DroneDeck(14599)
ve = win.vehicle

# build a trail and hand it to the map (update_vehicle assigns the reference)
ve.trail.extend([(47.0, 8.0), (47.1, 8.1), (47.2, 8.2)])
win.map.update_vehicle(47.2, 8.2, 0.0, None, ve.trail)
assert win.map.trail is ve.trail, "map must share the vehicle's trail list"
assert len(win.map.trail) == 3

# the context action clears the shared source
win._on_map_context("clear_trail", 47.2, 8.2)
assert ve.trail == [], ve.trail
assert win.map.trail == [], win.map.trail          # same object -> also empty
assert win.map.trail is ve.trail                    # still the same object (cleared in place)

# a subsequent fix starts a fresh trail on the same object (no stale copy left behind)
ve.trail.append((48.0, 9.0))
assert win.map.trail == [(48.0, 9.0)]

# Vehicle.clear_trail() directly empties in place
ve.trail.extend([(1.0, 1.0), (2.0, 2.0)])
before = ve.trail
ve.clear_trail()
assert ve.trail == [] and ve.trail is before

print("CLEARTRAIL PASSED")
