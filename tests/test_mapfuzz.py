#!/usr/bin/env python3
"""test_mapfuzz.py -- map rendering robustness against bad geometry. A corrupt / hand-edited .plan,
a broken GPS, or a fit-to-mission over garbage waypoints can present NaN/Inf/out-of-range lat/lon.
A non-finite tile or pixel value crashes the paintEvent: int(NaN) -> ValueError, int(Inf) ->
OverflowError, round(NaN) -> ValueError, or a NaN QPointF SEGFAULTs the native painter (all observed
before the iter96 fix). Renders the map (grab() forces the paintEvent) with adversarial center,
vehicle, trail, waypoints, fences, rally and home, asserting it never crashes. No link, no arming."""
import os
import sys
import math

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mapview
from mapview import MapView, deg2num

app = QApplication.instance() or QApplication([])

NAN, INF = float("nan"), float("inf")
BADS = (NAN, INF, -INF, 1e309 if False else 1e308, -1e308, 95.0, 1900.0, -1900.0)

# deg2num must return finite tile coords for ANY input (this is what protects every draw path)
for lat in BADS + (0.0, 47.0):
    for lon in BADS + (0.0, 8.0):
        x, y = deg2num(lat, lon, 15)
        assert math.isfinite(x) and math.isfinite(y), f"deg2num({lat},{lon}) -> {x},{y}"

# each map surface, rendered with non-finite / out-of-range values, must not crash the paintEvent
for bad in (NAN, INF, -INF):
    mv = MapView(); mv.resize(360, 280)

    mv.center = (bad, bad); mv.grab()                     # projection + graticule + scale bar
    mv.center = (bad, 8.0); mv.grab()
    mv.center = (47.0, bad); mv.grab()

    mv.center = (47.0, 8.0)
    mv.veh = (bad, bad, bad); mv.grab()                   # vehicle marker (SIGSEGV'd before the fix)

    mv.trail = [(47.0, 8.0), (bad, bad), (48.0, bad), (bad, 9.0)]; mv.grab()

    mv.set_mission([(bad, bad), (47.0, 8.0), (bad, 9.0)]); mv.grab()
    mv.set_rally([(bad, bad), (47.0, 8.0)]); mv.grab()
    mv.set_fence_shapes([(bad, bad), (47.0, 8.0), (48.0, 9.0)], [(bad, 8.0)],
                        [{"lat": bad, "lon": bad, "radius": bad, "incl": True},
                         {"lat": 47.0, "lon": 8.0, "radius": bad, "incl": False}])
    mv.grab()
    mv.home = (bad, bad); mv.grab()

# everything bad at once
mv = MapView(); mv.resize(360, 280)
mv.center = (NAN, INF); mv.veh = (NAN, NAN, NAN)
mv.trail = [(NAN, NAN), (INF, 8.0)]
mv.set_mission([(NAN, NAN), (INF, INF)]); mv.set_rally([(NAN, NAN)]); mv.home = (NAN, NAN)
mv.set_fence_shapes([(NAN, NAN)], [], [{"lat": NAN, "lon": NAN, "radius": NAN, "incl": False}])
mv.grab()

# DroneDeck-level: an out-of-range GPS position through the vehicle must not crash the live map
import main as m
win = m.DroneDeck(14588)
win._persist = False


class Msg:
    __slots__ = ("msgid", "sysid", "compid", "seq", "fields")
    def __init__(self, msgid, **f):
        self.msgid, self.sysid, self.compid, self.seq, self.fields = msgid, 1, 1, 0, f


import mavlink
import time
win.vehicle.consume([Msg(mavlink.GLOBAL_POSITION_INT, lat=950000000, lon=1900000000,
                         alt=0, relative_alt=0, vx=0, vy=0, vz=0, hdg=65535)])
win.map.veh = (95.0, 190.0, 655.35)
win.vehicle.last_heartbeat = time.monotonic()
win._refresh()
win.map.grab()

print("MAPFUZZ PASSED (map survives NaN/Inf/out-of-range geometry)")
