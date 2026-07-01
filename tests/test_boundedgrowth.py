#!/usr/bin/env python3
"""test_boundedgrowth.py -- a GCS must run for HOURS without leaking memory. Every container that
grows on incoming telemetry must stay bounded no matter how much data arrives. Feeds large bursts
headless and asserts each stays capped (and process RSS does not balloon):
  * vehicle.trail   -> TRAIL_MAX (4000)
  * vehicle.messages-> 200 (STATUSTEXT log)
  * self.traffic    -> TRAFFIC_MAX (2000) + stale-TTL expiry  (regression guard for the iter104 fix:
    ADSB targets were keyed by 24-bit ICAO and never pruned, so the dict AND the per-refresh sorted
    table rebuild grew without bound over a long flight / with a noisy source).
No link, no arming."""
import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mavlink
import main as m

app = QApplication.instance() or QApplication([])


class Msg:
    __slots__ = ("msgid", "sysid", "compid", "seq", "fields")
    def __init__(self, msgid, **f):
        self.msgid, self.sysid, self.compid, self.seq, self.fields = msgid, 1, 1, 0, f


def vmrss_kb():
    try:
        with open("/proc/self/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1])
    except OSError:
        pass
    return -1


win = m.DroneDeck(14702)
win._persist = False
ve = win.vehicle
rss0 = vmrss_kb()

# 1) trail: 60k moving positions -> capped at TRAIL_MAX ---------------------------------------------
for i in range(60000):
    lat = 470000000 + i * 2000          # ~each step is well past the _moved() threshold
    ve.consume([Msg(mavlink.GLOBAL_POSITION_INT, lat=lat, lon=85000000, alt=100000,
                    relative_alt=50000, vx=0, vy=0, vz=0, hdg=0)])
assert len(ve.trail) <= ve.TRAIL_MAX, f"trail unbounded: {len(ve.trail)} > {ve.TRAIL_MAX}"

# 2) messages: 50k STATUSTEXT -> capped at 200 -----------------------------------------------------
for i in range(50000):
    ve.consume([Msg(mavlink.STATUSTEXT, severity=6, text=f"status message number {i}")])
assert len(ve.messages) <= 200, f"messages unbounded: {len(ve.messages)}"

# 3) ADSB traffic: 60k DISTINCT ICAO in a flood -> capped at TRAFFIC_MAX ---------------------------
for base in range(0, 60000, 1000):
    win._update_traffic([Msg(mavlink.ADSB_VEHICLE, ICAO_address=base + k, lat=474000000,
                             lon=85000000, heading=0, altitude=100000, callsign="X")
                         for k in range(1000)])
assert len(win.traffic) <= win.TRAFFIC_MAX, f"traffic unbounded: {len(win.traffic)}"

# a single real target updating repeatedly does NOT grow the dict
win.traffic.clear()
for _ in range(500):
    win._update_traffic([Msg(mavlink.ADSB_VEHICLE, ICAO_address=0xABCDEF, lat=474000000,
                             lon=85000000, heading=90, altitude=100000, callsign="REAL")])
assert len(win.traffic) == 1, f"same ICAO should be one entry, got {len(win.traffic)}"

# stale targets expire by TTL
win.traffic[0x111] = {"lat": 47.0, "lon": 8.0, "heading": 0, "alt": 100, "callsign": "OLD",
                      "t": time.monotonic() - (win.TRAFFIC_TTL + 60)}
win._update_traffic([Msg(mavlink.ADSB_VEHICLE, ICAO_address=0x222, lat=474000000, lon=85000000,
                         heading=0, altitude=100000, callsign="NEW")])
assert 0x111 not in win.traffic and 0x222 in win.traffic, "stale ADSB not expired / fresh dropped"

# the traffic table rebuild still works after all this
win._refresh_traffic(ve)

# 4) multi-vehicle dict is protocol-bounded (sysid is a uint8) -- documents the natural cap ---------
assert len(win.vehicles) <= 256

# memory must not have ballooned across ~170k messages fed (allow generous slack for Qt/heap)
rss1 = vmrss_kb()
if rss0 > 0 and rss1 > 0:
    grew_mb = (rss1 - rss0) / 1024.0
    assert grew_mb < 200, f"RSS grew {grew_mb:.0f} MB over the burst -- possible leak"

print(f"BOUNDEDGROWTH PASSED (trail<={ve.TRAIL_MAX}, messages<=200, traffic<={win.TRAFFIC_MAX}; "
      f"RSS +{(rss1 - rss0) / 1024.0:.0f} MB)")
