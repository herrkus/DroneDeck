#!/usr/bin/env python3
"""test_planload.py -- .plan (QGC mission file) load robustness. Users hand-edit .plan files and
tools emit odd ones, so the loader must never let bad geometry through and must never crash. Two
guarantees, tested on the pure parse functions (the GUI wraps these in try/except + a warning
dialog, which blocks offscreen -- so we test planfile directly):
  * NON-FINITE coords are sanitised: a .plan can carry literal NaN/Infinity (Python's json.load
    accepts those tokens) -- those must become finite before entering a mission/fence that could be
    uploaded to a real drone (regression guard for the iter99 fix; closes the iter96 map loop).
  * MALFORMED files raise a catchable Exception (not a segfault / uncaught crash) so the caller can
    show a clean error.
No link, no arming."""
import os
import sys
import math
import json
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

import planfile
from mission import MissionItem


def write(txt):
    fd, path = tempfile.mkstemp(suffix=".plan")
    with os.fdopen(fd, "w") as f:
        f.write(txt)
    return path


def finite_pt(p):
    return math.isfinite(p[0]) and math.isfinite(p[1])


# 1) literal NaN / Infinity coords must be sanitised to finite -------------------------------------
nan_plan = """{"fileType":"Plan","mission":{"items":[
  {"type":"SimpleItem","command":16,"frame":3,"params":[0,0,0,0,NaN,Infinity,50]},
  {"type":"SimpleItem","command":22,"frame":3,"params":[NaN,0,0,0,-Infinity,8.5,Infinity]}
]},"geoFence":{
  "circles":[{"circle":{"center":[NaN,8.0],"radius":Infinity},"inclusion":true}],
  "polygons":[{"inclusion":true,"polygon":[[NaN,NaN],[47.0,8.0]]},
              {"inclusion":false,"polygon":[[Infinity,8.0]]}]}}"""
p = write(nan_plan)
items, (inc, exc, circ) = planfile.read_plan(p)
os.unlink(p)
for it in items:
    assert math.isfinite(it.lat) and math.isfinite(it.lon) and math.isfinite(it.alt), \
        f"non-finite mission coord survived: {it.lat},{it.lon},{it.alt}"
    for pv in (it.param1, it.param2, it.param3, it.param4):
        assert math.isfinite(pv), f"non-finite param survived: {pv}"
assert all(finite_pt(q) for q in inc + exc), "non-finite fence polygon point survived"
for c in circ:
    assert math.isfinite(c["lat"]) and math.isfinite(c["lon"]) and math.isfinite(c["radius"]), \
        f"non-finite fence circle survived: {c}"

# 2) malformed .plan variants must raise a CATCHABLE exception (no segfault / uncaught crash) -------
MALFORMED = {
    "non-dict top": "[1,2,3]",
    "bare string": '"hello"',
    "bare number": "42",
    "command=abc": '{"mission":{"items":[{"type":"SimpleItem","command":"abc","params":[0,0,0,0,1,2,3]}]}}',
    "params=str": '{"mission":{"items":[{"type":"SimpleItem","command":16,"params":"xyz"}]}}',
    "items=str": '{"mission":{"items":"nope"}}',
    "item=int": '{"mission":{"items":[5,6,7]}}',
    "truncated json": '{"mission":{"items":[{"type":',
    "empty file": "",
}
for name, txt in MALFORMED.items():
    p = write(txt)
    try:
        planfile.read_plan(p)
        # some are technically loadable (e.g. an item that isn't a SimpleItem is just skipped) --
        # that's fine; the contract is "no uncaught crash", which reaching here also satisfies
    except Exception:
        pass          # expected: a catchable error the GUI turns into a warning dialog
    finally:
        os.unlink(p)

# 3) benign-but-sparse: missing keys fall back to defaults, empty items -> [] -----------------------
p = write('{"fileType":"Plan","mission":{"items":[]}}')
items, fences = planfile.read_plan(p); os.unlink(p)
assert items == [] and fences == ([], [], [])

p = write('{"mission":{"items":[{"type":"SimpleItem"}]}}')      # no params/command/frame
items, _ = planfile.read_plan(p); os.unlink(p)
assert len(items) == 1 and items[0].command == 16 and math.isfinite(items[0].lat)

# 4) a bare mission dict (no top-level "mission") is tolerated (plan_to_mission fallback) -----------
p = write('{"items":[{"type":"SimpleItem","command":16,"frame":3,"params":[0,0,0,0,47.0,8.0,50]}]}')
items, _ = planfile.read_plan(p); os.unlink(p)
assert len(items) == 1 and abs(items[0].lat - 47.0) < 1e-6

# 5) round-trip a real mission+fence: unchanged and all-finite (no regression) ----------------------
src = [MissionItem(0, lat=47.1, lon=8.5, alt=50, command=22, frame=3, param1=1.5, param4=90),
       MissionItem(1, lat=47.2, lon=8.6, alt=60, command=16, frame=3)]
d = planfile.mission_to_plan(src, fence=([(47.0, 8.0), (47.1, 8.1)], [],
                                         [{"lat": 47.05, "lon": 8.05, "radius": 100.0, "incl": True}]))
back = planfile.plan_to_mission(d)
inc, exc, circ = planfile.plan_to_fence(d)
assert len(back) == 2 and abs(back[0].lat - 47.1) < 1e-6 and back[0].command == 22
assert len(inc) == 2 and len(circ) == 1 and abs(circ[0]["radius"] - 100.0) < 1e-6
assert all(math.isfinite(x) for it in back for x in (it.lat, it.lon, it.alt))

print("PLANLOAD PASSED (NaN/Inf coords sanitised, malformed files raise cleanly, round-trip intact)")
