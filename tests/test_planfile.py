"""Round-trip test for QGC .plan import/export (planfile.py): MissionItems -> .plan
dict -> MissionItems, a QGC-authored sample with null params, and a file round-trip.
Port-independent (no link), so it is always safe to run."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "app"))

from mission import MissionItem
import planfile

items = [MissionItem(0, 47.398, 8.546, 25.0, command=22, param1=15.0),
         MissionItem(1, 47.399, 8.547, 30.0, command=16, param4=90.0),
         MissionItem(2, 47.400, 8.548, 30.0, command=19, param1=12.0),
         MissionItem(3, 47.398, 8.546, 0.0, command=20, frame=2)]

plan = planfile.mission_to_plan(items, home=(47.398, 8.546, 488.0))
assert plan["fileType"] == "Plan" and plan["version"] == 1
assert plan["mission"]["version"] == 2 and len(plan["mission"]["items"]) == 4
assert plan["mission"]["plannedHomePosition"] == [47.398, 8.546, 488.0]

back = planfile.plan_to_mission(plan)
assert len(back) == 4
for a, b in zip(items, back):
    assert a.command == b.command and a.frame == b.frame
    assert abs(a.lat - b.lat) < 1e-9 and abs(a.lon - b.lon) < 1e-9 and abs(a.alt - b.alt) < 1e-9
    assert abs(a.param1 - b.param1) < 1e-9 and abs(a.param4 - b.param4) < 1e-9

# a QGC-authored plan (null params, string bools) parses cleanly
sample = {"fileType": "Plan", "version": 1, "mission": {"version": 2,
          "plannedHomePosition": [47.5, 8.5, 500],
          "items": [{"type": "SimpleItem", "command": 22, "frame": 3,
                     "autoContinue": True, "params": [0, 0, 0, None, 47.5, 8.5, 50]}]}}
loaded = planfile.plan_to_mission(sample)
assert len(loaded) == 1 and loaded[0].command == 22 and loaded[0].param4 == 0.0

# geoFence (inclusion polygon + circle) round-trips through the .plan too
inc = [(47.0, 8.0), (47.001, 8.0), (47.001, 8.001), (47.0, 8.001)]
circles = [{"lat": 47.002, "lon": 8.002, "radius": 50.0, "incl": True}]
plan_f = planfile.mission_to_plan(items, fence=(inc, [], circles))
assert len(plan_f["geoFence"]["polygons"]) == 1 and len(plan_f["geoFence"]["circles"]) == 1
i2, e2, c2 = planfile.plan_to_fence(plan_f)
assert i2 == inc and e2 == [] and len(c2) == 1 and abs(c2[0]["radius"] - 50.0) < 1e-9

tmp = os.path.join(os.path.dirname(__file__), "_plan_tmp.plan")
try:
    planfile.save_plan(tmp, items, fence=(inc, [], circles))
    rt, (ri, re, rc) = planfile.read_plan(tmp)
    assert len(rt) == 4 and rt[2].command == 19 and rt[3].frame == 2
    assert ri == inc and re == [] and len(rc) == 1 and rc[0]["incl"] is True
    print("PLANFILE PASSED")
finally:
    if os.path.exists(tmp):
        os.remove(tmp)
