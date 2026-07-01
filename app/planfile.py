"""planfile.py -- read/write QGroundControl .plan mission files (JSON).

A .plan is portable between DroneDeck and QGroundControl, so a plan drawn in either
tool opens in the other. We read/write the mission section (SimpleItems); geoFence and
rallyPoints are written empty (the GCS edits those through its own tools)."""
from __future__ import annotations
import json

from mission import MissionItem


def _f(v):
    """QGC stores unset params as JSON null -- coerce to 0.0."""
    return float(v) if v is not None else 0.0


def mission_to_plan(items, home=None, cruise=15.0, hover=5.0, firmware=12, vehicle=2):
    """list[MissionItem] -> a QGC .plan dict (fileType 'Plan', version 1)."""
    plan_items = []
    for i, it in enumerate(items):
        plan_items.append({
            "AMSLAltAboveTerrain": None,
            "Altitude": it.alt,
            "AltitudeMode": 1,
            "autoContinue": bool(it.autocontinue),
            "command": it.command,
            "doJumpId": i + 1,
            "frame": it.frame,
            "params": [it.param1, it.param2, it.param3, it.param4, it.lat, it.lon, it.alt],
            "type": "SimpleItem",
        })
    if home is None:
        home = (items[0].lat, items[0].lon, 0.0) if items else (0.0, 0.0, 0.0)
    home = list(home) + [0.0] * (3 - len(home))
    return {
        "fileType": "Plan",
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "groundStation": "DroneDeck",
        "mission": {
            "cruiseSpeed": cruise,
            "firmwareType": firmware,
            "globalPlanAltitudeMode": 1,
            "hoverSpeed": hover,
            "items": plan_items,
            "plannedHomePosition": [home[0], home[1], home[2]],
            "vehicleType": vehicle,
            "version": 2,
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1,
    }


def plan_to_mission(data):
    """A QGC .plan dict -> list[MissionItem]. Flattens SimpleItems, and expands a
    ComplexItem's precomputed child SimpleItems (survey/corridor transects) when present."""
    mission = data.get("mission", data)             # tolerate a bare mission dict
    out = []
    seq = 0
    for it in mission.get("items", []):
        simples = [it] if it.get("type") == "SimpleItem" else it.get("Items", [])
        for s in simples:
            if s.get("type") != "SimpleItem":
                continue
            p = (list(s.get("params", [])) + [None] * 7)[:7]
            out.append(MissionItem(
                seq, lat=_f(p[4]), lon=_f(p[5]), alt=_f(p[6]),
                command=int(s.get("command", 16)), frame=int(s.get("frame", 3)),
                autocontinue=1 if s.get("autoContinue", True) else 0,
                param1=_f(p[0]), param2=_f(p[1]), param3=_f(p[2]), param4=_f(p[3])))
            seq += 1
    return out


def save_plan(path, items, **kw):
    with open(path, "w") as f:
        json.dump(mission_to_plan(items, **kw), f, indent=4)


def load_plan(path):
    with open(path) as f:
        return plan_to_mission(json.load(f))
