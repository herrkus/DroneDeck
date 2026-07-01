"""planfile.py -- read/write QGroundControl .plan mission files (JSON).

A .plan is portable between DroneDeck and QGroundControl, so a plan drawn in either
tool opens in the other. We read/write the mission section (SimpleItems); geoFence and
rallyPoints are written empty (the GCS edits those through its own tools)."""
from __future__ import annotations
import json
import math

from mission import MissionItem


def _f(v):
    """QGC stores unset params as JSON null -- coerce to 0.0. Also coerce non-finite to 0.0: a .plan
    can carry NaN/Infinity (Python's json.load accepts those literal tokens, and a hand-edited or
    corrupt file can hold them), and a NaN/Inf coordinate must never enter a mission or fence that
    could then be uploaded to a real drone. Non-numeric junk still raises, so a blatantly corrupt
    file is rejected with a clear error by the caller rather than silently loading a bad waypoint."""
    if v is None:
        return 0.0
    f = float(v)
    return f if math.isfinite(f) else 0.0


def fence_to_plan(fence_inc=None, fence_exc=None, fence_circles=None):
    """DroneDeck fence shapes -> a QGC .plan geoFence dict."""
    polygons = []
    if fence_inc:
        polygons.append({"inclusion": True, "version": 1,
                         "polygon": [[la, lo] for la, lo in fence_inc]})
    if fence_exc:
        polygons.append({"inclusion": False, "version": 1,
                         "polygon": [[la, lo] for la, lo in fence_exc]})
    circles = [{"circle": {"center": [c["lat"], c["lon"]], "radius": c["radius"]},
                "inclusion": bool(c.get("incl", True)), "version": 1}
               for c in (fence_circles or [])]
    return {"circles": circles, "polygons": polygons, "version": 2}


def plan_to_fence(data):
    """A QGC .plan dict -> (fence_inc, fence_exc, fence_circles) DroneDeck shapes."""
    gf = data.get("geoFence", {})
    inc, exc, circles = [], [], []
    for poly in gf.get("polygons", []):
        pts = [(_f(p[0]), _f(p[1])) for p in poly.get("polygon", [])]
        if poly.get("inclusion", True):
            inc = pts
        else:
            exc = pts
    for c in gf.get("circles", []):
        cir = c.get("circle", {})
        ctr = (list(cir.get("center", [])) + [0.0, 0.0])[:2]
        circles.append({"lat": _f(ctr[0]), "lon": _f(ctr[1]),
                        "radius": _f(cir.get("radius", 0.0)),
                        "incl": bool(c.get("inclusion", True))})
    return inc, exc, circles


def mission_to_plan(items, home=None, fence=None, cruise=15.0, hover=5.0,
                    firmware=12, vehicle=2):
    """list[MissionItem] (+ optional fence=(inc, exc, circles)) -> a QGC .plan dict."""
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
        "geoFence": fence_to_plan(*fence) if fence else fence_to_plan(),
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


def read_plan(path):
    """Read a .plan -> (list[MissionItem], (fence_inc, fence_exc, fence_circles))."""
    with open(path) as f:
        data = json.load(f)
    return plan_to_mission(data), plan_to_fence(data)
