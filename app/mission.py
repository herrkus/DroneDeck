"""mission.py -- MAVLink mission protocol (upload / download) and survey grids.

Implements the GCS side of the standard mission micro-protocol over MISSION_INT
messages:

  upload   GCS -> MISSION_COUNT
           veh -> MISSION_REQUEST_INT(seq)   (repeated)
           GCS -> MISSION_ITEM_INT(seq)
           veh -> MISSION_ACK
  download GCS -> MISSION_REQUEST_LIST
           veh -> MISSION_COUNT(n)
           GCS -> MISSION_REQUEST_INT(seq)   (repeated)
           veh -> MISSION_ITEM_INT(seq)
           GCS -> MISSION_ACK

Each outstanding request is guarded by a timeout and retried; the whole exchange
aborts after too many retries. Modern ArduPilot/PX4 use the _INT variants.
"""
from __future__ import annotations
import math
from dataclasses import dataclass

from PySide6.QtCore import QObject, Signal, QTimer

import mavlink

TIMEOUT_MS = 1500
MAX_RETRIES = 5


@dataclass
class MissionItem:
    seq: int
    lat: float            # degrees
    lon: float            # degrees
    alt: float            # metres (relative)
    command: int = mavlink.MAV_CMD_NAV_WAYPOINT
    frame: int = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    autocontinue: int = 1
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0

    @property
    def cmd_name(self):
        return {16: "WAYPOINT", 22: "TAKEOFF", 21: "LAND", 20: "RTL",
                17: "LOITER_UNLIM", 19: "LOITER_TIME", 82: "SPLINE_WP",
                195: "ROI", 197: "ROI_NONE", 178: "CHANGE_SPEED",
                177: "JUMP"}.get(self.command,
                                                                            f"CMD{self.command}")


def survey_grid(points, spacing_m=35.0, alt=50.0):
    """Boustrophedon (lawnmower) waypoints covering the bounding box of `points`.
    A pragmatic area survey: parallel N-S sweeps spaced `spacing_m` apart."""
    if len(points) < 2:
        return []
    lats = [p[0] for p in points]
    lons = [p[1] for p in points]
    lat0, lat1 = min(lats), max(lats)
    lon0, lon1 = min(lons), max(lons)
    dlat = spacing_m / 111320.0
    n_lines = max(2, int((lat1 - lat0) / dlat) + 1)
    items = []
    y = lat0
    left_to_right = True
    seq = 0
    for _ in range(n_lines + 1):
        if y > lat1 + 1e-9:
            break
        ends = (lon0, lon1) if left_to_right else (lon1, lon0)
        for x in ends:
            items.append(MissionItem(seq, y, x, alt))
            seq += 1
        y += dlat
        left_to_right = not left_to_right
    return items


def _ll_to_m(lat, lon, lat0, lon0):
    """Equirectangular local frame (metres east/north) about (lat0, lon0)."""
    x = math.radians(lon - lon0) * 6378137.0 * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * 6378137.0
    return x, y


def _m_to_ll(x, y, lat0, lon0):
    lat = lat0 + math.degrees(y / 6378137.0)
    lon = lon0 + math.degrees(x / (6378137.0 * math.cos(math.radians(lat0))))
    return lat, lon


def _vertex_normals(pts):
    """Unit left-normals at each polyline vertex (bisector of the adjacent segments)."""
    n = len(pts)
    out = []
    for i in range(n):
        if i == 0:
            tx, ty = pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]
        elif i == n - 1:
            tx, ty = pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1]
        else:
            ax, ay = pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]
            bx, by = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
            la = math.hypot(ax, ay) or 1.0
            lb = math.hypot(bx, by) or 1.0
            tx, ty = ax / la + bx / lb, ay / la + by / lb    # segment-direction bisector
        tl = math.hypot(tx, ty) or 1.0
        out.append((-ty / tl, tx / tl))                      # rotate unit tangent +90deg
    return out


def corridor_scan(path, width_m=60.0, spacing_m=30.0, alt=50.0):
    """Parallel passes along a polyline centerline `path` [(lat,lon), ...], covering a
    corridor of `width_m` (measured perpendicular to the path) with passes `spacing_m`
    apart. Boustrophedon order so the vehicle snakes back and forth. Returns
    MissionItems -- for linear inspection (power lines, roads, pipelines, coastlines).

    Offsetting uses per-vertex segment-bisector normals in a local metres frame: exact
    on straight runs, good on gentle bends (no miter scaling, so very sharp corners
    pinch slightly)."""
    if len(path) < 2:
        return []
    lat0, lon0 = path[0]
    pts = [_ll_to_m(la, lo, lat0, lon0) for la, lo in path]
    normals = _vertex_normals(pts)
    n_pass = max(1, int(round(width_m / max(spacing_m, 1e-6))))
    offsets = [(-width_m / 2.0) + i * (width_m / n_pass) for i in range(n_pass + 1)]
    items = []
    seq = 0
    for k, off in enumerate(offsets):
        line = [(x + nx * off, y + ny * off) for (x, y), (nx, ny) in zip(pts, normals)]
        if k % 2:
            line = list(reversed(line))                      # snake between passes
        for x, y in line:
            la, lo = _m_to_ll(x, y, lat0, lon0)
            items.append(MissionItem(seq, la, lo, alt))
            seq += 1
    return items


def structure_scan(points, radius_m=30.0, layers=3, layer_height_m=8.0,
                   base_alt=15.0, n_points=16):
    """Orbit waypoints around the centroid of `points`, `radius_m` beyond the
    structure's own extent, repeated across `layers` altitude levels (base_alt, then
    +layer_height each). Each waypoint yaws to face the centre (param4), so a nose- or
    gimbal-mounted camera captures the facade -- for towers, masts, buildings, wind
    turbines. Alternate layers reverse direction so the path spirals up without a long
    reposition. A single point gives a plain radius_m orbit."""
    if not points:
        return []
    lat0, lon0 = points[0]
    pm = [_ll_to_m(la, lo, lat0, lon0) for la, lo in points]
    cx = sum(x for x, _ in pm) / len(pm)
    cy = sum(y for _, y in pm) / len(pm)
    struct_r = max((math.hypot(x - cx, y - cy) for x, y in pm), default=0.0)
    orbit_r = radius_m + struct_r                            # standoff from the extent
    n_points = max(3, int(n_points))
    items = []
    seq = 0
    for L in range(max(1, int(layers))):
        alt = base_alt + L * layer_height_m
        order = range(n_points) if L % 2 == 0 else range(n_points - 1, -1, -1)
        for i in order:
            th = 2.0 * math.pi * i / n_points
            x = cx + orbit_r * math.cos(th)
            y = cy + orbit_r * math.sin(th)
            yaw = math.degrees(math.atan2(cx - x, cy - y)) % 360.0   # face centre (bearing from N)
            la, lo = _m_to_ll(x, y, lat0, lon0)
            items.append(MissionItem(seq, la, lo, alt, param4=yaw))
            seq += 1
    return items


def _convex_hull(pts):
    """Counter-clockwise convex hull (Andrew's monotone chain) of [(x, y)] points."""
    pts = sorted(set(pts))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _inflate_convex(hull, margin):
    """Offset each edge of a CCW convex polygon outward by `margin`; new vertices are
    the intersections of consecutive offset edges."""
    n = len(hull)
    lines = []
    for i in range(n):
        a, b = hull[i], hull[(i + 1) % n]
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy) or 1.0
        ux, uy = dx / L, dy / L
        nx, ny = uy, -ux                           # outward normal (right of travel, CCW)
        lines.append(((a[0] + nx * margin, a[1] + ny * margin), (ux, uy)))
    out = []
    for i in range(n):
        (p1, d1), (p2, d2) = lines[(i - 1) % n], lines[i]
        den = d1[0] * (-d2[1]) - d1[1] * (-d2[0])
        if abs(den) < 1e-9:
            out.append(p2)                         # parallel edges -> take the offset point
        else:
            t = ((p2[0] - p1[0]) * (-d2[1]) - (p2[1] - p1[1]) * (-d2[0])) / den
            out.append((p1[0] + t * d1[0], p1[1] + t * d1[1]))
    return out


def fence_from_mission(points, margin_m=30.0):
    """Inclusion geofence polygon around `points`: their convex hull inflated outward
    by `margin_m` (a margin box when there are fewer than 3 non-collinear points). Every
    point ends up inside with roughly margin_m of clearance -- a quick safety boundary."""
    pts = [p for p in points if not (abs(p[0]) < 1e-9 and abs(p[1]) < 1e-9)]
    if not pts:
        return []
    lat0, lon0 = pts[0]
    pm = [_ll_to_m(la, lo, lat0, lon0) for la, lo in pts]
    hull = _convex_hull(pm)
    if len(hull) < 3:                              # 1-2 points or collinear -> margin box
        xs = [x for x, _ in pm]
        ys = [y for _, y in pm]
        hull_m = [(min(xs) - margin_m, min(ys) - margin_m), (max(xs) + margin_m, min(ys) - margin_m),
                  (max(xs) + margin_m, max(ys) + margin_m), (min(xs) - margin_m, max(ys) + margin_m)]
    else:
        hull_m = _inflate_convex(hull, margin_m)
    return [_m_to_ll(x, y, lat0, lon0) for x, y in hull_m]


def validate_mission(items):
    """Advisory pre-upload checks for common mission mistakes. Returns a list of
    human-readable warning strings (empty == looks fine). The autopilot still has the
    final say -- these just catch the errors that most often bite before flight."""
    warns = []
    if not items:
        return warns
    NAV_TAKEOFF, NAV_LAND, RTL, DO_JUMP = 22, 21, 20, 177
    DO_CMDS = {177, 178, 195, 197}                  # jump, change-speed, ROI, clear-ROI
    nav = [it for it in items if it.command not in DO_CMDS and it.command != RTL]

    if nav and nav[0].command != NAV_TAKEOFF:
        warns.append("Mission does not start with a Takeoff waypoint.")
    if items[-1].command not in (NAV_LAND, RTL):
        warns.append("Mission does not end with Land or Return-to-launch.")

    n = len(items)
    for it in items:
        if it.command == DO_JUMP and not (0 <= int(it.param1) < n):
            warns.append(f"WP {it.seq}: DO_JUMP target {int(it.param1)} is out of range (0..{n - 1}).")
    airborne = [it for it in nav if it.command != NAV_LAND]   # land alt 0 is expected
    for it in airborne:
        if it.alt <= 0:
            warns.append(f"WP {it.seq}: altitude is {it.alt:.0f} m (zero or negative).")
    for a, b in zip(airborne, airborne[1:]):
        if abs(b.alt - a.alt) > 120:
            warns.append(f"WP {a.seq}->{b.seq}: large altitude change ({a.alt:.0f} -> {b.alt:.0f} m).")
    return warns


class MissionProtocol(QObject):
    progress = Signal(str)          # human-readable step
    finished = Signal(bool, str)    # ok, message
    downloaded = Signal(list)       # list[MissionItem]

    def __init__(self, link_getter, target_getter, parent=None):
        super().__init__(parent)
        self._link = link_getter
        self._target = target_getter
        self.state = "idle"
        self.items = []
        self.expected = 0
        self.next_seq = 0
        self.retries = 0
        self.mtype = 0          # 0=mission, 1=fence, 2=rally
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self._on_timeout)

    # -- helpers --------------------------------------------------------------
    @property
    def busy(self):
        return self.state != "idle"

    def _ready(self):
        link = self._link()
        return link is not None and link.is_open and link.remote is not None

    def _arm(self):
        self.timer.start(TIMEOUT_MS)

    def _done(self, ok, msg):
        self.timer.stop()
        self.state = "idle"
        self.finished.emit(ok, msg)

    # -- public actions -------------------------------------------------------
    def upload(self, items, mission_type=0):
        if not self._ready():
            self.finished.emit(False, "no vehicle connected")
            return
        if self.busy:
            return
        self.mtype = mission_type
        self.items = [MissionItem(i, *(it.lat, it.lon, it.alt), command=it.command,
                                  frame=it.frame, autocontinue=it.autocontinue,
                                  param1=it.param1, param2=it.param2,
                                  param3=it.param3, param4=it.param4)
                      for i, it in enumerate(items)]
        self.state = "upload"
        self.retries = 0
        self.progress.emit(f"uploading {len(self.items)} items")
        self._link().send_mission_count(self._target(), len(self.items), self.mtype)
        self._arm()

    def download(self, mission_type=0):
        if not self._ready():
            self.finished.emit(False, "no vehicle connected")
            return
        if self.busy:
            return
        self.mtype = mission_type
        self.items = []
        self.state = "dl_count"
        self.retries = 0
        self.progress.emit("requesting mission")
        self._link().send_mission_request_list(self._target(), self.mtype)
        self._arm()

    def clear(self, mission_type=0):
        if not self._ready():
            self.finished.emit(False, "no vehicle connected")
            return
        if self.busy:
            return
        self.mtype = mission_type
        self.state = "clear"
        self.retries = 0
        self.progress.emit("clearing mission")
        self._link().send_mission_clear(self._target(), self.mtype)
        self._arm()

    # -- inbound --------------------------------------------------------------
    def handle_messages(self, batch):
        if self.state == "idle":
            return
        for m in batch:
            self.handle(m)

    def handle(self, m):
        mid = m.msgid
        if self.state == "upload" and mid == mavlink.MISSION_REQUEST_INT:
            seq = int(m.fields.get("seq", 0))
            if 0 <= seq < len(self.items):
                self.retries = 0
                self._link().send_mission_item(self._target(), self.items[seq], self.mtype)
                self.progress.emit(f"sent item {seq + 1}/{len(self.items)}")
                self._arm()
        elif self.state == "upload" and mid == mavlink.MISSION_ACK:
            res = int(m.fields.get("type", 0))
            self._done(res == 0, "upload complete" if res == 0 else f"upload rejected (type {res})")
        elif self.state == "dl_count" and mid == mavlink.MISSION_COUNT:
            self.expected = int(m.fields.get("count", 0))
            self.items = []
            self.next_seq = 0
            self.retries = 0
            if self.expected == 0:
                self._link().send_mission_ack(self._target(), 0, self.mtype)
                self.downloaded.emit([])
                self._done(True, "no mission on vehicle")
                return
            self.state = "dl_item"
            self._link().send_mission_request_int(self._target(), 0, self.mtype)
            self._arm()
        elif self.state == "dl_item" and mid == mavlink.MISSION_ITEM_INT:
            seq = int(m.fields.get("seq", -1))
            if seq != self.next_seq:                       # out of order: re-request
                self._link().send_mission_request_int(self._target(), self.next_seq, self.mtype)
                self._arm()
                return
            self.items.append(MissionItem(
                seq, m.fields["x"] / 1e7, m.fields["y"] / 1e7, float(m.fields["z"]),
                command=int(m.fields["command"]), frame=int(m.fields["frame"]),
                autocontinue=int(m.fields["autocontinue"]),
                param1=m.fields["param1"], param2=m.fields["param2"],
                param3=m.fields["param3"], param4=m.fields["param4"]))
            self.next_seq += 1
            self.retries = 0
            if self.next_seq >= self.expected:
                self._link().send_mission_ack(self._target(), 0, self.mtype)
                self.downloaded.emit(self.items)
                self._done(True, f"downloaded {len(self.items)} items")
            else:
                self.progress.emit(f"item {self.next_seq}/{self.expected}")
                self._link().send_mission_request_int(self._target(), self.next_seq, self.mtype)
                self._arm()
        elif self.state == "clear" and mid == mavlink.MISSION_ACK:
            res = int(m.fields.get("type", 0))
            self._done(res == 0, "mission cleared" if res == 0 else f"clear rejected (type {res})")

    def _on_timeout(self):
        self.retries += 1
        if self.retries > MAX_RETRIES:
            self._done(False, "mission protocol timed out")
            return
        if not self._ready():
            self._done(False, "link lost")
            return
        tgt = self._target()
        link = self._link()
        if self.state == "upload":
            link.send_mission_count(tgt, len(self.items), self.mtype)
        elif self.state == "dl_count":
            link.send_mission_request_list(tgt, self.mtype)
        elif self.state == "dl_item":
            link.send_mission_request_int(tgt, self.next_seq, self.mtype)
        elif self.state == "clear":
            link.send_mission_clear(tgt, self.mtype)
        self.progress.emit(f"retry {self.retries}/{MAX_RETRIES}")
        self._arm()
