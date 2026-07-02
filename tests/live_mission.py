#!/usr/bin/env python3
"""live_mission.py -- END-TO-END mission EXECUTION verification vs real PX4 (udp:14550).

Proves the core GCS path a mission actually flies: upload a small mission (takeoff -> waypoint ->
land), arm, switch to AUTO mission, and watch the vehicle STATE progress -- current waypoint advances,
it climbs, reaches the waypoint, then lands + auto-disarms. Non-destructive: saves + restores the
vehicle's existing mission. Always lands + disarms in a finally block. SKIPs (exit 0) with no vehicle.
NOT in run_all.sh.

Usage:  QT_QPA_PLATFORM=offscreen python3 tests/live_mission.py [udp_port]
"""
import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QCoreApplication
import mavlink
from mission import MissionItem

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 14550
app = QApplication.instance() or QApplication([])
import main as m

win = m.DroneDeck(PORT)
win._persist = False
win._connect()
ve = win.vehicle
fails = []


def pump(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        QCoreApplication.processEvents()
        time.sleep(0.01)


def step(label, cond, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        pump(0.2)
        if cond():
            print(f"  [ok] {label} ({time.monotonic()-t0:.1f}s)")
            return True
    print(f"  [XX] {label} -- TIMEOUT {timeout}s")
    fails.append(label)
    return False


def run_mission_action(action, *a):
    box = {}
    win.mission.finished.connect(lambda ok, msg: box.update(ok=ok, msg=msg))
    action(*a)
    t0 = time.monotonic()
    while "ok" not in box and time.monotonic() - t0 < 15:
        pump(0.3)
    return box.get("ok"), box.get("msg", "TIMEOUT")


t0 = time.monotonic()
while ve.msg_count == 0 and time.monotonic() - t0 < 15:
    pump(0.5)
if ve.msg_count == 0:
    print(f"LIVE MISSION SKIPPED (no MAVLink on udp:{PORT})")
    sys.exit(0)
t0 = time.monotonic()
while ve.fix_type < 3 and time.monotonic() - t0 < 45:
    pump(0.5)
pump(8)
sysid = win._sysid()
la, lo = ve.lat, ve.lon
print(f"=== sysid={sysid} autopilot={'PX4' if ve.autopilot==mavlink.MAV_AUTOPILOT_PX4 else ve.autopilot} "
      f"fix={ve.fix_type} at {la:.6f},{lo:.6f} ===")

# save existing mission to restore later
saved = {}
win.mission.downloaded.connect(lambda items: saved.setdefault("items", items))
run_mission_action(win.mission.download, 0)
print(f"saved existing mission: {len(saved.get('items', []))} item(s)")

# build: takeoff 30 m -> waypoint ~50 m north @ 30 m -> land at start ------------------------------
REL = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
mission = [
    MissionItem(0, la, lo, 30.0, command=mavlink.MAV_CMD_NAV_TAKEOFF, frame=REL),
    MissionItem(1, la + 0.00045, lo, 30.0, command=mavlink.MAV_CMD_NAV_WAYPOINT, frame=REL),
    MissionItem(2, la, lo, 0.0, command=mavlink.MAV_CMD_NAV_LAND, frame=REL),
]

try:
    print("=== UPLOAD mission (takeoff/waypoint/land) ===")
    ok, msg = run_mission_action(win.mission.upload, mission, 0)
    print(f"  upload: {'ACCEPT' if ok else 'REJECT'} ({msg})")
    if not ok:
        fails.append("mission upload")

    print("=== ARM ===")
    armed = False
    for attempt in (1, 2, 3):
        win.link.arm(sysid, True)
        t = time.monotonic()
        while not ve.armed and time.monotonic() - t < 8:
            pump(0.2)
        if ve.armed:
            armed = True
            print(f"  [ok] ARMED (attempt {attempt})")
            break
        pump(6)
    if not armed:
        fails.append("ARM")

    if armed and ok:
        print("=== START mission (AUTO.MISSION) ===")
        cmd = mavlink.mode_command(ve.autopilot, ve.mav_type, "AUTO.MISSION") or \
            mavlink.mode_command(ve.autopilot, ve.mav_type, "Auto")
        if cmd:
            win.link.set_mode(sysid, cmd[0], cmd[1] if len(cmd) > 1 else 0)
        step("entered a mission/auto mode", lambda: "AUTO" in (ve.mode or "").upper() or
             "MISSION" in (ve.mode or "").upper(), 10)
        print(f"  mode {ve.mode}")

        # progress: climb, advance past the takeoff item, then reach/return + land + disarm
        base = ve.alt_rel
        step("climbing (mission takeoff)", lambda: ve.alt_rel > base + 8.0, 40)
        step("advanced past waypoint 0", lambda: ve.current_wp >= 1 or ve.reached_wp >= 0, 40)
        print(f"  current_wp={ve.current_wp} reached_wp={ve.reached_wp} alt={ve.alt_rel:.1f} mode={ve.mode}")
        step("auto-DISARMED after mission land", lambda: not ve.armed, 90)

finally:
    print("=== cleanup: ensure grounded + disarmed, restore saved mission ===")
    if ve.armed:
        win.link.land(sysid)
        pump(10)
        if ve.armed:
            win.link.force_disarm(sysid)
            pump(3)
    run_mission_action(win.mission.clear, 0)
    if saved.get("items"):
        run_mission_action(win.mission.upload, saved["items"], 0)
        print(f"  restored {len(saved['items'])} saved item(s)")
    print(f"  final: armed={ve.armed} alt_rel={ve.alt_rel:.1f} mode={ve.mode}")
    try:
        win.link.close()
    except Exception:
        pass

if fails:
    print("LIVE MISSION FAILED:", fails)
    sys.exit(1)
print("LIVE MISSION PASSED (upload -> arm -> AUTO mission -> climb + waypoint progress -> land + disarm)")
