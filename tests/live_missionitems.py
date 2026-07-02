#!/usr/bin/env python3
"""live_missionitems.py -- prove which of the loop's new mission items a REAL autopilot accepts.

QGC filters the mission palette per autopilot; DroneDeck shows all commands, so an item its firmware
doesn't support would fail the whole upload (MAV_MISSION_UNSUPPORTED). This uploads a tiny 3-item
mission per candidate to the live vehicle on udp:14550, reads the MISSION_ACK, and reports accept/
reject -- so autopilot_mission_warnings() can warn about the rejects (like the existing RTL warning).

Non-destructive: downloads the current mission first and restores it at the end. SKIPs (exit 0) if no
vehicle. NOT in run_all.sh -- needs a live autopilot.

Usage:  QT_QPA_PLATFORM=offscreen python3 tests/live_missionitems.py [udp_port]
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


def pump(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        QCoreApplication.processEvents()
        time.sleep(0.01)


def run_once(action, *a):
    """Fire a MissionProtocol action and pump until its finished(ok,msg) fires (or 12s)."""
    box = {}

    def on_fin(ok, msg):
        box["ok"], box["msg"] = ok, msg
    win.mission.finished.connect(on_fin)
    action(*a)
    t = time.monotonic()
    while "ok" not in box and time.monotonic() - t < 12:
        pump(0.3)
    try:
        win.mission.finished.disconnect(on_fin)
    except Exception:
        pass
    return box.get("ok"), box.get("msg", "TIMEOUT")


# wait for a heartbeat, else skip
t0 = time.monotonic()
while ve.msg_count == 0 and time.monotonic() - t0 < 15:
    pump(0.5)
if ve.msg_count == 0:
    print(f"LIVE MISSIONITEMS SKIPPED (no MAVLink on udp:{PORT})")
    sys.exit(0)
pump(3)
ap = "PX4" if ve.autopilot == mavlink.MAV_AUTOPILOT_PX4 else \
     "ArduPilot" if ve.autopilot == mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA else str(ve.autopilot)
print(f"=== autopilot = {ap} ===")

la = ve.lat if ve.have_position else 47.3977
lo = ve.lon if ve.have_position else 8.5456

# save the current mission so we can restore it
saved = {}
win.mission.downloaded.connect(lambda items: saved.setdefault("items", items))
run_once(win.mission.download, 0)
print(f"saved existing mission: {len(saved.get('items', []))} item(s)")


def item(seq, cmd, frame, **pp):
    it = MissionItem(seq, la + 0.0002 * seq, lo, 30.0, command=cmd, frame=frame)
    for k, v in pp.items():
        setattr(it, k, v)
    return it


REL = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT      # 6, what a normal waypoint uses
MIS = 2                                              # MAV_FRAME_MISSION, for no-position items
cands = [
    ("baseline [wp,wp,wp]", None, None, {}),
    ("LOITER_TURNS 18", 18, REL, dict(param1=2, param3=30)),
    ("DELAY 93", 93, MIS, dict(param1=5)),
    ("SET_SERVO 183", 183, MIS, dict(param1=5, param2=1600)),
    ("CONDITION_YAW 115", 115, MIS, dict(param1=90)),
    ("CAM_TRIGG_DIST 206", 206, MIS, dict(param1=25)),
    ("SPLINE_WP 82", 82, REL, {}),
    ("ROI 195 (georef)", 195, REL, dict(param1=0)),   # DO_SET_ROI_LOCATION needs a real position
    ("JUMP 177", 177, MIS, dict(param1=0, param2=1)),
    ("LOITER_TO_ALT 31", 31, REL, dict(param1=0, param2=80)),   # radius in param2, alt in z
]
print("=== per-item upload acceptance ===")
verdict = {}
for name, cmd, frame, pp in cands:
    mid = item(1, cmd, frame, **pp) if cmd is not None else item(1, 16, REL)
    items = [item(0, 16, REL), mid, item(2, 16, REL)]
    ok, msg = run_once(win.mission.upload, items, 0)
    verdict[name] = ok
    print(f"  {name:22s} -> {'ACCEPT' if ok else 'REJECT'}  ({msg})")
    pump(1)

# restore
print("=== restore ===")
run_once(win.mission.clear, 0)
if saved.get("items"):
    ok, msg = run_once(win.mission.upload, saved["items"], 0)
    print(f"  restored {len(saved['items'])} saved item(s): {ok} ({msg})")
else:
    print("  (no prior mission to restore; left cleared)")

try:
    win.link.close()
except Exception:
    pass

rejects = [n for n, ok in verdict.items() if ok is False and n != "baseline [wp,wp,wp]"]
print("SUMMARY: rejects =", rejects if rejects else "none (all new items accepted)")
