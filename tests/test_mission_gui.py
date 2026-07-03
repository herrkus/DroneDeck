#!/usr/bin/env python3
"""test_mission_gui.py -- mission planning through the real GUI.

Drives the actual DroneDeck window: plans waypoints (as map clicks would),
uploads them to the test vehicle, wipes the local copy, downloads them back,
and confirms the plan survived the round trip. Saves a screenshot of the planned
path on the map.
"""
import os
import sys
import subprocess

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APP = os.path.join(ROOT, "app")
SIM = os.path.join(ROOT, "sim", "simulator.py")
sys.path.insert(0, APP)

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer

import main as appmain
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # for _ports
from _ports import free_udp_port
PORT = free_udp_port()

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "build", "dronedeck_mission.png")
os.makedirs(os.path.dirname(OUT), exist_ok=True)

app = QApplication([])
app.setStyleSheet(appmain.DARK_QSS)
win = appmain.DroneDeck(PORT)
win.setFixedSize(1240, 820)
win.show()

sim = subprocess.Popen([sys.executable, SIM, "--target", f"127.0.0.1:{PORT}"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

PLAN = [(54.6895, 25.2790), (54.6915, 25.2840), (54.6890, 25.2890), (54.6870, 25.2845)]
steps = []
win.mission.finished.connect(lambda ok, msg: steps.append((ok, msg)))
state = {}


def plan():
    if win.link.remote is not None:
        for la, lo in PLAN:
            win._add_waypoint(la, lo)
        state["planned"] = len(win.mission_items)


def upload():
    win._upload_mission()


def wipe_local():
    win.mission_items = []
    win._refresh_mission_view()
    state["after_wipe"] = len(win.mission_items)


def download():
    win._download_mission()


def finish():
    state["after_dl"] = len(win.mission_items)
    state["map_mission"] = len(win.map.mission)
    win.map.follow = False
    win.map.center = (54.6895, 25.2840)
    win.map.set_zoom(15)
    try:
        win.grab().save(OUT)
    except Exception as e:
        state["grab_error"] = repr(e)
    app.quit()


QTimer.singleShot(1000, plan)
QTimer.singleShot(1500, upload)
QTimer.singleShot(2600, wipe_local)
QTimer.singleShot(2800, download)
QTimer.singleShot(4600, finish)
app.exec()

sim.terminate()
try:
    sim.wait(timeout=2)
except Exception:
    sim.kill()

print("mission GUI results:")
for k, v in state.items():
    print(f"  {k:12}: {v}")
print("protocol steps:")
for ok, msg in steps:
    print(f"  {'ok ' if ok else 'ERR'} {msg}")

fail = []
if state.get("planned") != len(PLAN):
    fail.append(f"planned {state.get('planned')} != {len(PLAN)}")
if state.get("after_wipe") != 0:
    fail.append("local wipe failed")
if state.get("after_dl") != len(PLAN):
    fail.append(f"download repopulated {state.get('after_dl')} != {len(PLAN)}")
if state.get("map_mission") != len(PLAN):
    fail.append("map did not draw mission")
if not steps or not all(ok for ok, _ in steps):
    fail.append("a protocol step failed")

print(f"screenshot: {OUT}")
print("MISSION GUI FAILED: " + "; ".join(fail) if fail else "MISSION GUI PASSED")
sys.stdout.flush()
os._exit(1 if fail else 0)
