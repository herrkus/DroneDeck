#!/usr/bin/env python3
"""test_mission_edit.py -- waypoint editing (move / delete / reorder) in the GUI.

Pure local editing -- no vehicle needed. Drives the real handlers the map and the
mission list call, and checks the model and the map stay consistent.
"""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # for _ports
from _ports import free_udp_port
PORT = free_udp_port()

from PySide6.QtWidgets import QApplication
import main as appmain

app = QApplication([])
win = appmain.DroneDeck(PORT)

fail = []

# plan 4 waypoints
for la, lo in [(54.690, 25.280), (54.700, 25.290), (54.710, 25.300), (54.720, 25.310)]:
    win._add_waypoint(la, lo)
if len(win.mission_items) != 4:
    fail.append(f"add: expected 4, got {len(win.mission_items)}")
if len(win.map.mission) != 4:
    fail.append("map not drawing 4 waypoints")

# drag WP 1 to a new spot (as the map would emit)
win.mission_list.setCurrentRow(1)
win._wp_moved(1, 54.7055, 25.2955)
if abs(win.mission_items[1].lat - 54.7055) > 1e-6 or abs(win.mission_items[1].lon - 25.2955) > 1e-6:
    fail.append("drag did not move the waypoint")

# selecting on the map selects the list row
win._wp_selected(2)
if win.mission_list.currentRow() != 2:
    fail.append("map selection did not select the list row")

# delete WP 2 -> 3 left, contiguous seqs
win.mission_list.setCurrentRow(2)
win._wp_delete()
if len(win.mission_items) != 3 or [w.seq for w in win.mission_items] != [0, 1, 2]:
    fail.append(f"delete/renumber wrong: {[w.seq for w in win.mission_items]}")

# reorder: move row 0 down, then verify the first two swapped
first_before = (win.mission_items[0].lat, win.mission_items[0].lon)
win.mission_list.setCurrentRow(0)
win._wp_down()
if (win.mission_items[1].lat, win.mission_items[1].lon) != first_before:
    fail.append("reorder (down) did not swap")
if [w.seq for w in win.mission_items] != [0, 1, 2]:
    fail.append("reorder left seqs non-contiguous")

# edit altitude directly (dialog is modal; exercise the model+row update)
win.mission_items[0].alt = 123.0
win._update_wp_row(0)
if "123" not in win.mission_list.item(0).text():
    fail.append("altitude edit not reflected in the list")

print(f"waypoints: {[(round(w.lat,4), round(w.lon,4), w.alt) for w in win.mission_items]}")
print("MISSION EDIT FAILED: " + "; ".join(fail) if fail else "MISSION EDIT PASSED")
sys.stdout.flush()
os._exit(1 if fail else 0)
