#!/usr/bin/env python3
"""test_flightchip.py -- flight-phase chip in the top status strip. Driven by ve.landed_state
(EXTENDED_SYS_STATE) + ve.armed: shown only when meaningful (armed, or airborne/transitioning)
so a disarmed-on-ground vehicle isn't cluttered. FLYING green, TAKING OFF / LANDING amber,
ON GROUND grey. Pure UI -- captures the chip list handed to the status strip."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

import time

from PySide6.QtWidgets import QApplication
import main as m

app = QApplication([])
win = m.DroneDeck(14599)
cap = {}
win.status_strip.set_chips = lambda chips: cap.update(chips=chips)
ve = win.vehicle
ve.last_heartbeat = time.monotonic()      # link_alive is a property (recent heartbeat -> True)


def texts():
    return [c[0] for c in cap["chips"]]


def chip(name):
    return next((c for c in cap["chips"] if c[0] == name), None)


PHASES = ("ON GROUND", "FLYING", "TAKING OFF", "LANDING")

# disarmed on the ground -> no flight chip (DISARMED/READY already say it)
ve.armed = False
ve.have_ext_state = True
ve.landed_state = 1
win._update_status_strip(ve, True, 0)
assert not any(t in PHASES for t in texts()), texts()

# armed on the ground -> ON GROUND (grey)
ve.armed = True
win._update_status_strip(ve, True, 0)
assert chip("ON GROUND") and chip("ON GROUND")[1] == "#9aa0ac"

# in air -> FLYING (green)
ve.landed_state = 2
win._update_status_strip(ve, True, 0)
assert chip("FLYING") and chip("FLYING")[1] == "#37d67a"

# takeoff / landing -> amber
ve.landed_state = 3
win._update_status_strip(ve, True, 0)
assert chip("TAKING OFF") and chip("TAKING OFF")[1] == "#e0a030"
ve.landed_state = 4
win._update_status_strip(ve, True, 0)
assert chip("LANDING") and chip("LANDING")[1] == "#e0a030"

# airborne is shown even if the armed flag is momentarily false (safety: surface flight)
ve.armed = False
ve.landed_state = 2
win._update_status_strip(ve, True, 0)
assert "FLYING" in texts()

# no EXTENDED_SYS_STATE at all -> no flight chip
ve.have_ext_state = False
ve.armed = True
win._update_status_strip(ve, True, 0)
assert not any(t in PHASES for t in texts())

print("FLIGHTCHIP PASSED")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
