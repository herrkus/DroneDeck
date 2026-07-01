#!/usr/bin/env python3
"""test_failsafe.py -- MAV_STATE failsafe emphasis: the FLIGHT 'status' cell turns red for
Critical/Emergency, and a one-shot console note + toast fires on TRANSITION into the
failsafe band (not on every update). Uses an isolated UDP port and never connects."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import main as appmain
import panels
from vehicle import Vehicle

app = QApplication([])
win = appmain.DroneDeck(14599)          # isolated port; test never feeds it a link

notes, toasts = [], []
win.console.add_note = lambda text, color=None: notes.append((text, color))
win._notify = lambda text, color="#e0a030", ms=7000: toasts.append((text, color))

ve = Vehicle()
ve.sysid = 7


def step(s):
    ve.system_status = s
    win._check_failsafe(ve)


# normal states never alarm
for s in (3, 4, 3):
    step(s)
assert not notes and not toasts, (notes, toasts)

# entering Critical -> exactly one alarm
step(5)
assert len(notes) == 1 and len(toasts) == 1 and "CRITICAL" in notes[0][0], notes

# staying critical / worsening to emergency does NOT re-fire (already in failsafe band)
step(5)
step(6)
assert len(notes) == 1, "must not re-fire while remaining in the failsafe band"

# recover, then re-enter -> re-arms and fires again
step(3)
step(6)
assert len(notes) == 2 and "EMERGENCY" in notes[1][0], notes

# the FLIGHT 'status' cell is red for >=5, uncoloured otherwise
tp = panels.TelemetryPanel()
ve.system_status = 6
tp.update_all(ve, "UDP", 20.0, 100, 0)
assert "e05050" in tp.v["status"].styleSheet(), tp.v["status"].styleSheet()
ve.system_status = 3
tp.update_all(ve, "UDP", 20.0, 100, 0)
assert "e05050" not in tp.v["status"].styleSheet()

print("alarms:", [n[0] for n in notes])
print("FAILSAFE PASSED")
