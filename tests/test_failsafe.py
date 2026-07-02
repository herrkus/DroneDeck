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

# -- iter114: GCS-side low-battery annunciation, edge-triggered LOW then CRITICAL, hysteresis+re-arm -
bnotes, btoasts = [], []
win.console.add_note = lambda text, color=None: bnotes.append((text, color))
win._notify = lambda text, color="#e0a030", ms=7000: btoasts.append((text, color))
bv = Vehicle()
bv.sysid = 9


def bstep(rem):
    bv.battery_remaining = rem
    win._check_battery(bv)


# healthy battery never warns; -1 (no sensor / no estimate) never warns
for r in (95, 60, 31, -1):
    bstep(r)
assert not bnotes and not btoasts, (bnotes, btoasts)

# crossing below LOW (30%) -> exactly one amber LOW warning (console + toast)
bstep(28)
assert len(bnotes) == 1 and "LOW BATTERY" in bnotes[0][0] and bnotes[0][1] == "#e0a030", bnotes
assert len(btoasts) == 1, btoasts

# jittering around the threshold must NOT re-fire (hysteresis: 31 < 30+3 stays in the LOW band)
bstep(27); bstep(29); bstep(31)
assert len(bnotes) == 1, f"low-battery warning re-fired on jitter: {bnotes}"

# dropping into CRITICAL (<15%) -> one red CRITICAL warning
bstep(12)
assert len(bnotes) == 2 and "CRITICAL BATTERY" in bnotes[1][0] and bnotes[1][1] == "#e05050", bnotes

# staying critical (or a small recovery within hysteresis) does not re-fire
bstep(10); bstep(13)
assert len(bnotes) == 2, "critical warning re-fired while still critical"

# battery swap / recharge well above LOW re-arms; a later decline warns again
bstep(100)
bstep(25)
assert len(bnotes) == 3 and "LOW BATTERY" in bnotes[2][0], f"re-arm after recharge failed: {bnotes}"

print("alarms:", [n[0] for n in notes])
print("battery alarms:", [n[0] for n in bnotes])
print("FAILSAFE PASSED (+ iter114: low-battery LOW/CRITICAL annunciation, edge-triggered + re-arm)")
