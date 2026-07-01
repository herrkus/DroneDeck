#!/usr/bin/env python3
"""test_takeoffalt.py -- remembered takeoff altitude. The Takeoff dialog now pre-fills with the
last-used altitude (default 25 m) instead of resetting to a constant, and the value persists
across runs via QSettings. Pure UI; QInputDialog is stubbed, settings go to a temp INI so the
user's real config is untouched."""
import os
import sys
import tempfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication
import main as m

app = QApplication([])
win = m.DroneDeck(14599)

calls = []


class FakeLink:
    def takeoff(self, sysid, alt, lat, lon):
        calls.append((sysid, alt, lat, lon))


win.link = FakeLink()
win._has_vehicle = lambda: True
win._sysid = lambda: 1

seen = {}


def fake_ok(parent, title, label, value, lo, hi, dec):
    seen["default"] = value
    return (42.0, True)          # user accepts 42 m


m.QInputDialog.getDouble = staticmethod(fake_ok)

# fresh install defaults to 25 m, and the dialog is pre-filled with it
assert win._takeoff_alt == 25.0
win._takeoff()
assert seen["default"] == 25.0, seen
assert calls and abs(calls[-1][1] - 42.0) < 1e-9, calls    # takeoff used the chosen altitude
assert win._takeoff_alt == 42.0                            # remembered for next time

# the next Takeoff pre-fills with 42; a cancel must not command a takeoff or change the memory
def fake_cancel(parent, title, label, value, lo, hi, dec):
    seen["default2"] = value
    return (value, False)


m.QInputDialog.getDouble = staticmethod(fake_cancel)
n = len(calls)
win._takeoff()
assert seen["default2"] == 42.0, seen
assert len(calls) == n
assert win._takeoff_alt == 42.0

# persistence across runs: save to a temp INI, restore into a fresh window
ini = os.path.join(tempfile.mkdtemp(), "dd.ini")
win._takeoff_alt = 57.0
win.settings = QSettings(ini, QSettings.IniFormat)
win.save_settings()

win2 = m.DroneDeck(14598)
win2.settings = QSettings(ini, QSettings.IniFormat)
win2.load_settings()
assert win2._takeoff_alt == 57.0, win2._takeoff_alt

print("TAKEOFFALT PASSED")
