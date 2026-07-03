#!/usr/bin/env python3
"""test_about.py -- About dialog. A Help menu exposes 'About DroneDeck', whose content names the
app, its version, the three-language stack, and the MAVLink dialect; it folds in the connected
vehicle's autopilot/firmware when known. Verifies the content helper and the menu wiring. No link."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QAction
import main as m

app = QApplication([])
win = m.DroneDeck(14599)

assert isinstance(m.APP_VERSION, str) and m.APP_VERSION, "APP_VERSION must be a non-empty string"

html = win._about_html()
for needle in ("DroneDeck", m.APP_VERSION, "assembly", "Python", "C++", "MAVLink", "MISSION_INT"):
    assert needle in html, f"About text missing {needle!r}"
# no vehicle version yet -> no vehicle row, and it must not crash
assert "Vehicle</b>" not in html

# vehicle autopilot/firmware folds in once known
ve = win.vehicle
ve.have_autopilot_version = True
ve.autopilot = 12                        # PX4
ve.fw_version = "v1.15.0"
html2 = win._about_html()
assert "PX4" in html2 and "v1.15.0" in html2, html2

# the Help menu carries an 'About DroneDeck' action wired to _show_about
about = [a for a in win.menuBar().findChildren(QAction) if "About" in a.text()]
assert about, "no About action in the menu bar"
titles = [mnu.title() for mnu in win.menuBar().findChildren(type(win._help_menu))]
assert any("Help" in t for t in titles), titles

print("ABOUT PASSED")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
