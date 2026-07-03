#!/usr/bin/env python3
"""test_mapcenter.py -- map 'Center on vehicle' button/shortcut + Follow-checkbox sync.
QGC parity: a one-shot recenter (no follow change), and the Follow checkbox now tracks the
map's follow flag when panning auto-detaches it / a double-click re-attaches it. Pure UI."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import main as m

app = QApplication([])
win = m.DroneDeck(14599)
mp = win.map

# no fix yet -> center_on_vehicle reports False (and the slot degrades gracefully)
mp.veh = None
assert mp.center_on_vehicle() is False
win._center_on_vehicle()      # must not raise without a fix

# with a known position: detach (pan away), then Center recenters exactly on the vehicle
mp.update_vehicle(47.5, 8.5, 0.0, None, [])
mp.set_follow(False)
mp.center = (10.0, 10.0)
win._center_on_vehicle()
assert abs(mp.center[0] - 47.5) < 1e-6 and abs(mp.center[1] - 8.5) < 1e-6, mp.center

# panning auto-detaches follow -> the toolbar checkbox unticks (via followChanged)
win.chk_follow.setChecked(True)
mp.follow = True
mp.set_follow(False)          # what a pan does
assert not win.chk_follow.isChecked()
# a double-click re-attaches -> checkbox reticks
mp.set_follow(True)
assert win.chk_follow.isChecked()

# followChanged fires only on an actual change (no redundant emits / loops)
calls = []
mp.followChanged.connect(lambda v: calls.append(v))
mp.set_follow(True)           # already True -> no emit
assert calls == [], calls
mp.set_follow(False)          # change -> one emit
assert calls == [False], calls

# checkbox -> map direction still works (no signal loop left it stuck)
win.chk_follow.setChecked(False)
win.chk_follow.setChecked(True)
assert mp.follow is True

# the button + shortcut wiring exist
assert win.btn_center is not None

print("MAPCENTER PASSED")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
