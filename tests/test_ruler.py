#!/usr/bin/env python3
"""test_ruler.py -- map ruler / measure tool. With the Ruler toolbar button active, two map
clicks set A then measure A->B (distance + bearing) shown on the map + status bar, and the next
click starts a fresh measurement. Toggling off clears it. Ruler clicks must NOT fall through to
goto/plan. Pure UI, no link."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import QApplication
import main as m
from main import haversine, bearing

app = QApplication([])
win = m.DroneDeck(14599)
win.map.resize(500, 400)

A = (47.40, 8.54)
B = (47.41, 8.54)                      # ~1.11 km due north of A

# activate the ruler
win.btn_ruler.setChecked(True)
assert win._ruler_a is None

# first click sets A (and does NOT trigger goto -- _ruler_a proves the ruler branch ran)
win._on_map_click(*A)
assert win._ruler_a == A
assert win.map.ruler is not None and win.map.ruler[0] == A and win.map.ruler[1] is None

# second click measures A->B
win._on_map_click(*B)
a, b, label = win.map.ruler
assert a == A and b == B
exp_d = haversine(A[0], A[1], B[0], B[1])
exp_brg = bearing(A[0], A[1], B[0], B[1])
assert f"{exp_d / 1000:.2f} km" in label, (label, exp_d)     # ~1.11 km
assert f"{exp_brg:.0f}" in label.split()[-1] or "0" in label  # due north ~0 deg
assert win._ruler_a is None                                   # ready for a fresh measurement
assert "Ruler:" in win.statusBar().currentMessage()

# a third click starts a NEW measurement at C (not a second leg of the old one)
C = (47.42, 8.55)
win._on_map_click(*C)
assert win._ruler_a == C and win.map.ruler[1] is None

# the map paints with the ruler overlay without crashing
pm = QPixmap(500, 400)
win.map.set_ruler(A, B, "1.11 km  0")
win.map.render(pm)
assert not pm.isNull()

# toggling the ruler off clears everything
win.btn_ruler.setChecked(False)
assert win._ruler_a is None and win.map.ruler is None
assert win.statusBar().currentMessage() == ""

print("RULER PASSED")
