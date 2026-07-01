#!/usr/bin/env python3
"""test_scalebar.py -- map scale bar. scale_nice() turns metres-per-pixel into a 1/2/5 x 10^n
round distance near a target pixel width, returning (metres, bar_px, label); _draw_hud renders
it bottom-left. Verifies the picker at known zoom/latitude values and that MapView paints with
the bar without crashing. Pure UI, no link."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import QApplication
import mapview

app = QApplication([])

# zoom 16 @ lat 47.4 -> ~1.617 m/px -> 100 m bar (~62 px)
d, px, lbl = mapview.scale_nice(1.617)
assert d == 100.0 and abs(px - 61.8) < 1.5 and lbl == "100 m", (d, px, lbl)

# 50 m/px -> target 4500 m -> 2 km bar (40 px)
d, px, lbl = mapview.scale_nice(50.0)
assert d == 2000.0 and abs(px - 40.0) < 0.5 and lbl == "2 km", (d, px, lbl)

# 0.5 m/px -> target 45 m -> 20 m bar
d, px, lbl = mapview.scale_nice(0.5)
assert d == 20.0 and abs(px - 40.0) < 0.5 and lbl == "20 m", (d, px, lbl)

# very coarse -> 5*10^n and km label
d, px, lbl = mapview.scale_nice(1000.0)
assert d == 50000.0 and lbl == "50 km", (d, px, lbl)

# degenerate m/px -> empty (no divide-by-zero, no NaN)
assert mapview.scale_nice(0) == (0.0, 0.0, "")
assert mapview.scale_nice(-5) == (0.0, 0.0, "")

# the label is always a clean 1/2/5 leading digit
for mpp in (0.3, 1.0, 3.2, 8.8, 17.0, 120.0, 640.0):
    _, _, lbl = mapview.scale_nice(mpp)
    lead = lbl.split()[0]
    assert lead in ("1", "2", "5", "10", "20", "50", "100", "200", "500"), (mpp, lbl)

# MapView paints with the scale bar and doesn't crash
mv = mapview.MapView()
mv.resize(400, 300)
mv.center = (47.4, 8.5)
mv.zoom = 16
pm = QPixmap(400, 300)
mv.render(pm)                       # exercises paintEvent -> _draw_hud -> scale bar
assert not pm.isNull()

print("SCALEBAR PASSED")
