#!/usr/bin/env python3
"""test_cellbars.py -- per-cell battery bar graph. SystemsPanel keeps the Cells (V) text
row and adds a _CellBars widget: one bar per cell, filled over 3.2-4.2 V and coloured by
voltage. paintEvent must run with cells, with none, and across the colour thresholds.
Port-independent (panel widget only, no link)."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QPixmap
import panels
from vehicle import Vehicle

app = QApplication([])
sp = panels.SystemsPanel()
pm = QPixmap(220, 60)
sp.cell_bars.resize(220, 60)

# cells present -> bars mirror ve.cells and the text row is still populated (test_systems reads it)
v = Vehicle()
v.cells = [4.15, 3.65, 3.45, 3.90]
sp.update_from(v)
assert sp.cell_bars.cells == [4.15, 3.65, 3.45, 3.90]
assert "4.15" in sp.b_cells.text() and "3.45" in sp.b_cells.text()
sp.cell_bars.render(pm)

# no cells -> empty list, 'no cell data' branch renders cleanly
sp.update_from(Vehicle())
assert sp.cell_bars.cells == []
sp.cell_bars.render(pm)

# spanning the colour thresholds (red < 3.5 <= amber < 3.7 <= green) renders
v.cells = [3.20, 3.55, 3.75, 4.20]
sp.update_from(v)
assert sp.cell_bars.cells == [3.20, 3.55, 3.75, 4.20]
sp.cell_bars.render(pm)

print("CELLBARS PASSED")
