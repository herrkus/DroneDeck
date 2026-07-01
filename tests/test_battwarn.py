#!/usr/bin/env python3
"""test_battwarn.py -- battery low/critical warning colouring on the SystemsPanel readouts.
'Remaining' is coloured green >=40% / amber >=20% / red below (matching the status-strip chip);
'Voltage' is coloured by the weakest cell green >=3.7V / amber >=3.5V / red below (matching the
per-cell bars). Unknown values stay neutral. Pure UI, no link."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import panels
from vehicle import Vehicle

app = QApplication([])
sp = panels.SystemsPanel()
ve = Vehicle()
ve.voltage = 12.0

GREEN, AMBER, RED = "#37d67a", "#e0a030", "#e05050"

# remaining %: healthy -> green
ve.battery_remaining = 80
ve.cells = [4.0, 4.0, 4.0]
sp.update_from(ve)
assert GREEN in sp.b_rem.styleSheet(), sp.b_rem.styleSheet()
assert GREEN in sp.b_volt.styleSheet(), sp.b_volt.styleSheet()   # min cell 4.0 >= 3.7

# low -> amber, critical -> red, unknown -> neutral
ve.battery_remaining = 25
sp.update_from(ve)
assert AMBER in sp.b_rem.styleSheet()
ve.battery_remaining = 12
sp.update_from(ve)
assert RED in sp.b_rem.styleSheet()
ve.battery_remaining = -1
sp.update_from(ve)
assert sp.b_rem.styleSheet() == ""

# voltage coloured by the WEAKEST cell -- one sagging cell drags it red
ve.battery_remaining = 90
ve.cells = [3.9, 3.2, 3.9]        # min 3.2 < 3.5
sp.update_from(ve)
assert RED in sp.b_volt.styleSheet(), sp.b_volt.styleSheet()

# marginal pack -> amber
ve.cells = [3.6, 3.6, 3.6]        # 3.5 <= 3.6 < 3.7
sp.update_from(ve)
assert AMBER in sp.b_volt.styleSheet()

# no cell data -> voltage stays neutral (can't threshold pack V without cell count)
ve.cells = []
sp.update_from(ve)
assert sp.b_volt.styleSheet() == ""

# aggregated pack reported as a single "cell" (PX4 SITL: cells=[16.2]) must NOT be treated as a
# per-cell voltage -- a naive min() would show false green on a sagging pack. Stays neutral.
ve.cells = [16.2]
sp.update_from(ve)
assert sp.b_volt.styleSheet() == "", sp.b_volt.styleSheet()
ve.cells = [14.0]                 # a 4S pack at 3.5 V/cell -- must NOT show green
sp.update_from(ve)
assert sp.b_volt.styleSheet() == "", sp.b_volt.styleSheet()

print("BATTWARN PASSED")
