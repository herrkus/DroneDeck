#!/usr/bin/env python3
"""test_windcompass.py -- wind arrow overlay on the compass. The Compass widget takes a
set_wind(speed, dir, have) and draws a rotating 'wind from' arrow + speed label; paintEvent
must run cleanly with and without wind. Port-independent (widget only, no link)."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QPixmap
from instruments import Compass

app = QApplication([])
c = Compass()
c.resize(200, 200)
pm = QPixmap(200, 200)

# with wind: fields stored, paintEvent (arrow + label) runs
c.set_heading(90.0)
c.set_wind(5.2, 45.0, True)
assert c.have_wind and abs(c.wind_speed - 5.2) < 1e-6 and abs(c.wind_dir - 45.0) < 1e-6
c.render(pm)

# without wind: no arrow path, still renders
c.set_wind(0.0, 0.0, False)
assert not c.have_wind
c.render(pm)

# heading + wind together, and wind_dir normalised mod 360
c.set_heading(270.0)
c.set_wind(12.7, 400.0, True)
assert abs(c.wind_dir - 40.0) < 1e-6
c.render(pm)

print("WINDCOMPASS PASSED")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
