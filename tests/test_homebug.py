#!/usr/bin/env python3
"""test_homebug.py -- home-direction bug on the compass. Compass.set_home_bearing() places a
green 'H' marker on the rim at the bearing to launch (rotating with the card). Verifies the
state, that the bug actually paints (adds green pixels), that it sits on the correct side
(east vs west), and that main.bearing() gives the expected compass bearing. Pure UI, no link."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import QApplication
from instruments import Compass
import main as m

app = QApplication([])

c = Compass()
c.resize(200, 200)

# state
c.set_home_bearing(90.0, have=True)
assert c.home_bearing == 90.0 and c.have_home is True
c.set_home_bearing(0.0, have=False)
assert c.have_home is False


W = H = 200


def green_halves(img):
    """(left, right) counts of green-ish pixels -- the home bug is (80,220,130)."""
    left = right = 0
    for y in range(0, H, 2):
        for x in range(0, W, 2):
            col = img.pixelColor(x, y)
            if col.green() > 180 and col.red() < 160 and col.blue() < 180:
                if x < W // 2:
                    left += 1
                else:
                    right += 1
    return left, right


c.set_heading(0.0)

# baseline: no home bug
c.set_home_bearing(90.0, have=False)
pm0 = QPixmap(W, H)
c.render(pm0)
base_l, base_r = green_halves(pm0.toImage())

# home due EAST (90) with heading 0 -> bug on the RIGHT rim
c.set_home_bearing(90.0, have=True)
pm_e = QPixmap(W, H)
c.render(pm_e)
el, er = green_halves(pm_e.toImage())
assert (el + er) > (base_l + base_r), "home bug added no green pixels"
assert (er - base_r) > (el - base_l), (base_l, base_r, el, er)   # extra green is on the right

# home due WEST (270) -> bug on the LEFT rim
c.set_home_bearing(270.0, have=True)
pm_w = QPixmap(W, H)
c.render(pm_w)
wl, wr = green_halves(pm_w.toImage())
assert (wl - base_l) > (wr - base_r), (base_l, base_r, wl, wr)   # extra green is on the left

# bearing() sanity: due east ~90, due north ~0
assert abs(m.bearing(0.0, 0.0, 0.0, 1.0) - 90.0) < 1.0
assert abs((m.bearing(0.0, 0.0, 1.0, 0.0)) % 360.0) < 1.0

# _refresh gating: the home bug is HIDDEN within ~10 m of home (where the bearing is just GPS
# jitter -- caught in an airborne verification: it span 326->296->126 at dist~0), shown beyond it.
win = m.DroneDeck(14599)
ve = win.vehicle
ve.have_position = True
ve.home = (47.4000, 8.5400)
ve.lat, ve.lon = 47.400015, 8.540015          # ~2 m from home
win._refresh()
assert win.compass.have_home is False, "home bug must hide within 10 m of home"
ve.lat, ve.lon = 47.4013, 8.5413              # ~170 m from home
win._refresh()
assert win.compass.have_home is True, "home bug must show when clear of home"
exp = m.bearing(ve.lat, ve.lon, ve.home[0], ve.home[1])
assert abs((win.compass.home_bearing - exp + 180) % 360 - 180) < 0.5, (win.compass.home_bearing, exp)

print("HOMEBUG PASSED")
