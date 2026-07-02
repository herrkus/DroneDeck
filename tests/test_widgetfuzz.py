#!/usr/bin/env python3
"""test_widgetfuzz.py -- custom-painted-widget robustness against non-finite telemetry. The bar
widgets (vibration / battery-cell / servo) and status/health strips draw bar heights and fills from
live values; a drone can stream NaN/Inf (e.g. NaN vibration, an unknown cell voltage). The frac
clamp keeps them from crashing, but NaN would otherwise clamp to a FULL bar and the severity colour
fall through to RED -- a false critical alarm -- so non-finite now renders empty with '--'. This
grabs each painted widget (forces its paintEvent) with NaN/Inf/+-huge, and drives the SystemsPanel
integration path, asserting no crash. Complements test_telemfuzz (instruments+panel) / test_mapfuzz."""
import os
import sys
import math

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import panels
from vehicle import Vehicle

app = QApplication.instance() or QApplication([])

NAN, INF = float("nan"), float("inf")
BADS = (NAN, INF, -INF, 1e308, -1e308)

# 1) bar widgets: every non-finite / extreme value must render without crashing the paintEvent ------
for bad in BADS:
    vb = panels._VibBars(); vb.resize(180, 70)
    vb.set_values((bad, bad, bad)); vb.grab()
    vb.set_values((bad, 12.0, 95.0)); vb.grab()          # mixed finite/non-finite

    cb = panels._CellBars(); cb.resize(180, 60)
    cb.set_cells([bad, 3.7, bad, 0.0]); cb.grab()
    cb.set_cells([bad]); cb.grab()
    cb.set_cells([]); cb.grab()                          # empty -> "no cell data"

    sb = panels._ServoBars(); sb.resize(220, 60)
    sb.set_values([bad, bad, 1500, 1900, bad, 1000, 0, bad]); sb.grab()

# 2) status / health strips with odd / extreme inputs --------------------------------------------
ss = panels.StatusStrip(); ss.resize(400, 26)
ss.set_chips([]); ss.grab()
ss.set_chips([("ARMED", "#e05050", "#e05050", "#e05050"),   # 4-tuple filled badge
              ("", "#37d67a", "#37d67a"), ("GPS: 3D", "#37d67a", "#c8ccd2")]); ss.grab()

hp = panels.HealthPanel(); hp.resize(300, 120)
hp.set_health(0, 0, 0); hp.grab()
hp.set_health(0xFFFFFFFF, 0xFFFFFFFF, 0x00000000); hp.grab()      # all present, none healthy
hp.set_health(-1, -1, -1); hp.grab()

# 3) integration: SystemsPanel.update_from(vehicle) with NaN vibration / cells / servo ------------
sp = panels.SystemsPanel(); sp.resize(360, 600)
ve = Vehicle()
ve.have_vibration = True
ve.vibration = (NAN, INF, NAN)
ve.cells = [NAN, 3.7, INF, 0.0]
ve.servo_raw = [NAN, INF, 1500, 1000, NAN, 0, 0, 0]
sp.update_from(ve)                                       # must not raise
sp.grab()                                                # must not crash the bars' paints
sp.vib_bars.grab(); sp.cell_bars.grab(); sp.servo_bars.grab()

# and finite values still render (no regression)
ve2 = Vehicle()
ve2.have_vibration = True
ve2.vibration = (5.0, 12.0, 40.0)
ve2.cells = [4.1, 3.9, 3.6]
ve2.servo_raw = [1500, 1900, 1100, 1000, 0, 0, 0, 0]
sp.update_from(ve2); sp.grab()

# -- audit batch 8: the two UI paths the NaN sweep had missed --------------------------------------
import math as _math
from charts import ChartPanel
from instruments import AttitudeIndicator


class _VE:                                               # minimal chart source
    alt_rel = groundspeed = voltage = climb = 0.0


cp = ChartPanel()
_v = _VE()
_v.climb, _v.groundspeed, _v.voltage, _v.alt_rel = NAN, INF, 16.2, 50.0   # PX4 pre-EKF NaN burst
cp.sample(_v)
_v.climb, _v.groundspeed = 1.0, 5.0
cp.sample(_v); cp.sample(_v)
stored = [v for dq in cp.data.values() for _t, v in dq]
assert stored and all(_math.isfinite(v) for v in stored), "charts stored a non-finite sample"
cp.resize(320, 240); cp.grab()                           # NaN-free polyline -> no painter crash

adi = AttitudeIndicator(); adi.resize(200, 200)
for pd in (-85.0, -60.0, 0.0, 60.0, 85.0):               # steep dive/climb must still fill the ball
    adi.set_attitude(0.0, _math.radians(pd)); adi.grab()

print("WIDGETFUZZ PASSED (bar/status/health widgets survive NaN/Inf/extreme; SystemsPanel integ ok; "
      "charts drop non-finite samples; ADI fills at extreme pitch)")
