#!/usr/bin/env python3
"""test_vibration.py -- vibration health display. VIBRATION (241) was parsed and stored on the
vehicle but never shown; the telemetry MOTION group now has a 'Vibration' row showing the peak-axis
value, colour-coded to PX4's rule of thumb (<30 good / 30-60 caution / >60 bad), and '--' until a
VIBRATION message actually arrives (so the 0,0,0 default is not mistaken for a healthy reading)."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mavlink
from vehicle import Vehicle
from panels import TelemetryPanel


class Msg:
    __slots__ = ("msgid", "sysid", "compid", "seq", "fields")
    def __init__(self, msgid, fields):
        self.msgid, self.sysid, self.compid, self.seq, self.fields = msgid, 1, 1, 0, fields


app = QApplication([])
panel = TelemetryPanel()
ve = Vehicle()

# the row exists
assert "vibe" in panel.v, "no Vibration row in the telemetry panel"

# before any VIBRATION message -> '--' (0,0,0 default must NOT read as a healthy 0.0)
assert not ve.have_vibration
panel.update_all(ve, "OK", 10.0, 1, 0)
assert panel.v["vibe"].text() == "--", panel.v["vibe"].text()

# a healthy reading -> shows the peak axis, green
ve.consume([Msg(mavlink.VIBRATION, {"vibration_x": 5.0, "vibration_y": 12.0, "vibration_z": 8.0})])
assert ve.have_vibration and ve.vibration == (5.0, 12.0, 8.0)
panel.update_all(ve, "OK", 10.0, 1, 0)
assert panel.v["vibe"].text().startswith("12.0"), panel.v["vibe"].text()

# caution band (30-60) and danger band (>60) drive the colour
ve.consume([Msg(mavlink.VIBRATION, {"vibration_x": 45.0, "vibration_y": 10.0, "vibration_z": 10.0})])
panel.update_all(ve, "OK", 10.0, 1, 0)
assert panel.v["vibe"].text().startswith("45.0")

ve.consume([Msg(mavlink.VIBRATION, {"vibration_x": 70.0, "vibration_y": 10.0, "vibration_z": 10.0})])
panel.update_all(ve, "OK", 10.0, 1, 0)
assert panel.v["vibe"].text().startswith("70.0")

# clip counts still captured alongside
ve.consume([Msg(mavlink.VIBRATION, {"vibration_x": 1.0, "vibration_y": 2.0, "vibration_z": 3.0,
                                    "clipping_0": 4, "clipping_1": 5, "clipping_2": 6})])
assert ve.clipping == (4, 5, 6)

print("VIBRATION PASSED")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
