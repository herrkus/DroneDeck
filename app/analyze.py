"""analyze.py -- offline .tlog analysis: plot any numeric telemetry field over time.

QGroundControl's "Analyze Tools" equivalent: open a recorded .tlog, pick a signal
(altitude, battery, speed, attitude, ...) and see it plotted across the whole flight.
Frames are parsed through the same core.Parser the live link uses, so every message
the GCS understands is available here too.
"""
from __future__ import annotations
import os

from PySide6.QtCore import Qt, QRectF, QPointF
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QPolygonF
from PySide6.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
                               QComboBox, QWidget, QFileDialog)

import mavlink
import core
from tlog import read_tlog

_MONO = QFont("DejaVu Sans Mono", 8)

# (message, field, label, unit, scale) -- scale converts raw wire units to display units
PLOTTABLE = [
    ("GLOBAL_POSITION_INT", "relative_alt",       "Altitude (rel)",    "m",   1e-3),
    ("GLOBAL_POSITION_INT", "alt",                "Altitude (MSL)",    "m",   1e-3),
    ("VFR_HUD",             "groundspeed",        "Ground speed",      "m/s", 1.0),
    ("VFR_HUD",             "airspeed",           "Air speed",         "m/s", 1.0),
    ("VFR_HUD",             "climb",              "Climb rate",        "m/s", 1.0),
    ("VFR_HUD",             "throttle",           "Throttle",          "%",   1.0),
    ("VFR_HUD",             "heading",            "Heading",           "deg", 1.0),
    ("SYS_STATUS",          "voltage_battery",    "Battery voltage",   "V",   1e-3),
    ("SYS_STATUS",          "current_battery",    "Battery current",   "A",   1e-2),
    ("SYS_STATUS",          "battery_remaining",  "Battery remaining", "%",   1.0),
    ("ATTITUDE",            "roll",               "Roll",              "deg", 57.29578),
    ("ATTITUDE",            "pitch",              "Pitch",             "deg", 57.29578),
    ("ATTITUDE",            "yaw",                "Yaw",               "deg", 57.29578),
    ("GPS_RAW_INT",         "satellites_visible", "GPS satellites",    "",    1.0),
]


def extract_series(records):
    """records: [(t_us, frame_bytes)] -> ({label: [(t_s, value)]}, {label: unit}).

    Time is seconds since the first record. Uses a fresh core.Parser so it is
    independent of any live link, and only keeps the curated PLOTTABLE signals."""
    want = {}                                   # msgid -> [(field, label, scale)]
    units = {}
    for msg, field, label, unit, scale in PLOTTABLE:
        mid = getattr(mavlink, msg, None)
        if mid is None:
            continue
        want.setdefault(mid, []).append((field, label, scale))
        units[label] = unit
    parser = core.Parser()
    series = {}
    t0 = None
    for t_us, fr in records:
        if t0 is None:
            t0 = t_us
        t = (t_us - t0) / 1e6
        for m in parser.feed(fr):
            for field, label, scale in want.get(m.msgid, ()):
                if field in m.fields:
                    series.setdefault(label, []).append((t, float(m.fields[field]) * scale))
    return series, units


class LogPlot(QWidget):
    """A single time-series plotted with axes, grid and min/max/last readouts."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(260)
        self.pts = []
        self.label = ""
        self.unit = ""
        self.color = QColor("#37c0ff")

    def set_series(self, pts, label, unit):
        self.pts = list(pts)
        self.label = label
        self.unit = unit
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(22, 24, 29))
        w, h = self.width(), self.height()
        p.setFont(_MONO)
        L, R, T, B = 64, 12, 22, 22                 # axis margins
        plot = QRectF(L, T, max(1.0, w - L - R), max(1.0, h - T - B))
        p.setPen(QPen(QColor(60, 64, 72), 1))
        p.drawRect(plot)
        if len(self.pts) < 2:
            p.setPen(QColor(150, 156, 166))
            p.drawText(self.rect(), Qt.AlignCenter, "open a .tlog and pick a field")
            p.end()
            return
        ts = [t for t, _ in self.pts]
        vs = [v for _, v in self.pts]
        t_lo, t_hi = ts[0], ts[-1]
        v_lo, v_hi = min(vs), max(vs)
        if t_hi - t_lo < 1e-6:
            t_hi = t_lo + 1.0
        if v_hi - v_lo < 1e-6:
            v_lo, v_hi = v_lo - 1.0, v_hi + 1.0
        pad = (v_hi - v_lo) * 0.08
        v_lo -= pad
        v_hi += pad

        def X(t):
            return plot.left() + (t - t_lo) / (t_hi - t_lo) * plot.width()

        def Y(v):
            return plot.bottom() - (v - v_lo) / (v_hi - v_lo) * plot.height()

        for i in range(6):                          # horizontal grid + value labels
            gv = v_lo + (v_hi - v_lo) * i / 5.0
            y = Y(gv)
            p.setPen(QPen(QColor(38, 41, 48), 1))
            p.drawLine(int(plot.left()), int(y), int(plot.right()), int(y))
            p.setPen(QColor(150, 156, 166))
            p.drawText(QRectF(0, y - 7, L - 6, 14), Qt.AlignRight | Qt.AlignVCenter, f"{gv:.6g}")
        for i in range(7):                          # vertical grid + time labels
            gt = t_lo + (t_hi - t_lo) * i / 6.0
            x = X(gt)
            p.setPen(QPen(QColor(38, 41, 48), 1))
            p.drawLine(int(x), int(plot.top()), int(x), int(plot.bottom()))
            p.setPen(QColor(150, 156, 166))
            p.drawText(QRectF(x - 26, plot.bottom() + 3, 52, 14), Qt.AlignCenter, f"{gt:.0f}s")
        poly = QPolygonF()
        for t, v in self.pts:
            poly.append(QPointF(X(t), Y(v)))
        p.setPen(QPen(self.color, 1.6))
        p.drawPolyline(poly)
        p.setPen(QColor(210, 214, 220))
        p.drawText(QRectF(L, 2, plot.width(), 16), Qt.AlignLeft,
                   f"{self.label}   min {min(vs):.6g}  max {max(vs):.6g}  last {vs[-1]:.6g} {self.unit}")
        p.setPen(QColor(120, 126, 136))
        p.drawText(QRectF(L, 2, plot.width(), 16), Qt.AlignRight, f"{len(self.pts)} pts")
        p.end()


class AnalyzeDialog(QDialog):
    """Open a .tlog and plot a chosen numeric signal over the whole recording."""

    def __init__(self, parent=None, start_dir=None):
        super().__init__(parent)
        self.setWindowTitle("Analyze -- Log Plot")
        self.resize(760, 460)
        self._start_dir = start_dir or os.path.expanduser("~")
        self.series = {}
        self.units = {}
        lay = QVBoxLayout(self)
        top = QHBoxLayout()
        self.btn_open = QPushButton("Open .tlog...")
        self.btn_open.clicked.connect(lambda: self.open_tlog())
        self.combo = QComboBox()
        self.combo.setMinimumWidth(200)
        self.combo.currentTextChanged.connect(self._on_field)
        top.addWidget(self.btn_open)
        top.addWidget(QLabel("Field:"))
        top.addWidget(self.combo, 1)
        lay.addLayout(top)
        self.plot = LogPlot()
        lay.addWidget(self.plot, 1)
        self.status = QLabel("open a recorded .tlog to plot a signal over time")
        self.status.setStyleSheet("color:#8a90a0;")
        lay.addWidget(self.status)

    def open_tlog(self, path=None):
        if not path:
            path, _ = QFileDialog.getOpenFileName(
                self, "Open telemetry log", self._start_dir,
                "Telemetry logs (*.tlog);;All files (*)")
        if not path:
            return
        try:
            records = read_tlog(path)
        except Exception as e:
            self.status.setText(f"open failed: {e}")
            return
        self.series, self.units = extract_series(records)
        ordered = [label for (_, _, label, _, _) in PLOTTABLE if self.series.get(label)]
        self.combo.blockSignals(True)
        self.combo.clear()
        self.combo.addItems(ordered)
        self.combo.blockSignals(False)
        base = os.path.basename(path)
        if ordered:
            self.status.setText(f"{base}: {len(records)} frames, {len(ordered)} plottable signals")
            self.combo.setCurrentIndex(0)
            self._on_field(ordered[0])
        else:
            self.plot.set_series([], "", "")
            self.status.setText(f"{base}: {len(records)} frames, no plottable signals found")

    def _on_field(self, label):
        self.plot.set_series(self.series.get(label, []), label, self.units.get(label, ""))
