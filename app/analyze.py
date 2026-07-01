"""analyze.py -- offline .tlog analysis: plot any numeric telemetry field over time.

QGroundControl's "Analyze Tools" equivalent: open a recorded .tlog, pick a signal
(altitude, battery, speed, attitude, ...) and see it plotted across the whole flight.
Frames are parsed through the same core.Parser the live link uses, so every message
the GCS understands is available here too.
"""
from __future__ import annotations
import os
import bisect

from PySide6.QtCore import Qt, QRectF, QPointF
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QPolygonF
from PySide6.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
                               QWidget, QFileDialog, QListWidget, QListWidgetItem)

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


def series_to_csv(pts, label, unit):
    """Render one (t, value) series as CSV text: a header row + time_s,value rows."""
    head = f"time_s,{label} ({unit})" if unit else f"time_s,{label}"
    rows = [head]
    for t, v in pts:
        rows.append(f"{t:.6f},{v:.9g}")
    return "\n".join(rows) + "\n"


_COLORS = ["#37c0ff", "#7CFC00", "#ffb000", "#ff7a50", "#c07cff", "#ff5db1", "#e8e8e8"]


class LogPlot(QWidget):
    """One or more time-series over a shared time axis. A single series shows real Y
    labels + a hover tooltip; multiple series are each normalised to 0..1 and drawn in
    distinct colours with a legend whose values track the hover crosshair."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(260)
        self.setMouseTracking(True)                 # hover crosshair without a click
        self.series = []                            # [(label, unit, [(t, v), ...])]
        self._cursor_t = None                       # time under the cursor, or None
        self._geo = None                            # (plot QRectF, t_lo, t_hi) for pixel<->time

    def set_series(self, pts, label, unit):         # single series (back-compat)
        self.series = [(label, unit, list(pts))] if pts else []
        self._cursor_t = None
        self.update()

    def set_multi(self, items):                     # items = [(label, unit, pts), ...]
        self.series = [(l, u, list(p)) for (l, u, p) in items if p]
        self._cursor_t = None
        self.update()

    @staticmethod
    def _nearest(pts, t):
        if not pts:
            return None
        ts = [p[0] for p in pts]
        i = bisect.bisect_left(ts, t)
        if i <= 0:
            return pts[0]
        if i >= len(pts):
            return pts[-1]
        a, b = pts[i - 1], pts[i]
        return b if abs(b[0] - t) < abs(a[0] - t) else a

    def value_at(self, t):                          # nearest sample of the first series
        return self._nearest(self.series[0][2], t) if self.series else None

    def mouseMoveEvent(self, e):
        if self._geo is None or not self.series:
            return
        plot, t_lo, t_hi = self._geo
        x = e.position().x()
        self._cursor_t = (t_lo + (x - plot.left()) / plot.width() * (t_hi - t_lo)
                          if plot.left() <= x <= plot.right() else None)
        self.update()

    def leaveEvent(self, _):
        self._cursor_t = None
        self.update()

    def _time_span(self):
        lo = min(s[2][0][0] for s in self.series)
        hi = max(s[2][-1][0] for s in self.series)
        return (lo, lo + 1.0) if hi - lo < 1e-6 else (lo, hi)

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
        if not self.series or all(len(s[2]) < 2 for s in self.series):
            p.setPen(QColor(150, 156, 166))
            p.drawText(self.rect(), Qt.AlignCenter, "open a .tlog and tick one or more fields")
            p.end()
            return
        t_lo, t_hi = self._time_span()
        self._geo = (plot, t_lo, t_hi)              # let mouseMove map pixels back to time

        def X(t):
            return plot.left() + (t - t_lo) / (t_hi - t_lo) * plot.width()

        for i in range(7):                          # vertical grid + time labels (shared)
            gt = t_lo + (t_hi - t_lo) * i / 6.0
            x = X(gt)
            p.setPen(QPen(QColor(38, 41, 48), 1))
            p.drawLine(int(x), int(plot.top()), int(x), int(plot.bottom()))
            p.setPen(QColor(150, 156, 166))
            p.drawText(QRectF(x - 26, plot.bottom() + 3, 52, 14), Qt.AlignCenter, f"{gt:.0f}s")
        if len(self.series) == 1:
            self._paint_single(p, plot, L, X)
        else:
            self._paint_multi(p, plot, L, X)
        p.end()

    def _paint_single(self, p, plot, L, X):
        label, unit, pts = self.series[0]
        vs = [v for _, v in pts]
        v_lo, v_hi = min(vs), max(vs)
        if v_hi - v_lo < 1e-6:
            v_lo, v_hi = v_lo - 1.0, v_hi + 1.0
        pad = (v_hi - v_lo) * 0.08
        v_lo -= pad
        v_hi += pad

        def Y(v):
            return plot.bottom() - (v - v_lo) / (v_hi - v_lo) * plot.height()

        for i in range(6):                          # horizontal grid + value labels
            gv = v_lo + (v_hi - v_lo) * i / 5.0
            y = Y(gv)
            p.setPen(QPen(QColor(38, 41, 48), 1))
            p.drawLine(int(plot.left()), int(y), int(plot.right()), int(y))
            p.setPen(QColor(150, 156, 166))
            p.drawText(QRectF(0, y - 7, L - 6, 14), Qt.AlignRight | Qt.AlignVCenter, f"{gv:.6g}")
        poly = QPolygonF()
        for t, v in pts:
            poly.append(QPointF(X(t), Y(v)))
        p.setPen(QPen(QColor(_COLORS[0]), 1.6))
        p.drawPolyline(poly)
        p.setPen(QColor(210, 214, 220))
        p.drawText(QRectF(L, 2, plot.width(), 16), Qt.AlignLeft,
                   f"{label}   min {min(vs):.6g}  max {max(vs):.6g}  last {vs[-1]:.6g} {unit}")
        p.setPen(QColor(120, 126, 136))
        p.drawText(QRectF(L, 2, plot.width(), 16), Qt.AlignRight, f"{len(pts)} pts")
        if self._cursor_t is not None:              # hover crosshair + floating tooltip
            samp = self._nearest(pts, self._cursor_t)
            if samp is not None:
                cx, cy = X(samp[0]), Y(samp[1])
                p.setPen(QPen(QColor(120, 126, 136), 1, Qt.DashLine))
                p.drawLine(int(cx), int(plot.top()), int(cx), int(plot.bottom()))
                p.setPen(QPen(QColor(255, 210, 74), 1))
                p.setBrush(QColor(255, 210, 74))
                p.drawEllipse(QPointF(cx, cy), 3.0, 3.0)
                txt = f"t={samp[0]:.2f}s  {samp[1]:.6g} {unit}"
                tw = p.fontMetrics().horizontalAdvance(txt) + 10
                tx = cx + 8 if cx < plot.right() - tw - 8 else cx - tw - 8
                ty = max(plot.top() + 2.0, min(cy - 18.0, plot.bottom() - 18.0))
                box = QRectF(tx, ty, tw, 15)
                p.fillRect(box, QColor(20, 22, 27))
                p.setBrush(Qt.NoBrush)
                p.setPen(QPen(QColor(90, 94, 102), 1))
                p.drawRect(box)
                p.setPen(QColor(255, 210, 74))
                p.drawText(box.adjusted(5, 0, 0, 0), Qt.AlignVCenter | Qt.AlignLeft, txt)

    def _paint_multi(self, p, plot, L, X):
        if self._cursor_t is not None:
            cx = X(self._cursor_t)
            p.setPen(QPen(QColor(120, 126, 136), 1, Qt.DashLine))
            p.drawLine(int(cx), int(plot.top()), int(cx), int(plot.bottom()))
        ly = 2                                        # legend rows stack down from the top
        for k, (label, unit, pts) in enumerate(self.series):
            col = QColor(_COLORS[k % len(_COLORS)])
            vs = [v for _, v in pts]
            lo, hi = min(vs), max(vs)
            span = (hi - lo) or 1.0

            def Y(v, lo=lo, span=span):               # normalise each series to ~0..1
                return plot.bottom() - (0.02 + 0.96 * (v - lo) / span) * plot.height()

            poly = QPolygonF()
            for t, v in pts:
                poly.append(QPointF(X(t), Y(v)))
            p.setPen(QPen(col, 1.5))
            p.drawPolyline(poly)
            if self._cursor_t is not None:
                samp = self._nearest(pts, self._cursor_t)
                if samp is not None:
                    p.setBrush(col)
                    p.setPen(QPen(col, 1))
                    p.drawEllipse(QPointF(X(samp[0]), Y(samp[1])), 3.0, 3.0)
                    p.setBrush(Qt.NoBrush)
                    txt = f"{label}: {samp[1]:.4g} {unit}"
                else:
                    txt = label
            else:
                txt = f"{label}: {lo:.4g}..{hi:.4g} {unit}"
            p.fillRect(QRectF(L, ly + 3, 10, 8), col)   # colour swatch
            p.setPen(QColor(210, 214, 220))
            p.drawText(QRectF(L + 16, ly, plot.width() - 16, 14), Qt.AlignLeft, txt)
            ly += 15
        p.setPen(QColor(120, 126, 136))
        p.drawText(QRectF(L, 2, plot.width(), 14), Qt.AlignRight, "each 0..1 normalised")


class AnalyzeDialog(QDialog):
    """Open a .tlog and plot one or more numeric signals over the whole recording."""

    def __init__(self, parent=None, start_dir=None):
        super().__init__(parent)
        self.setWindowTitle("Analyze -- Log Plot")
        self.resize(820, 480)
        self._start_dir = start_dir or os.path.expanduser("~")
        self.series = {}
        self.units = {}
        lay = QVBoxLayout(self)
        top = QHBoxLayout()
        self.btn_open = QPushButton("Open .tlog...")
        self.btn_open.clicked.connect(lambda: self.open_tlog())
        self.btn_csv = QPushButton("Export CSV...")
        self.btn_csv.clicked.connect(self._export_csv)
        top.addWidget(self.btn_open)
        top.addWidget(QLabel("tick fields to overlay"))
        top.addStretch(1)
        top.addWidget(self.btn_csv)
        lay.addLayout(top)
        body = QHBoxLayout()
        self.fieldlist = QListWidget()
        self.fieldlist.setMaximumWidth(210)
        self.fieldlist.itemChanged.connect(lambda _it: self._replot())
        body.addWidget(self.fieldlist)
        self.plot = LogPlot()
        body.addWidget(self.plot, 1)
        lay.addLayout(body, 1)
        self.status = QLabel("open a recorded .tlog to plot signals over time")
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
        self.fieldlist.blockSignals(True)
        self.fieldlist.clear()
        for label in ordered:
            it = QListWidgetItem(label)
            it.setFlags(it.flags() | Qt.ItemIsUserCheckable)
            it.setCheckState(Qt.Unchecked)
            self.fieldlist.addItem(it)
        if ordered:                                   # tick the first field by default
            self.fieldlist.item(0).setCheckState(Qt.Checked)
        self.fieldlist.blockSignals(False)
        base = os.path.basename(path)
        if ordered:
            self.status.setText(f"{base}: {len(records)} frames, {len(ordered)} plottable signals")
            self._replot()
        else:
            self.plot.set_multi([])
            self.status.setText(f"{base}: {len(records)} frames, no plottable signals found")

    def _checked(self):
        return [self.fieldlist.item(i).text() for i in range(self.fieldlist.count())
                if self.fieldlist.item(i).checkState() == Qt.Checked]

    def _replot(self):
        labels = self._checked()
        self.plot.set_multi([(l, self.units.get(l, ""), self.series.get(l, [])) for l in labels])
        if len(labels) > 1:
            self.status.setText(f"overlaying {len(labels)} signals (each normalised 0..1)")

    def _export_csv(self):
        labels = self._checked()
        if not labels:
            self.status.setText("tick a field to export")
            return
        label = labels[0]                             # export the first ticked signal
        pts = self.series.get(label)
        if not pts:
            self.status.setText("no data in the selected series")
            return
        stem = label.replace(" ", "_").replace("(", "").replace(")", "")
        default = os.path.join(self._start_dir, stem + ".csv")
        path, _ = QFileDialog.getSaveFileName(self, "Export series as CSV", default, "CSV (*.csv)")
        if not path:
            return
        try:
            with open(path, "w") as f:
                f.write(series_to_csv(pts, label, self.units.get(label, "")))
        except OSError as e:
            self.status.setText(f"export failed: {e}")
            return
        self.status.setText(f"exported {len(pts)} rows to {os.path.basename(path)}")
