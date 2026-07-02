"""charts.py -- scrolling live telemetry charts (QGC "Analyze" style).

Keeps a rolling time window of a few key signals and paints them as stacked
line charts with QPainter. Fed by sample(vehicle) from the UI refresh loop;
repaints on its own modest timer so it costs nothing when hidden.
"""
from __future__ import annotations
import math
import time
from collections import deque

from PySide6.QtCore import Qt, QTimer, QRectF, QPointF
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QPolygonF
from PySide6.QtWidgets import QWidget

WINDOW = 60.0          # seconds shown
_MONO = QFont("DejaVu Sans Mono", 8)


class ChartPanel(QWidget):
    # (vehicle attr, label, unit, colour)
    SERIES = [
        ("alt_rel", "Altitude", "m", "#37c0ff"),
        ("groundspeed", "Ground spd", "m/s", "#7CFC00"),
        ("voltage", "Battery", "V", "#ffb000"),
        ("climb", "Climb", "m/s", "#ff7a50"),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(150)
        self.t0 = time.monotonic()
        self.data = {k: deque() for k, _, _, _ in self.SERIES}
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.update)
        self._timer.start(150)

    def sample(self, ve):
        t = time.monotonic() - self.t0
        vals = {"alt_rel": ve.alt_rel, "groundspeed": ve.groundspeed,
                "voltage": ve.voltage, "climb": ve.climb}
        for k, dq in self.data.items():
            try:
                v = float(vals.get(k, 0.0))
            except (TypeError, ValueError):
                v = None
            # Drop non-finite samples: a single NaN (PX4 streams NaN climb/speed before EKF
            # convergence) would make min/max/pad NaN and feed NaN QPointFs to drawPolyline --
            # the same NaN-painter crash class hardened across the rest of the UI.
            if v is not None and math.isfinite(v):
                dq.append((t, v))
            cut = t - WINDOW
            while dq and dq[0][0] < cut:
                dq.popleft()

    def clear(self):
        for dq in self.data.values():
            dq.clear()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(22, 24, 29))
        w, h = self.width(), self.height()
        now = time.monotonic() - self.t0
        n = len(self.SERIES)
        band = h / n
        p.setFont(_MONO)
        for i, (key, label, unit, col) in enumerate(self.SERIES):
            top = i * band
            r = QRectF(0, top, w, band)
            if i:
                p.setPen(QPen(QColor(45, 48, 56), 1))
                p.drawLine(0, int(top), w, int(top))
            dq = self.data[key]
            color = QColor(col)
            # current value + label
            cur = dq[-1][1] if dq else 0.0
            p.setPen(QColor(150, 156, 166))
            p.drawText(QRectF(6, top + 2, w - 12, 14), Qt.AlignLeft, f"{label}")
            p.setPen(color)
            p.drawText(QRectF(6, top + 2, w - 12, 14), Qt.AlignRight, f"{cur:6.1f} {unit}")
            if len(dq) < 2:
                continue
            ys = [v for _, v in dq]
            lo, hi = min(ys), max(ys)
            if hi - lo < 1e-6:
                lo, hi = lo - 1.0, hi + 1.0
            pad = (hi - lo) * 0.15
            lo -= pad
            hi += pad
            plot = QRectF(6, top + 16, w - 12, band - 20)
            poly = QPolygonF()
            for t, v in dq:
                x = plot.left() + (1.0 - (now - t) / WINDOW) * plot.width()
                y = plot.bottom() - (v - lo) / (hi - lo) * plot.height()
                poly.append(QPointF(x, y))
            p.setPen(QPen(color, 1.6))
            p.drawPolyline(poly)
        p.end()
