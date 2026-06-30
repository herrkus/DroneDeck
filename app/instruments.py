"""instruments.py -- QPainter flight instruments: attitude indicator + compass.

Pure custom-drawn QWidgets so there is no 3D model anywhere -- just the classic
ADI horizon, pitch ladder, bank scale and a heading card.
"""
from __future__ import annotations
import math

from PySide6.QtCore import Qt, QRectF, QPointF, QSize
from PySide6.QtGui import (QPainter, QColor, QPen, QBrush, QPolygonF, QFont,
                           QPainterPath, QRadialGradient)
from PySide6.QtWidgets import QWidget

SKY = QColor(48, 128, 196)
GROUND = QColor(140, 92, 50)
YELLOW = QColor(255, 206, 0)
LINE = QColor(245, 245, 245)


class AttitudeIndicator(QWidget):
    """Primary flight display: artificial horizon with airspeed/altitude tapes,
    a vertical-speed indicator and a heading strip -- QGroundControl style."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0
        self.airspeed = 0.0
        self.alt = 0.0
        self.heading = 0.0
        self.climb = 0.0
        self.setMinimumSize(200, 150)

    def sizeHint(self):
        return QSize(300, 220)

    def set_attitude(self, roll, pitch):
        self.roll, self.pitch = roll, pitch
        self.update()

    def set_data(self, roll, pitch, airspeed, alt, heading, climb):
        self.roll, self.pitch = roll, pitch
        self.airspeed, self.alt, self.heading, self.climb = airspeed, alt, heading, climb
        self.update()

    def paintEvent(self, _):
        w, h = self.width(), self.height()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(18, 19, 24))

        tape = max(30, min(48, int(w * 0.16)))
        head_h = 16
        hx0, hx1 = tape, w - tape
        hy0, hy1 = head_h, h
        hw, hh = hx1 - hx0, hy1 - hy0
        cx, cy = (hx0 + hx1) / 2.0, (hy0 + hy1) / 2.0
        ppd = hh / 50.0                       # +/-25 deg visible
        roll_deg = math.degrees(self.roll)
        pitch_deg = math.degrees(self.pitch)

        # ---- horizon (clipped to the centre rect) ---------------------------
        p.save()
        clip = QPainterPath()
        clip.addRoundedRect(QRectF(hx0, hy0, hw, hh), 6, 6)
        p.setClipPath(clip)
        p.translate(cx, cy)
        p.rotate(-roll_deg)
        horizon_y = pitch_deg * ppd
        big = max(hw, hh) * 3
        p.fillRect(QRectF(-big, -big, 2 * big, big + horizon_y), QBrush(SKY))
        p.fillRect(QRectF(-big, horizon_y, 2 * big, big), QBrush(GROUND))
        p.setPen(QPen(LINE, 2))
        p.drawLine(QPointF(-big, horizon_y), QPointF(big, horizon_y))
        p.setFont(QFont("DejaVu Sans", 7))
        for a in range(-30, 31, 10):
            if a == 0:
                continue
            y = (pitch_deg - a) * ppd
            if abs(y) > hh / 2 - 4:
                continue
            half = hw * 0.16 if a % 30 == 0 else hw * 0.09
            p.setPen(QPen(LINE, 1.3))
            p.drawLine(QPointF(-half, y), QPointF(half, y))
            p.drawText(QRectF(half + 3, y - 7, 26, 14), Qt.AlignVCenter | Qt.AlignLeft, str(abs(a)))
        # bank ticks
        p.setPen(QPen(LINE, 1.4))
        rr = min(hw, hh) * 0.48
        for t in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            p.save()
            p.rotate(t)
            ln = 9 if t % 30 == 0 else 5
            p.drawLine(QPointF(0, -rr), QPointF(0, -rr + ln))
            p.restore()
        p.restore()

        # bank pointer + aircraft symbol (fixed)
        p.save()
        p.translate(cx, cy)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(YELLOW))
        rr = min(hw, hh) * 0.48
        p.drawPolygon(QPolygonF([QPointF(0, -rr + 1), QPointF(-5, -rr + 11), QPointF(5, -rr + 11)]))
        p.setPen(QPen(YELLOW, 3))
        p.setBrush(Qt.NoBrush)
        wing = hw * 0.30
        p.drawLine(QPointF(-wing, 0), QPointF(-wing * 0.4, 0))
        p.drawLine(QPointF(wing * 0.4, 0), QPointF(wing, 0))
        p.setBrush(QBrush(YELLOW))
        p.drawEllipse(QPointF(0, 0), 2.5, 2.5)
        p.restore()

        # ---- tapes + heading strip ------------------------------------------
        self._tape(p, QRectF(0, hy0, tape, hh), self.airspeed, 4.0, 1, "%.0f", right=True)
        self._tape(p, QRectF(w - tape, hy0, tape, hh), self.alt, 10.0, 1, "%.0f", right=False)
        self._vsi(p, w - tape, hy0, hh)
        self._heading_strip(p, hx0, hw, head_h)
        p.end()

    def _tape(self, p, rect, value, step, _minor, fmt, right):
        p.fillRect(rect, QColor(0, 0, 0, 130))
        p.setClipRect(rect)
        cy = rect.center().y()
        ppu = rect.height() / (step * 10.0)   # show +/-5*step
        p.setFont(QFont("DejaVu Sans Mono", 7))
        base = math.floor(value / step) * step
        for k in range(-7, 8):
            v = base + k * step
            y = cy - (v - value) * ppu
            if y < rect.top() or y > rect.bottom():
                continue
            major = (round(v / step) % 5 == 0)
            p.setPen(QPen(QColor(210, 214, 220), 1.4 if major else 1))
            if right:
                p.drawLine(QPointF(rect.right() - (8 if major else 5), y), QPointF(rect.right(), y))
                if major:
                    p.drawText(QRectF(rect.left(), y - 7, rect.width() - 9, 14),
                               Qt.AlignVCenter | Qt.AlignRight, fmt % v)
            else:
                p.drawLine(QPointF(rect.left(), y), QPointF(rect.left() + (8 if major else 5), y))
                if major:
                    p.drawText(QRectF(rect.left() + 9, y - 7, rect.width() - 9, 14),
                               Qt.AlignVCenter | Qt.AlignLeft, fmt % v)
        p.setClipping(False)
        # current value box
        bh = 16
        box = QRectF(rect.left(), cy - bh / 2, rect.width(), bh)
        p.setBrush(QColor(10, 10, 12))
        p.setPen(QPen(YELLOW, 1.3))
        p.drawRect(box)
        p.setPen(QColor(255, 255, 255))
        p.setFont(QFont("DejaVu Sans Mono", 9, QFont.Bold))
        p.drawText(box, Qt.AlignCenter, f"{value:.0f}")

    def _vsi(self, p, x, y0, hh):
        # thin climb bar just inside the right tape
        bx = x - 5
        cy = y0 + hh / 2
        p.setPen(QPen(QColor(120, 124, 132), 1))
        p.drawLine(QPointF(bx, y0 + 4), QPointF(bx, y0 + hh - 4))
        mag = max(-5.0, min(5.0, self.climb))
        yend = cy - (mag / 5.0) * (hh / 2 - 6)
        p.setPen(QPen(QColor(120, 230, 140) if mag >= 0 else QColor(240, 140, 90), 3))
        p.drawLine(QPointF(bx, cy), QPointF(bx, yend))

    def _heading_strip(self, p, x0, hw, head_h):
        rect = QRectF(x0, 0, hw, head_h)
        p.fillRect(rect, QColor(0, 0, 0, 150))
        p.setClipRect(rect)
        cx = rect.center().x()
        ppd = hw / 60.0                       # +/-30 deg
        p.setFont(QFont("DejaVu Sans Mono", 7))
        base = math.floor(self.heading / 10.0) * 10
        cards = {0: "N", 90: "E", 180: "S", 270: "W"}
        for k in range(-5, 6):
            deg = (base + k * 10)
            x = cx + (deg - self.heading) * ppd
            d = deg % 360
            p.setPen(QPen(QColor(210, 214, 220), 1))
            p.drawLine(QPointF(x, rect.bottom() - 4), QPointF(x, rect.bottom()))
            p.drawText(QRectF(x - 14, 0, 28, head_h - 3), Qt.AlignCenter, cards.get(d, str(d)))
        p.setClipping(False)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(YELLOW))
        p.drawPolygon(QPolygonF([QPointF(cx, head_h), QPointF(cx - 5, head_h - 6), QPointF(cx + 5, head_h - 6)]))


class Compass(QWidget):
    """Heading-up compass card driven by heading (degrees)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.heading = 0.0
        self.setMinimumSize(130, 130)

    def sizeHint(self):
        return QSize(200, 200)

    def set_heading(self, hdg):
        self.heading = hdg % 360.0
        self.update()

    def paintEvent(self, _):
        w, h = self.width(), self.height()
        R = min(w, h) * 0.5 - 6
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.translate(w / 2.0, h / 2.0)

        grad = QRadialGradient(QPointF(0, 0), R)
        grad.setColorAt(0.0, QColor(34, 36, 42))
        grad.setColorAt(1.0, QColor(18, 19, 24))
        p.setBrush(QBrush(grad))
        p.setPen(QPen(QColor(20, 20, 24), 4))
        p.drawEllipse(QPointF(0, 0), R, R)

        cards = {0: "N", 90: "E", 180: "S", 270: "W"}
        p.save()
        p.rotate(-self.heading)
        for deg in range(0, 360, 10):
            p.save()
            p.rotate(deg)
            major = (deg % 30 == 0)
            ln = R * 0.14 if major else R * 0.07
            p.setPen(QPen(QColor(220, 220, 220), 2 if major else 1))
            p.drawLine(QPointF(0, -R + 3), QPointF(0, -R + 3 + ln))
            if deg in cards or major:
                label = cards.get(deg, str(deg // 10))
                p.setFont(QFont("DejaVu Sans", int(R * (0.13 if deg in cards else 0.09)),
                                QFont.Bold if deg in cards else QFont.Normal))
                p.setPen(QColor(255, 90, 90) if deg == 0 else QColor(235, 235, 235))
                p.save()
                p.translate(0, -R + 3 + ln + R * 0.13)
                p.rotate(self.heading - deg)        # keep labels upright
                p.drawText(QRectF(-20, -12, 40, 24), Qt.AlignCenter, label)
                p.restore()
            p.restore()
        p.restore()

        # fixed lubber index
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(YELLOW))
        p.drawPolygon(QPolygonF([QPointF(0, -R + 2),
                                 QPointF(-R * 0.06, -R + 2 + R * 0.12),
                                 QPointF(R * 0.06, -R + 2 + R * 0.12)]))

        # digital readout
        p.setBrush(QBrush(QColor(0, 0, 0, 160)))
        p.setPen(QPen(QColor(80, 80, 90), 1))
        box = QRectF(-R * 0.42, -R * 0.22, R * 0.84, R * 0.44)
        p.drawRoundedRect(box, 6, 6)
        p.setPen(QColor(0, 230, 120))
        p.setFont(QFont("DejaVu Sans Mono", int(R * 0.22), QFont.Bold))
        p.drawText(box, Qt.AlignCenter, f"{int(round(self.heading)) % 360:03d}")
        p.end()
