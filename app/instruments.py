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
    """Artificial horizon driven by roll/pitch (radians)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0
        self.setMinimumSize(140, 140)

    def sizeHint(self):
        return QSize(220, 220)

    def set_attitude(self, roll, pitch):
        self.roll, self.pitch = roll, pitch
        self.update()

    def paintEvent(self, _):
        w, h = self.width(), self.height()
        side = min(w, h)
        R = side * 0.5 - 6
        ppd = R / 32.0                       # pixels per degree of pitch
        roll_deg = math.degrees(self.roll)
        pitch_deg = math.degrees(self.pitch)

        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.translate(w / 2.0, h / 2.0)

        # circular instrument face
        face = QPainterPath()
        face.addEllipse(QPointF(0, 0), R, R)
        p.setClipPath(face)

        # ---- rotating horizon + pitch ladder --------------------------------
        p.save()
        p.rotate(-roll_deg)
        horizon_y = pitch_deg * ppd
        big = R * 3
        p.fillRect(QRectF(-big, -big, 2 * big, big + horizon_y), QBrush(SKY))
        p.fillRect(QRectF(-big, horizon_y, 2 * big, big), QBrush(GROUND))
        p.setPen(QPen(LINE, 2))
        p.drawLine(QPointF(-big, horizon_y), QPointF(big, horizon_y))

        p.setFont(QFont("DejaVu Sans", max(7, int(R * 0.06))))
        for a in range(-30, 31, 10):
            if a == 0:
                continue
            y = (pitch_deg - a) * ppd
            if abs(y) > R - 4:
                continue
            half = R * 0.26 if a % 30 == 0 else R * 0.15
            p.setPen(QPen(LINE, 1.4))
            p.drawLine(QPointF(-half, y), QPointF(half, y))
            p.drawText(QRectF(half + 4, y - 8, 34, 16),
                       Qt.AlignVCenter | Qt.AlignLeft, str(abs(a)))
            p.drawText(QRectF(-half - 38, y - 8, 34, 16),
                       Qt.AlignVCenter | Qt.AlignRight, str(abs(a)))

        # bank tick scale, fixed to the horizon (rotates with it)
        p.setPen(QPen(LINE, 1.6))
        for t in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            p.save()
            p.rotate(t)
            ln = R * 0.12 if t % 30 == 0 else R * 0.07
            p.drawLine(QPointF(0, -R), QPointF(0, -R + ln))
            p.restore()
        p.restore()

        # ---- fixed overlay: bank pointer + aircraft symbol ------------------
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(YELLOW))
        tri = QPolygonF([QPointF(0, -R + R * 0.13),
                         QPointF(-R * 0.05, -R + R * 0.13 + R * 0.08),
                         QPointF(R * 0.05, -R + R * 0.13 + R * 0.08)])
        p.drawPolygon(tri)

        pen = QPen(YELLOW, 3)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        wing = R * 0.55
        p.drawLine(QPointF(-wing, 0), QPointF(-wing * 0.35, 0))
        p.drawLine(QPointF(wing * 0.35, 0), QPointF(wing, 0))
        p.drawLine(QPointF(-wing * 0.35, 0), QPointF(-wing * 0.35, R * 0.10))
        p.drawLine(QPointF(wing * 0.35, 0), QPointF(wing * 0.35, R * 0.10))
        p.setBrush(QBrush(YELLOW))
        p.drawEllipse(QPointF(0, 0), 3, 3)

        # bezel
        p.setClipping(False)
        p.setPen(QPen(QColor(20, 20, 24), 6))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(0, 0), R, R)
        p.end()


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
