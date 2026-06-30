"""joystick.py -- on-screen virtual joystick for manual control.

Two spring-centred thumbsticks like QGroundControl's virtual joystick:
left = throttle (vertical) + yaw (horizontal), right = pitch (vertical) +
roll (horizontal). Axes are read by the manual-control send loop in main; the
pads can also be driven from the keyboard via set().
"""
from __future__ import annotations
import math

from PySide6.QtCore import Qt, QSize, QPointF, QRectF
from PySide6.QtGui import QPainter, QColor, QPen, QRadialGradient, QBrush, QFont
from PySide6.QtWidgets import QWidget, QHBoxLayout


class JoystickPad(QWidget):
    def __init__(self, hlabel="", vlabel="", parent=None):
        super().__init__(parent)
        self.ax = 0.0          # -1 (left) .. +1 (right)
        self.ay = 0.0          # -1 (up)   .. +1 (down)
        self.hlabel, self.vlabel = hlabel, vlabel
        self._dragging = False
        self.setMinimumSize(120, 120)

    def sizeHint(self):
        return QSize(160, 160)

    def set(self, ax, ay):
        self.ax = max(-1.0, min(1.0, ax))
        self.ay = max(-1.0, min(1.0, ay))
        self.update()

    def _geom(self):
        cx, cy = self.width() / 2.0, self.height() / 2.0
        R = min(cx, cy) - 10
        return cx, cy, R

    def _from_pos(self, pos):
        cx, cy, R = self._geom()
        dx, dy = (pos.x() - cx) / R, (pos.y() - cy) / R
        m = math.hypot(dx, dy)
        if m > 1.0:
            dx, dy = dx / m, dy / m
        self.set(dx, dy)

    def mousePressEvent(self, e):
        self._dragging = True
        self._from_pos(e.position())

    def mouseMoveEvent(self, e):
        if self._dragging:
            self._from_pos(e.position())

    def mouseReleaseEvent(self, e):
        self._dragging = False
        self.set(0.0, 0.0)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        cx, cy, R = self._geom()
        grad = QRadialGradient(QPointF(cx, cy), R)
        grad.setColorAt(0.0, QColor(40, 44, 52))
        grad.setColorAt(1.0, QColor(24, 26, 32))
        p.setBrush(QBrush(grad))
        p.setPen(QPen(QColor(60, 66, 78), 2))
        p.drawEllipse(QPointF(cx, cy), R, R)
        p.setPen(QPen(QColor(70, 76, 88), 1, Qt.DashLine))
        p.drawLine(QPointF(cx - R, cy), QPointF(cx + R, cy))
        p.drawLine(QPointF(cx, cy - R), QPointF(cx, cy + R))
        p.setPen(QColor(120, 126, 138))
        p.setFont(QFont("DejaVu Sans", 7))
        if self.hlabel:
            p.drawText(QRectF(cx, cy + 2, R - 4, 14), Qt.AlignRight | Qt.AlignTop, self.hlabel)
        if self.vlabel:
            p.drawText(QRectF(cx + 4, cy - R + 2, R - 8, 14), Qt.AlignLeft | Qt.AlignTop, self.vlabel)
        # knob
        kx, ky = cx + self.ax * R, cy + self.ay * R
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(70, 160, 230) if (self.ax or self.ay) else QColor(150, 156, 166))
        p.drawEllipse(QPointF(kx, ky), R * 0.22, R * 0.22)
        p.end()


class VirtualJoystick(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(10)
        self.left = JoystickPad("Yaw", "Thr")
        self.right = JoystickPad("Roll", "Pitch")
        lay.addWidget(self.left)
        lay.addWidget(self.right)

    def values(self):
        """(x=pitch, y=roll, z=thrust, r=yaw), each -1000..1000."""
        x = -self.right.ay * 1000.0     # stick up -> +pitch (forward)
        y = self.right.ax * 1000.0      # stick right -> +roll
        z = -self.left.ay * 1000.0      # stick up -> +thrust (climb)
        r = self.left.ax * 1000.0       # stick right -> +yaw (CW)
        return x, y, z, r

    def set_keys(self, fwd, right, up, yaw):
        """Drive the pads from keyboard axes (each -1..1)."""
        self.right.set(right, -fwd)
        self.left.set(yaw, -up)
