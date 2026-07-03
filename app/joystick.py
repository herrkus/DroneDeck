"""joystick.py -- manual control input: an on-screen virtual joystick plus a real hardware
gamepad/stick reader.

The virtual joystick is two spring-centred thumbsticks like QGroundControl's: left = throttle
(vertical) + yaw (horizontal), right = pitch (vertical) + roll (horizontal). HwJoystick reads a
real Linux joystick (/dev/input/jsN) and maps its axes to the same value contract, so the
manual-control send loop can source from either. Axes are read by that loop in main; the virtual
pads can also be driven from the keyboard via set_keys().
"""
from __future__ import annotations
import glob
import math
import os
import struct

from PySide6.QtCore import Qt, QSize, QPointF, QRectF
from PySide6.QtGui import QPainter, QColor, QPen, QRadialGradient, QBrush, QFont
from PySide6.QtWidgets import QWidget, QHBoxLayout

# Linux legacy joystick API (/dev/input/jsN): fixed 8-byte events, no external deps needed.
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
_JS_EVENT = struct.Struct("<IhBB")      # time(u32), value(i16), type(u8), number(u8)


def list_joysticks():
    """Paths of connected legacy-API joysticks, e.g. ['/dev/input/js0']."""
    return sorted(glob.glob("/dev/input/js*"))


class HwJoystick:
    """Reads a real Linux joystick (/dev/input/jsN) and maps its axes to MANUAL_CONTROL
    (x=pitch, y=roll, z=thrust, r=yaw, each -1000..1000) -- the SAME contract as
    VirtualJoystick.values(), so main's send loop can use either interchangeably.

    Dependency-free: parses the 8-byte js_event struct directly and reads non-blocking, so a
    missing/slow device never stalls the UI. Unplugging the device closes it cleanly (values()
    then returns neutral). The default axis map suits a common Mode-2 gamepad; override for others.
    """

    # role -> (axis index, invert).  Xbox-style: L-stick = throttle+yaw, R-stick = pitch+roll.
    DEFAULT_MAP = {
        "yaw":      (0, False),     # left stick X   -> yaw (right = +)
        "throttle": (1, True),      # left stick Y   -> thrust (up = -raw = climb)
        "roll":     (3, False),     # right stick X  -> roll (right = +)
        "pitch":    (4, True),      # right stick Y  -> pitch (up = -raw = forward)
    }

    def __init__(self, path, axis_map=None, deadzone=0.06):
        self.path = path
        self.axis_map = dict(axis_map or self.DEFAULT_MAP)
        self.deadzone = max(0.0, min(0.9, deadzone))
        self._axes = {}             # axis index -> raw value (-32767..32767)
        self._buttons = {}          # button index -> bool (current state)
        self._presses = set()       # button indices that had a rising edge since the last take_presses()
        self._fd = None
        self.open()

    def open(self):
        try:
            self._fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            self._fd = None

    @property
    def is_open(self):
        return self._fd is not None

    def close(self):
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None

    def feed(self, data):
        """Fold raw js_event bytes into the current axis + button state (also the test seam)."""
        for i in range(0, len(data) - 7, 8):
            _t, value, etype, number = _JS_EVENT.unpack_from(data, i)
            if etype & JS_EVENT_AXIS:               # axis motion
                self._axes[number] = value
            elif etype & JS_EVENT_BUTTON:           # button state; INIT bit set = the open-time snapshot
                if value and not (etype & JS_EVENT_INIT):   # rising edge only (ignore held-at-open)
                    self._presses.add(number)
                self._buttons[number] = bool(value)

    def take_presses(self):
        """Return the set of buttons pressed (rising edge) since the last call, and clear it.
        Returns empty if the device is closed -- so a disconnect never replays stale presses."""
        if self._fd is None:
            self._presses.clear()
            return set()
        p = self._presses
        self._presses = set()
        return p

    def poll(self):
        """Drain pending events without blocking. Closes on unplug (read raises OSError)."""
        if self._fd is None:
            return
        try:
            while True:
                chunk = os.read(self._fd, 8 * 64)
                if not chunk:
                    break
                self.feed(chunk)
        except BlockingIOError:
            pass                                    # no more events queued -- normal
        except OSError:
            self.close()                            # device went away

    def _axis(self, role):
        idx, invert = self.axis_map[role]
        v = max(-1.0, min(1.0, self._axes.get(idx, 0) / 32767.0))
        if abs(v) < self.deadzone:
            return 0.0
        return (-v if invert else v) * 1000.0

    def values(self):
        """(x=pitch, y=roll, z=thrust, r=yaw), each -1000..1000 -- matches VirtualJoystick."""
        if self._fd is None:
            return 0.0, 0.0, 0.0, 0.0               # disconnected -> neutral (fail safe)
        return self._axis("pitch"), self._axis("roll"), self._axis("throttle"), self._axis("yaw")


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
