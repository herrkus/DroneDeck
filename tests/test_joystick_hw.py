#!/usr/bin/env python3
"""test_joystick_hw.py -- real hardware gamepad/stick reader (iter133).

DroneDeck had only an on-screen virtual joystick; HwJoystick reads a Linux /dev/input/jsN device
and maps its axes to the same MANUAL_CONTROL contract (x=pitch, y=roll, z=thrust, r=yaw, -1000..1000)
so the manual send loop can use either. Verifies the js_event parsing, axis map + inversion,
normalization, deadzone, and fail-safe neutral on disconnect -- all without a real device, by opening
a temp file as the "device" and feeding synthetic js_event bytes. Also checks the GUI source combo."""
import os
import sys
import struct
import tempfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from joystick import HwJoystick, list_joysticks

fail = []
EV = struct.Struct("<IhBB")            # js_event: time, value, type, number


def axis(number, value):
    return EV.pack(0, value, 0x02, number)      # 0x02 = JS_EVENT_AXIS


def button(number, value):
    return EV.pack(0, value, 0x01, number)      # 0x01 = JS_EVENT_BUTTON


tf = tempfile.NamedTemporaryFile(delete=False)
tf.close()
js = HwJoystick(tf.name)                # temp file stands in for /dev/input/jsN
if not js.is_open:
    fail.append("HwJoystick should open a readable path")

# default map: yaw=axis0, throttle=axis1(inv), roll=axis3, pitch=axis4(inv) --------------------------
js.feed(axis(0, 32767))                 # yaw full right -> r = +1000
js.feed(axis(1, -32767))                # throttle stick up (raw -max), inverted -> z = +1000 (climb)
js.feed(axis(3, -32767))                # roll full left -> y = -1000
js.feed(axis(4, 32767))                 # pitch stick back (raw +max), inverted -> x = -1000
x, y, z, r = js.values()
if not (abs(x + 1000) < 1 and abs(y + 1000) < 1 and abs(z - 1000) < 1 and abs(r - 1000) < 1):
    fail.append(f"axis map/normalize wrong: x={x} y={y} z={z} r={r}")

# a mid value normalizes proportionally (16384/32767 ~ 0.5 -> ~500) ---------------------------------
js.feed(axis(0, 16384))
if not abs(js.values()[3] - 500) < 6:
    fail.append(f"half-deflection should be ~500, got {js.values()[3]}")

# deadzone: |raw| < 6% -> 0 ------------------------------------------------------------------------
js.feed(axis(0, 1500))                  # 1500/32767 = 0.046 < 0.06
if js.values()[3] != 0.0:
    fail.append(f"deadzone: small yaw should be 0, got {js.values()[3]}")

# buttons don't move axes; a concatenated multi-event blob parses -----------------------------------
before = js.values()
js.feed(button(3, 1) + button(4, 0))
if js.values() != before:
    fail.append("button events must not change axis values")

# recenter -> neutral ------------------------------------------------------------------------------
js.feed(axis(0, 0) + axis(1, 0) + axis(3, 0) + axis(4, 0))
if js.values() != (0.0, 0.0, 0.0, 0.0):
    fail.append(f"recentered axes should be neutral, got {js.values()}")

# disconnect -> fail-safe neutral + safe to poll ---------------------------------------------------
js.feed(axis(0, 32767))                 # deflect, then yank the device
js.close()
if js.is_open or js.values() != (0.0, 0.0, 0.0, 0.0):
    fail.append("a closed device must report neutral")
js.poll()                               # must not raise
os.unlink(tf.name)

# custom axis map honoured -------------------------------------------------------------------------
tf2 = tempfile.NamedTemporaryFile(delete=False)
tf2.close()
js2 = HwJoystick(tf2.name, axis_map={"yaw": (2, False), "throttle": (5, False),
                                     "roll": (0, False), "pitch": (1, False)}, deadzone=0.0)
js2.feed(axis(2, 32767))                # yaw now on axis 2
if abs(js2.values()[3] - 1000) < 1:
    pass
else:
    fail.append(f"custom axis map ignored: {js2.values()}")
js2.close()
os.unlink(tf2.name)

if not isinstance(list_joysticks(), list):
    fail.append("list_joysticks should return a list")

# GUI: the manual-input source combo exists and defaults to the virtual pad -------------------------
from PySide6.QtWidgets import QApplication
app = QApplication.instance() or QApplication([])
import main as m
win = m.DroneDeck(14740)
win._persist = False
if win.joy_source.count() < 1 or win.joy_source.itemData(0) is not None:
    fail.append("source combo should start with the Virtual pad (data None)")
if win.hw_joystick is not None:
    fail.append("hardware joystick should start unset (virtual)")

print("JOYSTICK_HW FAILED: " + "; ".join(fail) if fail else
      "JOYSTICK_HW PASSED (js_event parse + axis map/invert/normalize/deadzone; disconnect fail-safe; "
      "custom map; GUI source combo defaults to virtual)")
sys.stdout.flush()
os._exit(1 if fail else 0)
