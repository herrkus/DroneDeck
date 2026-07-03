#!/usr/bin/env python3
"""test_joystick_buttons.py -- gamepad button -> action mapping (1:1 QGC parity).

HwJoystick parsed axes but ignored buttons; QGC maps gamepad buttons to actions. Added rising-edge
button tracking (take_presses) and a default Xbox A/B/X/Y -> arm/disarm/RTL/land map fired from the
manual-control loop. Verifies: the open-time INIT snapshot is not a press, real presses are reported
once and cleared, releases don't count, a closed device replays nothing, and the main window routes
each mapped button to the right link call (guarded by a connected vehicle)."""
import os
import sys
import struct
import tempfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
from joystick import HwJoystick

fail = []
EV = struct.Struct("<IhBB")


def btn(number, value, init=False):
    etype = 0x01 | (0x80 if init else 0)      # JS_EVENT_BUTTON (+ JS_EVENT_INIT on open snapshot)
    return EV.pack(0, value, etype, number)


tf = tempfile.NamedTemporaryFile(delete=False)
tf.close()
js = HwJoystick(tf.name)

js.feed(btn(0, 1, init=True))                 # button held at open -> INIT snapshot, NOT a press
if js.take_presses():
    fail.append("INIT button snapshot must not count as a press")

js.feed(btn(2, 1))                            # real press
if js.take_presses() != {2}:
    fail.append("a real press should be reported once")
if js.take_presses():
    fail.append("presses must clear after being taken")

js.feed(btn(2, 0))                            # release
if js.take_presses():
    fail.append("a release must not count as a press")

js.feed(btn(1, 1) + btn(3, 1))               # two presses in one blob
if js.take_presses() != {1, 3}:
    fail.append("multiple presses should accumulate")

js.feed(btn(0, 1))
js.close()                                    # unplugged with a press pending
if js.take_presses():
    fail.append("a closed device must replay no presses")
os.unlink(tf.name)

# main window routes each mapped button to the right link call, guarded by a vehicle ----------------
from PySide6.QtWidgets import QApplication
app = QApplication.instance() or QApplication([])
import main as m
win = m.DroneDeck(16310)
win._persist = False
calls = []
win.link.arm = lambda s, on: calls.append(("arm", on))
win.link.rtl = lambda s: calls.append(("rtl",))
win.link.land = lambda s: calls.append(("land",))
win._has_vehicle = lambda: True
win._sysid = lambda: 1
for b in (0, 1, 2, 3):
    win._joy_button(b)
win._joy_button(9)                            # unmapped -> nothing
if calls != [("arm", True), ("arm", False), ("rtl",), ("land",)]:
    fail.append(f"button routing wrong: {calls}")

calls.clear()
win._has_vehicle = lambda: False
win._joy_button(0)
if calls:
    fail.append("no-vehicle: a button must not fire an action")

print("JOYSTICK_BUTTONS FAILED: " + "; ".join(fail) if fail else
      "JOYSTICK_BUTTONS PASSED (INIT snapshot ignored; rising-edge press reported once + cleared; "
      "release/closed replay nothing; A/B/X/Y -> arm/disarm/RTL/land routed + vehicle-guarded)")
sys.stdout.flush()
os._exit(1 if fail else 0)
