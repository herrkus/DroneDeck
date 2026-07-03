#!/usr/bin/env python3
"""test_motortest.py -- Vehicle Setup > Motors (DO_MOTOR_TEST 209), QGroundControl-style.

Motor test spins real motors, so the widget MUST refuse to command anything until the operator
acknowledges the propellers are removed. This verifies: the safety gate (no emission + disabled
buttons until acked, and re-armed correctly when the motor count changes), the emitted
(motor, throttle, duration, count) for single + sequence tests, that link.motor_test() builds the
correct COMMAND_LONG params (throttle-as-percent, sequence order only when count>0), and that the
command survives the real encode->parse wire. No arming, no real motors."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mavlink
import core
from calibration import MotorTestWidget
from link import UdpLink

app = QApplication.instance() or QApplication([])

# 1) SAFETY GATE: spin buttons disabled + nothing emitted until "props removed" is acknowledged -----
w = MotorTestWidget()
emits = []
w.motorTestRequested.connect(lambda m, t, d, c: emits.append((m, t, d, c)))
assert not w.btn_all.isEnabled() and all(not b.isEnabled() for b in w._motor_btns), \
    "motor spin was enabled without the props-removed acknowledgement"
w._test_one(2)                                   # attempt while unsafe -> must NOT emit
w._test_all()
assert emits == [], f"motor test fired without the safety acknowledgement: {emits}"

w.safety.setChecked(True)                         # acknowledge props removed
assert w.btn_all.isEnabled() and all(b.isEnabled() for b in w._motor_btns)

# 2) single-motor test emits (motor, throttle, duration, count=0) ----------------------------------
w.throttle.setValue(10)
w.duration.setValue(3)
w._test_one(3)
assert emits[-1] == (3, 10.0, 3.0, 0), emits[-1]

# 3) test-all emits count = number of motors, starting at motor 1 ----------------------------------
w.count.setValue(6)
w._test_all()
assert emits[-1] == (1, 10.0, 3.0, 6), emits[-1]

# 4) changing the motor count rebuilds the buttons but must NOT bypass the safety gate -------------
w.safety.setChecked(False)
w.count.setValue(4)
assert all(not b.isEnabled() for b in w._motor_btns) and not w.btn_all.isEnabled(), \
    "rebuilt motor buttons ignored the safety gate"

# 5) link.motor_test() builds the correct DO_MOTOR_TEST COMMAND_LONG params -------------------------
sent = []
lk = UdpLink()
lk.send_command_long = lambda tgt, cmd, params: sent.append((tgt, cmd, list(params)))
lk.motor_test(1, motor=3, throttle_pct=8, duration_s=2, count=0)         # single motor
tgt, cmd, p = sent[-1]
assert cmd == mavlink.MAV_CMD_DO_MOTOR_TEST == 209, cmd
assert p[0] == 3, "motor number wrong"
assert p[1] == mavlink.MOTOR_TEST_THROTTLE_PERCENT, "throttle type must be PERCENT"
assert p[2] == 8 and p[3] == 2, "throttle/duration wrong"
assert p[4] == 0 and p[5] == mavlink.MOTOR_TEST_ORDER_DEFAULT, "single-motor order/count wrong"
lk.motor_test(1, motor=1, throttle_pct=8, duration_s=2, count=4)         # all in sequence
p = sent[-1][2]
assert p[4] == 4 and p[5] == mavlink.MOTOR_TEST_ORDER_SEQUENCE, "sequence order not set for count>0"

# 6) real wire: encode a DO_MOTOR_TEST COMMAND_LONG -> parse -> command + params intact -------------
frame = core.encode_command_long(255, 190, 0, 1, 1, mavlink.MAV_CMD_DO_MOTOR_TEST,
                                 [3, 0, 8, 2, 0, 0, 0])
parser = core.Parser()
batch = parser.feed(frame)
cmds = [m for m in batch if m.msgid == mavlink.COMMAND_LONG]
assert cmds, "DO_MOTOR_TEST COMMAND_LONG did not decode"
f = cmds[0].fields
assert int(f["command"]) == 209 and abs(f["param1"] - 3) < 1e-6 and abs(f["param3"] - 8) < 1e-6

print("MOTORTEST PASSED (safety gate blocks spin without props-removed ack; correct DO_MOTOR_TEST "
      "params + sequence order; wire round-trip)")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
