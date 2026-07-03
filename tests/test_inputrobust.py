#!/usr/bin/env python3
"""test_inputrobust.py -- user text-input robustness. Hand-typed connection strings and console
commands must never crash the app:
  * LINK TARGET parse in _connect: a port outside 1..65535 (e.g. '99999', '-5', a huge number)
    reaches Qt's socket bind/connect as an out-of-uint16 value and raises OverflowError -- NOT
    ValueError -- which escaped _connect's handler and crashed the Connect action (iter105 fix:
    _parse_port validates the range, and the handler also catches OverflowError).
  * MAVLINK CONSOLE send: arbitrary/huge/non-ascii shell input must encode safely (utf-8 'replace',
    count masked to uint8, data truncated to the 70-byte SERIAL_CONTROL field) -- no crash.
No real vehicle, no arming; a failed connect must report, not raise."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mavlink
import main as m
from main import _parse_port
from panels import MavlinkConsole

app = QApplication.instance() or QApplication([])

# 1) _parse_port: valid in range, everything else raises ValueError (which _connect catches) --------
assert _parse_port("5760", 1) == 5760
assert _parse_port("", 14550) == 14550 and _parse_port("   ", 14550) == 14550
assert _parse_port("65535", 1) == 65535 and _parse_port("1", 1) == 1
for bad in ("0", "-1", "65536", "99999", "999999999999", "abc", "3.5", "0x10"):
    try:
        _parse_port(bad, 1)
        raise AssertionError(f"_parse_port accepted out-of-range/junk {bad!r}")
    except ValueError:
        pass

# 2) _connect with malformed targets across every transport -> no uncaught exception ---------------
win = m.DroneDeck(14722)
win._persist = False
CASES = [
    ("UDP", "99999"), ("UDP", "999999999999"), ("UDP", "-5"), ("UDP", "abc"), ("UDP", "0"),
    ("UDP", ""),
    ("TCP", "host:99999"), ("TCP", "host:abc"), ("TCP", "host:-1"), ("TCP", ":"), ("TCP", ""),
    ("TCP", "h:o:s:t:5760"), ("TCP", "127.0.0.1:70000"),
    ("Serial", "/dev/ttyNope:999999999"), ("Serial", ":abc"), ("Serial", ""),
    ("Replay", "/no/such.tlog@nan"), ("Replay", "@-5"), ("Replay", ""),
]
for t, tgt in CASES:
    win.transport_combo.setCurrentText(t)
    win.link_edit.setText(tgt)
    win._connect()                       # must not raise (a bad target -> status message, is_open False)
    if win.link is not None and win.link.is_open:
        win.link.close()

# a valid UDP target still connects (regression check that the guard didn't break the happy path)
win.transport_combo.setCurrentText("UDP")
win.link_edit.setText("")                # empty -> default port
win._connect()
assert win.link is not None and win.link.is_open, "valid UDP connect broke"
win.link.close()

# 3) MAVLINK CONSOLE: adversarial shell input encodes without crashing ------------------------------
con = MavlinkConsole()
sent = []
con.send_bytes.connect(lambda b: sent.append(b))
for text in ["", "reboot", "A" * 100000, "non-ascii é中文 \U0001f600",
             "nulls \x00\x01\x02 embedded", "  \t weird \r\n whitespace  "]:
    con.inp.setText(text)
    con._send()                          # emits send_bytes, must not raise
assert len(sent) == 6
# every emitted payload packs into a valid 81-byte SERIAL_CONTROL (count masked, data truncated)
for b in sent:
    payload = mavlink.enc_serial_control(mavlink.SERIAL_CONTROL_DEV_SHELL, 0, b)
    assert len(payload) == 81, f"SERIAL_CONTROL payload wrong size: {len(payload)}"

# and the console's own receive path decodes junk SERIAL_CONTROL without crashing
class Msg:
    __slots__ = ("msgid", "sysid", "compid", "seq", "fields")
    def __init__(self, msgid, **f):
        self.msgid, self.sysid, self.compid, self.seq, self.fields = msgid, 1, 1, 0, f


con.handle_messages([Msg(mavlink.SERIAL_CONTROL, count=255, data=b"\xff\xfe\x00 garbage \x80"),
                     Msg(mavlink.SERIAL_CONTROL, count=999, data=b"short")])   # count > len(data)

print("INPUTROBUST PASSED (out-of-range ports rejected cleanly; console encodes any input safely)")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
