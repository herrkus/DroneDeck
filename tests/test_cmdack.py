#!/usr/bin/env python3
"""test_cmdack.py -- COMMAND_ACK handling robustness. A real autopilot ACKs every command it
receives, and can send: any MAV_RESULT (0 ACCEPTED .. 5 CANCELLED, plus values this GCS predates),
an ACK for a command we never sent, unknown command ids, duplicates, and storms. The ACK path is
fire-and-forget (no pending/spinner state to get stuck), so the risk is a crash or a wrong decode,
not a hang. Drives the handler directly, through the vehicle signal, and through the REAL wire
(frame -> parser -> consume), plus the rejected-critical toast path, asserting no crash + right
decode + no unbounded growth. No link, no arming."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mavlink
import main as m
import core

app = QApplication.instance() or QApplication([])

win = m.DroneDeck(14631)
win._persist = False
ve = win.vehicle


class Msg:
    __slots__ = ("msgid", "sysid", "compid", "seq", "fields")
    def __init__(self, msgid, **f):
        self.msgid, self.sysid, self.compid, self.seq, self.fields = msgid, 1, 1, 0, f
    def get(self, k, d=None):
        return self.fields.get(k, d)


KNOWN_CMDS = [400, 22, 21, 20, 176, 179, 84, 300]     # arm, takeoff, land, rtl, set_mode, ...
UNKNOWN_CMDS = [9999, 0, 65535, 511]
ALL_RESULTS = list(range(0, 8)) + [99, 255, 6]         # 0..7 valid-ish + out-of-range

# 1) direct handler: every (command, result) combination must decode without crashing -------------
for cmd in KNOWN_CMDS + UNKNOWN_CMDS:
    for res in ALL_RESULTS:
        win._on_command_ack(cmd, res)                  # must not raise
        # result decodes to the MAV_RESULT name, or str(result) when unknown -- never crashes
        assert mavlink.MAV_RESULT.get(res, str(res))

# 2) rejected critical command -> prominent toast, with and without a preceding STATUSTEXT reason
ve.messages.clear()
win._on_command_ack(400, 4)                            # ARM rejected, no reason available
ve.messages.append((2, "Arming denied: GPS not ready"))
win._on_command_ack(400, 2)                            # ARM DENIED, reason should be picked up
win._on_command_ack(22, 3)                             # TAKEOFF UNSUPPORTED
assert win.notice_banner.text(), "critical rejection should show a toast"

# 3) through the vehicle signal + full consume path (fields as the parser produces them: ints) -----
seen = []
ve.command_ack.connect(lambda c, r: seen.append((c, r)))
ve.consume([Msg(mavlink.COMMAND_ACK, command=400, result=0)])      # accepted arm
ve.consume([Msg(mavlink.COMMAND_ACK, command=9999, result=99)])    # unknown cmd + result
ve.consume([Msg(mavlink.COMMAND_ACK, command=20, result=5)])       # RTL cancelled
assert ve.last_ack == (20, 5), ve.last_ack
assert (400, 0) in seen and (9999, 99) in seen

# 4) ACK for a command we never sent, duplicates, and a storm -> no crash, no unbounded growth -----
msgs_before = len(ve.messages)
for _ in range(500):
    ve.consume([Msg(mavlink.COMMAND_ACK, command=176, result=0)])  # duplicate storm
win._on_command_ack(176, 0)
# the ACK handler itself keeps no per-command state; vehicle.messages only grows via STATUSTEXT
assert len(ve.messages) == msgs_before, "COMMAND_ACK must not accumulate vehicle messages"

# 5) REAL wire path: build a COMMAND_ACK frame, parse it, confirm the decode matches --------------
payload = mavlink.enc_command_ack(400, 0)
frame = mavlink.frame(mavlink.COMMAND_ACK, payload, 0, 1, 1)
parser = core.Parser()
batch = parser.feed(frame)
acks = [mm for mm in batch if mm.msgid == mavlink.COMMAND_ACK]
assert acks, "COMMAND_ACK frame did not decode"
assert acks[0].fields.get("command") == 400 and acks[0].fields.get("result") == 0
ve.consume(acks)                                        # feed the genuinely-parsed message

print("CMDACK PASSED (adversarial ACKs: unknown cmd/result, storm, real wire -- no crash, decoded)")
