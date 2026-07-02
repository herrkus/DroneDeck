#!/usr/bin/env python3
"""test_command_retry.py -- confirmed-command delivery (ACK retry) hardening.

Real RF telemetry links drop packets; a fire-and-forget arm/mode/land command can silently vanish.
Link now resends confirm=True commands until a COMMAND_ACK arrives (up to ACK_MAX_TRIES), then emits
command_unacked so the GUI can warn -- matching QGroundControl/MAVSDK. Verifies: non-confirm sends
are not tracked; a confirmed command retries on the right schedule and gives up after ACK_MAX_TRIES;
a COMMAND_ACK (both via _match_acks and a real frame through _ingest) resolves it and stops resends;
an unrelated ACK is ignored; and the real arm/set_mode/land/rtl methods opt in to confirmation."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import core
import mavlink
from mavlink import Message
from link import Link

app = QApplication.instance() or QApplication([])
fail = []


class CaptureLink(Link):
    """A Link with a fake open transport that records every frame it 'sends'."""
    def __init__(self):
        super().__init__()
        self.sent = []
        self._open = True
        self.remote = True

    def _write(self, data):
        self.sent.append(data)


ARM = mavlink.MAV_CMD_COMPONENT_ARM_DISARM
MODE = mavlink.MAV_CMD_DO_SET_MODE

lk = CaptureLink()
acked, unacked = [], []
lk.command_result.connect(lambda c, r: acked.append((c, r)))
lk.command_unacked.connect(lambda c: unacked.append(c))

# 1) a plain (fire-and-forget) send is NOT tracked --------------------------------------------------
lk.send_command_long(1, mavlink.MAV_CMD_SET_CAMERA_ZOOM, [0, 0, 0, 0, 0, 0, 0])
if len(lk.sent) != 1 or lk._pending:
    fail.append(f"non-confirm send should not track (sent={len(lk.sent)}, pending={lk._pending})")

# 2) a confirmed send retries on schedule, then gives up after ACK_MAX_TRIES -------------------------
lk.sent.clear()
lk.send_command_long(1, ARM, [1, 0, 0, 0, 0, 0, 0], confirm=True)
if len(lk.sent) != 1 or ARM not in lk._pending or lk._pending[ARM]["tries"] != 1:
    fail.append(f"confirmed send should track with tries=1 (pending={lk._pending})")

now = lk._pending[ARM]["deadline"]
lk._check_acks(now - 0.01)                 # before the deadline -> no resend
if len(lk.sent) != 1:
    fail.append(f"no resend before deadline (sent={len(lk.sent)})")

# each expired deadline triggers one resend; ACK_MAX_TRIES total transmissions
for expected_sent in range(2, lk.ACK_MAX_TRIES + 1):
    lk._check_acks(now + 0.01)
    if len(lk.sent) != expected_sent:
        fail.append(f"expected {expected_sent} sends after resend, got {len(lk.sent)}")
    if ARM in lk._pending:
        now = lk._pending[ARM]["deadline"]

# tries now == ACK_MAX_TRIES; the next expiry gives up
lk._check_acks(now + 0.01)
if ARM in lk._pending:
    fail.append("command should be dropped after ACK_MAX_TRIES")
if len(lk.sent) != lk.ACK_MAX_TRIES:
    fail.append(f"total transmissions should equal ACK_MAX_TRIES={lk.ACK_MAX_TRIES}, got {len(lk.sent)}")
if unacked != [ARM]:
    fail.append(f"command_unacked should fire once with the command, got {unacked}")
if lk._ack_timer.isActive():
    fail.append("ack timer should stop once nothing is pending")

# 3) a COMMAND_ACK resolves a pending command and stops resends (via _match_acks) --------------------
lk.sent.clear(); acked.clear(); unacked.clear()
lk.send_command_long(1, MODE, [1, 4, 3, 0, 0, 0, 0], confirm=True)
lk._match_acks([Message(mavlink.COMMAND_ACK, 1, 1, 0, {"command": MODE, "result": 0})])
if MODE in lk._pending or acked != [(MODE, 0)]:
    fail.append(f"ACK should resolve pending + emit command_result (pending={lk._pending}, acked={acked})")
before = len(lk.sent)
lk._check_acks(lk._pending.get(MODE, {}).get("deadline", 0) + 100)   # far future
if len(lk.sent) != before:
    fail.append("no resend after an ACK cleared the command")

# 4) an unrelated ACK does not resolve a different pending command -----------------------------------
lk.sent.clear()
lk.send_command_long(1, MODE, [1, 4, 3, 0, 0, 0, 0], confirm=True)
lk._match_acks([Message(mavlink.COMMAND_ACK, 1, 1, 0, {"command": ARM, "result": 0})])
if MODE not in lk._pending:
    fail.append("an unrelated ACK must not clear a different command")

# 5) a real COMMAND_ACK frame through the RX funnel (_ingest) resolves it ----------------------------
lk2 = CaptureLink()
got = []
lk2.command_result.connect(lambda c, r: got.append((c, r)))
lk2.parser = core.Parser()
lk2.send_command_long(1, ARM, [0, 0, 0, 0, 0, 0, 0], confirm=True)      # disarm, confirmed
ack_frame = mavlink.frame(mavlink.COMMAND_ACK, mavlink.enc_command_ack(ARM, 0),
                          7, 1, 1, crc_fn=core.crc_extra)
lk2._ingest(ack_frame)
if ARM in lk2._pending or got != [(ARM, 0)]:
    fail.append(f"real ACK frame via _ingest should resolve pending (pending={lk2._pending}, got={got})")

# 6) the real safety-critical command methods opt into confirmation ----------------------------------
lk3 = CaptureLink()
lk3.arm(1)
lk3.set_mode(1, 4, 3)
lk3.land(1)
lk3.rtl(1)
for cid, label in [(ARM, "arm"), (MODE, "set_mode"),
                   (mavlink.MAV_CMD_NAV_LAND, "land"), (mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, "rtl")]:
    if cid not in lk3._pending:
        fail.append(f"{label} should send a confirmed (tracked) command")

# 7) COMMAND_INT confirmation: a dropped takeoff is just as critical, so it opts in + retries -------
lk4 = CaptureLink()
lk4.takeoff(1, 20.0, 47.0, 8.0)
TKO = mavlink.MAV_CMD_NAV_TAKEOFF
if TKO not in lk4._pending or lk4._pending[TKO]["tries"] != 1:
    fail.append("takeoff (COMMAND_INT) should be confirmed/tracked")
else:
    dl = lk4._pending[TKO]["deadline"]
    before = len(lk4.sent)
    lk4._check_acks(dl + 0.01)
    if len(lk4.sent) != before + 1 or lk4._pending[TKO]["tries"] != 2:
        fail.append(f"confirmed takeoff should resend (sent {before}->{len(lk4.sent)}, "
                    f"tries={lk4._pending.get(TKO, {}).get('tries')})")

print("COMMAND_RETRY FAILED: " + "; ".join(fail) if fail else
      "COMMAND_RETRY PASSED (no-track for fire-and-forget; retry schedule + give-up after "
      "ACK_MAX_TRIES + command_unacked; ACK via _match_acks and real frame via _ingest resolves + "
      "stops resends; unrelated ACK ignored; arm/set_mode/land/rtl opt in; COMMAND_INT takeoff "
      "confirmed + retries)")
sys.exit(1 if fail else 0)
