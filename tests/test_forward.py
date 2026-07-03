#!/usr/bin/env python3
"""test_forward.py -- MAVLink forwarding to a 2nd UDP endpoint (iter155, QGC parity).

QGC can re-broadcast received telemetry to a second UDP endpoint (a companion computer, a 2nd GCS,
MAVProxy, an OBS overlay). DroneDeck's link now does the same (one-way, like QGC): every received byte
is mirrored to the forward endpoint while normal parsing continues. Verifies over real UDP loopback: a
frame sent to the link is forwarded VERBATIM to the 2nd endpoint AND still parsed+emitted; clear_forward
stops it; and the target persists onto a freshly built link (survives reconnect)."""
import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
from PySide6.QtNetwork import QUdpSocket, QHostAddress
from PySide6.QtCore import QCoreApplication
import mavlink
import core
from link import UdpLink

app = QApplication.instance() or QApplication([])
fail = []
LINK_PORT, FWD_PORT = 14771, 14772


def pump(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        QCoreApplication.processEvents()
        time.sleep(0.005)


link = UdpLink()
if not link.open(port=LINK_PORT):
    print("FORWARD SKIPPED (could not bind test UDP port -- another process may hold it)")
    os._exit(0)
recv = QUdpSocket()
if not recv.bind(QHostAddress("127.0.0.1"), FWD_PORT):
    print("FORWARD SKIPPED (could not bind forward receiver port)")
    os._exit(0)
link.set_forward("127.0.0.1", FWD_PORT)

got = []
link.messages.connect(lambda b: got.extend(b))

payload = mavlink.enc_heartbeat(2, 12, 0, 0, 4)
frame = mavlink.frame_v2(mavlink.HEARTBEAT, payload, 5, 1, 1, crc_fn=core.crc_extra)
sender = QUdpSocket()
sender.writeDatagram(frame, QHostAddress("127.0.0.1"), LINK_PORT)
pump(1.0)

# 1) the frame was forwarded VERBATIM to the 2nd endpoint -------------------------------------------
fwd = None
while recv.hasPendingDatagrams():
    fwd = bytes(recv.receiveDatagram().data())
if fwd != frame:
    fail.append(f"forwarded bytes mismatch (got {len(fwd) if fwd else 0} B, want {len(frame)})")

# 2) the link still parsed + emitted the message (forwarding is non-destructive) --------------------
if not any(m.msgid == mavlink.HEARTBEAT for m in got):
    fail.append("link should still emit the parsed HEARTBEAT while forwarding")

# 3) clear_forward stops forwarding -----------------------------------------------------------------
link.clear_forward()
if link.forwarding:
    fail.append("clear_forward should disable forwarding")
sender.writeDatagram(frame, QHostAddress("127.0.0.1"), LINK_PORT)
pump(0.5)
leftover = False
while recv.hasPendingDatagrams():
    recv.receiveDatagram()
    leftover = True
if leftover:
    fail.append("no datagram should be forwarded after clear_forward")

link.close()

# 4) the forward target persists onto a freshly built link (survives reconnect) ---------------------
import main as mn
win = mn.DroneDeck(14773)
win._persist = False
win._fwd_target = ("127.0.0.1", 14774)
lk2 = win._make_link()
if not lk2.forwarding:
    fail.append("_make_link should re-apply the persisted forward target")
lk2.clear_forward()

print("FORWARD FAILED: " + "; ".join(fail) if fail else
      "FORWARD PASSED (received frame mirrored verbatim to the 2nd UDP endpoint + still parsed/emitted; "
      "clear_forward stops it; forward target persists onto a rebuilt link)")
sys.stdout.flush()
os._exit(1 if fail else 0)
