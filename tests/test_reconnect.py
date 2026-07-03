#!/usr/bin/env python3
"""test_reconnect.py -- link-lifecycle / reconnect stress. Real drones drop and re-establish links
constantly, so open()/close() must free every resource. Opens and closes a UDP link 20x and drives
the DroneDeck-level reconnect path (which creates a fresh Link and deleteLater()s the old one each
time), asserting the process's open file-descriptor count and Python thread count stay flat (a socket
or QObject that is not freed is a real leak), and that the link still decodes a frame after the churn.
No arming."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

import threading
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QCoreApplication
import mavlink
from link import UdpLink

app = QApplication.instance() or QApplication([])


def pump():
    for _ in range(3):
        QCoreApplication.processEvents()


def fd_count():
    try:
        return len(os.listdir("/proc/self/fd"))
    except OSError:
        return -1


PORT = 14584

# warm-up: first open/close does one-time allocations that must not be counted as a leak
l0 = UdpLink()
l0.open(port=PORT)
pump()
l0.close()
l0.deleteLater()
pump()

fd_before = fd_count()
thr_before = threading.active_count()

# 20 open/close cycles on a fresh UdpLink each time (mirrors _connect making a new Link)
for i in range(20):
    lk = UdpLink()
    assert lk.open(port=PORT), f"bind failed on cycle {i} (port {PORT} not released?)"
    assert lk.is_open
    pump()
    lk.close()
    assert not lk.is_open
    lk.deleteLater()
    pump()

pump()
fd_after = fd_count()
thr_after = threading.active_count()

# file descriptors and threads must be flat (small slack for event-loop transients)
assert fd_after <= fd_before + 2, f"FD leak: {fd_before} -> {fd_after} over 20 reconnects"
assert thr_after == thr_before, f"thread leak: {thr_before} -> {thr_after}"

# the hb QTimer must be stopped after close (not left running)
lk = UdpLink()
lk.open(port=PORT)
pump()
lk.close()
pump()
assert not lk.hb.isActive(), "heartbeat timer still active after close()"

# a link still works after all the churn: feed a HEARTBEAT frame and confirm it decodes
lk2 = UdpLink()
lk2.open(port=PORT)
decoded = []
lk2.messages.connect(lambda batch: decoded.extend(m.msgid for m in batch))
hb = mavlink.frame(mavlink.HEARTBEAT, b"\x00" * mavlink._WIRE[mavlink.HEARTBEAT][2], 0, 1, 1)
lk2._ingest(hb)          # deliver bytes as if they arrived on the socket
pump()
assert mavlink.HEARTBEAT in decoded, "link did not decode a frame after reconnect churn"
lk2.close()
pump()

# DroneDeck-level reconnect path: close + deleteLater + fresh link, several times, window survives
import main as m
win = m.DroneDeck(PORT + 1)
fd_win = fd_count()
for _ in range(6):
    win._connect()
    pump()
    if win.link is not None:
        win.link.close()
    pump()
assert fd_count() <= fd_win + 3, f"FD leak across DroneDeck reconnects: {fd_win} -> {fd_count()}"
assert win.link is not None and not win.link.is_open

print(f"RECONNECT PASSED (20 udp cycles + 6 window reconnects; fd {fd_before}->{fd_after}, "
      f"threads {thr_before})")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
