#!/usr/bin/env python3
"""test_upload_progress.py -- mission upload/download progress bar. MissionProtocol emits a
numeric progress_n(done, total) alongside its text progress, and the GUI drives a QProgressBar
from it: shown during a transfer, stepped per item, hidden on finish. Drives the protocol with
a mock link + synthetic MISSION_REQUEST_INT/ACK messages. Port-independent (no real link)."""
import os
import sys
import types

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mavlink
import main as m
from mission import MissionProtocol, MissionItem

app = QApplication([])


class MockLink:
    is_open = True
    remote = ("127.0.0.1", 14550)

    def send_mission_count(self, *a):
        pass

    def send_mission_item(self, *a):
        pass

    def send_mission_ack(self, *a):
        pass


def msg(mid, fields):
    return types.SimpleNamespace(msgid=mid, fields=fields)


# -- level 1: MissionProtocol.progress_n sequence over a 4-item upload -----------------
mp = MissionProtocol(lambda: MockLink(), lambda: 1)
emits, fin = [], []
mp.progress_n.connect(lambda d, t: emits.append((d, t)))
mp.finished.connect(lambda ok, s: fin.append((ok, s)))

items = [MissionItem(i, 47.0 + i * 0.001, 8.0, 50.0) for i in range(4)]
mp.upload(items)
for seq in range(4):                                   # vehicle requests each item in turn
    mp.handle(msg(mavlink.MISSION_REQUEST_INT, {"seq": seq}))
mp.handle(msg(mavlink.MISSION_ACK, {"type": 0}))       # vehicle accepts

assert emits == [(0, 4), (1, 4), (2, 4), (3, 4), (4, 4)], emits
assert fin == [(True, "upload complete")], fin

# -- level 2: GUI QProgressBar reacts to the numeric progress --------------------------
win = m.DroneDeck(14599)
assert win.mission_progress.isHidden()                 # hidden at rest
win._on_mission_progress_n(2, 5)
assert not win.mission_progress.isHidden()             # appears during transfer
assert win.mission_progress.value() == 2 and win.mission_progress.maximum() == 5
win._on_mission_progress_n(5, 5)
assert win.mission_progress.value() == 5
win._on_mission_finished(True, "upload complete")
assert win.mission_progress.isHidden()                 # gone once finished

print("UPLOAD PROGRESS PASSED")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
