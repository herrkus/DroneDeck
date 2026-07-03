#!/usr/bin/env python3
"""test_fence_enable.py -- runtime geofence enable/disable (DO_FENCE_ENABLE) -- 1:1 QGC parity.

DroneDeck could build + upload fence geometry but not turn enforcement on/off at runtime like QGC's
fence toggle. Added link.fence_enable + Tools-menu Enable/Disable Geofence. Verifies: the command
encodes DO_FENCE_ENABLE (207) with param1 = 1 (enable) / 0 (disable); the menu handler routes to it
and is guarded when there is no vehicle."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import mavlink
import core
from link import Link

fail = []


class CaptureLink(Link):
    def __init__(self):
        super().__init__()
        self.sent = []
        self._open = True
        self.remote = True

    def _write(self, data):
        self.sent.append(data)


def cmd_of(lk):
    msgs = core.Parser().feed(lk.sent[-1])
    return msgs[0].fields if msgs else None


# 1) enable / disable encode DO_FENCE_ENABLE with the right param1 -----------------------------------
lk = CaptureLink()
lk.fence_enable(1, True)
f = cmd_of(lk)
if not f or int(f.get("command", -1)) != mavlink.MAV_CMD_DO_FENCE_ENABLE or abs(f["param1"] - 1.0) > 1e-6:
    fail.append(f"enable should send DO_FENCE_ENABLE(207) param1=1, got cmd={f and f.get('command')} p1={f and f.get('param1')}")
lk.fence_enable(1, False)
f = cmd_of(lk)
if not f or abs(f["param1"]) > 1e-6:
    fail.append(f"disable should send param1=0, got {f and f.get('param1')}")

# 2) the menu handler routes to the link + is guarded with no vehicle --------------------------------
from PySide6.QtWidgets import QApplication
app = QApplication.instance() or QApplication([])
import main as m
win = m.DroneDeck(16110)
win._persist = False
calls = []
win.link.fence_enable = lambda sysid, on: calls.append(on)
win._sysid = lambda: 1

win._has_vehicle = lambda: False
win._fence_enable(True)
if calls:
    fail.append("no-vehicle: _fence_enable must not send")

win._has_vehicle = lambda: True
win._fence_enable(True)
win._fence_enable(False)
if calls != [True, False]:
    fail.append(f"handler should route enable/disable, got {calls}")

print("FENCE_ENABLE FAILED: " + "; ".join(fail) if fail else
      "FENCE_ENABLE PASSED (DO_FENCE_ENABLE 207 param1 enable=1/disable=0; menu handler routes + "
      "guards no-vehicle)")
sys.stdout.flush()
os._exit(1 if fail else 0)
