#!/usr/bin/env python3
"""test_confirm.py -- confirmation guards on destructive flight actions. RTL, Land, Arm, and
Disarm-while-armed must ask before sending; a 'No' answer must NOT reach the link. Disarming
an already-disarmed vehicle is harmless and sends without a prompt. Uses a recording link and
monkeypatches QMessageBox.question. Port-independent (no real link)."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication, QMessageBox
import main as m
import mavlink

app = QApplication([])
win = m.DroneDeck(14599)

# Wire-level: link.force_disarm must emit MAV_CMD_COMPONENT_ARM_DISARM with the force magic
# param2=21196 (a plain disarm is refused mid-flight -- PX4-verified). Capture + decode the frame.
_frames = []
win.link._send = lambda data: _frames.append(bytes(data))
win.link.force_disarm(7)
assert len(_frames) == 1
_dm = mavlink.PyParser().feed(_frames[0])
assert len(_dm) == 1 and _dm[0].msgid == mavlink.COMMAND_LONG
_ff = _dm[0].fields
assert _ff["command"] == mavlink.MAV_CMD_COMPONENT_ARM_DISARM
assert _ff["param1"] == 0.0 and abs(_ff["param2"] - 21196.0) < 1e-3 and _ff["target_system"] == 7


class RecLink:
    def __init__(self):
        self.calls = []

    def arm(self, sysid, arm):
        self.calls.append(("arm", arm))

    def land(self, sysid):
        self.calls.append(("land",))

    def rtl(self, sysid):
        self.calls.append(("rtl",))

    def force_disarm(self, sysid):
        self.calls.append(("force_disarm",))


win.link = RecLink()
win._has_vehicle = lambda: True
win._sysid = lambda: 1

answer = {"v": QMessageBox.StandardButton.No}
QMessageBox.question = staticmethod(lambda *a, **k: answer["v"])   # every guard consults this
YES, NO = QMessageBox.StandardButton.Yes, QMessageBox.StandardButton.No


def run(action, ans):
    win.link.calls.clear()
    answer["v"] = ans
    action()
    return list(win.link.calls)


# RTL -- declined sends nothing, confirmed sends
assert run(win._rtl, NO) == []
assert run(win._rtl, YES) == [("rtl",)]

# Land
assert run(win._land, NO) == []
assert run(win._land, YES) == [("land",)]

# Disarm while ARMED -- guarded
win.vehicle.armed = True
assert run(lambda: win._arm(False), NO) == []
assert run(lambda: win._arm(False), YES) == [("arm", False)]

# Arm -- guarded
assert run(lambda: win._arm(True), NO) == []
assert run(lambda: win._arm(True), YES) == [("arm", True)]

# Disarm while NOT armed -- harmless, no prompt shown, sends directly even though answer is No
win.vehicle.armed = False
assert run(lambda: win._arm(False), NO) == [("arm", False)]

# Emergency Stop -- guarded force-disarm; declined sends nothing, confirmed sends force_disarm
assert run(win._emergency_stop, NO) == []
assert run(win._emergency_stop, YES) == [("force_disarm",)]

print("CONFIRM PASSED")
