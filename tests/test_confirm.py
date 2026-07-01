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

app = QApplication([])
win = m.DroneDeck(14599)


class RecLink:
    def __init__(self):
        self.calls = []

    def arm(self, sysid, arm):
        self.calls.append(("arm", arm))

    def land(self, sysid):
        self.calls.append(("land",))

    def rtl(self, sysid):
        self.calls.append(("rtl",))


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

print("CONFIRM PASSED")
