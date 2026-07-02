#!/usr/bin/env python3
"""test_reboot.py -- Reboot Vehicle (PREFLIGHT_REBOOT_SHUTDOWN 246) from the params screen (iter131).

Many parameters need an autopilot reboot to take effect; QGC exposes a Reboot button in the params
view. Verifies link.reboot_vehicle builds the right COMMAND_LONG, ParamManager.reboot routes to it
(guarding a missing vehicle), and the ParamDialog button exists and CONFIRMS before sending."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication, QMessageBox
import mavlink
from link import UdpLink
from params import ParamManager, ParamDialog

app = QApplication.instance() or QApplication([])
fail = []

# 1) link.reboot_vehicle -> COMMAND_LONG 246, param1 = 1 (reboot autopilot) ------------------------
sent = []
lk = UdpLink()
lk.send_command_long = lambda tgt, cmd, params: sent.append((tgt, cmd, list(params)))
lk.reboot_vehicle(5)
tgt, cmd, p = sent[-1]
if cmd != mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN or mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN != 246:
    fail.append(f"cmd={cmd} (expect PREFLIGHT_REBOOT_SHUTDOWN 246)")
if p[0] != 1:
    fail.append(f"param1={p[0]} (expect 1 = reboot autopilot)")
if tgt != 5:
    fail.append(f"target={tgt}")


# 2) ParamManager.reboot routes to link.reboot_vehicle, guarding a missing vehicle -----------------
class FakeLink:
    is_open = True
    remote = ("127.0.0.1", 14550)

    def __init__(self):
        self.calls = []

    def reboot_vehicle(self, tgt):
        self.calls.append(tgt)


fake = FakeLink()
mgr = ParamManager(lambda: fake, lambda: 3)
if mgr.reboot() is not True:
    fail.append("reboot() should return True with a ready link")
if fake.calls != [3]:
    fail.append(f"reboot should target sysid 3: {fake.calls}")
if ParamManager(lambda: None, lambda: 3).reboot() is not False:
    fail.append("reboot() with no link should return False and send nothing")

# 3) ParamDialog: Reboot button present; _reboot confirms first, only sends on Yes -----------------
dlg = ParamDialog(mgr)
if getattr(dlg, "btn_reboot", None) is None or dlg.btn_reboot.text() != "Reboot vehicle":
    fail.append("Reboot vehicle button missing")
fake.calls.clear()
QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.StandardButton.No)
dlg._reboot()
if fake.calls:
    fail.append("a declined reboot must NOT send the command")
QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.StandardButton.Yes)
dlg._reboot()
if fake.calls != [3]:
    fail.append(f"a confirmed reboot should send exactly once: {fake.calls}")

print("REBOOT FAILED: " + "; ".join(fail) if fail else
      "REBOOT PASSED (COMMAND 246 param1=1; ParamManager.reboot routes+guards; dialog button confirms)")
sys.exit(1 if fail else 0)
