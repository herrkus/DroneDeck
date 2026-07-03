#!/usr/bin/env python3
"""test_parachute.py -- emergency Deploy Parachute (DO_PARACHUTE 208) from the Tools menu (iter132).

An irreversible emergency action, so it must build the right COMMAND_LONG (param1 = RELEASE 2) and
the GUI handler must guard a missing vehicle and only fire on an explicit confirm (default = Cancel)."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication, QMessageBox
import mavlink
from link import UdpLink

app = QApplication.instance() or QApplication([])
fail = []

# 1) link.deploy_parachute -> COMMAND_LONG 208, param1 = RELEASE (2) -------------------------------
sent = []
lk = UdpLink()
lk.send_command_long = lambda tgt, cmd, params: sent.append((tgt, cmd, list(params)))
lk.deploy_parachute(6)
tgt, cmd, p = sent[-1]
if cmd != mavlink.MAV_CMD_DO_PARACHUTE or mavlink.MAV_CMD_DO_PARACHUTE != 208:
    fail.append(f"cmd={cmd} (expect DO_PARACHUTE 208)")
if p[0] != mavlink.PARACHUTE_RELEASE or mavlink.PARACHUTE_RELEASE != 2:
    fail.append(f"param1={p[0]} (expect RELEASE 2)")
if tgt != 6:
    fail.append(f"target={tgt}")

# 2) GUI handler: guards no-vehicle; only deploys on an explicit Yes ------------------------------
import main as m
win = m.DroneDeck(14730)
win._persist = False
calls = []
win.link.deploy_parachute = lambda tgt: calls.append(tgt)
QMessageBox.information = staticmethod(lambda *a, **k: None)

win._has_vehicle = lambda: False
win._deploy_parachute()
if calls:
    fail.append("deployed parachute with no vehicle connected")

win._has_vehicle = lambda: True
win._sysid = lambda: 8

# Cancel/decline -> must NOT deploy. Patch QMessageBox.exec to return Cancel.
QMessageBox.exec = lambda self: QMessageBox.Cancel
win._deploy_parachute()
if calls:
    fail.append("a cancelled confirm still deployed the parachute")

# Confirm -> deploys exactly once, for the active sysid
QMessageBox.exec = lambda self: QMessageBox.Yes
win._deploy_parachute()
if calls != [8]:
    fail.append(f"confirmed deploy should fire once for sysid 8: {calls}")

print("PARACHUTE FAILED: " + "; ".join(fail) if fail else
      "PARACHUTE PASSED (DO_PARACHUTE 208 param1=RELEASE 2; GUI guards no-vehicle + requires explicit confirm)")
sys.stdout.flush()
os._exit(1 if fail else 0)
