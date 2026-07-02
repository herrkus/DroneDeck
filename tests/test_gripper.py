#!/usr/bin/env python3
"""test_gripper.py -- payload gripper + winch guided actions (iter121/iter122), QGC-parity.

Delivery drones drop/hold a payload via DO_GRIPPER (211): param1 = instance, param2 = action
(0 release, 1 grab); and pay out/reel in cable via DO_WINCH (42600). Verifies link.gripper/link.winch
build the right COMMAND_LONGs, that the GUI Payload menu has the expected entries, and that the
handlers guard a missing vehicle. No real gripper/winch."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication, QInputDialog
import mavlink
from link import UdpLink

app = QApplication.instance() or QApplication([])
fail = []

# 1) link.gripper -> DO_GRIPPER (211), instance in param1, action in param2 ------------------------
sent = []
lk = UdpLink()
lk.send_command_long = lambda tgt, cmd, params: sent.append((tgt, cmd, list(params)))
lk.gripper(3, mavlink.GRIPPER_ACTION_RELEASE)
tgt, cmd, p = sent[-1]
if cmd != mavlink.MAV_CMD_DO_GRIPPER or mavlink.MAV_CMD_DO_GRIPPER != 211:
    fail.append(f"cmd={cmd} (expect DO_GRIPPER 211)")
if p[0] != 1 or p[1] != mavlink.GRIPPER_ACTION_RELEASE:
    fail.append(f"release params: instance={p[0]} action={p[1]} (expect 1, 0)")
lk.gripper(3, mavlink.GRIPPER_ACTION_GRAB, instance=2)
p = sent[-1][2]
if p[0] != 2 or p[1] != mavlink.GRIPPER_ACTION_GRAB:
    fail.append(f"grab params: instance={p[0]} action={p[1]} (expect 2, 1)")

# 2) link.winch -> DO_WINCH (42600), length control params ----------------------------------------
lk.winch(3, mavlink.WINCH_LENGTH_CONTROL, length=2.5, rate=1.0)
tgt, cmd, p = sent[-1]
if cmd != mavlink.MAV_CMD_DO_WINCH or mavlink.MAV_CMD_DO_WINCH != 42600:
    fail.append(f"winch cmd={cmd} (expect DO_WINCH 42600)")
if p[1] != mavlink.WINCH_LENGTH_CONTROL or p[2] != 2.5 or p[3] != 1.0:
    fail.append(f"winch params: action={p[1]} length={p[2]} rate={p[3]} (expect 1, 2.5, 1.0)")
lk.winch(3, mavlink.WINCH_RELAXED)
if sent[-1][2][1] != mavlink.WINCH_RELAXED:
    fail.append("winch relax action should be 0")

# 3) GUI: Payload menu has gripper + winch entries; handlers guard no-vehicle, else send -----------
import main as m
win = m.DroneDeck(14720)
win._persist = False
labels = [a.text() for a in win._payload_menu.actions() if a.text()]   # skip the separator
if labels != ["Release payload", "Grab payload", "Winch (lower / raise)...", "Winch relax"]:
    fail.append(f"payload menu entries: {labels}")
if getattr(win.btn_payload, "_needs", None) != "conn":
    fail.append("payload button should need a connection")
gcalls, wcalls = [], []
win.link.gripper = lambda tgt, action: gcalls.append((tgt, action))
win.link.winch = lambda tgt, action, length=0.0, rate=1.0, instance=1: wcalls.append(
    (tgt, action, length))
QInputDialog.getDouble = staticmethod(lambda *a, **k: (3.0, True))
win._has_vehicle = lambda: False
win._gripper(mavlink.GRIPPER_ACTION_RELEASE)
win._winch()
if gcalls or wcalls:
    fail.append("sent payload command with no vehicle connected")
win._has_vehicle = lambda: True
win._sysid = lambda: 7
win._gripper(mavlink.GRIPPER_ACTION_GRAB)
if gcalls != [(7, mavlink.GRIPPER_ACTION_GRAB)]:
    fail.append(f"wrong gripper call: {gcalls}")
win._winch()
if wcalls != [(7, mavlink.WINCH_LENGTH_CONTROL, 3.0)]:
    fail.append(f"wrong winch call: {wcalls}")

print("GRIPPER FAILED: " + "; ".join(fail) if fail else
      "GRIPPER PASSED (DO_GRIPPER + DO_WINCH params; Payload menu Release/Grab/Winch; GUI guards no-vehicle)",
      flush=True)
os._exit(1 if fail else 0)
