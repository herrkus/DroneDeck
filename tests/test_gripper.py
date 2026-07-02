#!/usr/bin/env python3
"""test_gripper.py -- payload gripper release/grab guided action (iter121), QGC-parity.

Delivery drones drop/hold a payload via DO_GRIPPER (211): param1 = instance, param2 = action
(0 release, 1 grab). Verifies link.gripper builds the right COMMAND_LONG, that the GUI menu button
exists with Release/Grab entries, and that _gripper guards a missing vehicle. No real gripper."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
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

# 2) GUI: Payload menu button has Release + Grab; _gripper guards no-vehicle, else sends -----------
import main as m
win = m.DroneDeck(14720)
win._persist = False
labels = [a.text() for a in win._payload_menu.actions()]
if labels != ["Release payload", "Grab payload"]:
    fail.append(f"payload menu entries: {labels}")
if getattr(win.btn_payload, "_needs", None) != "conn":
    fail.append("payload button should need a connection")
calls = []
win.link.gripper = lambda tgt, action: calls.append((tgt, action))
win._has_vehicle = lambda: False
win._gripper(mavlink.GRIPPER_ACTION_RELEASE)
if calls:
    fail.append("sent gripper with no vehicle connected")
win._has_vehicle = lambda: True
win._sysid = lambda: 7
win._gripper(mavlink.GRIPPER_ACTION_GRAB)
if calls != [(7, mavlink.GRIPPER_ACTION_GRAB)]:
    fail.append(f"wrong gripper call: {calls}")

print("GRIPPER FAILED: " + "; ".join(fail) if fail else
      "GRIPPER PASSED (DO_GRIPPER instance/action params; Payload menu Release/Grab; GUI guards no-vehicle)",
      flush=True)
os._exit(1 if fail else 0)
