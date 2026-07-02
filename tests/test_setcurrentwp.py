#!/usr/bin/env python3
"""test_setcurrentwp.py -- MISSION_SET_CURRENT (41): jump the vehicle's active mission item (iter117).

DroneDeck only SENDS this message (the vehicle echoes MISSION_CURRENT back), so this verifies the
encoder payload, that link.set_current_wp() frames a WIRE-VALID MISSION_SET_CURRENT (correct msgid +
the CRC_EXTRA seed 28 -- a wrong seed would make a real vehicle silently reject it), and that the GUI
action guards a missing vehicle and targets the right sequence. No parser decode (never received)."""
import os
import sys
import struct

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication, QMessageBox
import mavlink
from link import UdpLink

app = QApplication.instance() or QApplication([])
fail = []

# 1) encoder + seed: payload is size-sorted seq(uint16), target_system, target_component -----------
if mavlink.enc_mission_set_current(5, 3, 1) != struct.pack("<HBB", 5, 3, 1):
    fail.append("encoder payload layout wrong")
if mavlink.CRC_EXTRA.get(mavlink.MISSION_SET_CURRENT) != 28:      # verified vs the raw MAVLink algo
    fail.append(f"CRC_EXTRA seed = {mavlink.CRC_EXTRA.get(mavlink.MISSION_SET_CURRENT)} (expect 28)")

# 2) link.set_current_wp frames a wire-VALID MISSION_SET_CURRENT -----------------------------------
sent = []
lk = UdpLink()
lk._send = lambda b: sent.append(bytes(b))
lk.set_current_wp(7, 4)                              # target system 7, jump to seq 4
fr = sent[-1]
if fr[0] != 0xFE:
    fail.append("expected a v1 frame (0xFE)")
if fr[5] != mavlink.MISSION_SET_CURRENT or mavlink.MISSION_SET_CURRENT != 41:
    fail.append(f"wrong msgid byte: {fr[5]}")
payload = fr[6:6 + fr[1]]
if payload != struct.pack("<HBB", 4, 7, 1):          # seq=4, target_sys=7, target_comp defaults to 1
    fail.append(f"framed payload wrong: {payload!r}")
crc = mavlink.crc16_mcrf4xx(fr[1:6] + payload, 28)   # recompute independently with seed 28
if struct.pack("<H", crc) != fr[-2:]:
    fail.append("MISSION_SET_CURRENT CRC invalid -- a real vehicle would reject the command")

# 3) GUI action guards no-vehicle, else sends set_current_wp(sysid, seq) for the chosen row ---------
import main as m
QMessageBox.information = staticmethod(lambda *a, **k: None)
win = m.DroneDeck(14690)
win._persist = False
calls = []
win.link.set_current_wp = lambda tgt, seq: calls.append((tgt, seq))
win.mission_items = [type("I", (), {})() for _ in range(3)]       # 3 dummy waypoints
win._has_vehicle = lambda: False
win._set_current_wp(2)
if calls:
    fail.append("sent MISSION_SET_CURRENT with no vehicle connected")
win._has_vehicle = lambda: True
win._sysid = lambda: 9
win._set_current_wp(2)
if calls != [(9, 2)]:
    fail.append(f"wrong target/seq sent: {calls}")
win._set_current_wp(99)                              # out of range -> ignored, not sent
if calls != [(9, 2)]:
    fail.append("an out-of-range waypoint index was sent")

print("SETCURRENTWP FAILED: " + "; ".join(fail) if fail else
      "SETCURRENTWP PASSED (encoder + seed 28 correct; wire-valid frame; GUI guards + targets right seq)",
      flush=True)
os._exit(1 if fail else 0)     # skip Qt offscreen teardown (segfault-prone); result already printed
