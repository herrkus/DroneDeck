#!/usr/bin/env python3
"""test_components.py -- MAVLink component enumeration (iter154, QGC parity).

QGC detects and lists the components on a vehicle (autopilot, gimbal, camera, companion computer, ...)
from their HEARTBEATs. DroneDeck deliberately ignores non-autopilot heartbeats in the FC-state path (so
a gimbal/camera can't flap armed/mode), but never surfaced them. Now the vehicle enumerates every
component (compid -> type/autopilot/last_seen) and exposes active_components(); Vehicle Info lists them.
Verifies: all components are recorded from heartbeats; the FC state is NOT disturbed by peripheral
heartbeats; stale components age out; compid->name mapping; and the Vehicle Info dialog shows them."""
import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import mavlink
from vehicle import Vehicle

fail = []
ARMED = mavlink.MAV_MODE_FLAG_SAFETY_ARMED
INVALID = mavlink.MAV_AUTOPILOT_INVALID


def hb(compid, autopilot, base_mode=0, mav_type=2, custom_mode=0):
    return mavlink.Message(mavlink.HEARTBEAT, 1, compid, 0,
                           {"type": mav_type, "autopilot": autopilot, "base_mode": base_mode,
                            "custom_mode": custom_mode, "system_status": 4, "mavlink_version": 3})


# 1) all components enumerated; FC state comes only from the autopilot heartbeat --------------------
ve = Vehicle()
ve.consume([hb(1, 12, base_mode=ARMED, custom_mode=0)])          # PX4 autopilot, ARMED
if not ve.armed:
    fail.append("autopilot heartbeat should arm the vehicle")
ve.consume([hb(154, INVALID, base_mode=0)])                      # gimbal (INVALID autopilot, base_mode 0)
ve.consume([hb(100, INVALID, base_mode=0)])                      # camera
if not ve.armed:
    fail.append("peripheral heartbeats must NOT clear the armed flag (state flap)")
names = dict(ve.active_components())
if set(names) != {1, 100, 154}:
    fail.append(f"components should be {{1,100,154}}, got {set(names)}")
elif [names[c] for c in (1, 100, 154)] != ["Autopilot", "Camera", "Gimbal"]:
    fail.append(f"component names wrong: {[names.get(c) for c in (1,100,154)]}")

# 2) stale components age out of active_components --------------------------------------------------
ve.components[154] = (ve.components[154][0], ve.components[154][1], time.monotonic() - 30)  # 30 s ago
active = dict(ve.active_components(ttl=5.0))
if 154 in active or 1 not in active or 100 not in active:
    fail.append(f"stale gimbal should age out (5 s ttl): active={sorted(active)}")

# 3) compid -> name mapping ------------------------------------------------------------------------
cases = {1: "Autopilot", 100: "Camera", 101: "Camera 2", 154: "Gimbal", 172: "Gimbal",
         156: "ADS-B", 158: "Peripheral", 191: "Companion", 190: "GCS", 220: "GPS", 240: "Component 240"}
for cid, want in cases.items():
    got = mavlink.component_name(cid)
    if got != want:
        fail.append(f"component_name({cid})={got!r}, want {want!r}")

# 4) Vehicle Info dialog lists the components -------------------------------------------------------
from PySide6.QtWidgets import QApplication, QMessageBox
app = QApplication.instance() or QApplication([])
import main as m
win = m.DroneDeck(14762)
win._persist = False
win.vehicle.consume([hb(1, 12, base_mode=ARMED), hb(154, INVALID), hb(100, INVALID)])
win.vehicle.have_autopilot_version = True
win._has_vehicle = lambda: True
win._sysid = lambda: 1
captured = {}
QMessageBox.information = staticmethod(lambda *a, **k: captured.update(text=a[2] if len(a) > 2 else ""))
win._show_vehicle_info()
if "Gimbal" not in captured.get("text", "") or "Camera" not in captured.get("text", ""):
    fail.append(f"Vehicle Info should list Gimbal + Camera, got: {captured.get('text','')[:200]}")

print("COMPONENTS FAILED: " + "; ".join(fail) if fail else
      "COMPONENTS PASSED (every component enumerated from heartbeats without flapping FC state; stale "
      "ones age out; compid->name mapping; Vehicle Info lists detected components)")
sys.stdout.flush()
os._exit(1 if fail else 0)
