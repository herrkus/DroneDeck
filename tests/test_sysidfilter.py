#!/usr/bin/env python3
"""test_sysidfilter.py -- heartbeat source-identity filtering (iter112 fix).

Real drones have several components beating on the SAME sysid (gimbal compid 154, companion
computer, camera -- all with autopilot=MAV_AUTOPILOT_INVALID and base_mode=0), and telemetry radios
are broadcast media so a NEIGHBOURING GCS's heartbeat (sysid 255, type=GCS) also arrives. Before the
fix: a gimbal heartbeat flipped armed True->False and mode 'POSCTL'->'MODE 0' every second (display
flapping on any real drone with a gimbal), a dead flight controller still looked alive because the
gimbal kept refreshing last_heartbeat, and a second GCS appeared as selectable phantom "vehicle #255"
(and was even sent telemetry-stream requests). Fix: Vehicle._on_heartbeat ignores autopilot-INVALID
heartbeats for state, and _route does not CREATE a vehicle from INVALID-only heartbeat traffic --
matching QGC's rule. Vehicle creation from first telemetry (heartbeat late/lost) is preserved."""
import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication, QMessageBox
import main as m
import mavlink

app = QApplication.instance() or QApplication([])
QMessageBox.information = QMessageBox.warning = QMessageBox.critical = staticmethod(lambda *a, **k: None)
QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.StandardButton.No)

PORT = 14791
ARMED = mavlink.MAV_MODE_FLAG_SAFETY_ARMED


class Msg:
    __slots__ = ("msgid", "sysid", "compid", "seq", "fields")
    def __init__(s, mid, sysid=1, compid=1, **f):
        s.msgid, s.sysid, s.compid, s.seq, s.fields = mid, sysid, compid, 0, f


def fc_hb(sysid=1, base_mode=ARMED | 1, custom_mode=3 << 16):        # PX4 POSCTL, armed
    return Msg(mavlink.HEARTBEAT, sysid=sysid, autopilot=12, type=2,
               base_mode=base_mode, custom_mode=custom_mode, system_status=4)


def gimbal_hb(sysid=1):                                              # peripheral: autopilot INVALID
    return Msg(mavlink.HEARTBEAT, sysid=sysid, compid=154, autopilot=8, type=26,
               base_mode=0, custom_mode=0, system_status=4)


def gcs_hb(sysid=255):                                               # another ground station
    return Msg(mavlink.HEARTBEAT, sysid=sysid, compid=190, autopilot=8, type=6,
               base_mode=0, custom_mode=0, system_status=4)


win = m.DroneDeck(PORT)
win._persist = False

# 1) peripheral heartbeat must NOT flap vehicle state ------------------------------------------------
win._route([fc_hb()])
ve = win.vehicles[1]
assert ve.armed is True and ve.mode == "POSCTL" and ve.mav_type == 2 and ve.autopilot == 12
win._route([gimbal_hb()])
assert ve.armed is True, "gimbal heartbeat flipped armed state"
assert ve.mode == "POSCTL", f"gimbal heartbeat flipped mode to {ve.mode!r}"
assert ve.mav_type == 2 and ve.autopilot == 12, "gimbal heartbeat overwrote type/autopilot"

# 2) a peripheral must not keep a dead FC looking alive ---------------------------------------------
ve.last_heartbeat = 1.0                       # long-stale FC
win._route([gimbal_hb()])
assert ve.last_heartbeat == 1.0, "peripheral heartbeat refreshed last_heartbeat (masks comm loss)"
win._route([fc_hb()])
assert time.monotonic() - ve.last_heartbeat < 5.0, "FC heartbeat should refresh last_heartbeat"

# 3) a neighbouring GCS is not a vehicle ------------------------------------------------------------
n_before = win.vehicle_combo.count()
win._route([gcs_hb()])
assert 255 not in win.vehicles, "second GCS tracked as a phantom vehicle"
assert win.vehicle_combo.count() == n_before, "phantom GCS entry appeared in the selector"

# 4) vehicle creation from first TELEMETRY (late/lost heartbeat) is preserved -----------------------
win._route([Msg(mavlink.ATTITUDE, sysid=7, roll=0.1, pitch=0.0, yaw=1.0)])
assert 7 in win.vehicles, "telemetry-first vehicle creation broke"

# 5) a real second vehicle still gets created from its FC heartbeat ---------------------------------
win._route([fc_hb(sysid=2)])
assert 2 in win.vehicles and win.vehicles[2].mode == "POSCTL"

# 6) peripheral NON-heartbeat traffic to an EXISTING vehicle is still consumed ----------------------
win._route([Msg(mavlink.STATUSTEXT, sysid=1, compid=154, severity=6, text="gimbal ok")])
assert any("gimbal ok" in t for _s, t in win.vehicles[1].messages), "peripheral statustext dropped"

# 7) a sysid first seen as GCS-only, later a real FC: gets created then ------------------------------
win._route([gcs_hb(sysid=250)])
assert 250 not in win.vehicles
win._route([fc_hb(sysid=250)])
assert 250 in win.vehicles, "real FC on a previously-skipped sysid must create the vehicle"

print("SYSIDFILTER PASSED (peripheral heartbeats don't flap state or mask comm loss; "
      "no phantom GCS vehicle; telemetry-first creation preserved)", flush=True)
os._exit(0)      # skip Qt offscreen teardown (segfault-prone); result already printed + flushed
