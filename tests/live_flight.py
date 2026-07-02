#!/usr/bin/env python3
"""live_flight.py -- END-TO-END flight verification against a REAL autopilot (PX4 SITL) on udp:14550.

Unlike live_smoke (read-only), this exercises the WRITE/command path through a full flight cycle and
checks the vehicle STATE actually responds: pre-arm readiness -> ARM -> TAKEOFF (climb) -> mode change
-> LAND (descend) -> DISARM. It always tries to LAND + DISARM at the end, even on partial failure, so
it leaves the sim on the ground. SKIPs cleanly (exit 0) if no vehicle. NOT in run_all.sh.

Usage:  QT_QPA_PLATFORM=offscreen python3 tests/live_flight.py [udp_port]
"""
import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QCoreApplication
import mavlink

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 14550
app = QApplication.instance() or QApplication([])
import main as m

win = m.DroneDeck(PORT)
win._persist = False
win._connect()
ve = win.vehicle
sysid = None
fails = []


def pump(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        QCoreApplication.processEvents()
        time.sleep(0.01)


def wait_for(cond, timeout, label):
    """Pump until cond() is true or timeout; return whether it happened."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        pump(0.2)
        if cond():
            print(f"  [ok] {label} ({time.monotonic()-t0:.1f}s)")
            return True
    print(f"  [XX] {label} -- TIMEOUT after {timeout}s")
    fails.append(label)
    return False


# wait for a heartbeat, else skip cleanly
t0 = time.monotonic()
while ve.msg_count == 0 and time.monotonic() - t0 < 15:
    pump(0.5)
if ve.msg_count == 0:
    print(f"LIVE FLIGHT SKIPPED (no MAVLink on udp:{PORT})")
    sys.exit(0)
# a fresh SITL needs GPS lock + EKF convergence, and PX4 needs a moment of our GCS heartbeat before it
# will arm -- settle up to 45s for a 3D fix, then a few more seconds, before touching anything.
t0 = time.monotonic()
while ve.fix_type < 3 and time.monotonic() - t0 < 45:
    pump(0.5)
pump(8)
sysid = win._sysid()
print(f"=== vehicle sysid={sysid} autopilot={'PX4' if ve.autopilot==mavlink.MAV_AUTOPILOT_PX4 else ve.autopilot} "
      f"mode={ve.mode} armed={ve.armed} alt_rel={ve.alt_rel:.1f} ===")

ready, reasons = (ve.preflight_status() if hasattr(ve, "preflight_status") else (True, []))
print(f"pre-arm: {'READY' if ready else 'NOT READY -- ' + '; '.join(reasons)}")

try:
    # 1) ARM (retry -- a fresh PX4 may reject the first attempt until health checks settle) ---------
    print("=== ARM ===")
    armed = False
    for attempt in (1, 2, 3):
        win.link.arm(sysid, True)
        t0 = time.monotonic()
        while not ve.armed and time.monotonic() - t0 < 8:
            pump(0.2)
        if ve.armed:
            armed = True
            print(f"  [ok] vehicle ARMED (attempt {attempt})")
            break
        print(f"  attempt {attempt}: not armed yet, settling...")
        pump(8)
    if not armed:
        print("  [XX] vehicle did not ARM")
        fails.append("vehicle ARMED")

    if armed:
        # 2) TAKEOFF to 20 m -----------------------------------------------------------------------
        print("=== TAKEOFF 20 m ===")
        base_alt = ve.alt_rel
        win.link.takeoff(sysid, 20.0, ve.lat, ve.lon)
        wait_for(lambda: ve.alt_rel > base_alt + 5.0, 30, "climbed >5 m (takeoff)")
        pump(6)
        print(f"  altitude now {ve.alt_rel:.1f} m, mode {ve.mode}")

        # 3) MODE CHANGE -> Hold/Loiter ------------------------------------------------------------
        print("=== MODE CHANGE ===")
        start_mode = ve.mode
        cmd = None
        for name in ("AUTO.LOITER", "Hold", "Loiter", "AUTO.RTL", "POSCTL"):
            cmd = mavlink.mode_command(ve.autopilot, ve.mav_type, name)
            if cmd:
                break
        if cmd:
            win.link.set_mode(sysid, cmd[0], cmd[1] if len(cmd) > 1 else 0)
            wait_for(lambda: ve.mode != start_mode, 8, f"mode changed from {start_mode} to {name}")
            print(f"  mode now {ve.mode}")
        else:
            print("  (no hold-like mode mapping -- skipping)")

    # 4) LAND --------------------------------------------------------------------------------------
    print("=== LAND ===")
    top = ve.alt_rel
    win.link.land(sysid)
    if top > 3.0:
        wait_for(lambda: ve.alt_rel < top - 3.0, 40, "descending (land)")
    wait_for(lambda: not ve.armed, 45, "auto-DISARMED after land")

finally:
    # safety net: force the sim back to a safe ground state no matter what happened above
    print("=== cleanup (ensure grounded + disarmed) ===")
    if ve.armed:
        win.link.land(sysid)
        pump(8)
        if ve.armed:
            win.link.force_disarm(sysid)
            pump(3)
    print(f"  final: armed={ve.armed} alt_rel={ve.alt_rel:.1f} mode={ve.mode}")
    try:
        win.link.close()
    except Exception:
        pass

if fails:
    print("LIVE FLIGHT FAILED:", fails)
    sys.exit(1)
print("LIVE FLIGHT PASSED (arm -> takeoff/climb -> mode change -> land/descend -> disarm all verified live)")
