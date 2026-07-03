#!/usr/bin/env python3
"""test_preflight.py -- arming-readiness (preflight) checks + dialog (iter156, QGC parity).

QGC gates arming on preflight checks and shows why. preflight.preflight_checks mirrors the conditions a
GCS can see from telemetry (GPS, estimator, home, battery, sensor health, armed) into a single READY /
NOT READY verdict. Verifies the pure evaluation across states (all-good, bad GPS, unhealthy sensor,
low/critical battery, no-telemetry, armed) and that the live dialog renders READY vs NOT READY."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import mavlink
from vehicle import Vehicle
import preflight

fail = []
SENS = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 5)   # gyro/accel/mag/baro/gps
EKF_OK = mavlink.ESTIMATOR_ATTITUDE | mavlink.ESTIMATOR_VELOCITY_HORIZ | mavlink.ESTIMATOR_POS_HORIZ_ABS


def good():
    ve = Vehicle()
    ve.fix_type, ve.satellites = 3, 12
    ve.ekf_flags = EKF_OK
    ve.home = (47.3, 8.5)
    ve.battery_remaining = 85
    ve.sensors_present = ve.sensors_enabled = ve.sensors_health = SENS
    ve.armed = False
    return ve


def status_of(checks, name):
    return next((s for n, s, _d in checks if n == name), None)


# 1) all-good vehicle -> everything passes, READY ---------------------------------------------------
c = preflight.preflight_checks(good())
if not preflight.is_ready(c):
    fail.append(f"good vehicle should be READY: {[(n, s) for n, s, _ in c]}")
if any(s != "pass" for _n, s, _d in c):
    fail.append(f"good vehicle should be all-pass: {[(n, s) for n, s, _ in c]}")

# 2) bad GPS -> GPS fail -> NOT READY ---------------------------------------------------------------
ve = good()
ve.fix_type, ve.satellites = 1, 3
c = preflight.preflight_checks(ve)
if status_of(c, "GPS") != "fail" or preflight.is_ready(c):
    fail.append("bad GPS should fail + block readiness")

# 3) an enabled-but-unhealthy sensor -> Sensors fail ------------------------------------------------
ve = good()
ve.sensors_health = SENS & ~(1 << 2)             # Mag present+enabled but unhealthy
c = preflight.preflight_checks(ve)
if status_of(c, "Sensors") != "fail":
    fail.append("an unhealthy enabled sensor should fail")
detail = next(d for n, _s, d in c if n == "Sensors")
if "Mag" not in detail:
    fail.append(f"Sensors detail should name Mag, got {detail!r}")

# 4) battery: warn band (15-24%) does not block; critical (<15%) does -------------------------------
ve = good(); ve.battery_remaining = 18
c = preflight.preflight_checks(ve)
if status_of(c, "Battery") != "warn" or not preflight.is_ready(c):
    fail.append("18% battery should warn but not block")
ve = good(); ve.battery_remaining = 9
c = preflight.preflight_checks(ve)
if status_of(c, "Battery") != "fail" or preflight.is_ready(c):
    fail.append("9% battery should fail + block")

# 5) fresh vehicle (no telemetry) -> unknowns + Home fail -> NOT READY ------------------------------
c = preflight.preflight_checks(Vehicle())
if status_of(c, "GPS") != "unknown" or status_of(c, "Estimator") != "unknown":
    fail.append("fresh vehicle GPS/Estimator should be unknown")
if status_of(c, "Home") != "fail" or preflight.is_ready(c):
    fail.append("fresh vehicle has no home -> NOT READY")

# 6) armed vehicle -> Armed warn (informational, not a block) ---------------------------------------
ve = good(); ve.armed = True
c = preflight.preflight_checks(ve)
if status_of(c, "Armed") != "warn" or not preflight.is_ready(c):
    fail.append("armed should warn but not block readiness")

# 7) dialog renders READY vs NOT READY --------------------------------------------------------------
from PySide6.QtWidgets import QApplication
app = QApplication.instance() or QApplication([])
dlg = preflight.PreflightDialog(lambda ve=good(): ve)
if "READY TO ARM" not in dlg.banner.text():
    fail.append(f"dialog with good vehicle should say READY, got {dlg.banner.text()!r}")
bad = good(); bad.fix_type = 0; bad.satellites = 0; bad.home = None
dlg2 = preflight.PreflightDialog(lambda: bad)
if dlg2.banner.text() != "NOT READY":
    fail.append(f"dialog with bad vehicle should say NOT READY, got {dlg2.banner.text()!r}")
if dlg2.table.rowCount() != len(preflight.preflight_checks(bad)):
    fail.append("dialog table should have one row per check")

print("PREFLIGHT FAILED: " + "; ".join(fail) if fail else
      "PREFLIGHT PASSED (all-good=READY all-pass; bad GPS/critical-battery/unhealthy-sensor fail + block; "
      "battery warn + armed warn don't block; fresh vehicle unknowns + no-home block; dialog READY/NOT "
      "READY)")
sys.stdout.flush()
os._exit(1 if fail else 0)
