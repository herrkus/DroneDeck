#!/usr/bin/env python3
"""test_telemetry_msgs.py -- DISTANCE_SENSOR + NAV_CONTROLLER_OUTPUT + POWER_STATUS parsing (iter140).

QGC surfaces rangefinder AGL, autopilot nav-controller output, and board power rails; DroneDeck parsed
none of them. Added to BOTH parsers (native C++/asm core + pure-Python fallback) with CRC_EXTRA seeds
verified by crc_extra_calc, vehicle handlers (cm->m, mV->V), and a rangefinder readout in the telemetry
panel. Verifies: both backends decode all three with identical fields + exact values; the vehicle
handlers convert and store correctly; the telemetry panel shows the AGL reading."""
import os
import sys
import struct

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import mavlink
import core
from vehicle import Vehicle

fail = []

# (struct fmt, known field values) -- keys must be the wire-order field names
CASES = {
    mavlink.POWER_STATUS: ("<HHH", {"Vcc": 5000, "Vservo": 5100, "flags": 1}),
    mavlink.NAV_CONTROLLER_OUTPUT: ("<fffffhhH", {
        "nav_roll": 1.5, "nav_pitch": -2.5, "alt_error": 3.0, "aspd_error": 0.5,
        "xtrack_error": -1.0, "nav_bearing": 90, "target_bearing": 95, "wp_dist": 120}),
    mavlink.DISTANCE_SENSOR: ("<IHHHBBBB", {
        "time_boot_ms": 1000, "min_distance": 20, "max_distance": 700, "current_distance": 350,
        "type": 0, "id": 1, "orientation": 25, "covariance": 0}),
}


def frame_for(mid):
    fmt, vals = CASES[mid]
    names = mavlink._WIRE[mid][1]
    payload = struct.pack(fmt, *[vals[n] for n in names])
    return mavlink.frame(mid, payload, 7, 1, 1, crc_fn=core.crc_extra)


# 1) both parser backends decode all three, with matching + exact field values -----------------------
print("native backend live:", core.NATIVE)
for mid, (fmt, vals) in CASES.items():
    fr = frame_for(mid)
    nat = core.Parser().feed(fr)                 # native C++/asm
    pyp = mavlink.PyParser().feed(fr)            # pure Python
    nm = mavlink.MSG_NAME[mid]
    if len(nat) != 1 or len(pyp) != 1:
        fail.append(f"{nm}: decode count native={len(nat)} py={len(pyp)}")
        continue
    for n, exp in vals.items():
        a, b = nat[0].fields.get(n), pyp[0].fields.get(n)
        if a is None or b is None or abs(a - exp) > 1e-3 or abs(b - exp) > 1e-3:
            fail.append(f"{nm}.{n}: native={a} py={b} expected={exp}")

# 2) vehicle handlers convert + store (cm->m, mV->V) -------------------------------------------------
ve = Vehicle()
for mid in CASES:
    ve.consume(core.Parser().feed(frame_for(mid)))
checks = [
    (abs((ve.rangefinder_m or 0) - 3.50) < 1e-6, f"rangefinder_m {ve.rangefinder_m} (want 3.50 m from 350 cm)"),
    (abs((ve.rangefinder_min_m or 0) - 0.20) < 1e-6, f"rangefinder_min_m {ve.rangefinder_min_m}"),
    (ve.rangefinder_orientation == 25, f"rangefinder_orientation {ve.rangefinder_orientation}"),
    (abs((ve.power_vcc or 0) - 5.0) < 1e-6, f"power_vcc {ve.power_vcc} (want 5.0 V from 5000 mV)"),
    (abs((ve.power_vservo or 0) - 5.1) < 1e-6, f"power_vservo {ve.power_vservo}"),
    (ve.power_flags == 1, f"power_flags {ve.power_flags}"),
    (ve.nav_bearing == 90, f"nav_bearing {ve.nav_bearing}"),
    (ve.nav_wp_dist == 120, f"nav_wp_dist {ve.nav_wp_dist}"),
    (abs((ve.nav_xtrack_error or 0) + 1.0) < 1e-3, f"nav_xtrack_error {ve.nav_xtrack_error}"),
    (abs((ve.nav_alt_error or 0) - 3.0) < 1e-3, f"nav_alt_error {ve.nav_alt_error}"),
]
for ok, msg in checks:
    if not ok:
        fail.append("vehicle: " + msg)

# a fresh vehicle (no such messages) must report None, not a bogus 0 -------------------------------
fresh = Vehicle()
if fresh.rangefinder_m is not None or fresh.power_vcc is not None or fresh.nav_bearing is not None:
    fail.append("fresh vehicle should have None rangefinder/power/nav until a message arrives")

# 3) the telemetry panel shows the AGL reading ------------------------------------------------------
from PySide6.QtWidgets import QApplication
from panels import TelemetryPanel
app = QApplication.instance() or QApplication([])
tp = TelemetryPanel()
tp.update_all(ve, "UDP", 10.0, 100, 0)
rf = tp.v["rangefinder"].text()
if "3.5" not in rf:
    fail.append(f"panel rangefinder shows '{rf}', expected ~3.50 m")
tp.update_all(fresh, "UDP", 10.0, 100, 0)
if tp.v["rangefinder"].text() != "--":
    fail.append(f"panel rangefinder should be '--' with no sensor, got '{tp.v['rangefinder'].text()}'")

print("TELEMETRY_MSGS FAILED: " + "; ".join(fail) if fail else
      "TELEMETRY_MSGS PASSED (DISTANCE_SENSOR/NAV_CONTROLLER_OUTPUT/POWER_STATUS decode identically on "
      "native+Python with exact values; vehicle handlers convert cm->m and mV->V; panel shows AGL; "
      "absent = None/--)")
sys.stdout.flush()
os._exit(1 if fail else 0)
