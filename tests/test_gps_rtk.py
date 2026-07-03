#!/usr/bin/env python3
"""test_gps_rtk.py -- RTK GPS status parsing GPS_RTK(127) + GPS2_RTK(128) (iter147, QGC parity).

QGC shows RTK receiver health (survey-grade positioning): baseline to the base station, satellites
used in the RTK solution, and the solution accuracy. DroneDeck parsed none of it. Added both messages
to BOTH parsers (native C++/asm core + pure-Python fallback; CRC_EXTRA 25/226 verified by
crc_extra_calc, identical 35-byte scalar layout), a vehicle handler (mm->m 3D baseline magnitude), and
an RTK readout in the telemetry GPS group. Verifies: both backends decode both messages with identical
+ exact fields; the handler computes the baseline magnitude and stores health/nsats/accuracy; the panel
shows the RTK line healthy/unhealthy and '--' when no RTK receiver is reporting."""
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

# baseline 1000/2000/2000 mm -> magnitude sqrt(1e6+4e6+4e6)=3000mm=3.0 m; accuracy 12mm; 20 sats healthy
VALS = {"time_last_baseline_ms": 123456, "tow": 500, "baseline_a_mm": 1000, "baseline_b_mm": 2000,
        "baseline_c_mm": 2000, "accuracy": 12, "iar_num_hypotheses": 5, "wn": 2200,
        "rtk_receiver_id": 0, "rtk_health": 1, "rtk_rate": 1, "nsats": 20, "baseline_coords_type": 1}


def frame_for(mid):
    names = mavlink._WIRE[mid][1]
    payload = struct.pack(mavlink._WIRE[mid][0], *[VALS[n] for n in names])
    return mavlink.frame(mid, payload, 7, 1, 1, crc_fn=core.crc_extra)


# 1) both backends decode both RTK messages, matching + exact values --------------------------------
print("native backend live:", core.NATIVE)
for mid in (mavlink.GPS_RTK, mavlink.GPS2_RTK):
    fr = frame_for(mid)
    nat = core.Parser().feed(fr)
    pyp = mavlink.PyParser().feed(fr)
    nm = mavlink.MSG_NAME[mid]
    if len(nat) != 1 or len(pyp) != 1:
        fail.append(f"{nm}: decode count native={len(nat)} py={len(pyp)}")
        continue
    for n, exp in VALS.items():
        a, b = nat[0].fields.get(n), pyp[0].fields.get(n)
        if a is None or b is None or abs(a - exp) > 1e-3 or abs(b - exp) > 1e-3:
            fail.append(f"{nm}.{n}: native={a} py={b} expected={exp}")

# 2) vehicle handler computes baseline magnitude (mm->m) + stores health/nsats/accuracy -------------
ve = Vehicle()
ve.consume(core.Parser().feed(frame_for(mavlink.GPS_RTK)))
if abs((ve.rtk_baseline_m or 0) - 3.0) > 1e-6:
    fail.append(f"rtk_baseline_m {ve.rtk_baseline_m} (want 3.0 m from 1000/2000/2000 mm)")
if ve.rtk_health != 1 or ve.rtk_nsats != 20 or ve.rtk_accuracy_mm != 12:
    fail.append(f"rtk state: health={ve.rtk_health} nsats={ve.rtk_nsats} acc={ve.rtk_accuracy_mm}")

# a fresh vehicle reports None (no bogus 0) until a GPS_RTK arrives ---------------------------------
fresh = Vehicle()
if fresh.rtk_health is not None or fresh.rtk_baseline_m is not None:
    fail.append("fresh vehicle should have None RTK state until GPS_RTK arrives")

# an unhealthy solution (rtk_health 0) must be tracked as such --------------------------------------
bad = dict(VALS)
bad["rtk_health"] = 0
bad_payload = struct.pack(mavlink._WIRE[mavlink.GPS_RTK][0],
                          *[bad[n] for n in mavlink._WIRE[mavlink.GPS_RTK][1]])
bad_frame = mavlink.frame(mavlink.GPS_RTK, bad_payload, 7, 1, 1, crc_fn=core.crc_extra)
ve2 = Vehicle()
ve2.consume(core.Parser().feed(bad_frame))
if ve2.rtk_health != 0:
    fail.append(f"unhealthy rtk_health should be 0, got {ve2.rtk_health}")

# 3) the telemetry panel shows the RTK line, coloured by health; '--' with no receiver --------------
from PySide6.QtWidgets import QApplication
from panels import TelemetryPanel
app = QApplication.instance() or QApplication([])
tp = TelemetryPanel()
tp.update_all(ve, "UDP", 10.0, 100, 0)
rtk = tp.v["rtk"].text()
if "OK" not in rtk or "20 sat" not in rtk or "12 mm" not in rtk:
    fail.append(f"panel RTK shows '{rtk}', expected OK + 20 sat + 12 mm")
tp.update_all(ve2, "UDP", 10.0, 100, 0)
if "no fix" not in tp.v["rtk"].text():
    fail.append(f"panel RTK for unhealthy should say 'no fix', got '{tp.v['rtk'].text()}'")
tp.update_all(fresh, "UDP", 10.0, 100, 0)
if tp.v["rtk"].text() != "--":
    fail.append(f"panel RTK should be '--' with no receiver, got '{tp.v['rtk'].text()}'")

print("GPS_RTK FAILED: " + "; ".join(fail) if fail else
      "GPS_RTK PASSED (GPS_RTK/GPS2_RTK decode identically on native+Python with exact values; handler "
      "computes 3D baseline mm->m + stores health/nsats/accuracy; panel shows RTK OK/no-fix and '--' "
      "when absent)")
sys.exit(1 if fail else 0)
