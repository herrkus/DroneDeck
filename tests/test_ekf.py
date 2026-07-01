"""Estimator-health messages: EKF_STATUS_REPORT (ArduPilot) and ESTIMATOR_STATUS (PX4).
Crafts frames and checks the C++ core and the Python parser decode them identically, and
that Vehicle maps both onto the shared ekf_* fields. Port-independent (no link)."""
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "app"))

import mavlink
import core
from vehicle import Vehicle


def both(msgid, payload):
    frame = mavlink.frame(msgid, payload, seq=7, sysid=3, compid=1, crc_fn=mavlink.crc16_mcrf4xx)
    cf = core.Parser().feed(frame)
    pf = mavlink.PyParser().feed(frame)
    assert len(cf) == 1 and len(pf) == 1, (msgid, len(cf), len(pf))
    assert cf[0].fields == pf[0].fields, (cf[0].fields, pf[0].fields)   # C++/Python parity
    return cf[0].fields


# EKF_STATUS_REPORT (193): 5 float variances + uint16 flags
ekf = both(mavlink.EKF_STATUS_REPORT, struct.pack("<5fH", 0.10, 0.20, 0.30, 0.40, 0.50, 831))
assert abs(ekf["velocity_variance"] - 0.10) < 1e-4 and ekf["flags"] == 831
v = Vehicle()
v._on_ekf_status(ekf)
assert v.have_ekf and abs(v.ekf_compass_var - 0.40) < 1e-4 and abs(v.ekf_variance_max() - 0.40) < 1e-4
assert v.ekf_ok()                                   # flags 831 healthy, worst var 0.40 < 1.0
v.ekf_pos_horiz_var = 1.5
assert not v.ekf_ok()                               # red variance
v.ekf_pos_horiz_var, v.ekf_flags = 0.2, 1
assert not v.ekf_ok()                               # missing velocity/position flags

# ESTIMATOR_STATUS (230): time_usec, 8 float ratios, uint16 flags
est = both(mavlink.ESTIMATOR_STATUS,
           struct.pack("<Q8fH", 123456, 0.12, 0.22, 0.32, 0.42, 0.05, 0.0, 1.5, 2.0, 831))
assert abs(est["vel_ratio"] - 0.12) < 1e-4 and abs(est["mag_ratio"] - 0.42) < 1e-4 and est["flags"] == 831
v2 = Vehicle()
v2._on_estimator_status(est)
assert v2.have_ekf and abs(v2.ekf_vel_var - 0.12) < 1e-4 and abs(v2.ekf_compass_var - 0.42) < 1e-4
assert abs(v2.ekf_variance_max() - 0.42) < 1e-4 and v2.ekf_ok()

# ESTIMATOR_STATUS flags -> named badges (panels helpers are pure functions, no Qt app)
import panels
states = dict((label, (on, risk)) for label, on, risk in panels.ekf_flag_states(831))
for cap in ("att", "vel-h", "vel-v", "pos-h-rel", "pos-h-abs", "pos-v-abs", "pred-h-rel", "pred-h-abs"):
    assert states[cap][0], cap                    # capability bits set in 831
for clear in ("pos-v-agl", "const-pos", "gps-glitch", "accel-err"):
    assert not states[clear][0], clear            # clear in 831
assert states["gps-glitch"][1] and states["accel-err"][1] and states["const-pos"][1]   # risk-marked
assert '#e05050">gps-glitch' in panels.ekf_flags_html(mavlink.ESTIMATOR_GPS_GLITCH)     # risk set -> red
assert '#37d67a">att' in panels.ekf_flags_html(mavlink.ESTIMATOR_ATTITUDE)              # capability set -> green

print("EKF PASSED")
