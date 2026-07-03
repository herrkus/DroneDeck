#!/usr/bin/env python3
"""test_ardupilot_dialect.py -- lock in ArduPilot vehicle handling (iter165, real-drone-ready).

Everything is proven against PX4 SITL, but a huge share of real drones (Pixhawk/Cube) run ArduPilot,
whose dialect differs from PX4 in ways that silently mislead a pilot if wrong:
  * flight modes -- ArduPilot puts the mode DIRECTLY in custom_mode and the table depends on the vehicle
    TYPE (an ArduPlane in custom_mode 0 is MANUAL; the copter table would call it STABILIZE), whereas PX4
    packs main/sub modes into custom_mode bitfields. Same number, different meaning per autopilot+type.
  * telemetry streams -- an ArduPilot vehicle sends almost nothing until asked via the legacy
    REQUEST_DATA_STREAM (PX4 uses SET_MESSAGE_INTERVAL); get this wrong and a freshly-connected ArduPilot
    drone shows a dead HUD.
The behaviour is already correct; this is the regression guard so a future change can't break plane/rover
pilots or the ArduPilot stream request. (PX4 stays covered by its own tests + live SITL.)"""
import os
import sys
import struct

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import mavlink
import core
from vehicle import Vehicle
from link import Link

fail = []
APM = mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
PX4 = mavlink.MAV_AUTOPILOT_PX4
CUSTOM = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
COPTER, PLANE, ROVER = 2, 1, 10


def mode(autopilot, mav_type, custom, base=CUSTOM):
    return mavlink.flight_mode_name(autopilot, mav_type, base, custom)


# 1) ArduPilot mode tables dispatch on vehicle TYPE -------------------------------------------------
cases = [
    (COPTER, 0, "STABILIZE"), (COPTER, 6, "RTL"), (COPTER, 9, "LAND"), (COPTER, 21, "SMART_RTL"),
    (PLANE, 0, "MANUAL"), (PLANE, 10, "AUTO"), (PLANE, 11, "RTL"), (PLANE, 21, "QRTL"),
    (ROVER, 0, "MANUAL"), (ROVER, 10, "AUTO"), (ROVER, 11, "RTL"),
]
for mav_type, cm, want in cases:
    got = mode(APM, mav_type, cm)
    if got != want:
        fail.append(f"APM type={mav_type} cm={cm}: got {got!r} want {want!r}")

# the critical dispatch: SAME custom_mode, different vehicle type -> different mode --------------------
if mode(APM, PLANE, 0) == mode(APM, COPTER, 0):
    fail.append("plane cm=0 (MANUAL) must NOT resolve like copter cm=0 (STABILIZE) -- type dispatch broken")

# 2) PX4 and ArduPilot read the SAME custom_mode differently (autopilot dispatch) --------------------
# ArduPilot cm=6 is RTL (direct); PX4 reads 6 as a bitfield -> main=(6>>16)=0 -> not RTL.
if mode(PX4, COPTER, 6) == "RTL":
    fail.append("PX4 must not decode raw custom_mode=6 as ArduPilot's RTL")
# PX4 AUTO.RTL is main=4 sub=5 packed high; confirm PX4 path still works end of spectrum
px4_auto_rtl = (5 << 24) | (4 << 16)
if mode(PX4, COPTER, px4_auto_rtl) != "AUTO.RTL":
    fail.append(f"PX4 AUTO.RTL bitfield decode wrong: {mode(PX4, COPTER, px4_auto_rtl)!r}")

# 3) no CUSTOM_MODE flag -> raw 'MODE n', never a table lookup --------------------------------------
if mode(APM, COPTER, 6, base=0) != "MODE 6":
    fail.append(f"without CUSTOM_MODE flag should be 'MODE 6', got {mode(APM, COPTER, 6, base=0)!r}")

# 4) end-to-end: ArduPilot heartbeats drive the right Vehicle mode + armed ---------------------------
def hb(mav_type, cm, armed=True):
    base = CUSTOM | (0x80 if armed else 0)
    pl = struct.pack("<IBBBBB", cm, mav_type, APM, base, 4, 3)
    return mavlink.frame(mavlink.HEARTBEAT, pl, 1, 1, 1, crc_fn=core.crc_extra)


for mav_type, cm, want in [(COPTER, 6, "RTL"), (PLANE, 10, "AUTO"), (ROVER, 11, "RTL")]:
    ve = Vehicle()
    ve.consume(core.Parser().feed(hb(mav_type, cm)))
    got = ve.mode
    if got != want or not ve.armed:
        fail.append(f"E2E type={mav_type} cm={cm}: mode {got!r} armed={ve.armed} (want {want!r}, armed)")

# disarmed heartbeat -> armed False
ve = Vehicle()
ve.consume(core.Parser().feed(hb(COPTER, 0, armed=False)))
if ve.armed:
    fail.append("disarmed ArduCopter heartbeat should leave armed=False")

# 5) request_data_streams sends REQUEST_DATA_STREAM for ArduPilot (else a real APM drone shows nothing)
class CaptureLink(Link):
    def __init__(self):
        super().__init__()
        self.sent = []
        self._open = True
        self.remote = True

    def _write(self, data):
        self.sent.append(data)


def sent_msgids(link):
    # read the msgid from the frame header directly -- REQUEST_DATA_STREAM is TX-only (GCS->vehicle) so
    # the parser has no decoder for it; parsing captured frames would silently drop it.
    ids = []
    for f in link.sent:
        if f and f[0] == 0xFE:                          # MAVLink v1
            ids.append(f[5])
        elif f and f[0] == 0xFD:                        # MAVLink v2
            ids.append(f[7] | (f[8] << 8) | (f[9] << 16))
    return ids


apm_link = CaptureLink()
apm_link.request_data_streams(1, autopilot=APM)
apm_ids = sent_msgids(apm_link)
if apm_ids.count(mavlink.REQUEST_DATA_STREAM) < 1:
    fail.append(f"ArduPilot connect must send REQUEST_DATA_STREAM, got msgids {set(apm_ids)}")
if mavlink.COMMAND_LONG not in apm_ids:                 # AUTOPILOT_VERSION request (CMD 512) still sent
    fail.append("ArduPilot connect should still request AUTOPILOT_VERSION")

px4_link = CaptureLink()
px4_link.request_data_streams(1, autopilot=PX4)
px4_ids = sent_msgids(px4_link)
# PX4 gets the modern SET_MESSAGE_INTERVAL commands ON TOP of the legacy stream requests
if px4_ids.count(mavlink.COMMAND_LONG) <= apm_ids.count(mavlink.COMMAND_LONG):
    fail.append("PX4 connect should add SET_MESSAGE_INTERVAL commands beyond ArduPilot's")

print("ARDUPILOT_DIALECT FAILED: " + "; ".join(fail) if fail else
      "ARDUPILOT_DIALECT PASSED (copter/plane/rover mode tables dispatch on vehicle type; same custom_mode "
      "means different modes on ArduPilot vs PX4; no-CUSTOM-flag -> 'MODE n'; ArduPilot heartbeats drive "
      "the right Vehicle mode+armed; REQUEST_DATA_STREAM sent for ArduPilot, +SET_MESSAGE_INTERVAL for PX4)")
sys.stdout.flush()
os._exit(1 if fail else 0)
