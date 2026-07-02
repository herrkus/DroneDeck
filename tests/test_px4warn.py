#!/usr/bin/env python3
"""test_px4warn.py -- autopilot-specific mission advisory. PX4 supports a narrower mission-command
set than ArduPilot and rejects the WHOLE upload (MAV_MISSION_UNSUPPORTED) for commands it lacks, so
autopilot_mission_warnings() warns first -- only for PX4, and only for the specific items. The
unsupported set was verified LIVE against PX4 SITL: RTL (iter80) plus Loiter-turns / Set-servo /
Condition-Yaw (iter126, tests/live_missionitems.py); Delay + Cam-trigger-distance are accepted. No link."""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

import mavlink
from mission import MissionItem, autopilot_mission_warnings

PX4 = mavlink.MAV_AUTOPILOT_PX4                      # 12
ARDU = mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA          # 3
WP = mavlink.MAV_CMD_NAV_WAYPOINT
RTL = mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
LAND = mavlink.MAV_CMD_NAV_LAND


def wp(seq, cmd):
    return MissionItem(seq, 47.4, 8.5, 25.0, command=cmd)


with_rtl = [wp(0, WP), wp(1, WP), MissionItem(2, 0.0, 0.0, 0.0, command=RTL)]
with_land = [wp(0, WP), MissionItem(1, 47.4, 8.5, 0.0, command=LAND)]

# PX4 + an RTL item -> exactly one advisory, mentioning RTL/Return-To-Launch
w = autopilot_mission_warnings(with_rtl, PX4)
assert len(w) == 1 and ("Return-To-Launch" in w[0] or "RTL" in w[0]), w

# PX4 without RTL -> no advisory
assert autopilot_mission_warnings(with_land, PX4) == []

# ArduPilot with RTL -> no advisory (ArduPilot accepts RTL mission items)
assert autopilot_mission_warnings(with_rtl, ARDU) == []

# unknown/generic autopilot with RTL -> no advisory (only PX4 is known to reject)
assert autopilot_mission_warnings(with_rtl, 0) == []

# empty mission -> no advisory, no crash
assert autopilot_mission_warnings([], PX4) == []

# each live-verified PX4-unsupported item warns (once), and mentions the item -----------------------
for cmd, needle in ((18, "Loiter"), (183, "servo"), (115, "Yaw")):
    w = autopilot_mission_warnings([wp(0, WP), wp(1, cmd)], PX4)
    assert len(w) == 1 and needle.lower() in w[0].lower(), (cmd, w)
    assert autopilot_mission_warnings([wp(0, WP), wp(1, cmd)], ARDU) == [], f"ArduPilot cmd {cmd}"

# live-verified PX4-ACCEPTED items must NOT warn (Delay 93, Cam-trigger-distance 206) ---------------
for cmd in (93, 206):
    assert autopilot_mission_warnings([wp(0, WP), wp(1, cmd)], PX4) == [], f"cmd {cmd} should not warn"

# a mission hitting several unsupported items -> one advisory each, deduped by command --------------
mixed = [wp(0, WP), wp(1, 18), wp(2, 115), wp(3, 115), MissionItem(4, 0, 0, 0, command=RTL)]
w = autopilot_mission_warnings(mixed, PX4)
assert len(w) == 3, w                               # loiter-turns, condition-yaw (once), RTL

print("PX4WARN PASSED")
