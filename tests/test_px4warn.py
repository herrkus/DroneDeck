#!/usr/bin/env python3
"""test_px4warn.py -- autopilot-specific mission advisory. PX4 rejects a NAV_RETURN_TO_LAUNCH
mission item (verified live in iter80: MAV_MISSION_UNSUPPORTED), so autopilot_mission_warnings()
warns before the upload fails -- but only for PX4, and only when an RTL item is present. No link."""
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

with_rtl = [MissionItem(0, 47.4, 8.5, 25.0, command=WP),
            MissionItem(1, 47.41, 8.51, 30.0, command=WP),
            MissionItem(2, 0.0, 0.0, 0.0, command=RTL)]
with_land = [MissionItem(0, 47.4, 8.5, 25.0, command=WP),
             MissionItem(1, 47.4, 8.5, 0.0, command=LAND)]

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

print("PX4WARN PASSED")
