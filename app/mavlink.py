"""mavlink.py -- MAVLink message catalogue, constants and a pure-Python encoder.

This module is the single source of truth for message field order, which MUST
match the decoder in core/dronecore.cpp exactly (tests/test_parity.py enforces
it). It depends on nothing native, so it can be used as a reference encoder by
the test telemetry source without the compiled core.
"""
from __future__ import annotations
import struct

# --- message ids ------------------------------------------------------------
HEARTBEAT = 0
SYS_STATUS = 1
GPS_RAW_INT = 24
ATTITUDE = 30
GLOBAL_POSITION_INT = 33
SERVO_OUTPUT_RAW = 36        # actuator PWM outputs (servo1..8_raw us)
RC_CHANNELS = 65
RADIO_STATUS = 109
REQUEST_DATA_STREAM = 66     # GCS->vehicle: legacy telemetry-stream request (ArduPilot)
ALTITUDE = 141
BATTERY_STATUS = 147
VIBRATION = 241
EKF_STATUS_REPORT = 193      # estimator health (ArduPilot): variances + status flags
ESTIMATOR_STATUS = 230       # estimator health (PX4): innovation ratios + status flags
WIND_COV = 231               # wind estimate: NED wind vector + variances
# ESTIMATOR_STATUS_FLAGS bits (shared by ESTIMATOR_STATUS + EKF_STATUS_REPORT)
ESTIMATOR_ATTITUDE = 1
ESTIMATOR_VELOCITY_HORIZ = 2
ESTIMATOR_VELOCITY_VERT = 4
ESTIMATOR_POS_HORIZ_REL = 8
ESTIMATOR_POS_HORIZ_ABS = 16
ESTIMATOR_POS_VERT_ABS = 32
ESTIMATOR_POS_VERT_AGL = 64
ESTIMATOR_CONST_POS_MODE = 128     # dead-reckoning: no position aiding (risk)
ESTIMATOR_PRED_POS_HORIZ_REL = 256
ESTIMATOR_PRED_POS_HORIZ_ABS = 512
ESTIMATOR_GPS_GLITCH = 1024        # GPS glitch detected (risk)
ESTIMATOR_ACCEL_ERROR = 2048       # bad accel data (risk)
MANUAL_CONTROL = 69
VFR_HUD = 74
COMMAND_INT = 75
COMMAND_LONG = 76
COMMAND_ACK = 77
SET_POSITION_TARGET_GLOBAL_INT = 86
STATUSTEXT = 253
# parameter protocol
PARAM_REQUEST_READ = 20
PARAM_REQUEST_LIST = 21
PARAM_VALUE = 22
PARAM_SET = 23
# mission protocol
MISSION_CURRENT = 42
MISSION_REQUEST_LIST = 43
MISSION_COUNT = 44
MISSION_CLEAR_ALL = 45
MISSION_ITEM_REACHED = 46
MISSION_ACK = 47
MISSION_REQUEST_INT = 51
MISSION_ITEM_INT = 73
# onboard log download protocol
LOG_REQUEST_LIST = 117
LOG_ENTRY = 118
LOG_REQUEST_DATA = 119
LOG_DATA = 120
LOG_REQUEST_END = 122
# MAVLink serial/shell passthrough (PX4 nsh console)
SERIAL_CONTROL = 126
SERIAL_CONTROL_DEV_SHELL = 10
SERIAL_CONTROL_FLAG_REPLY = 1
SERIAL_CONTROL_FLAG_RESPOND = 2
SERIAL_CONTROL_FLAG_EXCLUSIVE = 4
SERIAL_CONTROL_FLAG_MULTI = 16
ADSB_VEHICLE = 246
HOME_POSITION = 242          # authoritative home the autopilot uses for RTL (+ altitude)
EXTENDED_SYS_STATE = 245     # landed_state (ground/air/takeoff/landing) + vtol_state
MOUNT_ORIENTATION = 265      # gimbal attitude report (roll/pitch/yaw deg)
ESC_STATUS = 291             # per-ESC rpm / voltage / current (banks of 4 from 'index')

MSG_NAME = {
    HEARTBEAT: "HEARTBEAT",
    SYS_STATUS: "SYS_STATUS",
    GPS_RAW_INT: "GPS_RAW_INT",
    ATTITUDE: "ATTITUDE",
    GLOBAL_POSITION_INT: "GLOBAL_POSITION_INT",
    SERVO_OUTPUT_RAW: "SERVO_OUTPUT_RAW",
    VFR_HUD: "VFR_HUD",
    RC_CHANNELS: "RC_CHANNELS",
    RADIO_STATUS: "RADIO_STATUS",
    REQUEST_DATA_STREAM: "REQUEST_DATA_STREAM",
    ALTITUDE: "ALTITUDE",
    BATTERY_STATUS: "BATTERY_STATUS",
    VIBRATION: "VIBRATION",
    EKF_STATUS_REPORT: "EKF_STATUS_REPORT",
    ESTIMATOR_STATUS: "ESTIMATOR_STATUS",
    WIND_COV: "WIND_COV",
    MOUNT_ORIENTATION: "MOUNT_ORIENTATION",
    HOME_POSITION: "HOME_POSITION",
    EXTENDED_SYS_STATE: "EXTENDED_SYS_STATE",
    ESC_STATUS: "ESC_STATUS",
    MANUAL_CONTROL: "MANUAL_CONTROL",
    COMMAND_INT: "COMMAND_INT",
    COMMAND_LONG: "COMMAND_LONG",
    COMMAND_ACK: "COMMAND_ACK",
    SET_POSITION_TARGET_GLOBAL_INT: "SET_POSITION_TARGET_GLOBAL_INT",
    STATUSTEXT: "STATUSTEXT",
    PARAM_REQUEST_READ: "PARAM_REQUEST_READ",
    PARAM_REQUEST_LIST: "PARAM_REQUEST_LIST",
    PARAM_VALUE: "PARAM_VALUE",
    PARAM_SET: "PARAM_SET",
    MISSION_CURRENT: "MISSION_CURRENT",
    MISSION_REQUEST_LIST: "MISSION_REQUEST_LIST",
    MISSION_COUNT: "MISSION_COUNT",
    MISSION_CLEAR_ALL: "MISSION_CLEAR_ALL",
    MISSION_ITEM_REACHED: "MISSION_ITEM_REACHED",
    MISSION_ACK: "MISSION_ACK",
    MISSION_REQUEST_INT: "MISSION_REQUEST_INT",
    MISSION_ITEM_INT: "MISSION_ITEM_INT",
    LOG_REQUEST_LIST: "LOG_REQUEST_LIST",
    LOG_ENTRY: "LOG_ENTRY",
    LOG_REQUEST_DATA: "LOG_REQUEST_DATA",
    LOG_DATA: "LOG_DATA",
    LOG_REQUEST_END: "LOG_REQUEST_END",
    SERIAL_CONTROL: "SERIAL_CONTROL",
    ADSB_VEHICLE: "ADSB_VEHICLE",
}

# Per-message CRC_EXTRA seed bytes (derived + validated in tests/crc_extra_calc.py).
CRC_EXTRA = {
    HEARTBEAT: 50, SYS_STATUS: 124, GPS_RAW_INT: 24, ATTITUDE: 39,
    GLOBAL_POSITION_INT: 104, SERVO_OUTPUT_RAW: 222, VFR_HUD: 20,
    COMMAND_INT: 158, COMMAND_LONG: 152,
    COMMAND_ACK: 143, STATUSTEXT: 83, SET_POSITION_TARGET_GLOBAL_INT: 5,
    PARAM_REQUEST_READ: 214, PARAM_REQUEST_LIST: 159, PARAM_VALUE: 220, PARAM_SET: 168,
    MISSION_CURRENT: 28, MISSION_REQUEST_LIST: 132, MISSION_COUNT: 221,
    MISSION_CLEAR_ALL: 232, MISSION_ITEM_REACHED: 11, MISSION_ACK: 153,
    MISSION_REQUEST_INT: 196, MISSION_ITEM_INT: 38, MANUAL_CONTROL: 243, RC_CHANNELS: 118,
    ALTITUDE: 47, BATTERY_STATUS: 154, VIBRATION: 90, RADIO_STATUS: 185,
    EKF_STATUS_REPORT: 71, ESTIMATOR_STATUS: 163, WIND_COV: 105, MOUNT_ORIENTATION: 26,
    HOME_POSITION: 104, EXTENDED_SYS_STATE: 130, ESC_STATUS: 10, REQUEST_DATA_STREAM: 148,
    LOG_REQUEST_LIST: 128, LOG_ENTRY: 56, LOG_REQUEST_DATA: 116,
    LOG_DATA: 134, LOG_REQUEST_END: 203, ADSB_VEHICLE: 184,
    SERIAL_CONTROL: 194,
}

# Decoded-field order. Index i here is index i in Decoded.f[] from the C++ core.
FIELDS = {
    HEARTBEAT: ["type", "autopilot", "base_mode", "custom_mode", "system_status", "mavlink_version"],
    SYS_STATUS: ["voltage_battery", "current_battery", "battery_remaining", "load",
                 "onboard_present", "onboard_enabled", "onboard_health"],
    GPS_RAW_INT: ["fix_type", "satellites_visible", "lat", "lon", "alt", "eph", "vel", "cog"],
    ATTITUDE: ["roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed", "time_boot_ms"],
    SERVO_OUTPUT_RAW: ["time_usec"] + [f"servo{i}_raw" for i in range(1, 9)] + ["port"],
    GLOBAL_POSITION_INT: ["lat", "lon", "alt", "relative_alt", "vx", "vy", "vz", "hdg", "time_boot_ms"],
    VFR_HUD: ["airspeed", "groundspeed", "alt", "climb", "heading", "throttle"],
    RC_CHANNELS: (["time_boot_ms"] + [f"chan{i}_raw" for i in range(1, 19)]
                  + ["chancount", "rssi"]),
    RADIO_STATUS: ["rxerrors", "fixed", "rssi", "remrssi", "txbuf", "noise", "remnoise"],
    ALTITUDE: ["time_usec", "altitude_monotonic", "altitude_amsl", "altitude_local",
               "altitude_relative", "altitude_terrain", "bottom_clearance"],
    VIBRATION: ["time_usec", "vibration_x", "vibration_y", "vibration_z",
                "clipping_0", "clipping_1", "clipping_2"],
    EKF_STATUS_REPORT: ["velocity_variance", "pos_horiz_variance", "pos_vert_variance",
                        "compass_variance", "terrain_alt_variance", "flags"],
    ESTIMATOR_STATUS: ["time_usec", "vel_ratio", "pos_horiz_ratio", "pos_vert_ratio",
                       "mag_ratio", "hagl_ratio", "tas_ratio", "pos_horiz_accuracy",
                       "pos_vert_accuracy", "flags"],
    WIND_COV: ["time_usec", "wind_x", "wind_y", "wind_z", "var_horiz", "var_vert",
               "wind_alt", "horiz_accuracy", "vert_accuracy"],
    MOUNT_ORIENTATION: ["time_boot_ms", "roll", "pitch", "yaw", "yaw_absolute"],
    HOME_POSITION: ["latitude", "longitude", "altitude", "x", "y", "z",
                    "q0", "q1", "q2", "q3", "approach_x", "approach_y", "approach_z"],
    ESC_STATUS: (["time_usec"] + [f"rpm{i}" for i in range(1, 5)]
                 + [f"voltage{i}" for i in range(1, 5)]
                 + [f"current{i}" for i in range(1, 5)] + ["index"]),
    EXTENDED_SYS_STATE: ["vtol_state", "landed_state"],
    BATTERY_STATUS: (["current_consumed", "energy_consumed", "temperature"]
                     + [f"voltage{i}" for i in range(1, 11)]
                     + ["current_battery", "id", "battery_function", "type", "battery_remaining"]),
    MANUAL_CONTROL: ["x", "y", "z", "r", "buttons", "target"],
    COMMAND_INT: ["param1", "param2", "param3", "param4", "x", "y", "z", "command",
                  "target_system", "target_component", "frame", "current", "autocontinue"],
    COMMAND_LONG: ["command", "param1", "param2", "param3", "param4", "param5", "param6", "param7"],
    COMMAND_ACK: ["command", "result"],
    SET_POSITION_TARGET_GLOBAL_INT: ["lat_int", "lon_int", "alt", "type_mask"],
    STATUSTEXT: ["severity"],   # `text` is attached separately (string, not in f[])
    PARAM_REQUEST_LIST: ["target_system", "target_component"],
    PARAM_REQUEST_READ: ["param_index"],          # `param_id` attached as string
    PARAM_VALUE: ["param_value", "param_count", "param_index", "param_type"],  # +param_id string
    PARAM_SET: ["param_value", "param_type"],      # +param_id string
    MISSION_CURRENT: ["seq"],
    MISSION_REQUEST_LIST: ["target_system", "target_component", "mission_type"],
    MISSION_COUNT: ["count", "target_system", "target_component", "mission_type"],
    MISSION_CLEAR_ALL: ["target_system", "target_component", "mission_type"],
    MISSION_ITEM_REACHED: ["seq"],
    MISSION_ACK: ["target_system", "target_component", "type", "mission_type"],
    MISSION_REQUEST_INT: ["seq", "target_system", "target_component", "mission_type"],
    MISSION_ITEM_INT: ["seq", "frame", "command", "current", "autocontinue",
                       "param1", "param2", "param3", "param4", "x", "y", "z", "mission_type"],
    LOG_REQUEST_LIST: ["start", "end", "target_system", "target_component"],
    LOG_ENTRY: ["time_utc", "size", "id", "num_logs", "last_log_num"],
    LOG_REQUEST_DATA: ["ofs", "count", "id", "target_system", "target_component"],
    LOG_DATA: ["ofs", "id", "count"],        # `data` (90 bytes) attached separately
    SERIAL_CONTROL: ["baudrate", "timeout", "device", "flags", "count"],  # `data` (70 B) attached
    LOG_REQUEST_END: ["target_system", "target_component"],
    ADSB_VEHICLE: ["ICAO_address", "lat", "lon", "altitude", "heading", "hor_velocity",
                   "ver_velocity", "flags", "squawk", "altitude_type", "emitter_type",
                   "tslc"],                  # `callsign` attached separately
}

# --- selected enums ---------------------------------------------------------
MAV_TYPE_QUADROTOR = 2
MAV_AUTOPILOT_ARDUPILOTMEGA = 3
MAV_MODE_FLAG_SAFETY_ARMED = 0x80
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 0x01
MAV_STATE_ACTIVE = 4
GPS_FIX_TYPE_3D_FIX = 3
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_PREFLIGHT_CALIBRATION = 241     # param1 gyro, param2 mag, param5 accel(1)/level(2)
MAV_CMD_DO_ORBIT = 34
MAV_CMD_DO_SET_HOME = 179
MAV_CMD_DO_SET_ROI_LOCATION = 195
MAV_CMD_DO_SET_ROI_NONE = 197           # cancel any active region-of-interest
MAV_CMD_DO_LAND_START = 189             # marks where the landing sequence begins
MAV_CMD_DO_REPOSITION = 192
MAV_CMD_DO_CHANGE_SPEED = 178           # param1=type (0 air / 1 ground), param2=speed m/s
MAV_CMD_DO_PAUSE_CONTINUE = 193
MAV_CMD_DO_VTOL_TRANSITION = 3000       # param1 = MAV_VTOL_STATE (3 = MC, 4 = FW)
MAV_VTOL_STATE_MC = 3                    # multicopter / hover
MAV_VTOL_STATE_FW = 4                    # fixed-wing / forward flight

# EXTENDED_SYS_STATE enums -> human-readable (0 = undefined/unknown -> blank)
MAV_LANDED_STATE_TEXT = {0: "--", 1: "On ground", 2: "In air", 3: "Taking off", 4: "Landing"}
MAV_VTOL_STATE_TEXT = {0: "--", 1: "-> Fixed-wing", 2: "-> Multicopter",
                       3: "Multicopter", 4: "Fixed-wing"}
# camera + gimbal (all carried by COMMAND_LONG)
MAV_CMD_DO_MOUNT_CONTROL = 205          # gimbal: param1=pitch, param2=roll, param3=yaw
MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206     # param1=distance m (0 = off)
MAV_CMD_DO_DIGICAM_CONTROL = 203        # param5=1 -> trigger one shot
MAV_CMD_IMAGE_START_CAPTURE = 2000      # param3=count (1 = single)
MAV_CMD_VIDEO_START_CAPTURE = 2500
MAV_CMD_VIDEO_STOP_CAPTURE = 2501
MAV_CMD_SET_MESSAGE_INTERVAL = 511      # param1=msgid, param2=interval us (-1 off, 0 default)
MAV_CMD_REQUEST_MESSAGE = 512           # param1=msgid -- one-shot request (e.g. AUTOPILOT_VERSION)

# Legacy REQUEST_DATA_STREAM req_stream_id values (ArduPilot honours these).
MAV_DATA_STREAM_ALL = 0
MAV_DATA_STREAM_RAW_SENSORS = 1
MAV_DATA_STREAM_EXTENDED_STATUS = 2
MAV_DATA_STREAM_RC_CHANNELS = 3
MAV_DATA_STREAM_POSITION = 6
MAV_DATA_STREAM_EXTRA1 = 10
MAV_DATA_STREAM_EXTRA2 = 11
MAV_DATA_STREAM_EXTRA3 = 12
MAV_MOUNT_MODE_MAVLINK_TARGETING = 2
MAV_FRAME_GLOBAL = 0
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
MAV_FRAME_GLOBAL_INT = 5                 # AMSL altitude, lat/lon as int (MISSION_ITEM_INT)
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6    # altitude above home, lat/lon as int
MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11    # altitude above terrain (AGL), lat/lon as int
MAV_PARAM_TYPE_REAL32 = 9          # ArduPilot stores every parameter as REAL32
MAV_MISSION_TYPE_MISSION = 0
MAV_MISSION_TYPE_FENCE = 1
MAV_MISSION_TYPE_RALLY = 2
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002
MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION = 5003       # param1 = radius (m)
MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004       # param1 = radius (m)
MAV_CMD_NAV_RALLY_POINT = 5100

# MAV_SYS_STATUS_SENSOR bits (SYS_STATUS onboard_control_sensors_*). Curated set
# shown in the health panel: (bit, short label).
SENSOR_BITS = [
    (1 << 0, "Gyro"), (1 << 1, "Accel"), (1 << 2, "Mag"), (1 << 3, "Baro"),
    (1 << 4, "AirSpd"), (1 << 5, "GPS"), (1 << 6, "OptFlow"), (1 << 15, "Motors"),
    (1 << 16, "RC"), (1 << 20, "Fence"), (1 << 21, "AHRS"), (1 << 22, "Terrain"),
    (1 << 24, "Logging"), (1 << 25, "Battery"),
]
# SET_POSITION_TARGET type_mask: use position fields only (ignore vel/accel/yaw).
POSITION_TARGET_TYPEMASK_POS_ONLY = 0x0DF8

MAV_SEVERITY = {0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL", 3: "ERROR",
                4: "WARNING", 5: "NOTICE", 6: "INFO", 7: "DEBUG"}
MAV_RESULT = {0: "ACCEPTED", 1: "TEMP REJECTED", 2: "DENIED", 3: "UNSUPPORTED",
              4: "FAILED", 5: "IN PROGRESS", 6: "CANCELLED"}

MAV_AUTOPILOT_PX4 = 12

# --- flight mode tables (custom_mode -> name) -------------------------------
ARDUCOPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
    6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP",
    15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB",
    20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG",
    25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL",
}
ARDUPLANE_MODES = {
    0: "MANUAL", 1: "CIRCLE", 2: "STABILIZE", 3: "TRAINING", 4: "ACRO", 5: "FBWA",
    6: "FBWB", 7: "CRUISE", 8: "AUTOTUNE", 10: "AUTO", 11: "RTL", 12: "LOITER",
    13: "TAKEOFF", 14: "AVOID_ADSB", 15: "GUIDED", 17: "QSTABILIZE", 18: "QHOVER",
    19: "QLOITER", 20: "QLAND", 21: "QRTL", 22: "QAUTOTUNE", 23: "QACRO", 24: "THERMAL",
}
ARDUROVER_MODES = {
    0: "MANUAL", 1: "ACRO", 3: "STEERING", 4: "HOLD", 5: "LOITER", 6: "FOLLOW",
    7: "SIMPLE", 10: "AUTO", 11: "RTL", 12: "SMART_RTL", 15: "GUIDED", 16: "INITIALISING",
}
_PX4_MAIN = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 5: "ACRO", 6: "OFFBOARD",
             7: "STABILIZED", 8: "RATTITUDE", 4: "AUTO"}
_PX4_AUTO_SUB = {1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION", 5: "RTL",
                 6: "LAND", 7: "RTGS", 8: "FOLLOW", 10: "PRECLAND"}
# Forward map for SETTING PX4 modes: label -> (main_mode, sub_mode).
_PX4_MODE_SET = {
    "MANUAL": (1, 0), "ALTCTL": (2, 0), "POSCTL": (3, 0), "ACRO": (5, 0),
    "OFFBOARD": (6, 0), "STABILIZED": (7, 0), "AUTO.TAKEOFF": (4, 2),
    "AUTO.LOITER": (4, 3), "AUTO.MISSION": (4, 4), "AUTO.RTL": (4, 5), "AUTO.LAND": (4, 6),
}


def mode_table(autopilot, mav_type):
    """The custom_mode -> name table appropriate for this vehicle."""
    if mav_type == 1:                       # FIXED_WING
        return ARDUPLANE_MODES
    if mav_type in (10, 11):                # GROUND_ROVER / SURFACE_BOAT
        return ARDUROVER_MODES
    return ARDUCOPTER_MODES                  # copters + default


def flight_mode_name(autopilot, mav_type, base_mode, custom_mode):
    custom_mode = int(custom_mode)
    if int(autopilot) == MAV_AUTOPILOT_PX4:
        main = (custom_mode >> 16) & 0xFF
        sub = (custom_mode >> 24) & 0xFF
        if main == 4:
            return "AUTO." + _PX4_AUTO_SUB.get(sub, str(sub))
        return _PX4_MAIN.get(main, f"MODE {main}")
    if not (int(base_mode) & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED):
        return f"MODE {custom_mode}"
    return mode_table(autopilot, mav_type).get(custom_mode, f"MODE {custom_mode}")


def crc16_mcrf4xx(data: bytes, extra: int) -> int:
    """Pure-Python CRC-16/MCRF4XX over `data` then the message `extra` seed.

    The native core computes this in assembly; this fallback keeps the encoder
    usable on its own and serves as an independent cross-check in the tests.
    """
    crc = 0xFFFF
    for b in data:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    b = extra
    tmp = (b ^ (crc & 0xFF)) & 0xFF
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def frame(msgid: int, payload: bytes, seq: int, sysid: int, compid: int, crc_fn=None) -> bytes:
    """Wrap a payload in a MAVLink v1 frame. `crc_fn(data, extra) -> int` lets a
    caller substitute the native assembly CRC; defaults to the Python one."""
    if crc_fn is None:
        crc_fn = crc16_mcrf4xx
    head = bytes((0xFE, len(payload), seq & 0xFF, sysid & 0xFF, compid & 0xFF, msgid & 0xFF))
    crc = crc_fn(head[1:] + payload, CRC_EXTRA[msgid])
    return head + payload + struct.pack("<H", crc)


def frame_v2(msgid: int, payload: bytes, seq: int, sysid: int, compid: int, crc_fn=None) -> bytes:
    """Wrap a payload in a MAVLink v2 frame (0xFD, unsigned). Needed for messages
    with extension fields (e.g. SERIAL_CONTROL's target_system) that some autopilots
    only honour over v2 -- PX4's nsh shell is one."""
    if crc_fn is None:
        crc_fn = crc16_mcrf4xx
    head = bytes((0xFD, len(payload) & 0xFF, 0, 0, seq & 0xFF, sysid & 0xFF, compid & 0xFF,
                  msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF))
    crc = crc_fn(head[1:] + payload, CRC_EXTRA[msgid])
    return head + payload + struct.pack("<H", crc)


# --- payload encoders (wire order matches the C++ decoder offsets) -----------
def enc_heartbeat(mav_type=MAV_TYPE_QUADROTOR, autopilot=MAV_AUTOPILOT_ARDUPILOTMEGA,
                  base_mode=0, custom_mode=0, system_status=MAV_STATE_ACTIVE):
    return struct.pack("<IBBBBB", custom_mode, mav_type, autopilot, base_mode, system_status, 3)


def enc_sys_status(voltage_mv, current_ca, remaining_pct, load=250,
                   present=0, enabled=0, health=0):
    return struct.pack("<IIIHHhHHHHHHb", int(present) & 0xFFFFFFFF,
                       int(enabled) & 0xFFFFFFFF, int(health) & 0xFFFFFFFF,
                       load, int(voltage_mv), int(current_ca),
                       0, 0, 0, 0, 0, 0, int(remaining_pct))


def enc_gps_raw_int(lat, lon, alt_mm, fix=GPS_FIX_TYPE_3D_FIX, sats=12, vel_cms=0, cog_cdeg=0):
    return struct.pack("<QiiiHHHHBB", 0, int(lat), int(lon), int(alt_mm),
                       9999, 9999, int(vel_cms), int(cog_cdeg), fix, sats)


def enc_attitude(roll, pitch, yaw, t_ms, rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0):
    return struct.pack("<Iffffff", t_ms & 0xFFFFFFFF, roll, pitch, yaw,
                       rollspeed, pitchspeed, yawspeed)


def enc_global_position_int(lat, lon, alt_mm, rel_alt_mm, hdg_cdeg, t_ms, vx=0, vy=0, vz=0):
    return struct.pack("<IiiiihhhH", t_ms & 0xFFFFFFFF, int(lat), int(lon), int(alt_mm),
                       int(rel_alt_mm), int(vx), int(vy), int(vz), int(hdg_cdeg) & 0xFFFF)


def enc_vfr_hud(airspeed, groundspeed, alt, climb, heading_deg, throttle_pct):
    return struct.pack("<ffffhH", airspeed, groundspeed, alt, climb,
                       int(heading_deg), int(throttle_pct))


def enc_rc_channels(chans, rssi=200, time_boot_ms=0):
    chans = [int(c) & 0xFFFF for c in list(chans)[:18]]
    count = len(chans)
    chans = chans + [65535] * (18 - count)     # UINT16_MAX = channel not used
    return struct.pack("<I18HBB", int(time_boot_ms) & 0xFFFFFFFF, *chans, count, rssi & 0xFF)


def enc_radio_status(rssi, remrssi, noise=40, remnoise=40, txbuf=100, rxerrors=0, fixed=0):
    return struct.pack("<HHBBBBB", int(rxerrors) & 0xFFFF, int(fixed) & 0xFFFF,
                       int(rssi) & 0xFF, int(remrssi) & 0xFF, int(txbuf) & 0xFF,
                       int(noise) & 0xFF, int(remnoise) & 0xFF)


def enc_altitude(altitude_monotonic, altitude_amsl, altitude_local, altitude_relative,
                 altitude_terrain=0.0, bottom_clearance=0.0, time_usec=0):
    return struct.pack("<Qffffff", int(time_usec), altitude_monotonic, altitude_amsl,
                       altitude_local, altitude_relative, altitude_terrain, bottom_clearance)


def enc_vibration(vibration_x, vibration_y, vibration_z, clipping_0=0, clipping_1=0,
                  clipping_2=0, time_usec=0):
    return struct.pack("<QfffIII", int(time_usec), vibration_x, vibration_y, vibration_z,
                       int(clipping_0), int(clipping_1), int(clipping_2))


def enc_battery_status(voltages, current_battery=-1, current_consumed=-1, energy_consumed=-1,
                       battery_remaining=100, temperature=2500, bat_id=0,
                       battery_function=0, bat_type=0):
    v = [int(x) & 0xFFFF for x in list(voltages)[:10]]
    v = v + [65535] * (10 - len(v))                 # UINT16_MAX = cell not present
    return struct.pack("<iih10HhBBBb", int(current_consumed), int(energy_consumed),
                       int(temperature), *v, int(current_battery), bat_id & 0xFF,
                       battery_function & 0xFF, bat_type & 0xFF, int(battery_remaining))


def enc_manual_control(target, x, y, z, r, buttons=0):
    """x=pitch, y=roll, z=thrust, r=yaw, each -1000..1000 (z 0..1000 = throttle)."""
    clamp = lambda v: max(-1000, min(1000, int(v)))
    return struct.pack("<hhhhHB", clamp(x), clamp(y), clamp(z), clamp(r),
                       buttons & 0xFFFF, target & 0xFF)


def enc_request_data_stream(target_system, req_stream_id, req_message_rate, start_stop,
                            target_component=1):
    # wire order (size desc): req_message_rate(u16), then the u8 fields
    return struct.pack("<HBBBB", int(req_message_rate) & 0xFFFF,
                       int(target_system) & 0xFF, int(target_component) & 0xFF,
                       int(req_stream_id) & 0xFF, 1 if start_stop else 0)


def enc_log_request_list(start=0, end=0xFFFF, target_system=1, target_component=1):
    return struct.pack("<HHBB", start & 0xFFFF, end & 0xFFFF, target_system, target_component)


def enc_log_entry(log_id, num_logs, last_log_num, size, time_utc=0):
    return struct.pack("<IIHHH", int(time_utc) & 0xFFFFFFFF, int(size) & 0xFFFFFFFF,
                       log_id & 0xFFFF, num_logs & 0xFFFF, last_log_num & 0xFFFF)


def enc_log_request_data(log_id, ofs=0, count=0xFFFFFFFF, target_system=1, target_component=1):
    return struct.pack("<IIHBB", int(ofs) & 0xFFFFFFFF, int(count) & 0xFFFFFFFF,
                       log_id & 0xFFFF, target_system, target_component)


def enc_log_data(log_id, ofs, data):
    data = bytes(data[:90])
    count = len(data)
    data = data + b"\x00" * (90 - count)
    return struct.pack("<IHB90s", int(ofs) & 0xFFFFFFFF, log_id & 0xFFFF, count, data)


def enc_log_request_end(target_system=1, target_component=1):
    return struct.pack("<BB", target_system, target_component)


def enc_adsb_vehicle(icao, lat, lon, alt_mm, heading_cdeg, callsign="", emitter_type=0,
                     hor_velocity=0, ver_velocity=0, flags=0, squawk=0,
                     altitude_type=0, tslc=0):
    cs = callsign.encode("ascii", "replace")[:9]
    cs = cs + b"\x00" * (9 - len(cs))
    return struct.pack("<IiiiHHhHHB9sBB", icao & 0xFFFFFFFF, int(lat), int(lon), int(alt_mm),
                       int(heading_cdeg) & 0xFFFF, hor_velocity & 0xFFFF, int(ver_velocity),
                       flags & 0xFFFF, squawk & 0xFFFF, altitude_type & 0xFF, cs,
                       emitter_type & 0xFF, tslc & 0xFF)


def enc_command_int(command, params4, x, y, z, target_system=1, target_component=1, frame=6):
    p = [float(v) for v in (list(params4) + [0.0] * 4)[:4]]
    return struct.pack("<ffffiifHBBBBB", p[0], p[1], p[2], p[3], int(x), int(y), float(z),
                       int(command) & 0xFFFF, target_system & 0xFF, target_component & 0xFF,
                       frame & 0xFF, 0, 0)


def enc_command_long(command, params7, target_system=1, target_component=1, confirmation=0):
    p = [float(x) for x in (list(params7) + [0.0] * 7)[:7]]
    return struct.pack("<fffffffHBBB", *p, command & 0xFFFF,
                       target_system & 0xFF, target_component & 0xFF, confirmation & 0xFF)


def enc_statustext(severity, text):
    return struct.pack("<B50s", int(severity) & 0xFF, text.encode("utf-8")[:50])


def enc_command_ack(command, result):
    return struct.pack("<HB", int(command) & 0xFFFF, int(result) & 0xFF)


def enc_set_position_target_global_int(lat_deg, lon_deg, alt_rel,
                                       type_mask=POSITION_TARGET_TYPEMASK_POS_ONLY,
                                       frame=MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                       target_system=1, target_component=1, time_boot_ms=0):
    return struct.pack("<Iii" + "f" * 9 + "HBBB",
                       time_boot_ms & 0xFFFFFFFF, int(lat_deg * 1e7), int(lon_deg * 1e7),
                       float(alt_rel), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       type_mask & 0xFFFF, target_system & 0xFF, target_component & 0xFF,
                       frame & 0xFF)


def _pid(param_id):
    return param_id.encode("ascii", "replace")[:16] if isinstance(param_id, str) else bytes(param_id)[:16]


def enc_param_request_list(target_system=1, target_component=1):
    return struct.pack("<BB", target_system & 0xFF, target_component & 0xFF)


def enc_param_request_read(param_id="", param_index=-1, target_system=1, target_component=1):
    return struct.pack("<hBB16s", int(param_index), target_system & 0xFF,
                       target_component & 0xFF, _pid(param_id))


def enc_param_value(param_id, value, param_type=MAV_PARAM_TYPE_REAL32, count=1, index=0):
    return struct.pack("<fHH16sB", float(value), int(count) & 0xFFFF, int(index) & 0xFFFF,
                       _pid(param_id), param_type & 0xFF)


_PARAM_INT_FMT = {1: ("<B", 1), 2: ("<b", 1), 3: ("<H", 2), 4: ("<h", 2),
                  5: ("<I", 4), 6: ("<i", 4), 7: ("<q", 8), 8: ("<q", 8)}


def param_decode(raw_float, ptype):
    """PX4 'bytewise' params: for integer types the param_value field carries the
    raw integer bits packed into the float32 slot -- reinterpret them. ArduPilot
    reports everything as REAL32, so this is a no-op there."""
    if ptype in _PARAM_INT_FMT:
        fmt, size = _PARAM_INT_FMT[ptype]
        return float(struct.unpack(fmt, (struct.pack("<f", raw_float) + b"\x00" * 8)[:size])[0])
    return raw_float


def param_encode(value, ptype):
    """Inverse of param_decode: pack an integer value's bits into the float32
    PARAM_SET slot (PX4). Floats pass straight through."""
    if ptype in _PARAM_INT_FMT:
        fmt, size = _PARAM_INT_FMT[ptype]
        return struct.unpack("<f", (struct.pack(fmt, int(round(value))) + b"\x00" * 8)[:4])[0]
    return float(value)


def enc_param_set(param_id, value, param_type=MAV_PARAM_TYPE_REAL32,
                  target_system=1, target_component=1):
    return struct.pack("<fBB16sB", param_encode(value, param_type), target_system & 0xFF,
                       target_component & 0xFF, _pid(param_id), param_type & 0xFF)


def enc_serial_control(device, flags, data=b"", timeout=0, baudrate=0, count=None,
                       target_system=1, target_component=1):
    # target_system/target_component are MAVLink extension fields -- PX4's shell
    # only answers when it is addressed, so we always include them (they don't
    # affect the CRC_EXTRA seed, which is computed over the base fields only).
    if isinstance(data, str):
        data = data.encode("utf-8", "replace")
    data = bytes(data)
    if count is None:
        count = len(data)
    data = (data + b"\x00" * 70)[:70]
    return struct.pack("<IHBBB70sBB", int(baudrate) & 0xFFFFFFFF, int(timeout) & 0xFFFF,
                       int(device) & 0xFF, int(flags) & 0xFF, int(count) & 0xFF, data,
                       int(target_system) & 0xFF, int(target_component) & 0xFF)


def enc_mission_count(count, target_system=1, target_component=1, mission_type=0):
    return struct.pack("<HBBB", int(count) & 0xFFFF, target_system & 0xFF,
                       target_component & 0xFF, mission_type & 0xFF)


def enc_mission_request_list(target_system=1, target_component=1, mission_type=0):
    return struct.pack("<BBB", target_system & 0xFF, target_component & 0xFF, mission_type & 0xFF)


def enc_mission_request_int(seq, target_system=1, target_component=1, mission_type=0):
    return struct.pack("<HBBB", int(seq) & 0xFFFF, target_system & 0xFF,
                       target_component & 0xFF, mission_type & 0xFF)


def enc_mission_ack(result=0, target_system=1, target_component=1, mission_type=0):
    return struct.pack("<BBBB", target_system & 0xFF, target_component & 0xFF,
                       int(result) & 0xFF, mission_type & 0xFF)


def enc_mission_clear_all(target_system=1, target_component=1, mission_type=0):
    return struct.pack("<BBB", target_system & 0xFF, target_component & 0xFF, mission_type & 0xFF)


def enc_mission_item_int(seq, lat_deg, lon_deg, alt, command=MAV_CMD_NAV_WAYPOINT,
                         frame=MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, current=0, autocontinue=1,
                         param1=0.0, param2=0.0, param3=0.0, param4=0.0,
                         target_system=1, target_component=1, mission_type=0):
    return struct.pack("<ffffiifHHBBBBBB",
                       float(param1), float(param2), float(param3), float(param4),
                       int(lat_deg * 1e7), int(lon_deg * 1e7), float(alt),
                       int(seq) & 0xFFFF, int(command) & 0xFFFF,
                       target_system & 0xFF, target_component & 0xFF,
                       frame & 0xFF, current & 0xFF, autocontinue & 0xFF, mission_type & 0xFF)


def crc16(data: bytes) -> int:
    """CRC-16/MCRF4XX over data only (no extra seed)."""
    crc = 0xFFFF
    for b in data:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def frame_total(buf, i):
    """Total on-wire length of the v1/v2 frame starting at buf[i], or None if the
    buffer doesn't yet hold all of it. Framing only -- no CRC check."""
    n = len(buf)
    b = buf[i]
    if b != 0xFE and b != 0xFD:
        return None
    v2 = (b == 0xFD)
    hdr = 10 if v2 else 6
    if i + 1 >= n:
        return None
    payload = buf[i + 1]
    sig = 0
    if v2:
        if i + 2 >= n:
            return None
        if buf[i + 2] & 0x01:
            sig = 13
    total = hdr + payload + 2 + sig
    return total if i + total <= n else None


def split_frames(buf):
    """Split a byte buffer into complete MAVLink frames. Returns (frames, leftover)
    where leftover is the trailing incomplete frame (inter-frame junk is dropped)."""
    frames = []
    n = len(buf)
    i = 0
    while i < n:
        if buf[i] != 0xFE and buf[i] != 0xFD:
            i += 1
            continue
        total = frame_total(buf, i)
        if total is None:
            return frames, bytes(buf[i:])      # incomplete frame -> keep tail
        frames.append(bytes(buf[i:i + total]))
        i += total
    return frames, b""


# ===========================================================================
#  Pure-Python decode path -- the portable fallback used when the native
#  C++/assembly core is unavailable (non-x86-64, Windows without a rebuild,
#  or a fresh checkout that hasn't run build.sh). Slower, but identical output.
# ===========================================================================
class Message:
    """A decoded message; mirrors core.Message so callers don't care which path
    produced it."""
    __slots__ = ("msgid", "name", "sysid", "compid", "seq", "fields")

    def __init__(self, msgid, sysid, compid, seq, fields):
        self.msgid = msgid
        self.name = MSG_NAME.get(msgid, f"MSG_{msgid}")
        self.sysid = sysid
        self.compid = compid
        self.seq = seq
        self.fields = fields

    def __repr__(self):
        return f"<{self.name} sys{self.sysid} {self.fields}>"


# msgid -> (struct format, field names in wire order, full payload length)
_WIRE = {
    HEARTBEAT: ("<IBBBBB",
                ["custom_mode", "type", "autopilot", "base_mode", "system_status", "mavlink_version"], 9),
    SYS_STATUS: ("<IIIHHhHHHHHHb",
                 ["onboard_present", "onboard_enabled", "onboard_health", "load",
                  "voltage_battery", "current_battery", "drop_rate_comm", "errors_comm",
                  "ec1", "ec2", "ec3", "ec4", "battery_remaining"], 31),
    GPS_RAW_INT: ("<QiiiHHHHBB",
                  ["time_usec", "lat", "lon", "alt", "eph", "epv", "vel", "cog",
                   "fix_type", "satellites_visible"], 30),
    ATTITUDE: ("<Iffffff",
               ["time_boot_ms", "roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed"], 28),
    SERVO_OUTPUT_RAW: ("<I8HB",
                       ["time_usec"] + [f"servo{i}_raw" for i in range(1, 9)] + ["port"], 21),
    GLOBAL_POSITION_INT: ("<IiiiihhhH",
                          ["time_boot_ms", "lat", "lon", "alt", "relative_alt",
                           "vx", "vy", "vz", "hdg"], 28),
    VFR_HUD: ("<ffffhH",
              ["airspeed", "groundspeed", "alt", "climb", "heading", "throttle"], 20),
    RC_CHANNELS: ("<I18HBB",
                  ["time_boot_ms"] + [f"chan{i}_raw" for i in range(1, 19)]
                  + ["chancount", "rssi"], 42),
    RADIO_STATUS: ("<HHBBBBB",
                   ["rxerrors", "fixed", "rssi", "remrssi", "txbuf", "noise", "remnoise"], 9),
    ALTITUDE: ("<Qffffff",
               ["time_usec", "altitude_monotonic", "altitude_amsl", "altitude_local",
                "altitude_relative", "altitude_terrain", "bottom_clearance"], 32),
    VIBRATION: ("<QfffIII",
                ["time_usec", "vibration_x", "vibration_y", "vibration_z",
                 "clipping_0", "clipping_1", "clipping_2"], 32),
    EKF_STATUS_REPORT: ("<5fH",
                        ["velocity_variance", "pos_horiz_variance", "pos_vert_variance",
                         "compass_variance", "terrain_alt_variance", "flags"], 22),
    ESTIMATOR_STATUS: ("<Q8fH",
                       ["time_usec", "vel_ratio", "pos_horiz_ratio", "pos_vert_ratio",
                        "mag_ratio", "hagl_ratio", "tas_ratio", "pos_horiz_accuracy",
                        "pos_vert_accuracy", "flags"], 42),
    WIND_COV: ("<Q8f",
               ["time_usec", "wind_x", "wind_y", "wind_z", "var_horiz", "var_vert",
                "wind_alt", "horiz_accuracy", "vert_accuracy"], 40),
    MOUNT_ORIENTATION: ("<I4f",
                        ["time_boot_ms", "roll", "pitch", "yaw", "yaw_absolute"], 20),
    HOME_POSITION: ("<3i10f",
                    ["latitude", "longitude", "altitude", "x", "y", "z",
                     "q0", "q1", "q2", "q3", "approach_x", "approach_y", "approach_z"], 52),
    ESC_STATUS: ("<Q4i4f4fB",
                 ["time_usec"] + [f"rpm{i}" for i in range(1, 5)]
                 + [f"voltage{i}" for i in range(1, 5)]
                 + [f"current{i}" for i in range(1, 5)] + ["index"], 57),
    EXTENDED_SYS_STATE: ("<2B", ["vtol_state", "landed_state"], 2),
    BATTERY_STATUS: ("<iih10HhBBBb",
                     ["current_consumed", "energy_consumed", "temperature"]
                     + [f"voltage{i}" for i in range(1, 11)]
                     + ["current_battery", "id", "battery_function", "type", "battery_remaining"], 36),
    MANUAL_CONTROL: ("<hhhhHB", ["x", "y", "z", "r", "buttons", "target"], 11),
    COMMAND_INT: ("<ffffiifHBBBBB",
                  ["param1", "param2", "param3", "param4", "x", "y", "z", "command",
                   "target_system", "target_component", "frame", "current", "autocontinue"], 35),
    COMMAND_LONG: ("<fffffffHBBB",
                   ["param1", "param2", "param3", "param4", "param5", "param6", "param7",
                    "command", "target_system", "target_component", "confirmation"], 33),
    COMMAND_ACK: ("<HB", ["command", "result"], 3),
    STATUSTEXT: ("<B50s", ["severity", "text"], 51),
    SET_POSITION_TARGET_GLOBAL_INT: (
        "<Iii" + "f" * 9 + "HBBB",
        ["time_boot_ms", "lat_int", "lon_int", "alt", "vx", "vy", "vz",
         "afx", "afy", "afz", "yaw", "yaw_rate", "type_mask",
         "target_system", "target_component", "coordinate_frame"], 53),
    PARAM_REQUEST_LIST: ("<BB", ["target_system", "target_component"], 2),
    PARAM_REQUEST_READ: ("<hBB16s", ["param_index", "target_system", "target_component",
                                     "param_id"], 20),
    PARAM_VALUE: ("<fHH16sB", ["param_value", "param_count", "param_index", "param_id",
                               "param_type"], 25),
    PARAM_SET: ("<fBB16sB", ["param_value", "target_system", "target_component", "param_id",
                             "param_type"], 23),
    MISSION_CURRENT: ("<H", ["seq"], 2),
    MISSION_REQUEST_LIST: ("<BBB", ["target_system", "target_component", "mission_type"], 3),
    MISSION_COUNT: ("<HBBB", ["count", "target_system", "target_component", "mission_type"], 5),
    MISSION_CLEAR_ALL: ("<BBB", ["target_system", "target_component", "mission_type"], 3),
    MISSION_ITEM_REACHED: ("<H", ["seq"], 2),
    MISSION_ACK: ("<BBBB", ["target_system", "target_component", "type", "mission_type"], 4),
    MISSION_REQUEST_INT: ("<HBBB", ["seq", "target_system", "target_component", "mission_type"], 5),
    MISSION_ITEM_INT: ("<ffffiifHHBBBBBB",
                       ["param1", "param2", "param3", "param4", "x", "y", "z",
                        "seq", "command", "target_system", "target_component",
                        "frame", "current", "autocontinue", "mission_type"], 38),
    LOG_REQUEST_LIST: ("<HHBB", ["start", "end", "target_system", "target_component"], 6),
    LOG_ENTRY: ("<IIHHH", ["time_utc", "size", "id", "num_logs", "last_log_num"], 14),
    LOG_REQUEST_DATA: ("<IIHBB", ["ofs", "count", "id", "target_system", "target_component"], 12),
    LOG_DATA: ("<IHB90s", ["ofs", "id", "count", "data"], 97),
    SERIAL_CONTROL: ("<IHBBB70s", ["baudrate", "timeout", "device", "flags", "count", "data"], 79),
    LOG_REQUEST_END: ("<BB", ["target_system", "target_component"], 2),
    ADSB_VEHICLE: ("<IiiiHHhHHB9sBB",
                   ["ICAO_address", "lat", "lon", "altitude", "heading", "hor_velocity",
                    "ver_velocity", "flags", "squawk", "altitude_type", "callsign",
                    "emitter_type", "tslc"], 38),
}


def mode_number(autopilot, mav_type, name):
    """Reverse of flight_mode_name: mode label -> custom_mode number, or None."""
    for num, nm in mode_table(autopilot, mav_type).items():
        if nm == name:
            return num
    return None


def mode_names(autopilot, mav_type):
    """Ordered list of settable mode labels for this vehicle's autopilot."""
    if int(autopilot) == MAV_AUTOPILOT_PX4:
        return list(_PX4_MODE_SET.keys())
    t = mode_table(autopilot, mav_type)
    return [t[k] for k in sorted(t)]


def mode_command(autopilot, mav_type, name):
    """DO_SET_MODE (param2, param3) for a mode label, or None if unavailable.
    PX4 takes (main_mode, sub_mode); ArduPilot takes (custom_mode, 0)."""
    if int(autopilot) == MAV_AUTOPILOT_PX4:
        ms = _PX4_MODE_SET.get(name)
        return (float(ms[0]), float(ms[1])) if ms else None
    num = mode_number(autopilot, mav_type, name)
    return (float(num), 0.0) if num is not None else None


class PyParser:
    """Streaming MAVLink v1/v2 parser in pure Python. Same CRC-gated resync as
    the C++ core (core/dronecore.cpp)."""

    def __init__(self):
        self.buf = bytearray()
        self.ok = 0
        self.drop = 0

    def feed(self, data: bytes):
        self.buf += data
        out = []
        n = len(self.buf)
        pos = 0
        first_incomplete = -1
        while pos < n:
            b = self.buf[pos]
            if b != 0xFE and b != 0xFD:
                pos += 1
                continue
            v2 = (b == 0xFD)
            hdr = 10 if v2 else 6
            if pos + 1 >= n:
                first_incomplete = pos if first_incomplete < 0 else first_incomplete
                pos += 1
                continue
            payload = self.buf[pos + 1]
            sig = 0
            if v2:
                if pos + 2 >= n:
                    first_incomplete = pos if first_incomplete < 0 else first_incomplete
                    pos += 1
                    continue
                if self.buf[pos + 2] & 0x01:
                    sig = 13
            total = hdr + payload + 2 + sig
            if pos + total > n:
                first_incomplete = pos if first_incomplete < 0 else first_incomplete
                pos += 1
                continue
            if v2:
                msgid = self.buf[pos + 7] | (self.buf[pos + 8] << 8) | (self.buf[pos + 9] << 16)
            else:
                msgid = self.buf[pos + 5]
            spec = _WIRE.get(msgid)
            extra = CRC_EXTRA.get(msgid)
            if spec is None or extra is None:
                pos += 1
                continue
            crc = crc16_mcrf4xx(bytes(self.buf[pos + 1: pos + hdr + payload]), extra)
            off = pos + hdr + payload
            if crc != (self.buf[off] | (self.buf[off + 1] << 8)):
                self.drop += 1
                pos += 1
                continue
            if v2:
                seq, sysid, compid = self.buf[pos + 4], self.buf[pos + 5], self.buf[pos + 6]
            else:
                seq, sysid, compid = self.buf[pos + 2], self.buf[pos + 3], self.buf[pos + 4]
            fmt, names, full = spec
            pl = bytes(self.buf[pos + hdr: pos + hdr + payload])
            pl = pl[:full] if len(pl) >= full else pl + b"\x00" * (full - len(pl))
            fields = dict(zip(names, struct.unpack(fmt, pl)))
            for sk in ("text", "param_id", "callsign"):   # char[] fields -> str
                if isinstance(fields.get(sk), bytes):
                    fields[sk] = fields[sk].split(b"\x00")[0].decode("utf-8", "replace")
            out.append(Message(msgid, sysid, compid, seq, fields))
            self.ok += 1
            pos += total
            first_incomplete = -1
        keep = first_incomplete if first_incomplete >= 0 else pos
        if keep > 0:
            del self.buf[:keep]
        return out
