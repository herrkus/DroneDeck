#!/usr/bin/env python3
"""crc_extra_calc.py -- derive MAVLink CRC_EXTRA seeds from message definitions.

CRC_EXTRA is the X25 CRC over "NAME " + for each non-extension field (in wire
order) "type name " (+ a length byte for arrays), folded to one byte. Getting it
wrong makes frames silently fail CRC, so rather than trust a remembered table we
compute it here and VALIDATE the calculator against the 7 messages already known
to work in our core. Any new message's seed is then trustworthy.

Field lists below are in MAVLink wire order (sorted by type size desc, stable),
extension fields excluded -- exactly what the CRC uses.
"""


def x25(data, crc=0xFFFF):
    for b in data:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def crc_extra(name, fields):
    crc = x25((name + " ").encode())
    for (typ, fname, arrlen) in fields:
        crc = x25((typ + " ").encode(), crc)
        crc = x25((fname + " ").encode(), crc)
        if arrlen:
            crc = x25(bytes([arrlen]), crc)
    return (crc & 0xFF) ^ (crc >> 8)


# (type, name, array_len) in wire order, extensions excluded.
KNOWN = {
    "HEARTBEAT": (50, [("uint32_t", "custom_mode", 0), ("uint8_t", "type", 0),
                       ("uint8_t", "autopilot", 0), ("uint8_t", "base_mode", 0),
                       ("uint8_t", "system_status", 0), ("uint8_t", "mavlink_version", 0)]),
    "SYS_STATUS": (124, [("uint32_t", "onboard_control_sensors_present", 0),
                         ("uint32_t", "onboard_control_sensors_enabled", 0),
                         ("uint32_t", "onboard_control_sensors_health", 0),
                         ("uint16_t", "load", 0), ("uint16_t", "voltage_battery", 0),
                         ("int16_t", "current_battery", 0), ("uint16_t", "drop_rate_comm", 0),
                         ("uint16_t", "errors_comm", 0), ("uint16_t", "errors_count1", 0),
                         ("uint16_t", "errors_count2", 0), ("uint16_t", "errors_count3", 0),
                         ("uint16_t", "errors_count4", 0), ("int8_t", "battery_remaining", 0)]),
    "GPS_RAW_INT": (24, [("uint64_t", "time_usec", 0), ("int32_t", "lat", 0),
                         ("int32_t", "lon", 0), ("int32_t", "alt", 0), ("uint16_t", "eph", 0),
                         ("uint16_t", "epv", 0), ("uint16_t", "vel", 0), ("uint16_t", "cog", 0),
                         ("uint8_t", "fix_type", 0), ("uint8_t", "satellites_visible", 0)]),
    "ATTITUDE": (39, [("uint32_t", "time_boot_ms", 0), ("float", "roll", 0),
                      ("float", "pitch", 0), ("float", "yaw", 0), ("float", "rollspeed", 0),
                      ("float", "pitchspeed", 0), ("float", "yawspeed", 0)]),
    "GLOBAL_POSITION_INT": (104, [("uint32_t", "time_boot_ms", 0), ("int32_t", "lat", 0),
                                  ("int32_t", "lon", 0), ("int32_t", "alt", 0),
                                  ("int32_t", "relative_alt", 0), ("int16_t", "vx", 0),
                                  ("int16_t", "vy", 0), ("int16_t", "vz", 0), ("uint16_t", "hdg", 0)]),
    "VFR_HUD": (20, [("float", "airspeed", 0), ("float", "groundspeed", 0), ("float", "alt", 0),
                     ("float", "climb", 0), ("int16_t", "heading", 0), ("uint16_t", "throttle", 0)]),
    "COMMAND_LONG": (152, [("float", "param1", 0), ("float", "param2", 0), ("float", "param3", 0),
                           ("float", "param4", 0), ("float", "param5", 0), ("float", "param6", 0),
                           ("float", "param7", 0), ("uint16_t", "command", 0),
                           ("uint8_t", "target_system", 0), ("uint8_t", "target_component", 0),
                           ("uint8_t", "confirmation", 0)]),
}

NEW = {
    "STATUSTEXT": (253, [("uint8_t", "severity", 0), ("char", "text", 50)]),
    "PARAM_REQUEST_READ": (20, [("int16_t", "param_index", 0), ("uint8_t", "target_system", 0),
                                ("uint8_t", "target_component", 0), ("char", "param_id", 16)]),
    "PARAM_REQUEST_LIST": (21, [("uint8_t", "target_system", 0), ("uint8_t", "target_component", 0)]),
    "PARAM_VALUE": (22, [("float", "param_value", 0), ("uint16_t", "param_count", 0),
                         ("uint16_t", "param_index", 0), ("char", "param_id", 16),
                         ("uint8_t", "param_type", 0)]),
    "PARAM_SET": (23, [("float", "param_value", 0), ("uint8_t", "target_system", 0),
                       ("uint8_t", "target_component", 0), ("char", "param_id", 16),
                       ("uint8_t", "param_type", 0)]),
    "COMMAND_ACK": (77, [("uint16_t", "command", 0), ("uint8_t", "result", 0)]),
    "SET_MODE": (11, [("uint32_t", "custom_mode", 0), ("uint8_t", "target_system", 0),
                      ("uint8_t", "base_mode", 0)]),
    "MISSION_REQUEST_LIST": (43, [("uint8_t", "target_system", 0), ("uint8_t", "target_component", 0)]),
    "MISSION_COUNT": (44, [("uint16_t", "count", 0), ("uint8_t", "target_system", 0),
                           ("uint8_t", "target_component", 0)]),
    "MISSION_REQUEST_INT": (51, [("uint16_t", "seq", 0), ("uint8_t", "target_system", 0),
                                 ("uint8_t", "target_component", 0)]),
    "MISSION_ITEM_INT": (73, [("float", "param1", 0), ("float", "param2", 0), ("float", "param3", 0),
                              ("float", "param4", 0), ("int32_t", "x", 0), ("int32_t", "y", 0),
                              ("float", "z", 0), ("uint16_t", "seq", 0), ("uint16_t", "command", 0),
                              ("uint8_t", "target_system", 0), ("uint8_t", "target_component", 0),
                              ("uint8_t", "frame", 0), ("uint8_t", "current", 0),
                              ("uint8_t", "autocontinue", 0)]),
    "MISSION_ACK": (47, [("uint8_t", "target_system", 0), ("uint8_t", "target_component", 0),
                         ("uint8_t", "type", 0)]),
    "MISSION_CURRENT": (42, [("uint16_t", "seq", 0)]),
    "MISSION_ITEM_REACHED": (46, [("uint16_t", "seq", 0)]),
    "MISSION_SET_CURRENT": (41, [("uint16_t", "seq", 0), ("uint8_t", "target_system", 0),
                                 ("uint8_t", "target_component", 0)]),
    "MISSION_CLEAR_ALL": (45, [("uint8_t", "target_system", 0), ("uint8_t", "target_component", 0)]),
    "MANUAL_CONTROL": (69, [("int16_t", "x", 0), ("int16_t", "y", 0), ("int16_t", "z", 0),
                            ("int16_t", "r", 0), ("uint16_t", "buttons", 0),
                            ("uint8_t", "target", 0)]),
    "LOG_REQUEST_LIST": (117, [("uint16_t", "start", 0), ("uint16_t", "end", 0),
                               ("uint8_t", "target_system", 0), ("uint8_t", "target_component", 0)]),
    "LOG_ENTRY": (118, [("uint32_t", "time_utc", 0), ("uint32_t", "size", 0),
                        ("uint16_t", "id", 0), ("uint16_t", "num_logs", 0),
                        ("uint16_t", "last_log_num", 0)]),
    "LOG_REQUEST_DATA": (119, [("uint32_t", "ofs", 0), ("uint32_t", "count", 0),
                               ("uint16_t", "id", 0), ("uint8_t", "target_system", 0),
                               ("uint8_t", "target_component", 0)]),
    "LOG_DATA": (120, [("uint32_t", "ofs", 0), ("uint16_t", "id", 0),
                       ("uint8_t", "count", 0), ("uint8_t", "data", 90)]),
    "LOG_REQUEST_END": (122, [("uint8_t", "target_system", 0), ("uint8_t", "target_component", 0)]),
    "RC_CHANNELS": (118, [("uint32_t", "time_boot_ms", 0)]
                    + [("uint16_t", f"chan{i}_raw", 0) for i in range(1, 19)]
                    + [("uint8_t", "chancount", 0), ("uint8_t", "rssi", 0)]),
    "RADIO_STATUS": (185, [("uint16_t", "rxerrors", 0), ("uint16_t", "fixed", 0),
                           ("uint8_t", "rssi", 0), ("uint8_t", "remrssi", 0),
                           ("uint8_t", "txbuf", 0), ("uint8_t", "noise", 0),
                           ("uint8_t", "remnoise", 0)]),
    "REQUEST_DATA_STREAM": (148, [("uint16_t", "req_message_rate", 0), ("uint8_t", "target_system", 0),
                                  ("uint8_t", "target_component", 0), ("uint8_t", "req_stream_id", 0),
                                  ("uint8_t", "start_stop", 0)]),
    "COMMAND_INT": (158, [("float", "param1", 0), ("float", "param2", 0), ("float", "param3", 0),
                          ("float", "param4", 0), ("int32_t", "x", 0), ("int32_t", "y", 0),
                          ("float", "z", 0), ("uint16_t", "command", 0),
                          ("uint8_t", "target_system", 0), ("uint8_t", "target_component", 0),
                          ("uint8_t", "frame", 0), ("uint8_t", "current", 0),
                          ("uint8_t", "autocontinue", 0)]),
    "ALTITUDE": (47, [("uint64_t", "time_usec", 0), ("float", "altitude_monotonic", 0),
                      ("float", "altitude_amsl", 0), ("float", "altitude_local", 0),
                      ("float", "altitude_relative", 0), ("float", "altitude_terrain", 0),
                      ("float", "bottom_clearance", 0)]),
    "VIBRATION": (90, [("uint64_t", "time_usec", 0), ("float", "vibration_x", 0),
                       ("float", "vibration_y", 0), ("float", "vibration_z", 0),
                       ("uint32_t", "clipping_0", 0), ("uint32_t", "clipping_1", 0),
                       ("uint32_t", "clipping_2", 0)]),
    "BATTERY_STATUS": (154, [("int32_t", "current_consumed", 0), ("int32_t", "energy_consumed", 0),
                             ("int16_t", "temperature", 0), ("uint16_t", "voltages", 10),
                             ("int16_t", "current_battery", 0), ("uint8_t", "id", 0),
                             ("uint8_t", "battery_function", 0), ("uint8_t", "type", 0),
                             ("int8_t", "battery_remaining", 0)]),
    "ADSB_VEHICLE": (246, [("uint32_t", "ICAO_address", 0), ("int32_t", "lat", 0),
                           ("int32_t", "lon", 0), ("int32_t", "altitude", 0),
                           ("uint16_t", "heading", 0), ("uint16_t", "hor_velocity", 0),
                           ("int16_t", "ver_velocity", 0), ("uint16_t", "flags", 0),
                           ("uint16_t", "squawk", 0), ("uint8_t", "altitude_type", 0),
                           ("char", "callsign", 9), ("uint8_t", "emitter_type", 0),
                           ("uint8_t", "tslc", 0)]),
    "SET_POSITION_TARGET_GLOBAL_INT": (86, [
        ("uint32_t", "time_boot_ms", 0), ("int32_t", "lat_int", 0), ("int32_t", "lon_int", 0),
        ("float", "alt", 0), ("float", "vx", 0), ("float", "vy", 0), ("float", "vz", 0),
        ("float", "afx", 0), ("float", "afy", 0), ("float", "afz", 0), ("float", "yaw", 0),
        ("float", "yaw_rate", 0), ("uint16_t", "type_mask", 0), ("uint8_t", "target_system", 0),
        ("uint8_t", "target_component", 0), ("uint8_t", "coordinate_frame", 0)]),
    # yaw_absolute is a MAVLink extension field -> excluded from the CRC (still sent on wire).
    # Computes 26 with it excluded, 77 if wrongly included -- a classic silent-CRC pitfall.
    "MOUNT_ORIENTATION": (265, [("uint32_t", "time_boot_ms", 0), ("float", "roll", 0),
                                ("float", "pitch", 0), ("float", "yaw", 0)]),
    # servo9..16_raw are extensions -> excluded from the CRC. Wire order sorts port (uint8)
    # last after the uint32 + eight uint16 servo fields. Computes 222.
    "SERVO_OUTPUT_RAW": (36, [("uint32_t", "time_usec", 0)]
                         + [("uint16_t", f"servo{i}_raw", 0) for i in range(1, 9)]
                         + [("uint8_t", "port", 0)]),
    # time_usec is an extension -> excluded. q is float[4] (array length byte in the CRC).
    # 3 int32 + 10 float base = 52 bytes. Computes 104.
    "HOME_POSITION": (242, [("int32_t", "latitude", 0), ("int32_t", "longitude", 0),
                            ("int32_t", "altitude", 0), ("float", "x", 0), ("float", "y", 0),
                            ("float", "z", 0), ("float", "q", 4), ("float", "approach_x", 0),
                            ("float", "approach_y", 0), ("float", "approach_z", 0)]),
}

if __name__ == "__main__":
    print("=== validate calculator against known-good CRC_EXTRA ===")
    ok = True
    for name, (expect, fields) in KNOWN.items():
        got = crc_extra(name, fields)
        flag = "OK" if got == expect else "MISMATCH"
        if got != expect:
            ok = False
        print(f"  {name:22} computed={got:3}  expected={expect:3}  {flag}")
    print("\nCALCULATOR VALIDATED" if ok else "\nCALCULATOR BROKEN -- field defs wrong")
    print("\n=== derived CRC_EXTRA for new messages (msgid: extra) ===")
    for name, (mid, fields) in NEW.items():
        print(f"  {mid:3}: {crc_extra(name, fields):3},   # {name}")
