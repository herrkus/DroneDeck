#!/usr/bin/env python3
"""test_parity.py -- cross-language contract test.

Encodes every supported message in pure Python (mavlink.py), parses it back
through the native C++/asm core (core.py), and checks the decoded fields match.
Also checks the assembly CRC agrees with the independent Python CRC. This is the
guard against the C++ decoder and the Python layouts drifting apart.
"""
import os
import sys
import math

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "app"))

import mavlink
import core

fails = 0


def check(cond, msg):
    global fails
    if not cond:
        print(f"  FAIL: {msg}")
        fails += 1


def approx(a, b, tol=1e-3):
    return abs(a - b) <= tol * max(1.0, abs(b))


def roundtrip(msgid, payload, crc_fn):
    frame = mavlink.frame(msgid, payload, seq=7, sysid=3, compid=1, crc_fn=crc_fn)
    p = core.Parser()
    msgs = p.feed(frame)
    check(len(msgs) == 1, f"{mavlink.MSG_NAME[msgid]}: expected 1 msg, got {len(msgs)}")
    return msgs[0] if msgs else None


def main():
    print("DroneDeck cross-language parity test")

    # The simulator uses the assembly CRC; the test also exercises the pure one.
    for crc_name, crc_fn in (("asm", core.crc_extra), ("py", mavlink.crc16_mcrf4xx)):
        print(f"[crc backend: {crc_name}]")

        m = roundtrip(mavlink.HEARTBEAT,
                      mavlink.enc_heartbeat(mav_type=2, base_mode=0x81, custom_mode=5), crc_fn)
        if m:
            check(m.name == "HEARTBEAT", "heartbeat name")
            check(m.fields["type"] == 2, f"hb type {m.fields['type']}")
            check(m.fields["base_mode"] == 0x81, f"hb base_mode {m.fields['base_mode']}")
            check(m.fields["custom_mode"] == 5, f"hb custom_mode {m.fields['custom_mode']}")

        m = roundtrip(mavlink.ATTITUDE,
                      mavlink.enc_attitude(0.1, -0.2, 1.5, 12345), crc_fn)
        if m:
            check(approx(m.fields["roll"], 0.1), f"att roll {m.fields['roll']}")
            check(approx(m.fields["pitch"], -0.2), f"att pitch {m.fields['pitch']}")
            check(approx(m.fields["yaw"], 1.5), f"att yaw {m.fields['yaw']}")
            check(m.fields["time_boot_ms"] == 12345, "att time")

        m = roundtrip(mavlink.GLOBAL_POSITION_INT,
                      mavlink.enc_global_position_int(
                          int(54.6872e7), int(25.2797e7), 120000, 50000, 27000, 999), crc_fn)
        if m:
            check(m.fields["lat"] == int(54.6872e7), f"gpi lat {m.fields['lat']}")
            check(m.fields["lon"] == int(25.2797e7), f"gpi lon {m.fields['lon']}")
            check(m.fields["alt"] == 120000, "gpi alt")
            check(m.fields["relative_alt"] == 50000, "gpi rel_alt")
            check(m.fields["hdg"] == 27000, f"gpi hdg {m.fields['hdg']}")

        sens = (1 << 0) | (1 << 1) | (1 << 5) | (1 << 25)        # gyro|accel|gps|battery
        m = roundtrip(mavlink.SYS_STATUS,
                      mavlink.enc_sys_status(12600, 1500, 87, present=sens,
                                             enabled=sens, health=sens ^ (1 << 5)), crc_fn)
        if m:
            check(m.fields["voltage_battery"] == 12600, "sys voltage")
            check(m.fields["current_battery"] == 1500, "sys current")
            check(m.fields["battery_remaining"] == 87, "sys remaining")
            check(int(m.fields["onboard_present"]) == sens, "sys sensors present")
            check(int(m.fields["onboard_health"]) == sens ^ (1 << 5), "sys sensors health")

        m = roundtrip(mavlink.GPS_RAW_INT,
                      mavlink.enc_gps_raw_int(int(54.6872e7), int(25.2797e7), 121000,
                                              fix=3, sats=14), crc_fn)
        if m:
            check(m.fields["fix_type"] == 3, "gps fix")
            check(m.fields["satellites_visible"] == 14, "gps sats")
            check(m.fields["lat"] == int(54.6872e7), "gps lat")

        m = roundtrip(mavlink.VFR_HUD,
                      mavlink.enc_vfr_hud(12.5, 13.0, 120.0, 1.2, 270, 65), crc_fn)
        if m:
            check(approx(m.fields["airspeed"], 12.5), "vfr airspeed")
            check(approx(m.fields["groundspeed"], 13.0), "vfr groundspeed")
            check(m.fields["heading"] == 270, "vfr heading")
            check(m.fields["throttle"] == 65, "vfr throttle")

        m = roundtrip(mavlink.RC_CHANNELS,
                      mavlink.enc_rc_channels([1100, 1500, 1900, 1000], rssi=180), crc_fn)
        if m:
            check(int(m.fields["chan1_raw"]) == 1100, "rc chan1")
            check(int(m.fields["chan3_raw"]) == 1900, "rc chan3")
            check(int(m.fields["chancount"]) == 4, "rc chancount")
            check(int(m.fields["chan5_raw"]) == 65535, "rc unused chan")
            check(int(m.fields["rssi"]) == 180, "rc rssi")

        m = roundtrip(mavlink.RADIO_STATUS,
                      mavlink.enc_radio_status(190, 185, noise=42, rxerrors=3), crc_fn)
        if m:
            check(int(m.fields["rssi"]) == 190, "radio rssi")
            check(int(m.fields["remrssi"]) == 185, "radio remrssi")
            check(int(m.fields["noise"]) == 42, "radio noise")
            check(int(m.fields["rxerrors"]) == 3, "radio rxerrors")

        m = roundtrip(mavlink.ALTITUDE,
                      mavlink.enc_altitude(105.0, 120.5, -5.0, 50.0, 12.0, 3.0), crc_fn)
        if m:
            check(approx(m.fields["altitude_amsl"], 120.5), "altitude amsl")
            check(approx(m.fields["altitude_relative"], 50.0), "altitude rel")
            check(approx(m.fields["bottom_clearance"], 3.0), "altitude bottom")

        m = roundtrip(mavlink.VIBRATION,
                      mavlink.enc_vibration(0.5, 0.7, 1.2, 3, 4, 5), crc_fn)
        if m:
            check(approx(m.fields["vibration_x"], 0.5), "vibration x")
            check(approx(m.fields["vibration_z"], 1.2), "vibration z")
            check(int(m.fields["clipping_2"]) == 5, "vibration clip2")

        m = roundtrip(mavlink.BATTERY_STATUS,
                      mavlink.enc_battery_status([16800, 16750], current_battery=1500,
                                                 current_consumed=2500, battery_remaining=88,
                                                 temperature=2600), crc_fn)
        if m:
            check(int(m.fields["voltage1"]) == 16800, "battery cell1")
            check(int(m.fields["voltage2"]) == 16750, "battery cell2")
            check(int(m.fields["voltage3"]) == 65535, "battery cell3 unused")
            check(int(m.fields["current_battery"]) == 1500, "battery current")
            check(int(m.fields["current_consumed"]) == 2500, "battery consumed")
            check(int(m.fields["battery_remaining"]) == 88, "battery remaining")
            check(int(m.fields["temperature"]) == 2600, "battery temperature")

        m = roundtrip(mavlink.COMMAND_INT,
                      mavlink.enc_command_int(mavlink.MAV_CMD_DO_ORBIT,
                                              [50.0, float("nan"), 0, 0],
                                              int(54.6872e7), int(25.2797e7), 30.0, frame=6), crc_fn)
        if m:
            check(int(m.fields["command"]) == mavlink.MAV_CMD_DO_ORBIT, "cmdint command")
            check(int(m.fields["x"]) == int(54.6872e7), f"cmdint x {m.fields.get('x')}")
            check(int(m.fields["y"]) == int(25.2797e7), f"cmdint y {m.fields.get('y')}")
            check(approx(m.fields["z"], 30.0), "cmdint z")
            check(int(m.fields["frame"]) == 6, "cmdint frame")
            check(approx(m.fields["param1"], 50.0), "cmdint param1")

        m = roundtrip(mavlink.MANUAL_CONTROL,
                      mavlink.enc_manual_control(1, 500, -250, 800, -100, 3), crc_fn)
        if m:
            check(int(m.fields["x"]) == 500, f"manual x {m.fields['x']}")
            check(int(m.fields["y"]) == -250, f"manual y {m.fields['y']}")
            check(int(m.fields["z"]) == 800, "manual z")
            check(int(m.fields["r"]) == -100, "manual r")
            check(int(m.fields["buttons"]) == 3 and int(m.fields["target"]) == 1, "manual btn/target")

        m = roundtrip(mavlink.LOG_ENTRY,
                      mavlink.enc_log_entry(2, 3, 2, 4096, time_utc=1700000000), crc_fn)
        if m:
            check(int(m.fields["id"]) == 2, "log_entry id")
            check(int(m.fields["num_logs"]) == 3, "log_entry num_logs")
            check(int(m.fields["size"]) == 4096, "log_entry size")

        blob = bytes((i * 7) & 0xFF for i in range(90))
        m = roundtrip(mavlink.LOG_DATA, mavlink.enc_log_data(2, 180, blob), crc_fn)
        if m:
            check(int(m.fields["ofs"]) == 180, "log_data ofs")
            check(int(m.fields["count"]) == 90, "log_data count")
            check(bytes(m.fields["data"])[:90] == blob, "log_data blob round-trip")

        m = roundtrip(mavlink.ADSB_VEHICLE,
                      mavlink.enc_adsb_vehicle(0xABCDEF, int(54.70e7), int(25.30e7),
                                               120000, 9000, "TEST123", emitter_type=3), crc_fn)
        if m:
            check(int(m.fields["ICAO_address"]) == 0xABCDEF, "adsb icao")
            check(int(m.fields["lat"]) == int(54.70e7), "adsb lat")
            check(int(m.fields["heading"]) == 9000, "adsb heading")
            check(m.fields.get("callsign") == "TEST123", f"adsb callsign {m.fields.get('callsign')!r}")

        m = roundtrip(mavlink.COMMAND_ACK, mavlink.enc_command_ack(400, 0), crc_fn)
        if m:
            check(m.fields["command"] == 400, f"ack command {m.fields.get('command')}")
            check(m.fields["result"] == 0, f"ack result {m.fields.get('result')}")

        m = roundtrip(mavlink.STATUSTEXT, mavlink.enc_statustext(6, "DroneDeck online"), crc_fn)
        if m:
            check(m.fields["severity"] == 6, f"statustext severity {m.fields.get('severity')}")
            check(m.fields.get("text") == "DroneDeck online",
                  f"statustext text {m.fields.get('text')!r}")

        m = roundtrip(mavlink.SET_POSITION_TARGET_GLOBAL_INT,
                      mavlink.enc_set_position_target_global_int(54.6872, 25.2797, 50.0), crc_fn)
        if m:  # encoder uses int(deg*1e7); decode must reproduce it bit-for-bit
            check(m.fields["lat_int"] == int(54.6872 * 1e7), f"spt lat {m.fields.get('lat_int')}")
            check(m.fields["lon_int"] == int(25.2797 * 1e7), f"spt lon {m.fields.get('lon_int')}")
            check(approx(m.fields["alt"], 50.0), f"spt alt {m.fields.get('alt')}")

        # --- mission protocol messages ---
        m = roundtrip(mavlink.MISSION_COUNT, mavlink.enc_mission_count(7), crc_fn)
        if m:
            check(m.fields["count"] == 7, f"mission_count {m.fields.get('count')}")

        m = roundtrip(mavlink.MISSION_REQUEST_INT, mavlink.enc_mission_request_int(3), crc_fn)
        if m:
            check(m.fields["seq"] == 3, f"mission_request seq {m.fields.get('seq')}")

        m = roundtrip(mavlink.MISSION_ACK, mavlink.enc_mission_ack(0), crc_fn)
        if m:
            check(m.fields["type"] == 0, f"mission_ack type {m.fields.get('type')}")

        m = roundtrip(mavlink.MISSION_ITEM_INT,
                      mavlink.enc_mission_item_int(2, 54.6872, 25.2797, 50.0,
                                                   command=16, current=1, param1=5.0), crc_fn)
        if m:
            check(m.fields["seq"] == 2, f"item seq {m.fields.get('seq')}")
            check(m.fields["command"] == 16, f"item command {m.fields.get('command')}")
            check(m.fields["current"] == 1, f"item current {m.fields.get('current')}")
            check(m.fields["x"] == int(54.6872 * 1e7), f"item x {m.fields.get('x')}")
            check(m.fields["y"] == int(25.2797 * 1e7), f"item y {m.fields.get('y')}")
            check(approx(m.fields["z"], 50.0), f"item z {m.fields.get('z')}")
            check(approx(m.fields["param1"], 5.0), f"item param1 {m.fields.get('param1')}")

        # --- parameter protocol messages ---
        m = roundtrip(mavlink.PARAM_VALUE,
                      mavlink.enc_param_value("WPNAV_SPEED", 500.0, count=12, index=3), crc_fn)
        if m:
            check(m.fields.get("param_id") == "WPNAV_SPEED", f"param id {m.fields.get('param_id')!r}")
            check(approx(m.fields["param_value"], 500.0), f"param value {m.fields.get('param_value')}")
            check(m.fields["param_count"] == 12, f"param count {m.fields.get('param_count')}")
            check(m.fields["param_index"] == 3, f"param index {m.fields.get('param_index')}")

        m = roundtrip(mavlink.PARAM_SET, mavlink.enc_param_set("RTL_ALT", 1500.0), crc_fn)
        if m:
            check(m.fields.get("param_id") == "RTL_ALT", f"set id {m.fields.get('param_id')!r}")
            check(approx(m.fields["param_value"], 1500.0), f"set value {m.fields.get('param_value')}")

    # The native COMMAND_INT encoder must round-trip through the parser.
    raw = core.encode_command_int(3, 1, 9, 1, 1, 6, mavlink.MAV_CMD_DO_SET_HOME,
                                  [0, 0, 0, 0], int(54.70e7), int(25.30e7), 20.0)
    mm = core.Parser().feed(raw)
    check(len(mm) == 1, f"native command_int: {len(mm)} msgs")
    if mm:
        check(int(mm[0].fields["command"]) == mavlink.MAV_CMD_DO_SET_HOME, "native command_int cmd")
        check(int(mm[0].fields["x"]) == int(54.70e7), f"native command_int x {mm[0].fields.get('x')}")
        check(approx(mm[0].fields["z"], 20.0), "native command_int z")

    # Assembly CRC must equal the independent Python CRC, byte-for-byte.
    sample = mavlink.enc_attitude(0.3, 0.4, 0.5, 7)
    head = bytes((len(sample), 0, 3, 1, mavlink.ATTITUDE))
    asm = core.crc_extra(head + sample, mavlink.CRC_EXTRA[mavlink.ATTITUDE])
    py = mavlink.crc16_mcrf4xx(head + sample, mavlink.CRC_EXTRA[mavlink.ATTITUDE])
    check(asm == py, f"crc asm {asm:#06x} vs py {py:#06x}")
    print(f"[crc] asm == py : {asm:#06x}")

    print("\nPARITY FAILED" if fails else "\nPARITY PASSED")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
