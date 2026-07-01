#!/usr/bin/env python3
"""simulator.py -- TEST telemetry source for DroneDeck (NOT a drone model).

A standalone signal generator + tiny flight model so the ground station can be
exercised without hardware. It streams MAVLink telemetry over UDP and responds
to GCS commands:
  - ARM / DISARM
  - DO_SET_MODE (mode shown in the GCS updates)
  - NAV_TAKEOFF / NAV_LAND / RETURN_TO_LAUNCH / DO_PAUSE_CONTINUE
  - SET_POSITION_TARGET_GLOBAL_INT (click-on-map "Goto": it flies there)
Default behaviour is a lazy LOITER circle with a draining battery. When a real
drone is connected this is simply not used.

Frames use the project's assembly CRC when the native core is present.

Usage:  python3 sim/simulator.py [--target 127.0.0.1:14550] [--sysid 1]
"""
from __future__ import annotations
import argparse
import math
import os
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "app"))

import mavlink

try:
    import core
    CRC_FN = core.crc_extra
    PARSER = core.Parser()
    CRC_BACKEND = "assembly core"
except Exception as e:                       # pragma: no cover
    CRC_FN = None
    PARSER = None
    CRC_BACKEND = f"pure-Python ({e.__class__.__name__})"

HOME_LAT, HOME_LON = 54.6872, 25.2797
RADIUS_M = 140.0
SPEED = 12.0
CRUISE_ALT = 80.0
M_PER_DEG = 111320.0

LOITER, AUTO, GUIDED, RTL, LAND = 5, 3, 4, 6, 9
ALT_HOLD = 2          # manual stick control with altitude hold (MANUAL_CONTROL)

# fake dataflash logs served over the LOG_* protocol: (id, size_bytes, time_utc)
SIM_LOGS = [(1, 4096, 1719000000), (2, 12345, 1719600000), (3, 800, 1719700000)]


def _logbyte(log_id, k):
    """Deterministic fake log content byte at absolute offset k."""
    return (k * 31 + log_id * 7) & 0xFF


def step_toward(clat, clon, tlat, tlon, max_m, cos_lat):
    """Move (clat,clon) toward (tlat,tlon) by up to max_m metres.
    Returns (lat, lon, remaining_m, heading_deg_or_None)."""
    dn = (tlat - clat) * M_PER_DEG
    de = (tlon - clon) * M_PER_DEG * cos_lat
    dist = math.hypot(dn, de)
    if dist <= 1e-9:
        return clat, clon, 0.0, None
    step = min(max_m, dist)
    f = step / dist
    return (clat + dn * f / M_PER_DEG,
            clon + de * f / (M_PER_DEG * cos_lat),
            dist - step,
            math.degrees(math.atan2(de, dn)) % 360.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="127.0.0.1:14550")
    ap.add_argument("--sysid", type=int, default=1)
    args = ap.parse_args()
    host, _, port = args.target.partition(":")
    target = (host, int(port or 14550))
    sysid = args.sysid

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", 0))
    sock.setblocking(False)
    cos_lat = math.cos(math.radians(HOME_LAT))

    print(f"DroneDeck TEST telemetry source -> {target[0]}:{target[1]} "
          f"(sysid {sysid}, CRC: {CRC_BACKEND})")
    print("LOITER circle by default; responds to GCS commands. Ctrl-C to stop. "
          "This is a test source, not a drone model.")

    seq = 0

    def send(msgid, payload):
        nonlocal seq
        try:
            sock.sendto(mavlink.frame(msgid, payload, seq, sysid, 1, crc_fn=CRC_FN), target)
        except OSError:
            pass
        seq = (seq + 1) & 0xFF

    # --- flight state --------------------------------------------------------
    lat, lon, alt = HOME_LAT, HOME_LON, CRUISE_ALT
    heading = 90.0
    mode = LOITER
    armed = True
    guided_target = None
    theta = 0.0
    # centre so that at theta=0 the vehicle sits exactly at (lat,lon): no jump.
    loiter_center = [lat - RADIUS_M / M_PER_DEG, lon]
    missions = {0: [], 1: [], 2: []}   # 0=mission (flown in AUTO), 1=fence, 2=rally
    auto_idx = 0          # current waypoint index in AUTO
    up_expected = 0       # >0 while receiving an upload
    up_items = []
    up_next = 0
    up_mtype = 0
    sim_params = {        # a small ArduCopter-like parameter set to browse/edit
        "SYSID_THISMAV": 1.0, "WPNAV_SPEED": 500.0, "WPNAV_RADIUS": 200.0,
        "RTL_ALT": 1500.0, "FENCE_ENABLE": 0.0, "FENCE_ALT_MAX": 100.0,
        "BATT_LOW_VOLT": 10.5, "ANGLE_MAX": 4500.0, "PILOT_SPEED_UP": 250.0,
        "ARMING_CHECK": 1.0, "GPS_TYPE": 1.0, "FS_THR_ENABLE": 1.0,
        "LOITER_SPEED": 1250.0, "LOG_BITMASK": 176126.0,
    }
    param_order = list(sim_params.keys())

    def enter_loiter():
        nonlocal theta, loiter_center
        theta = 0.0
        loiter_center = [lat - RADIUS_M / M_PER_DEG, lon]

    send(mavlink.STATUSTEXT, mavlink.enc_statustext(6, "DroneDeck SITL: ready"))

    next_t = {"hb": 0.0, "att": 0.0, "pos": 0.0, "hud": 0.0, "sys": 0.0, "gps": 0.0,
              "txt": 3.0, "adsb": 0.5, "rc": 0.0, "alt2": 0.0, "vib": 0.0, "bat": 0.0}
    period = {"hb": 0.25, "att": 0.04, "pos": 0.2, "hud": 0.2, "sys": 1.0, "gps": 1.0,
              "txt": 9.0, "adsb": 1.0, "rc": 0.1, "alt2": 0.2, "vib": 1.0, "bat": 1.0}

    t0 = time.monotonic()
    last = t0
    climb = 0.0
    groundspeed = SPEED
    man_x = man_y = man_z = man_r = 0.0   # last MANUAL_CONTROL sticks
    man_t = -99.0                          # time of last manual input

    try:
        while True:
            now = time.monotonic()
            dt = max(1e-3, now - last)
            last = now
            t = now - t0

            plat, plon, palt = lat, lon, alt
            climb_rate = 3.0
            target_alt = CRUISE_ALT

            do_orbit = False
            if not armed:
                target_alt = alt
            elif mode == LOITER:
                do_orbit = True
            elif mode == AUTO:
                nav = [it for it in missions[0] if it["command"] in (16, 22)
                       and not (abs(it["lat"]) < 1e-6 and abs(it["lon"]) < 1e-6)]
                if nav:
                    if auto_idx >= len(nav):
                        auto_idx = len(nav) - 1
                    tgt = nav[auto_idx]
                    lat, lon, rem, hdg = step_toward(lat, lon, tgt["lat"], tgt["lon"],
                                                     SPEED * dt, cos_lat)
                    if hdg is not None:
                        heading = hdg
                    target_alt = tgt["alt"] if tgt["alt"] > 0 else CRUISE_ALT
                    if rem < 5.0 and auto_idx < len(nav) - 1:
                        auto_idx += 1
                else:
                    do_orbit = True
            elif mode == GUIDED:
                if guided_target:
                    lat, lon, _rem, hdg = step_toward(lat, lon, guided_target[0],
                                                      guided_target[1], SPEED * dt, cos_lat)
                    if hdg is not None:
                        heading = hdg
                    target_alt = guided_target[2]
                else:
                    target_alt = alt
            elif mode == RTL:
                lat, lon, rem, hdg = step_toward(lat, lon, HOME_LAT, HOME_LON, SPEED * dt, cos_lat)
                if hdg is not None:
                    heading = hdg
                target_alt = CRUISE_ALT if rem > 3.0 else 0.0
            elif mode == LAND:
                target_alt = 0.0
            elif mode == ALT_HOLD:
                # manual stick control: move per pitch/roll, turn per yaw, climb per thrust
                if armed and (t - man_t) < 1.5:
                    MAN_SPEED, YAW_RATE = 12.0, 120.0    # m/s, deg/s at full deflection
                    heading = (heading + (man_r / 1000.0) * YAW_RATE * dt) % 360.0
                    fwd = (man_x / 1000.0) * MAN_SPEED
                    lat_v = (man_y / 1000.0) * MAN_SPEED
                    hr = math.radians(heading)
                    dn = fwd * math.cos(hr) - lat_v * math.sin(hr)
                    de = fwd * math.sin(hr) + lat_v * math.cos(hr)
                    lat += dn * dt / M_PER_DEG
                    lon += de * dt / (M_PER_DEG * cos_lat)
                    target_alt = alt + (man_z / 1000.0) * climb_rate
                else:
                    target_alt = alt                     # hover / altitude hold

            if do_orbit:
                theta += (SPEED / RADIUS_M) * dt
                lat = loiter_center[0] + RADIUS_M * math.cos(theta) / M_PER_DEG
                lon = loiter_center[1] + RADIUS_M * math.sin(theta) / (M_PER_DEG * cos_lat)
                heading = math.degrees(math.atan2(math.cos(theta), -math.sin(theta))) % 360.0
                target_alt = CRUISE_ALT

            if armed:
                alt += max(-climb_rate * dt, min(climb_rate * dt, target_alt - alt))
                if mode in (RTL, LAND) and target_alt == 0.0 and alt <= 0.5:
                    armed = False
                    send(mavlink.STATUSTEXT, mavlink.enc_statustext(5, "Landed, disarmed"))

            dn = (lat - plat) * M_PER_DEG
            de = (lon - plon) * M_PER_DEG * cos_lat
            groundspeed = math.hypot(dn, de) / dt
            climb = (alt - palt) / dt

            # bank into the turn while orbiting; near-level otherwise
            bank = math.atan2(SPEED * SPEED, RADIUS_M * 9.81) if do_orbit else 0.0
            roll = bank + 0.03 * math.sin(t * 1.7)
            pitch = 0.03 * math.sin(t * 0.9)
            yaw = math.radians(heading)

            base_mode = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            if armed:
                base_mode |= mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            batt_pct = max(2, int(100 - t * 0.5))
            voltage = 10.5 + (batt_pct / 100.0) * 2.1
            current = 18.0 + 4.0 * math.sin(t)

            if t >= next_t["hb"]:
                next_t["hb"] += period["hb"]
                send(mavlink.HEARTBEAT, mavlink.enc_heartbeat(
                    mav_type=mavlink.MAV_TYPE_QUADROTOR, base_mode=base_mode, custom_mode=mode))
            if t >= next_t["att"]:
                next_t["att"] += period["att"]
                send(mavlink.ATTITUDE, mavlink.enc_attitude(roll, pitch, yaw, int(t * 1000)))
            if t >= next_t["pos"]:
                next_t["pos"] += period["pos"]
                send(mavlink.GLOBAL_POSITION_INT, mavlink.enc_global_position_int(
                    int(lat * 1e7), int(lon * 1e7), int(alt * 1000), int(alt * 1000),
                    int(heading * 100), int(t * 1000),
                    vx=int(dn / dt * 100), vy=int(de / dt * 100), vz=int(-climb * 100)))
            if t >= next_t["hud"]:
                next_t["hud"] += period["hud"]
                send(mavlink.VFR_HUD, mavlink.enc_vfr_hud(
                    groundspeed, groundspeed, alt, climb, heading, 55 if armed else 0))
            if t >= next_t["sys"]:
                next_t["sys"] += period["sys"]
                # present/enabled: gyro, accel, mag, baro, gps, motors, rc, ahrs, logging, battery
                sens = ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 5) | (1 << 15)
                        | (1 << 16) | (1 << 21) | (1 << 24) | (1 << 25))
                health = sens                      # all healthy (drop GPS bit if no fix)
                send(mavlink.SYS_STATUS, mavlink.enc_sys_status(
                    int(voltage * 1000), int(current * 100), batt_pct,
                    present=sens, enabled=sens, health=health))
            if t >= next_t["alt2"]:
                next_t["alt2"] += period["alt2"]
                send(mavlink.ALTITUDE, mavlink.enc_altitude(
                    alt, alt + 100.0, alt, alt, alt, alt))   # monotonic, amsl, local, rel, terrain, bottom
            if t >= next_t["vib"]:
                next_t["vib"] += period["vib"]
                base = 3.0 + (8.0 if armed else 0.0)         # higher vibration when flying
                send(mavlink.VIBRATION, mavlink.enc_vibration(
                    base + 2.0 * math.sin(t * 1.3), base + 2.0 * math.cos(t * 1.1),
                    base * 1.2 + math.sin(t * 0.7), 0, 0, 0))
            if t >= next_t["bat"]:
                next_t["bat"] += period["bat"]
                per_cell = int(voltage * 1000 / 3)           # 3S pack, mV per cell
                send(mavlink.BATTERY_STATUS, mavlink.enc_battery_status(
                    [per_cell, per_cell, per_cell], current_battery=int(current * 100),
                    current_consumed=int(t * 2), energy_consumed=int(t * 3),
                    battery_remaining=batt_pct, temperature=int(2500 + 300 * math.sin(t * 0.05))))
            if t >= next_t["gps"]:
                next_t["gps"] += period["gps"]
                send(mavlink.GPS_RAW_INT, mavlink.enc_gps_raw_int(
                    int(lat * 1e7), int(lon * 1e7), int(alt * 1000), fix=3, sats=14,
                    vel_cms=int(groundspeed * 100), cog_cdeg=int(heading * 100)))
            if t >= next_t["rc"]:
                next_t["rc"] += period["rc"]
                # synthetic transmitter: each channel sweeps its full range at its
                # own rate so RC calibration has real min/max to capture
                chans = []
                for ch in range(8):
                    span = 380 + 20 * ch
                    chans.append(int(1500 + span * math.sin(t * (1.6 + 0.25 * ch) + ch)))
                send(mavlink.RC_CHANNELS, mavlink.enc_rc_channels(chans, rssi=210))
            if t >= next_t["txt"]:
                next_t["txt"] += period["txt"]
                send(mavlink.STATUSTEXT, mavlink.enc_statustext(
                    6, f"{mavlink.ARDUCOPTER_MODES.get(mode, mode)}  alt {alt:.0f}m  batt {batt_pct}%"))
            if sysid == 1 and t >= next_t["adsb"]:
                next_t["adsb"] += period["adsb"]
                for idx, (icao, cs, radius, rate) in enumerate(
                        [(0xA00001, "DRN001", 300.0, 0.020), (0xA00002, "HEL022", 550.0, -0.013)]):
                    ang = t * rate + idx * 2.0
                    tlat = HOME_LAT + (radius * math.cos(ang)) / M_PER_DEG
                    tlon = HOME_LON + (radius * math.sin(ang)) / (M_PER_DEG * cos_lat)
                    hdg = (math.degrees(ang) + 90.0) % 360.0
                    send(mavlink.ADSB_VEHICLE, mavlink.enc_adsb_vehicle(
                        icao, int(tlat * 1e7), int(tlon * 1e7), 120000 + idx * 20000,
                        int(hdg * 100), cs, emitter_type=(2 if idx == 0 else 7),
                        hor_velocity=1500))

            # --- inbound GCS commands -------------------------------------
            if PARSER is not None:
                try:
                    while True:
                        data, _addr = sock.recvfrom(2048)
                        for m in PARSER.feed(data):
                            if m.msgid == mavlink.COMMAND_LONG:
                                cmd = int(m.fields.get("command", 0))
                                send(mavlink.COMMAND_ACK, mavlink.enc_command_ack(cmd, 0))
                                if cmd == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                                    armed = m.fields.get("param1", 0) >= 0.5
                                    if armed:
                                        enter_loiter()
                                    send(mavlink.STATUSTEXT, mavlink.enc_statustext(
                                        5, "Armed" if armed else "Disarmed"))
                                elif cmd == mavlink.MAV_CMD_DO_SET_MODE:
                                    mode = int(m.fields.get("param2", LOITER))
                                    if mode in (LOITER, AUTO):
                                        enter_loiter()
                                    send(mavlink.STATUSTEXT, mavlink.enc_statustext(
                                        6, "Mode " + mavlink.ARDUCOPTER_MODES.get(mode, str(mode))))
                                elif cmd == mavlink.MAV_CMD_NAV_TAKEOFF:
                                    armed = True
                                    mode = GUIDED
                                    guided_target = (lat, lon, float(m.fields.get("param7", 30) or 30))
                                    send(mavlink.STATUSTEXT, mavlink.enc_statustext(5, "Taking off"))
                                elif cmd == mavlink.MAV_CMD_NAV_LAND:
                                    mode = LAND
                                    send(mavlink.STATUSTEXT, mavlink.enc_statustext(5, "Landing"))
                                elif cmd == mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                                    mode = RTL
                                    send(mavlink.STATUSTEXT, mavlink.enc_statustext(5, "Returning to launch"))
                                elif cmd == mavlink.MAV_CMD_DO_PAUSE_CONTINUE:
                                    if m.fields.get("param1", 0) >= 0.5:
                                        mode = LOITER
                                        enter_loiter()
                                    else:
                                        mode = GUIDED
                                        guided_target = (lat, lon, alt)
                                    send(mavlink.STATUSTEXT, mavlink.enc_statustext(5, "Pause/Continue"))
                                elif cmd in (mavlink.MAV_CMD_IMAGE_START_CAPTURE,
                                             mavlink.MAV_CMD_DO_DIGICAM_CONTROL):
                                    send(mavlink.STATUSTEXT, mavlink.enc_statustext(6, "Photo captured"))
                                elif cmd == mavlink.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                                    d = m.fields.get("param1", 0)
                                    send(mavlink.STATUSTEXT, mavlink.enc_statustext(
                                        6, f"Camera trigger dist {d:.0f} m" if d > 0 else "Camera trigger off"))
                                elif cmd in (mavlink.MAV_CMD_VIDEO_START_CAPTURE,
                                             mavlink.MAV_CMD_VIDEO_STOP_CAPTURE):
                                    on = (cmd == mavlink.MAV_CMD_VIDEO_START_CAPTURE)
                                    send(mavlink.STATUSTEXT, mavlink.enc_statustext(
                                        6, "Video recording" if on else "Video stopped"))
                                elif cmd == mavlink.MAV_CMD_DO_MOUNT_CONTROL:
                                    send(mavlink.STATUSTEXT, mavlink.enc_statustext(
                                        6, f"Gimbal pitch {m.fields.get('param1', 0):.0f} "
                                           f"yaw {m.fields.get('param3', 0):.0f}"))
                                elif cmd == mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
                                    if m.fields.get("param1", 0) >= 1:
                                        cal_lines = ["Calibrating gyroscopes",
                                                     "Gyro calibration successful"]
                                    elif m.fields.get("param2", 0) >= 1:
                                        cal_lines = ["Compass calibration: rotate through all orientations",
                                                     "Compass calibration successful"]
                                    elif m.fields.get("param5", 0) == 2:
                                        cal_lines = ["Calibrating level horizon",
                                                     "Level calibration successful"]
                                    else:
                                        cal_lines = ["Place vehicle level and press OK",
                                                     "Place vehicle on its LEFT side",
                                                     "Place vehicle on its RIGHT side",
                                                     "Place vehicle nose DOWN",
                                                     "Place vehicle nose UP",
                                                     "Place vehicle on its BACK",
                                                     "Calibration successful"]
                                    for cal_line in cal_lines:
                                        send(mavlink.STATUSTEXT, mavlink.enc_statustext(6, cal_line))
                            elif m.msgid == mavlink.SET_POSITION_TARGET_GLOBAL_INT:
                                guided_target = (m.fields.get("lat_int", 0) / 1e7,
                                                 m.fields.get("lon_int", 0) / 1e7,
                                                 float(m.fields.get("alt", CRUISE_ALT)))
                                mode = GUIDED
                                send(mavlink.STATUSTEXT, mavlink.enc_statustext(6, "Goto target set"))
                            elif m.msgid == mavlink.MANUAL_CONTROL:
                                man_x = float(m.fields.get("x", 0))
                                man_y = float(m.fields.get("y", 0))
                                man_z = float(m.fields.get("z", 0))
                                man_r = float(m.fields.get("r", 0))
                                man_t = t
                                if armed and mode != ALT_HOLD:
                                    mode = ALT_HOLD
                                    send(mavlink.STATUSTEXT,
                                         mavlink.enc_statustext(6, "Manual control (ALT_HOLD)"))
                            elif m.msgid == mavlink.LOG_REQUEST_LIST:
                                last = SIM_LOGS[-1][0]
                                for lid, sz, utc in SIM_LOGS:
                                    send(mavlink.LOG_ENTRY,
                                         mavlink.enc_log_entry(lid, len(SIM_LOGS), last, sz, utc))
                            elif m.msgid == mavlink.LOG_REQUEST_DATA:
                                lid = int(m.fields.get("id", 0))
                                match = [s for s in SIM_LOGS if s[0] == lid]
                                if match:
                                    size = match[0][1]
                                    ofs = int(m.fields.get("ofs", 0))
                                    while ofs < size:
                                        n = min(90, size - ofs)
                                        chunk = bytes(_logbyte(lid, ofs + i) for i in range(n))
                                        send(mavlink.LOG_DATA, mavlink.enc_log_data(lid, ofs, chunk))
                                        ofs += n
                            elif m.msgid == mavlink.LOG_REQUEST_END:
                                pass
                            # ---- mission protocol (vehicle side, per mission_type) ----
                            elif m.msgid == mavlink.MISSION_COUNT:
                                up_mtype = int(m.fields.get("mission_type", 0))
                                up_expected = int(m.fields.get("count", 0))
                                up_items = []
                                up_next = 0
                                if up_expected == 0:
                                    missions[up_mtype] = []
                                    if up_mtype == 0:
                                        auto_idx = 0
                                    send(mavlink.MISSION_ACK,
                                         mavlink.enc_mission_ack(0, mission_type=up_mtype))
                                else:
                                    send(mavlink.MISSION_REQUEST_INT,
                                         mavlink.enc_mission_request_int(0, mission_type=up_mtype))
                            elif m.msgid == mavlink.MISSION_ITEM_INT:
                                seq = int(m.fields.get("seq", -1))
                                if up_expected and seq == up_next:
                                    up_items.append({"seq": seq,
                                                     "lat": m.fields["x"] / 1e7,
                                                     "lon": m.fields["y"] / 1e7,
                                                     "alt": float(m.fields["z"]),
                                                     "command": int(m.fields["command"]),
                                                     "frame": int(m.fields["frame"]),
                                                     "p1": float(m.fields["param1"]),
                                                     "p2": float(m.fields["param2"]),
                                                     "p3": float(m.fields["param3"]),
                                                     "p4": float(m.fields["param4"])})
                                    up_next += 1
                                    if up_next >= up_expected:
                                        missions[up_mtype] = up_items
                                        up_expected = 0
                                        if up_mtype == 0:
                                            auto_idx = 0
                                        send(mavlink.MISSION_ACK,
                                             mavlink.enc_mission_ack(0, mission_type=up_mtype))
                                        kind = {0: "Mission", 1: "Fence", 2: "Rally"}.get(up_mtype, "?")
                                        send(mavlink.STATUSTEXT, mavlink.enc_statustext(
                                            6, f"{kind} received: {len(up_items)} items"))
                                    else:
                                        send(mavlink.MISSION_REQUEST_INT,
                                             mavlink.enc_mission_request_int(up_next, mission_type=up_mtype))
                            elif m.msgid == mavlink.MISSION_REQUEST_LIST:
                                mt = int(m.fields.get("mission_type", 0))
                                send(mavlink.MISSION_COUNT,
                                     mavlink.enc_mission_count(len(missions.get(mt, [])), mission_type=mt))
                            elif m.msgid == mavlink.MISSION_REQUEST_INT:
                                mt = int(m.fields.get("mission_type", 0))
                                seq = int(m.fields.get("seq", 0))
                                lst = missions.get(mt, [])
                                if 0 <= seq < len(lst):
                                    it = lst[seq]
                                    send(mavlink.MISSION_ITEM_INT, mavlink.enc_mission_item_int(
                                        it["seq"], it["lat"], it["lon"], it["alt"],
                                        command=it["command"], frame=it.get("frame", 6),
                                        param1=it.get("p1", 0.0), param2=it.get("p2", 0.0),
                                        param3=it.get("p3", 0.0), param4=it.get("p4", 0.0),
                                        current=1 if seq == 0 else 0, mission_type=mt))
                            elif m.msgid == mavlink.MISSION_CLEAR_ALL:
                                mt = int(m.fields.get("mission_type", 0))
                                missions[mt] = []
                                if mt == 0:
                                    auto_idx = 0
                                send(mavlink.MISSION_ACK, mavlink.enc_mission_ack(0, mission_type=mt))
                                kind = {0: "Mission", 1: "Fence", 2: "Rally"}.get(mt, "?")
                                send(mavlink.STATUSTEXT, mavlink.enc_statustext(6, f"{kind} cleared"))
                            # ---- parameter protocol (vehicle side) ----
                            elif m.msgid == mavlink.PARAM_REQUEST_LIST:
                                for i, name in enumerate(param_order):
                                    send(mavlink.PARAM_VALUE, mavlink.enc_param_value(
                                        name, sim_params[name], count=len(param_order), index=i))
                            elif m.msgid == mavlink.PARAM_SET:
                                name = m.fields.get("param_id", "")
                                if name in sim_params:
                                    sim_params[name] = float(m.fields.get("param_value", 0.0))
                                    i = param_order.index(name)
                                    send(mavlink.PARAM_VALUE, mavlink.enc_param_value(
                                        name, sim_params[name], count=len(param_order), index=i))
                                    send(mavlink.STATUSTEXT, mavlink.enc_statustext(
                                        6, f"{name} = {sim_params[name]:g}"))
                            elif m.msgid == mavlink.PARAM_REQUEST_READ:
                                idx = int(m.fields.get("param_index", -1))
                                name = m.fields.get("param_id", "")
                                if 0 <= idx < len(param_order):
                                    name = param_order[idx]
                                if name in sim_params:
                                    i = param_order.index(name)
                                    send(mavlink.PARAM_VALUE, mavlink.enc_param_value(
                                        name, sim_params[name], count=len(param_order), index=i))
                except BlockingIOError:
                    pass
                except OSError:
                    pass

            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\n[sim] stopped")


if __name__ == "__main__":
    main()
