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

    def enter_loiter():
        nonlocal theta, loiter_center
        theta = 0.0
        loiter_center = [lat - RADIUS_M / M_PER_DEG, lon]

    send(mavlink.STATUSTEXT, mavlink.enc_statustext(6, "DroneDeck SITL: ready"))

    next_t = {"hb": 0.0, "att": 0.0, "pos": 0.0, "hud": 0.0, "sys": 0.0, "gps": 0.0, "txt": 3.0}
    period = {"hb": 0.25, "att": 0.04, "pos": 0.2, "hud": 0.2, "sys": 1.0, "gps": 1.0, "txt": 9.0}

    t0 = time.monotonic()
    last = t0
    climb = 0.0
    groundspeed = SPEED

    try:
        while True:
            now = time.monotonic()
            dt = max(1e-3, now - last)
            last = now
            t = now - t0

            plat, plon, palt = lat, lon, alt
            climb_rate = 3.0
            target_alt = CRUISE_ALT

            if not armed:
                target_alt = alt
            elif mode in (LOITER, AUTO):
                theta += (SPEED / RADIUS_M) * dt
                lat = loiter_center[0] + RADIUS_M * math.cos(theta) / M_PER_DEG
                lon = loiter_center[1] + RADIUS_M * math.sin(theta) / (M_PER_DEG * cos_lat)
                heading = math.degrees(math.atan2(math.cos(theta), -math.sin(theta))) % 360.0
                target_alt = CRUISE_ALT
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
            bank = math.atan2(SPEED * SPEED, RADIUS_M * 9.81) if mode in (LOITER, AUTO) and armed else 0.0
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
                send(mavlink.SYS_STATUS, mavlink.enc_sys_status(
                    int(voltage * 1000), int(current * 100), batt_pct))
            if t >= next_t["gps"]:
                next_t["gps"] += period["gps"]
                send(mavlink.GPS_RAW_INT, mavlink.enc_gps_raw_int(
                    int(lat * 1e7), int(lon * 1e7), int(alt * 1000), fix=3, sats=14,
                    vel_cms=int(groundspeed * 100), cog_cdeg=int(heading * 100)))
            if t >= next_t["txt"]:
                next_t["txt"] += period["txt"]
                send(mavlink.STATUSTEXT, mavlink.enc_statustext(
                    6, f"{mavlink.ARDUCOPTER_MODES.get(mode, mode)}  alt {alt:.0f}m  batt {batt_pct}%"))

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
                            elif m.msgid == mavlink.SET_POSITION_TARGET_GLOBAL_INT:
                                guided_target = (m.fields.get("lat_int", 0) / 1e7,
                                                 m.fields.get("lon_int", 0) / 1e7,
                                                 float(m.fields.get("alt", CRUISE_ALT)))
                                mode = GUIDED
                                send(mavlink.STATUSTEXT, mavlink.enc_statustext(6, "Goto target set"))
                except BlockingIOError:
                    pass
                except OSError:
                    pass

            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\n[sim] stopped")


if __name__ == "__main__":
    main()
