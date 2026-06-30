#!/usr/bin/env python3
"""simulator.py -- TEST telemetry source for DroneDeck (NOT a drone model).

A standalone signal generator so the ground station can be exercised without
hardware: it streams MAVLink HEARTBEAT/ATTITUDE/GPS/VFR_HUD/SYS_STATUS over UDP
while flying a lazy circle with a draining battery, and flips its armed flag
when it receives an ARM/DISARM command from the GCS. When a real drone is
connected, this is simply not used -- point the app at the drone instead.

Frames are checksummed with the project's assembly CRC when the native core is
present, otherwise with the pure-Python fallback.

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

# Prefer the native assembly CRC; fall back to pure Python if not built.
try:
    import core
    CRC_FN = core.crc_extra
    PARSER = core.Parser()
    CRC_BACKEND = "assembly core"
except Exception as e:                       # pragma: no cover
    CRC_FN = None
    PARSER = None
    CRC_BACKEND = f"pure-Python ({e.__class__.__name__})"

# Home location (Vilnius) -- arbitrary; replace freely.
HOME_LAT, HOME_LON = 54.6872, 25.2797
RADIUS_M = 140.0
SPEED = 12.0                                  # m/s along the circle
ALT = 80.0


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

    print(f"DroneDeck TEST telemetry source -> {target[0]}:{target[1]} "
          f"(sysid {sysid}, CRC: {CRC_BACKEND})")
    print("Flying a circle; Ctrl-C to stop. This is a test source, not a drone model.")

    seq = 0
    def send(msgid, payload):
        nonlocal seq
        frame = mavlink.frame(msgid, payload, seq, sysid, 1, crc_fn=CRC_FN)
        seq = (seq + 1) & 0xFF
        try:
            sock.sendto(frame, target)
        except OSError:
            pass

    armed = False
    mode = 5                                   # LOITER (ArduCopter custom_mode)
    send(mavlink.STATUSTEXT, mavlink.enc_statustext(6, "DroneDeck SITL: ready"))
    w = SPEED / RADIUS_M                       # angular rate (rad/s)
    cos_lat = math.cos(math.radians(HOME_LAT))
    t0 = time.monotonic()
    next_t = {"hb": 0.0, "att": 0.0, "pos": 0.0, "hud": 0.0, "sys": 0.0, "gps": 0.0, "txt": 3.0}
    period = {"hb": 0.25, "att": 0.04, "pos": 0.2, "hud": 0.2, "sys": 1.0, "gps": 1.0, "txt": 9.0}

    try:
        while True:
            now = time.monotonic()
            t = now - t0
            theta = w * t

            # circular kinematics
            north = RADIUS_M * math.cos(theta)
            east = RADIUS_M * math.sin(theta)
            lat = HOME_LAT + north / 111320.0
            lon = HOME_LON + east / (111320.0 * cos_lat)
            vn = -RADIUS_M * w * math.sin(theta)
            ve = RADIUS_M * w * math.cos(theta)
            heading = math.degrees(math.atan2(ve, vn)) % 360.0
            yaw = math.radians(heading)
            bank = math.atan2(SPEED * SPEED, RADIUS_M * 9.81)      # coordinated turn
            roll = bank + 0.05 * math.sin(t * 1.7)
            pitch = 0.04 * math.sin(t * 0.9)
            alt = ALT + 3.0 * math.sin(t * 0.5)
            climb = 3.0 * 0.5 * math.cos(t * 0.5)

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
                    int(lat * 1e7), int(lon * 1e7), int(alt * 1000), int((alt - ALT + 3) * 1000),
                    int(heading * 100), int(t * 1000),
                    vx=int(vn * 100), vy=int(ve * 100), vz=int(-climb * 100)))
            if t >= next_t["hud"]:
                next_t["hud"] += period["hud"]
                send(mavlink.VFR_HUD, mavlink.enc_vfr_hud(
                    SPEED, SPEED, alt, climb, heading, 55 if armed else 0))
            if t >= next_t["sys"]:
                next_t["sys"] += period["sys"]
                send(mavlink.SYS_STATUS, mavlink.enc_sys_status(
                    int(voltage * 1000), int(current * 100), batt_pct))
            if t >= next_t["gps"]:
                next_t["gps"] += period["gps"]
                send(mavlink.GPS_RAW_INT, mavlink.enc_gps_raw_int(
                    int(lat * 1e7), int(lon * 1e7), int(alt * 1000), fix=3, sats=14,
                    vel_cms=int(SPEED * 100), cog_cdeg=int(heading * 100)))
            if t >= next_t["txt"]:
                next_t["txt"] += period["txt"]
                send(mavlink.STATUSTEXT, mavlink.enc_statustext(
                    6, f"alt {alt:.0f}m  spd {SPEED:.0f}m/s  batt {batt_pct}%"))

            # handle inbound GCS commands, if the native parser is available
            if PARSER is not None:
                try:
                    while True:
                        data, _addr = sock.recvfrom(2048)
                        for m in PARSER.feed(data):
                            if m.msgid != mavlink.COMMAND_LONG:
                                continue
                            cmd = int(m.fields.get("command", 0))
                            send(mavlink.COMMAND_ACK, mavlink.enc_command_ack(cmd, 0))  # ACCEPTED
                            if cmd == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                                armed = m.fields.get("param1", 0) >= 0.5
                                send(mavlink.STATUSTEXT, mavlink.enc_statustext(
                                    5, "Armed" if armed else "Disarmed"))
                                print(f"[sim] {'ARMED' if armed else 'DISARMED'} by GCS")
                except BlockingIOError:
                    pass
                except OSError:
                    pass

            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\n[sim] stopped")


if __name__ == "__main__":
    main()
