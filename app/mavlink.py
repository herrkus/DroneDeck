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
VFR_HUD = 74
COMMAND_LONG = 76

MSG_NAME = {
    HEARTBEAT: "HEARTBEAT",
    SYS_STATUS: "SYS_STATUS",
    GPS_RAW_INT: "GPS_RAW_INT",
    ATTITUDE: "ATTITUDE",
    GLOBAL_POSITION_INT: "GLOBAL_POSITION_INT",
    VFR_HUD: "VFR_HUD",
    COMMAND_LONG: "COMMAND_LONG",
}

# Per-message CRC_EXTRA seed bytes (from the MAVLink common dialect).
CRC_EXTRA = {
    HEARTBEAT: 50, SYS_STATUS: 124, GPS_RAW_INT: 24, ATTITUDE: 39,
    GLOBAL_POSITION_INT: 104, VFR_HUD: 20, COMMAND_LONG: 152,
}

# Decoded-field order. Index i here is index i in Decoded.f[] from the C++ core.
FIELDS = {
    HEARTBEAT: ["type", "autopilot", "base_mode", "custom_mode", "system_status", "mavlink_version"],
    SYS_STATUS: ["voltage_battery", "current_battery", "battery_remaining", "load"],
    GPS_RAW_INT: ["fix_type", "satellites_visible", "lat", "lon", "alt", "eph", "vel", "cog"],
    ATTITUDE: ["roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed", "time_boot_ms"],
    GLOBAL_POSITION_INT: ["lat", "lon", "alt", "relative_alt", "vx", "vy", "vz", "hdg", "time_boot_ms"],
    VFR_HUD: ["airspeed", "groundspeed", "alt", "climb", "heading", "throttle"],
    COMMAND_LONG: ["command", "param1", "param2", "param3", "param4", "param5", "param6", "param7"],
}

# --- selected enums ---------------------------------------------------------
MAV_TYPE_QUADROTOR = 2
MAV_AUTOPILOT_ARDUPILOTMEGA = 3
MAV_MODE_FLAG_SAFETY_ARMED = 0x80
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 0x01
MAV_STATE_ACTIVE = 4
GPS_FIX_TYPE_3D_FIX = 3
MAV_CMD_COMPONENT_ARM_DISARM = 400


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


# --- payload encoders (wire order matches the C++ decoder offsets) -----------
def enc_heartbeat(mav_type=MAV_TYPE_QUADROTOR, autopilot=MAV_AUTOPILOT_ARDUPILOTMEGA,
                  base_mode=0, custom_mode=0, system_status=MAV_STATE_ACTIVE):
    return struct.pack("<IBBBBB", custom_mode, mav_type, autopilot, base_mode, system_status, 3)


def enc_sys_status(voltage_mv, current_ca, remaining_pct, load=250):
    return struct.pack("<IIIHHhHHHHHHb", 0, 0, 0, load, int(voltage_mv), int(current_ca),
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


def enc_command_long(command, params7, target_system=1, target_component=1, confirmation=0):
    p = [float(x) for x in (list(params7) + [0.0] * 7)[:7]]
    return struct.pack("<fffffffHBBB", *p, command & 0xFFFF,
                       target_system & 0xFF, target_component & 0xFF, confirmation & 0xFF)


def crc16(data: bytes) -> int:
    """CRC-16/MCRF4XX over data only (no extra seed)."""
    crc = 0xFFFF
    for b in data:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


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
    GLOBAL_POSITION_INT: ("<IiiiihhhH",
                          ["time_boot_ms", "lat", "lon", "alt", "relative_alt",
                           "vx", "vy", "vz", "hdg"], 28),
    VFR_HUD: ("<ffffhH",
              ["airspeed", "groundspeed", "alt", "climb", "heading", "throttle"], 20),
    COMMAND_LONG: ("<fffffffHBBB",
                   ["param1", "param2", "param3", "param4", "param5", "param6", "param7",
                    "command", "target_system", "target_component", "confirmation"], 33),
}


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
            out.append(Message(msgid, sysid, compid, seq, fields))
            self.ok += 1
            pos += total
            first_incomplete = -1
        keep = first_incomplete if first_incomplete >= 0 else pos
        if keep > 0:
            del self.buf[:keep]
        return out
