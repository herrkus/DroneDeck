"""vehicle.py -- the live vehicle state model.

Consumes decoded Messages and exposes scaled, human-unit state plus Qt signals
the widgets bind to. One Vehicle == one connected system id.
"""
from __future__ import annotations
import math
import time

from PySide6.QtCore import QObject, Signal

import mavlink


class Vehicle(QObject):
    updated = Signal()              # emitted after a batch of messages is applied
    text_status = Signal(str)       # human-readable connection notes
    status_text = Signal(int, str)  # STATUSTEXT: severity, text
    command_ack = Signal(int, int)  # COMMAND_ACK: command, result

    TRAIL_MAX = 4000

    def __init__(self):
        super().__init__()
        # attitude (radians)
        self.roll = self.pitch = self.yaw = 0.0
        # position
        self.lat = self.lon = 0.0
        self.alt_msl = 0.0          # m
        self.alt_rel = 0.0          # m above home
        self.heading = 0.0          # deg
        self.vx = self.vy = self.vz = 0.0
        # vfr hud
        self.airspeed = self.groundspeed = 0.0
        self.climb = 0.0
        self.throttle = 0
        # power
        self.voltage = 0.0
        self.current = 0.0
        self.battery_remaining = -1
        # gps
        self.fix_type = 0
        self.satellites = 0
        # status
        self.sysid = 0
        self.mav_type = 0
        self.autopilot = 0
        self.base_mode = 0
        self.custom_mode = 0
        self.system_status = 0
        self.armed = False
        self.mode = "--"
        self.messages = []          # [(severity, text), ...] STATUSTEXT log
        self.last_ack = None        # (command, result)
        # bookkeeping
        self.have_position = False
        self.home = None            # (lat, lon)
        self.trail = []             # [(lat, lon), ...]
        self.last_heartbeat = 0.0
        self.msg_count = 0

    # -- ingest ---------------------------------------------------------------
    def consume(self, msgs):
        for m in msgs:
            self.msg_count += 1
            if self.sysid == 0 and m.sysid:
                self.sysid = m.sysid
            handler = self._H.get(m.msgid)
            if handler:
                handler(self, m.fields)
        if msgs:
            self.updated.emit()

    def _on_heartbeat(self, f):
        self.base_mode = int(f.get("base_mode", 0))
        self.custom_mode = int(f.get("custom_mode", 0))
        self.mav_type = int(f.get("type", 0))
        self.autopilot = int(f.get("autopilot", 0))
        self.system_status = int(f.get("system_status", 0))
        self.armed = bool(self.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.mode = mavlink.flight_mode_name(self.autopilot, self.mav_type,
                                             self.base_mode, self.custom_mode)
        self.last_heartbeat = time.monotonic()

    def _on_statustext(self, f):
        sev = int(f.get("severity", 6))
        text = str(f.get("text", ""))
        self.messages.append((sev, text))
        if len(self.messages) > 200:
            self.messages.pop(0)
        self.status_text.emit(sev, text)

    def _on_command_ack(self, f):
        cmd = int(f.get("command", 0))
        res = int(f.get("result", 0))
        self.last_ack = (cmd, res)
        self.command_ack.emit(cmd, res)

    def _on_attitude(self, f):
        self.roll = f.get("roll", 0.0)
        self.pitch = f.get("pitch", 0.0)
        self.yaw = f.get("yaw", 0.0)

    def _on_global_position(self, f):
        self.lat = f.get("lat", 0) / 1e7
        self.lon = f.get("lon", 0) / 1e7
        self.alt_msl = f.get("alt", 0) / 1000.0
        self.alt_rel = f.get("relative_alt", 0) / 1000.0
        self.heading = (f.get("hdg", 0) / 100.0) % 360.0
        self.vx, self.vy, self.vz = f.get("vx", 0) / 100.0, f.get("vy", 0) / 100.0, f.get("vz", 0) / 100.0
        if abs(self.lat) > 1e-6 or abs(self.lon) > 1e-6:
            self.have_position = True
            if self.home is None:
                self.home = (self.lat, self.lon)
            if not self.trail or self._moved(self.trail[-1], (self.lat, self.lon)):
                self.trail.append((self.lat, self.lon))
                if len(self.trail) > self.TRAIL_MAX:
                    self.trail.pop(0)

    @staticmethod
    def _moved(a, b):
        return abs(a[0] - b[0]) > 2e-6 or abs(a[1] - b[1]) > 2e-6

    def _on_sys_status(self, f):
        self.voltage = f.get("voltage_battery", 0) / 1000.0
        self.current = f.get("current_battery", 0) / 100.0
        self.battery_remaining = int(f.get("battery_remaining", -1))

    def _on_gps_raw(self, f):
        self.fix_type = int(f.get("fix_type", 0))
        self.satellites = int(f.get("satellites_visible", 0))

    def _on_vfr_hud(self, f):
        self.airspeed = f.get("airspeed", 0.0)
        self.groundspeed = f.get("groundspeed", 0.0)
        self.climb = f.get("climb", 0.0)
        self.throttle = int(f.get("throttle", 0))
        if f.get("heading") is not None and not self.have_position:
            self.heading = float(f["heading"]) % 360.0

    _H = {
        mavlink.HEARTBEAT: _on_heartbeat,
        mavlink.ATTITUDE: _on_attitude,
        mavlink.GLOBAL_POSITION_INT: _on_global_position,
        mavlink.SYS_STATUS: _on_sys_status,
        mavlink.GPS_RAW_INT: _on_gps_raw,
        mavlink.VFR_HUD: _on_vfr_hud,
        mavlink.STATUSTEXT: _on_statustext,
        mavlink.COMMAND_ACK: _on_command_ack,
    }

    # -- derived --------------------------------------------------------------
    @property
    def link_alive(self) -> bool:
        return (time.monotonic() - self.last_heartbeat) < 3.0 if self.last_heartbeat else False

    @property
    def fix_text(self) -> str:
        return {0: "No GPS", 1: "No Fix", 2: "2D", 3: "3D", 4: "DGPS",
                5: "RTK Float", 6: "RTK Fixed"}.get(self.fix_type, str(self.fix_type))

    @property
    def type_text(self) -> str:
        return {0: "Generic", 1: "Fixed Wing", 2: "Quadrotor", 3: "Coaxial",
                4: "Helicopter", 13: "Hexarotor", 14: "Octorotor",
                10: "Ground Rover", 12: "Submarine"}.get(self.mav_type, f"Type {self.mav_type}")
