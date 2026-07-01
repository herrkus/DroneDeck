"""vehicle.py -- the live vehicle state model.

Consumes decoded Messages and exposes scaled, human-unit state plus Qt signals
the widgets bind to. One Vehicle == one connected system id.
"""
from __future__ import annotations
import math
import time

from PySide6.QtCore import QObject, Signal

import mavlink


def _haversine(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres (kept local so vehicle.py has no UI/main dependency)."""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(min(1.0, math.sqrt(a)))


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
        self.battery_consumed = -1          # mAh drawn (BATTERY_STATUS)
        self.battery_time = -1              # s remaining, if the autopilot reports it
        self.battery_temp = None            # deg C (None = unknown)
        self.cells = []                     # per-cell voltages (V)
        # vibration + detailed altitude
        self.vibration = (0.0, 0.0, 0.0)    # m/s^2 on x/y/z
        self.clipping = (0, 0, 0)           # accel clip counts
        self.alt_terrain = None             # m above terrain (ALTITUDE.bottom_clearance)
        # telemetry radio link quality (RADIO_STATUS); None = no radio reporting
        self.radio_rssi = None
        self.radio_remrssi = None
        self.radio_noise = None
        # RC transmitter signal (RC_CHANNELS.rssi), 0-100%; None = no RC reporting
        self.rc_rssi = None
        # sensor health bitmasks (SYS_STATUS)
        self.sensors_present = 0
        self.sensors_enabled = 0
        self.sensors_health = 0
        # estimator (EKF_STATUS_REPORT): variances (0 healthy .. >1 bad) + status flags
        self.ekf_flags = 0
        self.ekf_vel_var = 0.0
        self.ekf_pos_horiz_var = 0.0
        self.ekf_pos_vert_var = 0.0
        self.ekf_compass_var = 0.0
        self.ekf_terrain_var = 0.0
        self.have_ekf = False               # True once a report has arrived
        # wind estimate (WIND_COV), NED m/s
        self.wind_x = 0.0                    # north component
        self.wind_y = 0.0                    # east component
        self.wind_z = 0.0                    # down component
        self.have_wind = False
        # gimbal / mount attitude (MOUNT_ORIENTATION), degrees
        self.gimbal_roll = 0.0
        self.gimbal_pitch = 0.0
        self.gimbal_yaw = 0.0                # relative to vehicle
        self.have_gimbal = False
        # gps
        self.fix_type = 0
        self.satellites = 0
        self.eph = None                     # HDOP*100 from GPS_RAW_INT (None = unknown)
        self.pos_horiz_acc = None           # m, from ESTIMATOR_STATUS
        self.pos_vert_acc = None
        self.current_wp = -1        # active mission waypoint seq (from MISSION_CURRENT)
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
        self.distance_traveled = 0.0   # m, cumulative ground track since connect (odometer)
        self._odo_pos = None           # last position counted toward the odometer
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
            # odometer: accumulate ground track in >=0.5 m steps (rejects GPS jitter), and
            # resync without counting on a >=1 km jump (glitch / home reset).
            if self._odo_pos is None:
                self._odo_pos = (self.lat, self.lon)
            else:
                d = _haversine(self._odo_pos[0], self._odo_pos[1], self.lat, self.lon)
                if 0.5 <= d < 1000.0:
                    self.distance_traveled += d
                    self._odo_pos = (self.lat, self.lon)
                elif d >= 1000.0:
                    self._odo_pos = (self.lat, self.lon)

    @staticmethod
    def _moved(a, b):
        return abs(a[0] - b[0]) > 2e-6 or abs(a[1] - b[1]) > 2e-6

    def _on_radio_status(self, f):
        self.radio_rssi = int(f.get("rssi", 0))
        self.radio_remrssi = int(f.get("remrssi", 0))
        self.radio_noise = int(f.get("noise", 0))

    def _on_rc_channels(self, f):
        # RC_CHANNELS.rssi is 0..254 (0 = no signal, 254 = full); 255 = unknown/not-reporting.
        r = int(f.get("rssi", 255))
        self.rc_rssi = None if r == 255 else round(r * 100.0 / 254.0)

    def _on_altitude(self, f):
        self.alt_msl = f.get("altitude_amsl", self.alt_msl)
        self.alt_rel = f.get("altitude_relative", self.alt_rel)
        self.alt_terrain = f.get("bottom_clearance", None)

    def _on_vibration(self, f):
        self.vibration = (f.get("vibration_x", 0.0), f.get("vibration_y", 0.0),
                          f.get("vibration_z", 0.0))
        self.clipping = (int(f.get("clipping_0", 0)), int(f.get("clipping_1", 0)),
                         int(f.get("clipping_2", 0)))

    def _on_battery_status(self, f):
        cells = []
        for i in range(1, 11):
            mv = int(f.get(f"voltage{i}", 65535))
            if mv == 65535:
                break
            cells.append(mv / 1000.0)
        self.cells = cells
        if cells:
            self.voltage = sum(cells)               # pack voltage from the cells
        cur = int(f.get("current_battery", -1))
        if cur >= 0:
            self.current = cur / 100.0
        cons = int(f.get("current_consumed", -1))
        if cons >= 0:
            self.battery_consumed = cons
        rem = int(f.get("battery_remaining", -1))
        if -100 <= rem <= 100 and rem >= 0:
            self.battery_remaining = rem
        temp = int(f.get("temperature", 32767))
        self.battery_temp = None if temp == 32767 else temp / 100.0
        tr = int(f.get("time_remaining", 0))            # 0 == not provided (MAVLink)
        self.battery_time = tr if tr > 0 else -1

    def battery_time_estimate(self):
        """Seconds of flight left. Prefer the autopilot's BATTERY_STATUS.time_remaining;
        otherwise derive it from consumed mAh + remaining % + present current draw.
        Returns -1 when it cannot be estimated."""
        if self.battery_time > 0:
            return self.battery_time
        rem, cons, cur = self.battery_remaining, self.battery_consumed, self.current
        if 0 < rem < 100 and cons > 0 and cur > 0.05:
            remaining_mah = cons * rem / (100.0 - rem)  # derive pack size, then what's left
            return remaining_mah / (cur * 1000.0) * 3600.0
        return -1

    def _on_ekf_status(self, f):
        self.ekf_flags = int(f.get("flags", 0))
        self.ekf_vel_var = float(f.get("velocity_variance", 0.0))
        self.ekf_pos_horiz_var = float(f.get("pos_horiz_variance", 0.0))
        self.ekf_pos_vert_var = float(f.get("pos_vert_variance", 0.0))
        self.ekf_compass_var = float(f.get("compass_variance", 0.0))
        self.ekf_terrain_var = float(f.get("terrain_alt_variance", 0.0))
        self.have_ekf = True

    def _on_estimator_status(self, f):
        # PX4's ESTIMATOR_STATUS carries innovation test ratios (same 0-good..>1-bad scale
        # and identical status-flag bits as ArduPilot's EKF_STATUS_REPORT), so it feeds the
        # very same estimator-health fields.
        self.ekf_flags = int(f.get("flags", 0))
        self.ekf_vel_var = float(f.get("vel_ratio", 0.0))
        self.ekf_pos_horiz_var = float(f.get("pos_horiz_ratio", 0.0))
        self.ekf_pos_vert_var = float(f.get("pos_vert_ratio", 0.0))
        self.ekf_compass_var = float(f.get("mag_ratio", 0.0))
        self.ekf_terrain_var = float(f.get("hagl_ratio", 0.0))
        self.pos_horiz_acc = float(f.get("pos_horiz_accuracy", 0.0)) or None
        self.pos_vert_acc = float(f.get("pos_vert_accuracy", 0.0)) or None
        self.have_ekf = True

    def _on_wind_cov(self, f):
        self.wind_x = float(f.get("wind_x", 0.0))
        self.wind_y = float(f.get("wind_y", 0.0))
        self.wind_z = float(f.get("wind_z", 0.0))
        self.have_wind = True

    def _on_mount_orientation(self, f):
        self.gimbal_roll = float(f.get("roll", 0.0))
        self.gimbal_pitch = float(f.get("pitch", 0.0))
        self.gimbal_yaw = float(f.get("yaw", 0.0))
        self.have_gimbal = True

    def wind_speed(self):
        """Horizontal wind speed, m/s."""
        return math.hypot(self.wind_x, self.wind_y)

    def wind_dir(self):
        """Compass bearing (deg) the wind blows FROM (meteorological convention). wind_x/y
        is the vector the air moves TOWARD in NED (x=north, y=east); 'from' is the reverse."""
        return math.degrees(math.atan2(-self.wind_y, -self.wind_x)) % 360.0

    def ekf_variance_max(self):
        """Worst of the four core variances (0 good .. >1 bad); QGC colours from this."""
        return max(self.ekf_vel_var, self.ekf_pos_horiz_var,
                   self.ekf_pos_vert_var, self.ekf_compass_var)

    def ekf_ok(self):
        """Healthy when attitude + horizontal velocity + a horizontal position solution
        are all flagged and no core variance is in the red (>= 1.0)."""
        need = (mavlink.ESTIMATOR_ATTITUDE | mavlink.ESTIMATOR_VELOCITY_HORIZ
                | mavlink.ESTIMATOR_POS_HORIZ_ABS)
        return (self.ekf_flags & need) == need and self.ekf_variance_max() < 1.0

    def _on_sys_status(self, f):
        self.voltage = f.get("voltage_battery", 0) / 1000.0
        self.current = f.get("current_battery", 0) / 100.0
        self.battery_remaining = int(f.get("battery_remaining", -1))
        self.sensors_present = int(f.get("onboard_present", 0))
        self.sensors_enabled = int(f.get("onboard_enabled", 0))
        self.sensors_health = int(f.get("onboard_health", 0))

    def _on_gps_raw(self, f):
        self.fix_type = int(f.get("fix_type", 0))
        self.satellites = int(f.get("satellites_visible", 0))
        eph = int(f.get("eph", 65535))
        self.eph = None if eph in (65535, 0) else eph

    def _on_vfr_hud(self, f):
        self.airspeed = f.get("airspeed", 0.0)
        self.groundspeed = f.get("groundspeed", 0.0)
        self.climb = f.get("climb", 0.0)
        self.throttle = int(f.get("throttle", 0))
        if f.get("heading") is not None and not self.have_position:
            self.heading = float(f["heading"]) % 360.0

    def _on_mission_current(self, f):
        self.current_wp = int(f.get("seq", -1))

    _H = {
        mavlink.HEARTBEAT: _on_heartbeat,
        mavlink.ATTITUDE: _on_attitude,
        mavlink.GLOBAL_POSITION_INT: _on_global_position,
        mavlink.SYS_STATUS: _on_sys_status,
        mavlink.ALTITUDE: _on_altitude,
        mavlink.VIBRATION: _on_vibration,
        mavlink.EKF_STATUS_REPORT: _on_ekf_status,
        mavlink.ESTIMATOR_STATUS: _on_estimator_status,
        mavlink.WIND_COV: _on_wind_cov,
        mavlink.MOUNT_ORIENTATION: _on_mount_orientation,
        mavlink.BATTERY_STATUS: _on_battery_status,
        mavlink.RADIO_STATUS: _on_radio_status,
        mavlink.RC_CHANNELS: _on_rc_channels,
        mavlink.GPS_RAW_INT: _on_gps_raw,
        mavlink.VFR_HUD: _on_vfr_hud,
        mavlink.STATUSTEXT: _on_statustext,
        mavlink.COMMAND_ACK: _on_command_ack,
        mavlink.MISSION_CURRENT: _on_mission_current,
    }

    # -- derived --------------------------------------------------------------
    @property
    def link_alive(self) -> bool:
        return (time.monotonic() - self.last_heartbeat) < 3.0 if self.last_heartbeat else False

    def preflight_status(self):
        """QGC-style arming readiness from live telemetry -> (ready: bool, reasons: list).
        Empty reasons == ready to arm. Only meaningful once telemetry is flowing."""
        reasons = []
        if self.fix_type < 3:
            reasons.append("no 3D GPS fix")
        elif self.satellites and self.satellites < 6:
            reasons.append(f"only {self.satellites} sats")
        bad = [name for bit, name in mavlink.SENSOR_BITS
               if (self.sensors_present & bit) and (self.sensors_enabled & bit)
               and not (self.sensors_health & bit)]
        if bad:
            reasons.append("unhealthy: " + ", ".join(bad[:4]))
        if 0 <= self.battery_remaining < 20:
            reasons.append(f"battery {self.battery_remaining}%")
        return (not reasons, reasons)

    @property
    def fix_text(self) -> str:
        return {0: "No GPS", 1: "No Fix", 2: "2D", 3: "3D", 4: "DGPS",
                5: "RTK Float", 6: "RTK Fixed"}.get(self.fix_type, str(self.fix_type))

    @property
    def type_text(self) -> str:
        return {0: "Generic", 1: "Fixed Wing", 2: "Quadrotor", 3: "Coaxial",
                4: "Helicopter", 13: "Hexarotor", 14: "Octorotor",
                10: "Ground Rover", 12: "Submarine"}.get(self.mav_type, f"Type {self.mav_type}")
