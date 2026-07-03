"""link.py -- MAVLink links over UDP, TCP and serial.

A common Link base owns the native parser, the 1 Hz GCS heartbeat and all
command framing; the transports (UdpLink / TcpLink / SerialLink) only move
bytes. Incoming bytes feed the parser and decoded Messages are emitted in
batches on the Qt main loop.
"""
from __future__ import annotations

import struct
import time

from PySide6.QtCore import QObject, Signal, QTimer, QIODeviceBase
from PySide6.QtNetwork import QUdpSocket, QTcpSocket, QHostAddress

import core
import ftp
import mavlink

try:
    from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo
    HAVE_SERIAL = True
except Exception:                       # pragma: no cover
    HAVE_SERIAL = False


class Link(QObject):
    messages = Signal(list)        # list[core.Message]
    state = Signal(bool)           # open / closed
    info = Signal(str)             # human-readable status line
    command_result = Signal(int, int)   # a tracked command was acknowledged: (command, result)
    command_unacked = Signal(int)       # a tracked command got no ACK after all retries: (command)

    # Confirmed-command delivery. Critical commands are resent until the vehicle returns a
    # COMMAND_ACK, so a frame dropped on a lossy RF link doesn't silently fail (QGC/MAVSDK do this).
    ACK_INTERVAL = 1.0             # seconds between resends of an unacknowledged command
    ACK_MAX_TRIES = 4              # total transmissions before giving up (~3 s of retries)

    def __init__(self, gcs_sysid=255, gcs_compid=mavlink.MAV_COMP_ID_MISSIONPLANNER):
        super().__init__()
        self.parser: core.Parser | None = None
        self.remote = None         # truthy once a peer is known (can send)
        self.gcs_sysid = gcs_sysid
        self.gcs_compid = gcs_compid
        self.seq = 0
        self._rtcm_seq = 0         # GPS_RTCM_DATA sequence id (0..31), increments per RTCM message
        self.rx_bytes = 0
        self.rx_mav_v2 = False     # a v2 (0xFD) frame has been received
        self.rx_mav_v1 = False     # a v1 (0xFE) frame has been received
        self._open = False
        self.recorder = None       # TlogWriter while recording, else None
        self.hb = QTimer(self)
        self.hb.setInterval(1000)
        self.hb.timeout.connect(self._send_heartbeat)
        self._pending = {}         # confirmed commands awaiting ACK: command id -> {resend, tries, deadline}
        self._ack_timer = QTimer(self)
        self._ack_timer.setInterval(250)   # poll finer than ACK_INTERVAL so deadlines land promptly
        self._ack_timer.timeout.connect(self._check_acks)

    # -- transport hooks (overridden) ----------------------------------------
    def open(self, **kw) -> bool:
        raise NotImplementedError

    def _write(self, data: bytes):
        pass

    def _teardown(self):
        pass

    # -- lifecycle ------------------------------------------------------------
    @property
    def is_open(self) -> bool:
        return self._open

    def _begin(self):
        self.parser = core.Parser()
        self.rx_bytes = 0
        self.rx_mav_v2 = self.rx_mav_v1 = False
        self._open = True
        self.hb.start()

    def _note_framing(self, data):
        """Record the wire protocol version of the first complete frame in `data` (0xFD=v2,
        0xFE=v1), reusing the proven frame_total scanner. Cheap: usually resolves at index 0,
        and short-circuits once both versions have been seen."""
        if self.rx_mav_v2 and self.rx_mav_v1:
            return
        n = len(data)
        i = 0
        while i < n:
            b = data[i]
            if (b == 0xFD or b == 0xFE) and mavlink.frame_total(data, i) is not None:
                if b == 0xFD:
                    self.rx_mav_v2 = True
                else:
                    self.rx_mav_v1 = True
                return
            i += 1

    @property
    def mavlink_version_str(self):
        """Human-readable negotiated MAVLink wire version, or None until a frame arrives."""
        if self.rx_mav_v2 and self.rx_mav_v1:
            return "2.0 (1.0 also seen)"
        if self.rx_mav_v2:
            return "2.0"
        if self.rx_mav_v1:
            return "1.0"
        return None

    def close(self):
        self.hb.stop()
        self._ack_timer.stop()
        self._pending.clear()
        self._teardown()
        self.remote = None
        self._open = False
        self.state.emit(False)

    def _record(self, data: bytes):
        r = self.recorder
        if r is not None:
            r.write(data, int(time.time() * 1e6))

    def _ingest(self, data: bytes):
        if not data:
            return
        self.rx_bytes += len(data)
        self._record(data)
        self._note_framing(data)
        batch = self.parser.feed(data)
        self._dispatch(batch)

    def _dispatch(self, batch):
        """Post-parse hook shared by EVERY transport. UDP/TCP/serial each parse in their own RX loop;
        routing all of them through here keeps the confirmed-command ACK matching alive on every
        transport, so it can't silently drift (it did on UDP -- ACKs were never matched, so every
        confirmed command was resent 4x and falsely warned on a healthy link)."""
        if not batch:
            return
        if self._pending:
            self._match_acks(batch)
        self.messages.emit(batch)

    # -- tx -------------------------------------------------------------------
    def _next_seq(self) -> int:
        s = self.seq
        self.seq = (self.seq + 1) & 0xFF
        return s

    def _send(self, data: bytes):
        if data and self._open and self.remote is not None:
            self._write(data)

    def _send_heartbeat(self):
        self._send(core.encode_heartbeat(self.gcs_sysid, self.gcs_compid, self._next_seq()))

    def _send_msg(self, msgid: int, payload: bytes):
        self._send(mavlink.frame(msgid, payload, self._next_seq(),
                                 self.gcs_sysid, self.gcs_compid, crc_fn=core.crc_extra))

    def _tx_command_long(self, target_sys, command, params):
        self._send(core.encode_command_long(self.gcs_sysid, self.gcs_compid, self._next_seq(),
                                            target_sys, 1, command, params))

    def send_command_long(self, target_sys: int, command: int, params, confirm=False):
        """Send a COMMAND_LONG. With confirm=True the command is resent until the vehicle ACKs it
        (up to ACK_MAX_TRIES), so a dropped frame on a lossy link doesn't silently fail; on failure
        command_unacked fires, on success command_result. Safe only for idempotent commands."""
        self._tx_command_long(target_sys, command, params)
        if confirm:
            self._track(command, lambda: self._tx_command_long(target_sys, command, params))

    def _track(self, command, resend):
        # Register a just-sent command as awaiting ACK. A later send of the SAME command id (e.g.
        # disarm after arm -- both COMPONENT_ARM_DISARM 400) supersedes the frame we resend, but we
        # bump `issued` and require that many ACKs before stopping: COMMAND_ACK doesn't echo which
        # invocation it answers, so a stale ACK for the first send must NOT cancel the second send's
        # retry (that could leave a dropped disarm un-retried while the GCS reports success).
        cmd = int(command)
        now = time.monotonic()
        p = self._pending.get(cmd)
        if p is None:
            self._pending[cmd] = {"resend": resend, "tries": 1, "deadline": now + self.ACK_INTERVAL,
                                  "issued": 1, "acked": 0}
        else:
            p["resend"] = resend          # the newest command's frame is what we now resend
            p["tries"] = 1
            p["deadline"] = now + self.ACK_INTERVAL
            p["issued"] += 1
        if not self._ack_timer.isActive():
            self._ack_timer.start()

    def _match_acks(self, batch):
        for m in batch:
            if m.msgid == mavlink.COMMAND_ACK:
                cmd = m.fields.get("command")
                p = self._pending.get(cmd)
                if p is not None:
                    p["acked"] += 1
                    if p["acked"] >= p["issued"]:      # every issued invocation acknowledged
                        del self._pending[cmd]
                        self.command_result.emit(int(cmd), int(m.fields.get("result", 0)))
        if not self._pending:
            self._ack_timer.stop()

    def _check_acks(self, now=None):
        """Resend commands still awaiting a COMMAND_ACK; give up (and signal) after ACK_MAX_TRIES."""
        if now is None:
            now = time.monotonic()
        for command in list(self._pending):
            p = self._pending.get(command)      # a signal handler below may have mutated _pending
            if p is None:
                continue
            if now < p["deadline"]:
                continue
            if p["tries"] >= self.ACK_MAX_TRIES:
                del self._pending[command]
                self.command_unacked.emit(int(command))
            else:
                p["resend"]()
                p["tries"] += 1
                p["deadline"] = now + self.ACK_INTERVAL
        if not self._pending:
            self._ack_timer.stop()

    def _tx_command_int(self, target_sys, command, params4, x, y, z, frame):
        # frame 6 = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT; x/y are lat/lon * 1e7 (int, precise)
        self._send(core.encode_command_int(self.gcs_sysid, self.gcs_compid, self._next_seq(),
                                           target_sys, 1, frame, command, params4, x, y, z))

    def send_command_int(self, target_sys, command, params4, x, y, z, frame=6, confirm=False):
        """Send a COMMAND_INT; confirm=True resends until ACKed (see send_command_long)."""
        self._tx_command_int(target_sys, command, params4, x, y, z, frame)
        if confirm:
            self._track(command,
                        lambda: self._tx_command_int(target_sys, command, params4, x, y, z, frame))

    def orbit(self, target_sys, lat, lon, radius, alt):
        self.send_command_int(target_sys, mavlink.MAV_CMD_DO_ORBIT,
                              [radius, float("nan"), 0, float("nan")],
                              int(lat * 1e7), int(lon * 1e7), alt)

    def set_roi(self, target_sys, lat, lon, alt):
        self.send_command_int(target_sys, mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
                              [0, 0, 0, 0], int(lat * 1e7), int(lon * 1e7), alt)

    def set_home(self, target_sys, lat, lon, alt_amsl):
        # z is the HOME ELEVATION in metres AMSL (frame 0 = GLOBAL). The old code sent the
        # vehicle's RELATIVE altitude in frame 6, corrupting the RTL/altitude reference by the
        # field elevation (home at "30 m AMSL" on a 500 m-elevation field).
        self.send_command_int(target_sys, mavlink.MAV_CMD_DO_SET_HOME,
                              [0, 0, 0, 0], int(lat * 1e7), int(lon * 1e7), alt_amsl, frame=0)

    # -- commands -------------------------------------------------------------
    def arm(self, target_sys: int, arm: bool = True):
        # confirm=True: arm/disarm is safety-critical -- resend until ACKed so a dropped frame
        # doesn't leave the operator thinking it armed (or disarmed) when it didn't.
        self.send_command_long(target_sys, mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                               [1.0 if arm else 0.0, 0, 0, 0, 0, 0, 0], confirm=True)

    def force_disarm(self, target_sys: int):
        # param2 = 21196 is the MAVLink force magic: disarm even with motors spinning
        # (emergency motor kill). A plain disarm (param2=0) is refused mid-flight -- PX4-verified.
        self.send_command_long(target_sys, mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                               [0.0, 21196.0, 0, 0, 0, 0, 0], confirm=True)

    def set_mode(self, target_sys: int, custom_mode, sub_mode=0):
        # ArduPilot: custom_mode is the mode number (sub_mode 0). PX4: custom_mode is the
        # main mode and sub_mode the sub mode -- both ride in DO_SET_MODE param2/param3.
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_SET_MODE,
                               [mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode, sub_mode, 0, 0, 0, 0],
                               confirm=True)

    def takeoff(self, target_sys: int, alt: float, lat: float = 0.0, lon: float = 0.0):
        # COMMAND_INT + GLOBAL_RELATIVE_ALT so `alt` is metres above the launch point.
        # A COMMAND_LONG NAV_TAKEOFF carries absolute AMSL, which PX4 reads as below
        # ground and refuses to climb -- verified against real PX4 SITL.
        self.send_command_int(target_sys, mavlink.MAV_CMD_NAV_TAKEOFF, [0, 0, 0, 0],
                              int(lat * 1e7), int(lon * 1e7), alt, frame=6, confirm=True)

    def land(self, target_sys: int):
        self.send_command_long(target_sys, mavlink.MAV_CMD_NAV_LAND, [0, 0, 0, 0, 0, 0, 0], confirm=True)

    def rtl(self, target_sys: int):
        self.send_command_long(target_sys, mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, [0] * 7, confirm=True)

    def pause(self, target_sys: int, cont: bool = False):
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_PAUSE_CONTINUE,
                               [1 if cont else 0, 0, 0, 0, 0, 0, 0])

    def change_altitude(self, target_sys: int, lat: float, lon: float, alt: float):
        # DO_REPOSITION to the current lat/lon at a new altitude above home. param2=1
        # (MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) tells PX4 to switch into guided
        # reposition so it actually flies to the new altitude in place.
        self.send_command_int(target_sys, mavlink.MAV_CMD_DO_REPOSITION,
                              [-1.0, 1.0, 0.0, float("nan")],
                              int(lat * 1e7), int(lon * 1e7), alt, frame=6)

    def change_speed(self, target_sys: int, speed: float, speed_type: int = 1):
        # DO_CHANGE_SPEED: type 1 = ground speed, param2 = m/s, param3 = -1 (no throttle change).
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_CHANGE_SPEED,
                               [float(speed_type), float(speed), -1.0, 0, 0, 0, 0])

    def reboot_vehicle(self, target_sys: int):
        """PREFLIGHT_REBOOT_SHUTDOWN with param1=1: reboot the autopilot -- e.g. to apply parameters
        that need a restart. Firmware refuses it while armed, which is the safety we want."""
        self.send_command_long(target_sys, mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                               [1, 0, 0, 0, 0, 0, 0])

    def deploy_parachute(self, target_sys: int):
        """DO_PARACHUTE with action RELEASE (2): deploy the parachute NOW -- emergency recovery.
        Irreversible in flight; callers must confirm first."""
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_PARACHUTE,
                               [mavlink.PARACHUTE_RELEASE, 0, 0, 0, 0, 0, 0])

    def gripper(self, target_sys: int, action: int, instance: int = 1):
        """DO_GRIPPER: release (0) or grab (1) a payload gripper -- delivery-drone payload drop."""
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_GRIPPER,
                               [float(instance), float(action), 0, 0, 0, 0, 0])

    def winch(self, target_sys: int, action: int, length: float = 0.0, rate: float = 1.0,
              instance: int = 1):
        """DO_WINCH: pay out / reel in a payload winch. action 1 = length control (length metres,
        + lowers), rate m/s; action 0 = relax (free spool)."""
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_WINCH,
                               [float(instance), float(action), float(length), float(rate), 0, 0, 0])

    def change_heading(self, target_sys: int, lat: float, lon: float, alt: float, heading_deg: float):
        # DO_REPOSITION holding the current position + altitude, param4 = yaw heading (deg) -> point
        # the nose to `heading_deg`. Same primitive as change_altitude (which passes yaw = NaN to keep
        # heading); param2 = 1 (CHANGE_MODE) switches PX4 into guided reposition.
        self.send_command_int(target_sys, mavlink.MAV_CMD_DO_REPOSITION,
                              [-1.0, 1.0, 0.0, float(heading_deg) % 360.0],
                              int(lat * 1e7), int(lon * 1e7), alt, frame=6)

    def vtol_transition(self, target_sys: int, state: int):
        # DO_VTOL_TRANSITION: param1 = MAV_VTOL_STATE (3 = multicopter, 4 = fixed-wing).
        # param2 = 0 -> normal (non-immediate) transition.
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_VTOL_TRANSITION,
                               [float(state), 0, 0, 0, 0, 0, 0])

    def request_data_streams(self, target_sys: int, autopilot: int = 0):
        """Ask the vehicle to stream telemetry. Real autopilots (ArduPilot especially)
        send almost nothing until requested, so this is what makes a freshly-connected
        drone actually show data. Sends the legacy REQUEST_DATA_STREAM (ArduPilot) and,
        for PX4, the modern SET_MESSAGE_INTERVAL for the key messages."""
        for stream_id, hz in ((mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2),  # SYS_STATUS/GPS/batt
                              (mavlink.MAV_DATA_STREAM_POSITION, 3),         # GLOBAL_POSITION_INT
                              (mavlink.MAV_DATA_STREAM_EXTRA1, 10),          # ATTITUDE
                              (mavlink.MAV_DATA_STREAM_EXTRA2, 5),           # VFR_HUD
                              (mavlink.MAV_DATA_STREAM_EXTRA3, 2),
                              (mavlink.MAV_DATA_STREAM_RC_CHANNELS, 3),
                              (mavlink.MAV_DATA_STREAM_RAW_SENSORS, 2)):
            self._send_msg(mavlink.REQUEST_DATA_STREAM,
                           mavlink.enc_request_data_stream(target_sys, stream_id, hz, 1))
        if int(autopilot) == mavlink.MAV_AUTOPILOT_PX4:
            for msgid, hz in ((mavlink.ATTITUDE, 20), (mavlink.GLOBAL_POSITION_INT, 5),
                              (mavlink.VFR_HUD, 5), (mavlink.SYS_STATUS, 2),
                              (mavlink.GPS_RAW_INT, 2), (mavlink.ALTITUDE, 5)):
                self.send_command_long(target_sys, mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                       [float(msgid), 1_000_000.0 / hz, 0, 0, 0, 0, 0])
        # one-shot: firmware version + capabilities (both autopilots answer CMD 512)
        self.send_command_long(target_sys, mavlink.MAV_CMD_REQUEST_MESSAGE,
                               [float(mavlink.AUTOPILOT_VERSION), 0, 0, 0, 0, 0, 0])

    def send_manual_control(self, target_sys, x, y, z, r, buttons=0):
        self._send_msg(mavlink.MANUAL_CONTROL,
                       mavlink.enc_manual_control(target_sys, x, y, z, r, buttons))

    # -- camera + gimbal ------------------------------------------------------
    def trigger_camera(self, target_sys):
        self.send_command_long(target_sys, mavlink.MAV_CMD_IMAGE_START_CAPTURE,
                               [0, 0, 1, 0, 0, 0, 0])      # interval 0, count 1

    def set_trigger_distance(self, target_sys, metres):
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_SET_CAM_TRIGG_DIST,
                               [float(metres), 0, 0, 0, 0, 0, 0])

    def video_capture(self, target_sys, start):
        cmd = (mavlink.MAV_CMD_VIDEO_START_CAPTURE if start
               else mavlink.MAV_CMD_VIDEO_STOP_CAPTURE)
        self.send_command_long(target_sys, cmd, [0, 0, 0, 0, 0, 0, 0])

    def set_camera_mode(self, target_sys, mode):
        """Switch the camera between photo (0) and video (1) capture mode (SET_CAMERA_MODE)."""
        self.send_command_long(target_sys, mavlink.MAV_CMD_SET_CAMERA_MODE,
                               [0, float(mode), 0, 0, 0, 0, 0])

    def camera_zoom(self, target_sys, step):
        """Step the camera zoom in (+1) or out (-1) by one increment (SET_CAMERA_ZOOM, step type)."""
        self.send_command_long(target_sys, mavlink.MAV_CMD_SET_CAMERA_ZOOM,
                               [float(mavlink.ZOOM_TYPE_STEP), float(step), 0, 0, 0, 0, 0])

    # -- motor test (Vehicle Setup > Motors) ----------------------------------
    def motor_test(self, target_sys, motor, throttle_pct, duration_s, count=0):
        """Spin motor(s) at a throttle % to verify motor order/direction (props OFF).
        motor = 1-based motor number; count = 0 tests just that motor, count > 0 tests `count`
        motors in sequence starting from `motor`. Throttle is always sent as a percent (0..100)."""
        order = (mavlink.MOTOR_TEST_ORDER_SEQUENCE if count > 0
                 else mavlink.MOTOR_TEST_ORDER_DEFAULT)
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_MOTOR_TEST,
                               [float(int(motor)), float(mavlink.MOTOR_TEST_THROTTLE_PERCENT),
                                float(throttle_pct), float(duration_s), float(int(count)),
                                float(order), 0.0])

    def set_gimbal(self, target_sys, pitch_deg, yaw_deg):
        # DO_MOUNT_CONTROL (gimbal v1): param1=pitch, param2=roll, param3=yaw, param7=mode
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                               [float(pitch_deg), 0.0, float(yaw_deg), 0, 0, 0,
                                mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING])

    def fence_enable(self, target_sys, enable: bool):
        # DO_FENCE_ENABLE: param1 = 1 enable / 0 disable the geofence at runtime (ArduPilot; PX4
        # enforces the fence via GF_* params rather than this command).
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_FENCE_ENABLE,
                               [1.0 if enable else 0.0, 0, 0, 0, 0, 0, 0])

    def set_gimbal_v2(self, target_sys, pitch_deg, yaw_deg, gimbal_id=0):
        # DO_GIMBAL_MANAGER_PITCHYAW (gimbal v2 manager -- the protocol QGC uses for modern gimbals):
        # p1=pitch deg, p2=yaw deg, p3=pitch rate (NaN = command the angle, not a rate), p4=yaw rate
        # (NaN), p5=gimbal manager flags (0 = default), p7=gimbal device id (0 = all / primary).
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                               [float(pitch_deg), float(yaw_deg), float("nan"), float("nan"),
                                0.0, 0.0, float(int(gimbal_id))])

    def accel_cal_position(self, target_sys, position):
        # ACCELCAL_VEHICLE_POS: advance ArduPilot's interactive 6-position accel calibration.
        # param1 = position (1 level, 2 left, 3 right, 4 nose-down, 5 nose-up, 6 back), sent after
        # the vehicle prompts for each orientation via STATUSTEXT.
        self.send_command_long(target_sys, mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS,
                               [float(int(position)), 0, 0, 0, 0, 0, 0])

    def start_mag_cal(self, target_sys, retry=True, autosave=True):
        # DO_START_MAG_CAL: begin ArduPilot's onboard compass calibration. p1 = mag mask (0 = all
        # compasses), p2 = retry on failure, p3 = autosave on success, p4 = delay s, p5 = autoreboot.
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_START_MAG_CAL,
                               [0.0, 1.0 if retry else 0.0, 1.0 if autosave else 0.0, 0.0, 0.0, 0, 0])

    def accept_mag_cal(self, target_sys):
        # DO_ACCEPT_MAG_CAL: accept + persist the computed offsets (p1 = mag mask, 0 = all).
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL, [0, 0, 0, 0, 0, 0, 0])

    def cancel_mag_cal(self, target_sys):
        # DO_CANCEL_MAG_CAL: abort an in-progress compass cal (p1 = mag mask, 0 = all).
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_CANCEL_MAG_CAL, [0, 0, 0, 0, 0, 0, 0])

    RTCM_FRAG_LEN = 180                     # GPS_RTCM_DATA data[] size; also the per-fragment max
    RTCM_MAX_FRAGS = 4                       # fragment id is 2 bits -> at most 4 fragments (720 bytes)

    def inject_rtcm(self, data: bytes) -> int:
        """Inject an RTCM3 correction message (from an NTRIP caster or a base-station receiver) to the
        vehicle as GPS_RTCM_DATA(233), fragmenting exactly like QGC: a message shorter than 180 bytes
        goes in a single unfragmented frame; a longer one is split into up to 4 fragments (the protocol
        cap -- messages over 720 bytes are truncated, logged below). The low flag bit marks a fragmented
        stream, bits 1-2 carry the fragment id, and bits 3-7 the per-message sequence id. Returns the
        number of GPS_RTCM_DATA frames sent."""
        data = bytes(data or b"")
        if not data:
            return 0
        seq = self._rtcm_seq & 0x1F
        sent = 0
        if len(data) < self.RTCM_FRAG_LEN:
            self._send_rtcm(seq << 3, data)          # single, unfragmented (LSB clear)
            sent = 1
        else:
            frags = [data[i:i + self.RTCM_FRAG_LEN]
                     for i in range(0, len(data), self.RTCM_FRAG_LEN)]
            if len(frags) > self.RTCM_MAX_FRAGS:
                dropped = sum(len(f) for f in frags[self.RTCM_MAX_FRAGS:])
                frags = frags[:self.RTCM_MAX_FRAGS]
                print(f"[rtcm] message {len(data)} B exceeds {self.RTCM_MAX_FRAGS}-fragment cap; "
                      f"dropped last {dropped} B")
            for fid, chunk in enumerate(frags):
                self._send_rtcm(1 | (fid << 1) | (seq << 3), chunk)   # LSB set = fragmented
                sent += 1
        self._rtcm_seq = (self._rtcm_seq + 1) & 0x1F  # one sequence id per whole RTCM message
        return sent

    def _send_rtcm(self, flags: int, chunk: bytes):
        # GPS_RTCM_DATA payload: flags(u8), len(u8), data(u8[180], zero-padded).
        payload = bytes((flags & 0xFF, len(chunk) & 0xFF)) + chunk + b"\x00" * (self.RTCM_FRAG_LEN - len(chunk))
        self._send_msg(mavlink.GPS_RTCM_DATA, payload)

    def send_ftp(self, target_sys, seq, session, opcode, offset=0, data=b"", target_comp=1):
        """Send one MAVLink FTP request packet inside FILE_TRANSFER_PROTOCOL (target_network 0). Returns
        the encoded FTP packet so a caller/client can track the seq it just issued."""
        pkt = ftp.encode(seq, session, opcode, offset, data)
        self._send_msg(mavlink.FILE_TRANSFER_PROTOCOL,
                       bytes((0, target_sys & 0xFF, target_comp & 0xFF)) + pkt)
        return pkt

    def terrain_check(self, lat, lon):
        """Ask the vehicle to report its terrain status at (lat, lon degrees) via TERRAIN_CHECK; the
        vehicle answers with TERRAIN_REPORT (height above terrain + tiles pending/loaded)."""
        self._send_msg(mavlink.TERRAIN_CHECK, struct.pack("<ii", int(lat * 1e7), int(lon * 1e7)))

    def calibrate(self, target_sys, kind):
        """kind: 'gyro' | 'accel' | 'level' | 'compass' -> MAV_CMD_PREFLIGHT_CALIBRATION."""
        p = [0, 0, 0, 0, 0, 0, 0]
        if kind == "gyro":
            p[0] = 1
        elif kind == "compass":
            p[1] = 1
        elif kind == "accel":
            p[4] = 1
        elif kind == "level":
            p[4] = 2
        self.send_command_long(target_sys, mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, p)

    # -- onboard log download -------------------------------------------------
    def request_log_list(self, target_sys, start=0, end=0xFFFF):
        self._send_msg(mavlink.LOG_REQUEST_LIST,
                       mavlink.enc_log_request_list(start, end, target_sys, 1))

    def request_log_data(self, target_sys, log_id, ofs=0, count=0xFFFFFFFF):
        self._send_msg(mavlink.LOG_REQUEST_DATA,
                       mavlink.enc_log_request_data(log_id, ofs, count, target_sys, 1))

    def log_request_end(self, target_sys):
        self._send_msg(mavlink.LOG_REQUEST_END, mavlink.enc_log_request_end(target_sys, 1))

    def goto(self, target_sys: int, lat: float, lon: float, alt_rel: float):
        # One-shot guided position target: honored by ArduPilot in GUIDED mode. PX4 IGNORES this
        # outside an OFFBOARD setpoint stream -- use reposition() for PX4 (main._guided_goto picks).
        self._send_msg(mavlink.SET_POSITION_TARGET_GLOBAL_INT,
                       mavlink.enc_set_position_target_global_int(lat, lon, alt_rel,
                                                                  target_system=target_sys))

    def reposition(self, target_sys: int, lat: float, lon: float, alt_rel: float):
        # Guided "fly to" via DO_REPOSITION + MAV_DO_REPOSITION_FLAGS_CHANGE_MODE (param2=1) --
        # the primitive PX4 actually honors from Hold/Position (same one the PX4-verified
        # change_altitude uses); param1=-1 default speed, yaw NaN = unchanged.
        self.send_command_int(target_sys, mavlink.MAV_CMD_DO_REPOSITION,
                              [-1.0, 1.0, 0.0, float("nan")],
                              int(lat * 1e7), int(lon * 1e7), alt_rel, frame=6)

    # -- parameter protocol ---------------------------------------------------
    def request_params(self, target_sys: int):
        self._send_msg(mavlink.PARAM_REQUEST_LIST, mavlink.enc_param_request_list(target_sys))

    def request_param_read(self, target_sys: int, param_id: str = "", index: int = -1):
        self._send_msg(mavlink.PARAM_REQUEST_READ,
                       mavlink.enc_param_request_read(param_id, index, target_sys))

    def set_param(self, target_sys: int, param_id: str, value: float,
                  ptype: int = mavlink.MAV_PARAM_TYPE_REAL32, bytewise: bool = True):
        # bytewise: integer-param encoding -- True for PX4 (union bits), False for ArduPilot /
        # spec-default C-cast. Callers pass mavlink.param_bytewise(vehicle.autopilot).
        self._send_msg(mavlink.PARAM_SET,
                       mavlink.enc_param_set(param_id, value, ptype, target_sys, bytewise=bytewise))

    def send_serial_control(self, data=b"", device=mavlink.SERIAL_CONTROL_DEV_SHELL,
                            flags=(mavlink.SERIAL_CONTROL_FLAG_RESPOND
                                   | mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
                                   | mavlink.SERIAL_CONTROL_FLAG_MULTI)):
        # Talk to the autopilot's nsh shell (PX4) over SERIAL_CONTROL. RESPOND asks
        # for output, MULTI keeps it streaming; the reply arrives as SERIAL_CONTROL.
        # Sent as MAVLink v2 -- PX4's shell only answers v2 (extension fields).
        payload = mavlink.enc_serial_control(device, flags, data)
        self._send(mavlink.frame_v2(mavlink.SERIAL_CONTROL, payload, self._next_seq(),
                                    self.gcs_sysid, self.gcs_compid, crc_fn=core.crc_extra))

    # -- mission protocol (mission_type: 0=mission, 1=fence, 2=rally) ----------
    def send_mission_count(self, target_sys: int, count: int, mission_type: int = 0):
        self._send_msg(mavlink.MISSION_COUNT,
                       mavlink.enc_mission_count(count, target_sys, mission_type=mission_type))

    def send_mission_item(self, target_sys: int, item, mission_type: int = 0):
        self._send_msg(mavlink.MISSION_ITEM_INT, mavlink.enc_mission_item_int(
            item.seq, item.lat, item.lon, item.alt, command=item.command, frame=item.frame,
            autocontinue=item.autocontinue, param1=item.param1, param2=item.param2,
            param3=item.param3, param4=item.param4, target_system=target_sys,
            mission_type=mission_type))

    def send_mission_request_list(self, target_sys: int, mission_type: int = 0):
        self._send_msg(mavlink.MISSION_REQUEST_LIST,
                       mavlink.enc_mission_request_list(target_sys, mission_type=mission_type))

    def send_mission_request_int(self, target_sys: int, seq: int, mission_type: int = 0):
        self._send_msg(mavlink.MISSION_REQUEST_INT,
                       mavlink.enc_mission_request_int(seq, target_sys, mission_type=mission_type))

    def set_current_wp(self, target_sys: int, seq: int):
        """MISSION_SET_CURRENT: make waypoint `seq` the active mission item -- skip ahead to it or
        restart the mission from it while flying (the vehicle echoes MISSION_CURRENT back)."""
        self._send_msg(mavlink.MISSION_SET_CURRENT,
                       mavlink.enc_mission_set_current(seq, target_sys))

    def send_mission_ack(self, target_sys: int, result: int = 0, mission_type: int = 0):
        self._send_msg(mavlink.MISSION_ACK,
                       mavlink.enc_mission_ack(result, target_sys, mission_type=mission_type))

    def send_mission_clear(self, target_sys: int, mission_type: int = 0):
        self._send_msg(mavlink.MISSION_CLEAR_ALL,
                       mavlink.enc_mission_clear_all(target_sys, mission_type=mission_type))


class UdpLink(Link):
    """GCS-standard: bind a local UDP port, learn the vehicle from its first packet."""

    def open(self, port=14550, bind_addr="0.0.0.0", **kw) -> bool:
        self.close()
        self.sock = QUdpSocket(self)
        if not self.sock.bind(QHostAddress(bind_addr), int(port)):
            self.info.emit(f"UDP bind failed on :{port} ({self.sock.errorString()})")
            self.sock.deleteLater()
            self.sock = None
            self.state.emit(False)
            return False
        self._begin()
        self.sock.readyRead.connect(self._on_ready)
        self.info.emit(f"listening for telemetry on UDP :{port}")
        self.state.emit(True)
        return True

    def _on_ready(self):
        batch = []
        while self.sock is not None and self.sock.hasPendingDatagrams():
            dg = self.sock.receiveDatagram()
            data = bytes(dg.data())
            self.rx_bytes += len(data)
            self._record(data)
            self._note_framing(data)
            addr, aport = dg.senderAddress(), dg.senderPort()
            if self.remote is None:
                self.remote = (addr, aport)
                self.info.emit(f"telemetry from {addr.toString()}:{aport}")
            elif self.remote[1] != aport or self.remote[0] != addr:
                # Re-target TX at the most recent sender (QGC does the same). A radio bridge or
                # SITL restart resumes telemetry from a NEW source port; keeping the old endpoint
                # means RX looks alive while every command + GCS heartbeat goes to a dead port
                # (the vehicle then declares GCS-loss). Info is throttled -- two interleaved
                # senders on one port would otherwise spam it every datagram.
                self.remote = (addr, aport)
                now = time.monotonic()
                if now - getattr(self, "_remote_move_t", 0.0) > 5.0:
                    self._remote_move_t = now
                    self.info.emit(f"telemetry endpoint moved to {addr.toString()}:{aport}")
            batch.extend(self.parser.feed(data))
        self._dispatch(batch)

    def _write(self, data: bytes):
        if self.sock is not None and self.remote is not None:
            self.sock.writeDatagram(data, self.remote[0], self.remote[1])

    def _teardown(self):
        if getattr(self, "sock", None) is not None:
            self.sock.close()
            self.sock.deleteLater()
            self.sock = None


class TcpLink(Link):
    """Connect to a TCP telemetry endpoint (e.g. SITL on 5760)."""

    def open(self, host="127.0.0.1", port=5760, **kw) -> bool:
        self.close()
        self.sock = QTcpSocket(self)
        self.sock.readyRead.connect(lambda: self._ingest(bytes(self.sock.readAll())))
        self.sock.connected.connect(self._on_connected)
        self.sock.disconnected.connect(self._on_dropped)
        self.sock.errorOccurred.connect(self._on_error)
        self._begin()
        self.info.emit(f"connecting TCP {host}:{port} ...")
        self.sock.connectToHost(str(host), int(port))
        self.state.emit(True)
        return True

    def _on_connected(self):
        self.remote = True
        self.info.emit("TCP connected")

    def _on_error(self, _e):
        # connection refused / host unreachable / remote closed: without this the UI shows the
        # link open forever while heartbeats are written into a dead socket.
        if not self._open:
            return
        msg = self.sock.errorString() if self.sock is not None else "socket error"
        self.info.emit(f"TCP error: {msg} -- link closed")
        self.close()

    def _on_dropped(self):
        if self._open:
            self.info.emit("TCP connection closed by remote -- link closed")
            self.close()

    def _write(self, data: bytes):
        if self.sock is not None:
            self.sock.write(data)

    def _teardown(self):
        if getattr(self, "sock", None) is not None:
            self.sock.close()
            self.sock.deleteLater()
            self.sock = None


class SerialLink(Link):
    """USB / SiK telemetry radio over a serial port."""

    def open(self, port="", baud=57600, **kw) -> bool:
        self.close()
        if not HAVE_SERIAL:
            self.info.emit("serial not available (install qt6-serialport)")
            self.state.emit(False)
            return False
        self.sp = QSerialPort(self)
        self.sp.setPortName(port)
        self.sp.setBaudRate(int(baud))
        if not self.sp.open(QIODeviceBase.OpenModeFlag.ReadWrite):
            self.info.emit(f"serial open failed: {port} ({self.sp.errorString()})")
            self.sp.deleteLater()
            self.sp = None
            self.state.emit(False)
            return False
        self._begin()
        self.sp.readyRead.connect(lambda: self._ingest(bytes(self.sp.readAll())))
        self.remote = True
        self.info.emit(f"serial {port} @ {baud}")
        self.state.emit(True)
        return True

    def _write(self, data: bytes):
        if self.sp is not None:
            self.sp.write(data)

    def _teardown(self):
        if getattr(self, "sp", None) is not None:
            self.sp.close()
            self.sp.deleteLater()
            self.sp = None

    @staticmethod
    def available_ports():
        if not HAVE_SERIAL:
            return []
        return [p.portName() for p in QSerialPortInfo.availablePorts()]


class ReplayLink(Link):
    """Play a recorded .tlog back through the GCS at its original cadence.

    Read-only (remote stays None, so commands are disabled); frames are fed to the
    parser on a timer scaled by `speed`."""

    def open(self, path="", speed=1.0, **kw) -> bool:
        self.close()
        from tlog import read_tlog, replay_schedule
        try:
            self._records = read_tlog(str(path))
        except OSError as e:
            self.info.emit(f"replay open failed: {e}")
            self.state.emit(False)
            return False
        if not self._records:
            self.info.emit(f"replay: no frames in {path}")
            self.state.emit(False)
            return False
        self.parser = core.Parser()
        self.rx_bytes = 0
        self._open = True
        self._speed = max(0.1, float(speed))
        self._i = 0
        # clamp non-monotonic / corrupt timestamps so a single bad frame can't stall playback
        self._sched = replay_schedule(self._records)
        self._wall0 = time.monotonic()
        self._tick = QTimer(self)
        self._tick.setInterval(20)
        self._tick.timeout.connect(self._pump)
        self._tick.start()
        self.info.emit(f"replaying {len(self._records)} frames from {path} @ {self._speed:g}x")
        self.state.emit(True)
        return True

    def _pump(self):
        elapsed_us = (time.monotonic() - self._wall0) * 1e6 * self._speed
        batch = []
        while self._i < len(self._records):
            _t_us, fr = self._records[self._i]
            if self._sched[self._i] > elapsed_us:
                break
            self.rx_bytes += len(fr)
            batch.extend(self.parser.feed(fr))
            self._i += 1
        self._dispatch(batch)
        if self._i >= len(self._records):
            self._tick.stop()
            self.info.emit("replay complete")

    def _teardown(self):
        t = getattr(self, "_tick", None)
        if t is not None:
            t.stop()
