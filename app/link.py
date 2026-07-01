"""link.py -- MAVLink links over UDP, TCP and serial.

A common Link base owns the native parser, the 1 Hz GCS heartbeat and all
command framing; the transports (UdpLink / TcpLink / SerialLink) only move
bytes. Incoming bytes feed the parser and decoded Messages are emitted in
batches on the Qt main loop.
"""
from __future__ import annotations

import time

from PySide6.QtCore import QObject, Signal, QTimer, QIODeviceBase
from PySide6.QtNetwork import QUdpSocket, QTcpSocket, QHostAddress

import core
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

    def __init__(self, gcs_sysid=255, gcs_compid=0):
        super().__init__()
        self.parser: core.Parser | None = None
        self.remote = None         # truthy once a peer is known (can send)
        self.gcs_sysid = gcs_sysid
        self.gcs_compid = gcs_compid
        self.seq = 0
        self.rx_bytes = 0
        self._open = False
        self.recorder = None       # TlogWriter while recording, else None
        self.hb = QTimer(self)
        self.hb.setInterval(1000)
        self.hb.timeout.connect(self._send_heartbeat)

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
        self._open = True
        self.hb.start()

    def close(self):
        self.hb.stop()
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
        batch = self.parser.feed(data)
        if batch:
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

    def send_command_long(self, target_sys: int, command: int, params):
        self._send(core.encode_command_long(self.gcs_sysid, self.gcs_compid, self._next_seq(),
                                            target_sys, 1, command, params))

    def send_command_int(self, target_sys, command, params4, x, y, z, frame=6):
        # frame 6 = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT; x/y are lat/lon * 1e7 (int, precise)
        self._send(core.encode_command_int(self.gcs_sysid, self.gcs_compid, self._next_seq(),
                                           target_sys, 1, frame, command, params4, x, y, z))

    def orbit(self, target_sys, lat, lon, radius, alt):
        self.send_command_int(target_sys, mavlink.MAV_CMD_DO_ORBIT,
                              [radius, float("nan"), 0, float("nan")],
                              int(lat * 1e7), int(lon * 1e7), alt)

    def set_roi(self, target_sys, lat, lon, alt):
        self.send_command_int(target_sys, mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
                              [0, 0, 0, 0], int(lat * 1e7), int(lon * 1e7), alt)

    def set_home(self, target_sys, lat, lon, alt):
        self.send_command_int(target_sys, mavlink.MAV_CMD_DO_SET_HOME,
                              [0, 0, 0, 0], int(lat * 1e7), int(lon * 1e7), alt)

    # -- commands -------------------------------------------------------------
    def arm(self, target_sys: int, arm: bool = True):
        self.send_command_long(target_sys, mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                               [1.0 if arm else 0.0, 0, 0, 0, 0, 0, 0])

    def set_mode(self, target_sys: int, custom_mode: int):
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_SET_MODE,
                               [mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode, 0, 0, 0, 0, 0])

    def takeoff(self, target_sys: int, alt: float, lat: float = 0.0, lon: float = 0.0):
        # COMMAND_INT + GLOBAL_RELATIVE_ALT so `alt` is metres above the launch point.
        # A COMMAND_LONG NAV_TAKEOFF carries absolute AMSL, which PX4 reads as below
        # ground and refuses to climb -- verified against real PX4 SITL.
        self.send_command_int(target_sys, mavlink.MAV_CMD_NAV_TAKEOFF, [0, 0, 0, 0],
                              int(lat * 1e7), int(lon * 1e7), alt, frame=6)

    def land(self, target_sys: int):
        self.send_command_long(target_sys, mavlink.MAV_CMD_NAV_LAND, [0, 0, 0, 0, 0, 0, 0])

    def rtl(self, target_sys: int):
        self.send_command_long(target_sys, mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, [0] * 7)

    def pause(self, target_sys: int, cont: bool = False):
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_PAUSE_CONTINUE,
                               [1 if cont else 0, 0, 0, 0, 0, 0, 0])

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

    def set_gimbal(self, target_sys, pitch_deg, yaw_deg):
        # DO_MOUNT_CONTROL: param1=pitch, param2=roll, param3=yaw, param7=mode
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                               [float(pitch_deg), 0.0, float(yaw_deg), 0, 0, 0,
                                mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING])

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
        self._send_msg(mavlink.SET_POSITION_TARGET_GLOBAL_INT,
                       mavlink.enc_set_position_target_global_int(lat, lon, alt_rel,
                                                                  target_system=target_sys))

    # -- parameter protocol ---------------------------------------------------
    def request_params(self, target_sys: int):
        self._send_msg(mavlink.PARAM_REQUEST_LIST, mavlink.enc_param_request_list(target_sys))

    def request_param_read(self, target_sys: int, param_id: str = "", index: int = -1):
        self._send_msg(mavlink.PARAM_REQUEST_READ,
                       mavlink.enc_param_request_read(param_id, index, target_sys))

    def set_param(self, target_sys: int, param_id: str, value: float,
                  ptype: int = mavlink.MAV_PARAM_TYPE_REAL32):
        self._send_msg(mavlink.PARAM_SET, mavlink.enc_param_set(param_id, value, ptype, target_sys))

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
            if self.remote is None:
                self.remote = (dg.senderAddress(), dg.senderPort())
                self.info.emit(f"telemetry from {dg.senderAddress().toString()}:{dg.senderPort()}")
            batch.extend(self.parser.feed(data))
        if batch:
            self.messages.emit(batch)

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
        self.sock.errorOccurred.connect(
            lambda _e: self.info.emit(f"TCP error: {self.sock.errorString()}"))
        self._begin()
        self.info.emit(f"connecting TCP {host}:{port} ...")
        self.sock.connectToHost(str(host), int(port))
        self.state.emit(True)
        return True

    def _on_connected(self):
        self.remote = True
        self.info.emit("TCP connected")

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
        from tlog import read_tlog
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
        self._t0 = self._records[0][0]
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
            t_us, fr = self._records[self._i]
            if (t_us - self._t0) > elapsed_us:
                break
            self.rx_bytes += len(fr)
            batch.extend(self.parser.feed(fr))
            self._i += 1
        if batch:
            self.messages.emit(batch)
        if self._i >= len(self._records):
            self._tick.stop()
            self.info.emit("replay complete")

    def _teardown(self):
        t = getattr(self, "_tick", None)
        if t is not None:
            t.stop()
