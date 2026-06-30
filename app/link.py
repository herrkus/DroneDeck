"""link.py -- MAVLink links over UDP, TCP and serial.

A common Link base owns the native parser, the 1 Hz GCS heartbeat and all
command framing; the transports (UdpLink / TcpLink / SerialLink) only move
bytes. Incoming bytes feed the parser and decoded Messages are emitted in
batches on the Qt main loop.
"""
from __future__ import annotations

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

    def _ingest(self, data: bytes):
        if not data:
            return
        self.rx_bytes += len(data)
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

    # -- commands -------------------------------------------------------------
    def arm(self, target_sys: int, arm: bool = True):
        self.send_command_long(target_sys, mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                               [1.0 if arm else 0.0, 0, 0, 0, 0, 0, 0])

    def set_mode(self, target_sys: int, custom_mode: int):
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_SET_MODE,
                               [mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode, 0, 0, 0, 0, 0])

    def takeoff(self, target_sys: int, alt: float):
        self.send_command_long(target_sys, mavlink.MAV_CMD_NAV_TAKEOFF, [0, 0, 0, 0, 0, 0, alt])

    def land(self, target_sys: int):
        self.send_command_long(target_sys, mavlink.MAV_CMD_NAV_LAND, [0, 0, 0, 0, 0, 0, 0])

    def rtl(self, target_sys: int):
        self.send_command_long(target_sys, mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, [0] * 7)

    def pause(self, target_sys: int, cont: bool = False):
        self.send_command_long(target_sys, mavlink.MAV_CMD_DO_PAUSE_CONTINUE,
                               [1 if cont else 0, 0, 0, 0, 0, 0, 0])

    def goto(self, target_sys: int, lat: float, lon: float, alt_rel: float):
        self._send_msg(mavlink.SET_POSITION_TARGET_GLOBAL_INT,
                       mavlink.enc_set_position_target_global_int(lat, lon, alt_rel,
                                                                  target_system=target_sys))


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
