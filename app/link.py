"""link.py -- MAVLink-over-UDP link.

Event-driven on the Qt main loop (QUdpSocket.readyRead): incoming datagrams are
fed straight into the native parser, decoded Messages are emitted in a batch.
The remote endpoint is learned from the first packet, and a 1 Hz GCS heartbeat
plus on-demand commands are sent back to it.
"""
from __future__ import annotations

from PySide6.QtCore import QObject, Signal, QTimer
from PySide6.QtNetwork import QUdpSocket, QHostAddress

import core
import mavlink


class UdpLink(QObject):
    messages = Signal(list)        # list[core.Message]
    state = Signal(bool)           # bound / unbound
    info = Signal(str)             # human-readable status line

    def __init__(self, gcs_sysid=255, gcs_compid=0):
        super().__init__()
        self.sock: QUdpSocket | None = None
        self.parser: core.Parser | None = None
        self.remote = None         # (QHostAddress, port)
        self.gcs_sysid = gcs_sysid
        self.gcs_compid = gcs_compid
        self.seq = 0
        self.rx_bytes = 0
        self.hb = QTimer(self)
        self.hb.setInterval(1000)
        self.hb.timeout.connect(self._send_heartbeat)

    # -- lifecycle ------------------------------------------------------------
    def open(self, port=14550, bind_addr="0.0.0.0") -> bool:
        self.close()
        self.parser = core.Parser()
        self.sock = QUdpSocket(self)
        if not self.sock.bind(QHostAddress(bind_addr), int(port)):
            self.info.emit(f"bind failed on :{port} ({self.sock.errorString()})")
            self.sock = None
            self.state.emit(False)
            return False
        self.sock.readyRead.connect(self._on_ready)
        self.hb.start()
        self.info.emit(f"listening for telemetry on UDP :{port}")
        self.state.emit(True)
        return True

    def close(self):
        self.hb.stop()
        if self.sock is not None:
            self.sock.close()
            self.sock.deleteLater()
            self.sock = None
        self.remote = None
        self.state.emit(False)

    @property
    def is_open(self) -> bool:
        return self.sock is not None

    # -- rx -------------------------------------------------------------------
    def _on_ready(self):
        batch = []
        while self.sock is not None and self.sock.hasPendingDatagrams():
            dg = self.sock.receiveDatagram()
            data = bytes(dg.data())
            self.rx_bytes += len(data)
            if self.remote is None:
                self.remote = (dg.senderAddress(), dg.senderPort())
                self.info.emit(
                    f"telemetry from {dg.senderAddress().toString()}:{dg.senderPort()}")
            batch.extend(self.parser.feed(data))
        if batch:
            self.messages.emit(batch)

    # -- tx -------------------------------------------------------------------
    def _next_seq(self) -> int:
        s = self.seq
        self.seq = (self.seq + 1) & 0xFF
        return s

    def _send(self, data: bytes):
        if self.sock is not None and self.remote is not None and data:
            self.sock.writeDatagram(data, self.remote[0], self.remote[1])

    def _send_heartbeat(self):
        self._send(core.encode_heartbeat(self.gcs_sysid, self.gcs_compid, self._next_seq()))

    def send_command_long(self, target_sys: int, command: int, params):
        self._send(core.encode_command_long(
            self.gcs_sysid, self.gcs_compid, self._next_seq(),
            target_sys, 1, command, params))

    def arm(self, target_sys: int, arm: bool = True):
        self.send_command_long(target_sys, mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                               [1.0 if arm else 0.0, 0, 0, 0, 0, 0, 0])
