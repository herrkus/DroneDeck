"""logdownload.py -- onboard (dataflash) log download over MAVLink.

Drives the LOG_REQUEST_LIST -> LOG_ENTRY -> LOG_REQUEST_DATA -> LOG_DATA ->
LOG_REQUEST_END handshake, writing the chosen log to a file with progress.
"""
from __future__ import annotations
import os

from PySide6.QtCore import QObject, Signal, QTimer

import mavlink

CHUNK = 90


class LogManager(QObject):
    entries = Signal(list)              # [{"id", "size", "time_utc"}, ...]
    progress = Signal(int, int)         # bytes_received, total
    finished = Signal(bool, str, str)   # ok, path, message

    def __init__(self, link_getter, target_getter, log_dir, parent=None):
        super().__init__(parent)
        self._link = link_getter
        self._target = target_getter
        self.log_dir = log_dir
        self.state = "idle"
        self._entries = {}
        self._num_logs = None
        self.dl_id = None
        self.dl_size = 0
        self.dl_got = 0
        self.fh = None
        self.path = ""
        self._list_timer = QTimer(self)
        self._list_timer.setSingleShot(True)
        self._list_timer.timeout.connect(self._finalize_list)

    def _ready(self):
        link = self._link()
        return link is not None and link.is_open and link.remote is not None

    # -- requests -------------------------------------------------------------
    def request_list(self):
        if not self._ready():
            self.finished.emit(False, "", "no vehicle connected")
            return
        self._entries = {}
        self._num_logs = None
        self.state = "listing"
        self._link().request_log_list(self._target())
        self._list_timer.start(1500)

    def download(self, log_id):
        if not self._ready():
            self.finished.emit(False, "", "no vehicle connected")
            return
        ent = self._entries.get(log_id)
        if ent is None:
            self.finished.emit(False, "", f"log {log_id} not in list")
            return
        os.makedirs(self.log_dir, exist_ok=True)
        self.path = os.path.join(self.log_dir, f"log_{log_id}.bin")
        try:
            self.fh = open(self.path, "wb")
        except OSError as e:
            self.finished.emit(False, "", f"cannot open {self.path}: {e}")
            return
        self.dl_id = log_id
        self.dl_size = int(ent["size"])
        self.dl_got = 0
        self.state = "downloading"
        self.progress.emit(0, self.dl_size)
        self._link().request_log_data(self._target(), log_id, 0, 0xFFFFFFFF)

    # -- inbound --------------------------------------------------------------
    def handle_messages(self, batch):
        if self.state == "idle":
            return
        for m in batch:
            self._handle(m)

    def _handle(self, m):
        if self.state == "listing" and m.msgid == mavlink.LOG_ENTRY:
            self._num_logs = int(m.fields.get("num_logs", 0))
            if self._num_logs == 0:
                self._list_timer.stop()
                self.state = "idle"
                self.entries.emit([])
                return
            lid = int(m.fields.get("id", 0))
            self._entries[lid] = {"id": lid, "size": int(m.fields.get("size", 0)),
                                  "time_utc": int(m.fields.get("time_utc", 0))}
            self._list_timer.start(800)             # finalize shortly after the last one
            if len(self._entries) >= self._num_logs:
                self._finalize_list()
        elif self.state == "downloading" and m.msgid == mavlink.LOG_DATA:
            if int(m.fields.get("id", -1)) != self.dl_id:
                return
            ofs = int(m.fields.get("ofs", 0))
            count = int(m.fields.get("count", 0))
            data = bytes(m.fields.get("data", b""))[:count]
            self.fh.seek(ofs)
            self.fh.write(data)
            self.dl_got = max(self.dl_got, ofs + count)
            self.progress.emit(min(self.dl_got, self.dl_size), self.dl_size)
            if count < CHUNK or self.dl_got >= self.dl_size:
                self._finish_download()

    def _finalize_list(self):
        if self.state != "listing":
            return
        self.state = "idle"
        self.entries.emit(sorted(self._entries.values(), key=lambda e: e["id"]))

    def _finish_download(self):
        try:
            self.fh.close()
        except Exception:
            pass
        self.fh = None
        self.state = "idle"
        if self._ready():
            self._link().log_request_end(self._target())
        self.finished.emit(True, self.path, f"saved {self.dl_got} bytes -> {os.path.basename(self.path)}")
