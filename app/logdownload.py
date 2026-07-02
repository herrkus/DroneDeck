"""logdownload.py -- onboard (dataflash) log download over MAVLink.

Drives the LOG_REQUEST_LIST -> LOG_ENTRY -> LOG_REQUEST_DATA -> LOG_DATA ->
LOG_REQUEST_END handshake, writing the chosen log to a file with progress.
"""
from __future__ import annotations
import os

from PySide6.QtCore import QObject, Signal, QTimer

import mavlink

CHUNK = 90
DL_TIMEOUT_MS = 1500
DL_MAX_RETRIES = 8


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
        self.dl_got = 0        # highest contiguous byte received from 0 (the true progress)
        self.dl_retries = 0
        self.fh = None
        self.path = ""
        self._list_timer = QTimer(self)
        self._list_timer.setSingleShot(True)
        self._list_timer.timeout.connect(self._finalize_list)
        self._dl_timer = QTimer(self)
        self._dl_timer.setSingleShot(True)
        self._dl_timer.timeout.connect(self._on_dl_timeout)

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
        if self.state == "downloading":
            # busy guard: abandoning a download without closing leaked the file handle and
            # corrupted the first file. Cleanly cancel the in-flight one first.
            self._close_fh()
            self._dl_timer.stop()
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
        self.dl_retries = 0
        self.state = "downloading"
        self.progress.emit(0, self.dl_size)
        self._link().request_log_data(self._target(), log_id, 0, 0xFFFFFFFF)
        self._dl_timer.start(DL_TIMEOUT_MS)

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
            if self.fh is not None:
                self.fh.seek(ofs)
                self.fh.write(data)
            # advance the contiguous-from-zero high-water mark. A chunk that lands past dl_got is
            # written but does NOT advance it -- the gap before it is re-requested on timeout, so a
            # dropped mid-stream chunk can't leave a silent zero-filled hole in the log.
            if ofs <= self.dl_got:
                self.dl_got = max(self.dl_got, ofs + count)
            self.dl_retries = 0
            self.progress.emit(min(self.dl_got, self.dl_size), self.dl_size)
            # done only when the whole file is contiguously present (a short final chunk with no gap
            # also completes it); a short chunk sitting AFTER a gap must NOT end the download.
            if self.dl_got >= self.dl_size or (count < CHUNK and ofs + count >= self.dl_size):
                self._finish_download(True)
            else:
                self._dl_timer.start(DL_TIMEOUT_MS)

    def _on_dl_timeout(self):
        if self.state != "downloading":
            return
        self.dl_retries += 1
        if self.dl_retries > DL_MAX_RETRIES or not self._ready():
            self._finish_download(False)
            return
        # re-request from the first missing byte (the contiguous high-water mark), filling the gap
        # forward instead of hanging with the file open forever when a chunk (or the last one) drops.
        self._link().request_log_data(self._target(), self.dl_id, self.dl_got, 0xFFFFFFFF)
        self._dl_timer.start(DL_TIMEOUT_MS)

    def _finalize_list(self):
        if self.state != "listing":
            return
        self.state = "idle"
        self.entries.emit(sorted(self._entries.values(), key=lambda e: e["id"]))

    def _close_fh(self):
        try:
            if self.fh is not None:
                self.fh.close()
        except Exception:
            pass
        self.fh = None

    def _finish_download(self, ok):
        self._dl_timer.stop()
        self._close_fh()
        self.state = "idle"
        if self._ready():
            self._link().log_request_end(self._target())
        if ok:
            self.finished.emit(True, self.path,
                               f"saved {self.dl_got} bytes -> {os.path.basename(self.path)}")
        else:
            self.finished.emit(False, self.path,
                               f"log download incomplete ({self.dl_got}/{self.dl_size} bytes) -- "
                               f"gave up after {self.dl_retries} retries")
