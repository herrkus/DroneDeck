"""ftpbrowser.py -- MAVLink FTP file browser dialog.

Exposes the vehicle's filesystem (PX4/ArduPilot) over MAVLink FTP: navigate directories and download a
file to disk -- the QGC "Analyze > Log Download" equivalent for PX4, whose logs live under
/fs/microsd/log and are only reachable over FTP. Thin UI shell over ftp.FtpClient (all protocol logic +
proofs live there and in tests/live_ftp.py); this drives it from the live link and renders results.
"""
import os

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QListWidget, QListWidgetItem,
                               QPushButton, QLabel, QFileDialog, QMessageBox)

import mavlink
import ftp


class FtpBrowserDialog(QDialog):
    """Browse + download over MAVLink FTP, fed live from the link while open."""

    START_DIRS = ("/fs/microsd/log", "/")     # PX4 logs first, then root

    def __init__(self, link_getter, sysid_getter, save_dir, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Vehicle Files — MAVLink FTP")
        self.resize(560, 440)
        self._link = link_getter
        self._sysid = sysid_getter
        self._save_dir = save_dir
        self._path = "/"
        self._pending_path = "/"
        self._pending_save = None
        self._entries = []
        self._busy = False
        self._client = ftp.FtpClient(self._send)

        self.path_lbl = QLabel(self._path)
        self.path_lbl.setStyleSheet("font-weight:bold;")
        self.list = QListWidget()
        self.list.itemDoubleClicked.connect(self._activate)
        self.status = QLabel("")
        self.status.setWordWrap(True)

        self.btn_up = QPushButton("Up")
        self.btn_up.clicked.connect(self._go_up)
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(lambda: self._list(self._path))
        self.btn_dl = QPushButton("Download")
        self.btn_dl.clicked.connect(self._download_selected)
        btn_close = QPushButton("Close")
        btn_close.clicked.connect(self.close)
        row = QHBoxLayout()
        for b in (self.btn_up, self.btn_refresh, self.btn_dl):
            row.addWidget(b)
        row.addStretch(1)
        row.addWidget(btn_close)

        lay = QVBoxLayout(self)
        lay.addWidget(self.path_lbl)
        lay.addWidget(self.list, 1)
        lay.addWidget(self.status)
        lay.addLayout(row)

        # resend-on-stall while an operation is in flight (localhost is reliable, real radios drop)
        self._last_seq = -1
        self._timer = QTimer(self)
        self._timer.setInterval(900)
        self._timer.timeout.connect(self._tick)

        self._link_obj = self._link()
        if self._link_obj is not None:
            self._link_obj.messages.connect(self._on_messages)

        # open on the first start dir that answers; begin with the PX4 log path
        self._start_idx = 0
        self._list(self.START_DIRS[0])

    # --- transport ---------------------------------------------------------
    def _send(self, seq, session, opcode, offset, data, size):
        link = self._link()
        if link is not None:
            link.send_ftp(self._sysid(), seq, session, opcode, offset, data, size=size, target_comp=1)

    def _on_messages(self, batch):
        for m in batch:
            if m.msgid == mavlink.FILE_TRANSFER_PROTOCOL and "payload" in m.fields:
                self._client.handle(ftp.decode(m.fields["payload"]))

    def _tick(self):
        if self._client.active:
            if self._client.seq == self._last_seq:      # no progress since last tick -> resend
                self._client.resend()
            self._last_seq = self._client.seq
        else:
            self._timer.stop()

    def _begin(self, note):
        self._busy = True
        self._last_seq = self._client.seq
        self._set_enabled(False)
        self.status.setText(note)
        self._timer.start()

    def _end(self):
        self._busy = False
        self._timer.stop()
        self._set_enabled(True)

    def _set_enabled(self, on):
        for b in (self.btn_up, self.btn_refresh, self.btn_dl):
            b.setEnabled(on)

    # --- directory listing -------------------------------------------------
    def _list(self, path):
        if self._busy:
            return
        self._pending_path = path
        self._begin(f"Listing {path} ...")
        self._client.list_directory(path, self._on_list_done)

    def _on_list_done(self, entries, error):
        self._end()
        if error:
            # a start dir that isn't there -> fall through to the next candidate
            if self._start_idx + 1 < len(self.START_DIRS) and self._pending_path in self.START_DIRS:
                self._start_idx += 1
                self._list(self.START_DIRS[self._start_idx])
                return
            self.status.setText(f"Could not list {self._pending_path}: {error}")
            return
        self._path = self._pending_path
        self._entries = entries or []
        self.path_lbl.setText(self._path)
        self._populate()
        self.status.setText(f"{len(self._entries)} item(s)")

    def _populate(self):
        self.list.clear()
        if self._path not in ("/", ""):
            up = QListWidgetItem("[..]")
            up.setData(Qt.UserRole, {"type": "up"})
            self.list.addItem(up)
        dirs = sorted((e for e in self._entries if e["type"] == "dir"), key=lambda e: e["name"])
        files = sorted((e for e in self._entries if e["type"] == "file"), key=lambda e: e["name"])
        for e in dirs:
            it = QListWidgetItem(f"[{e['name']}]")
            it.setData(Qt.UserRole, e)
            self.list.addItem(it)
        for e in files:
            sz = f"  ({e['size']} B)" if e.get("size") is not None else ""
            it = QListWidgetItem(f"{e['name']}{sz}")
            it.setData(Qt.UserRole, e)
            self.list.addItem(it)

    def _join(self, name):
        return (self._path.rstrip("/") + "/" + name) if self._path != "/" else "/" + name

    def _parent(self):
        p = self._path.rstrip("/")
        return p.rsplit("/", 1)[0] or "/"

    def _go_up(self):
        if self._path not in ("/", ""):
            self._list(self._parent())

    def _activate(self, item):
        e = item.data(Qt.UserRole)
        if not e:
            return
        if e["type"] == "up":
            self._go_up()
        elif e["type"] == "dir":
            self._list(self._join(e["name"]))
        elif e["type"] == "file":
            self._download(e)

    # --- download ----------------------------------------------------------
    def _download_selected(self):
        it = self.list.currentItem()
        e = it.data(Qt.UserRole) if it else None
        if not e or e.get("type") != "file":
            self.status.setText("Select a file to download.")
            return
        self._download(e)

    def _download(self, entry):
        if self._busy:
            return
        default = os.path.join(self._save_dir, entry["name"])
        path, _ = QFileDialog.getSaveFileName(self, "Save file as", default)
        if not path:
            return
        self._do_download(entry, path)

    def _do_download(self, entry, save_path):
        """Read `entry` over FTP and write it to save_path. Split out so tests can bypass the dialog."""
        if self._busy:
            return
        self._pending_save = save_path
        self._begin(f"Downloading {entry['name']} ...")
        self._client.read_file(self._join(entry["name"]), self._on_read_done)

    def _on_read_done(self, data, error):
        self._end()
        if error:
            self.status.setText(f"Download failed: {error}")
            return
        try:
            with open(self._pending_save, "wb") as f:
                f.write(data or b"")
        except OSError as ex:
            self.status.setText(f"Could not write {self._pending_save}: {ex}")
            return
        self.status.setText(f"Saved {len(data or b'')} B to {self._pending_save}")

    def closeEvent(self, e):
        self._timer.stop()
        if self._link_obj is not None:
            try:
                self._link_obj.messages.disconnect(self._on_messages)
            except (RuntimeError, TypeError):
                pass
        super().closeEvent(e)
