"""params.py -- MAVLink parameter protocol (download + set) and the editor dialog.

  download  GCS -> PARAM_REQUEST_LIST
            veh -> PARAM_VALUE x N   (each carries param_count + param_index)
            (missing indices are re-requested individually with PARAM_REQUEST_READ)
  set       GCS -> PARAM_SET ; veh echoes PARAM_VALUE with the stored value

ArduPilot stores every parameter as REAL32, so the editor treats values as floats.
"""
from __future__ import annotations

from PySide6.QtCore import QObject, Signal, QTimer, Qt
from PySide6.QtGui import QFont, QColor
from PySide6.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLineEdit, QPushButton,
                               QLabel, QTableWidget, QTableWidgetItem, QHeaderView)

import mavlink

TIMEOUT_MS = 1500
MAX_RETRIES = 6


class ParamManager(QObject):
    progress = Signal(str)
    finished = Signal(bool, str)
    param = Signal(str, float, int, int)    # name, value, index, count
    updated = Signal(str, float)            # name, value (any PARAM_VALUE)

    def __init__(self, link_getter, target_getter, parent=None):
        super().__init__(parent)
        self._link = link_getter
        self._target = target_getter
        self.values = {}                    # name -> float
        self.index_of = {}                  # name -> index
        self.expected = None
        self.received = set()
        self.state = "idle"
        self.retries = 0
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self._on_timeout)

    def _ready(self):
        link = self._link()
        return link is not None and link.is_open and link.remote is not None

    def download(self):
        if not self._ready():
            self.finished.emit(False, "no vehicle connected")
            return
        self.values, self.index_of, self.received = {}, {}, set()
        self.expected = None
        self.state = "download"
        self.retries = 0
        self.progress.emit("requesting parameters")
        self._link().request_params(self._target())
        self.timer.start(TIMEOUT_MS)

    def set(self, name, value):
        if not self._ready():
            self.finished.emit(False, "no vehicle connected")
            return
        self._link().set_param(self._target(), name, float(value))
        self.progress.emit(f"set {name} = {value:g}")

    def handle_messages(self, batch):
        for m in batch:
            if m.msgid == mavlink.PARAM_VALUE:
                self._on_value(m)

    def _on_value(self, m):
        name = m.fields.get("param_id", "")
        if not name:
            return
        val = float(m.fields.get("param_value", 0.0))
        idx = int(m.fields.get("param_index", 0))
        cnt = int(m.fields.get("param_count", 0))
        self.values[name] = val
        self.index_of[name] = idx
        self.updated.emit(name, val)
        if self.state == "download":
            self.expected = cnt
            self.received.add(idx)
            self.retries = 0
            self.param.emit(name, val, idx, cnt)
            self.progress.emit(f"{len(self.received)}/{cnt} parameters")
            if cnt and len(self.received) >= cnt:
                self.state = "idle"
                self.timer.stop()
                self.finished.emit(True, f"{cnt} parameters")
            else:
                self.timer.start(TIMEOUT_MS)
        else:
            self.param.emit(name, val, idx, cnt or len(self.values))

    def _on_timeout(self):
        if self.state != "download":
            return
        self.retries += 1
        if self.retries > MAX_RETRIES or not self._ready():
            got = len(self.received)
            self.state = "idle"
            self.finished.emit(got > 0, f"got {got}/{self.expected or '?'} parameters (timed out)")
            return
        link, tgt = self._link(), self._target()
        if self.expected is None:
            link.request_params(tgt)            # never heard back at all
        else:
            missing = [i for i in range(self.expected) if i not in self.received]
            for i in missing[:12]:              # re-request a batch of gaps
                link.request_param_read(tgt, index=i)
            self.progress.emit(f"re-requesting {len(missing)} missing (try {self.retries})")
        self.timer.start(TIMEOUT_MS)


class ParamDialog(QDialog):
    """Searchable parameter table; edit a value and Write to push a PARAM_SET."""

    def __init__(self, manager, parent=None):
        super().__init__(parent)
        self.mgr = manager
        self.setWindowTitle("Parameters")
        self.resize(560, 620)
        self._loading = False
        self.rows = {}                          # name -> row index
        self.edited = {}                        # name -> new value (pending write)

        lay = QVBoxLayout(self)
        top = QHBoxLayout()
        self.search = QLineEdit()
        self.search.setPlaceholderText("filter by name...")
        self.search.textChanged.connect(self._filter)
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self.mgr.download)
        self.btn_write = QPushButton("Write changed")
        self.btn_write.clicked.connect(self._write)
        top.addWidget(self.search, 1)
        top.addWidget(self.btn_refresh)
        top.addWidget(self.btn_write)
        lay.addLayout(top)

        self.table = QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["Parameter", "Value"])
        self.table.verticalHeader().setVisible(False)
        self.table.setFont(QFont("DejaVu Sans Mono", 9))
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table.itemChanged.connect(self._item_changed)
        lay.addWidget(self.table, 1)

        self.status = QLabel("no parameters loaded")
        self.status.setStyleSheet("color:#8a90a0;")
        lay.addWidget(self.status)

        self.mgr.param.connect(self._on_param)
        self.mgr.updated.connect(self._on_updated)
        self.mgr.progress.connect(self.status.setText)
        self.mgr.finished.connect(lambda ok, msg: self.status.setText(msg))

    def _on_param(self, name, value, index, count):
        self._loading = True
        if name not in self.rows:
            row = self.table.rowCount()
            self.table.insertRow(row)
            nitem = QTableWidgetItem(name)
            nitem.setFlags(nitem.flags() & ~Qt.ItemIsEditable)
            self.table.setItem(row, 0, nitem)
            self.table.setItem(row, 1, QTableWidgetItem(""))
            self.rows[name] = row
        vitem = self.table.item(self.rows[name], 1)
        vitem.setText(f"{value:g}")
        vitem.setForeground(QColor("#d6dae2"))
        self.edited.pop(name, None)
        self._loading = False
        self._apply_filter_row(self.rows[name])

    def _on_updated(self, name, value):
        if name in self.rows:
            self._on_param(name, value, self.mgr.index_of.get(name, 0), len(self.mgr.values))

    def _item_changed(self, item):
        if self._loading or item.column() != 1:
            return
        name = self.table.item(item.row(), 0).text()
        try:
            self.edited[name] = float(item.text())
            item.setForeground(QColor("#ffd24a"))      # pending write
        except ValueError:
            item.setForeground(QColor("#ff6b6b"))

    def _write(self):
        if not self.edited:
            self.status.setText("no edited values to write")
            return
        for name, val in list(self.edited.items()):
            self.mgr.set(name, val)
        self.status.setText(f"writing {len(self.edited)} parameter(s)...")

    def _filter(self, _text):
        for row in range(self.table.rowCount()):
            self._apply_filter_row(row)

    def _apply_filter_row(self, row):
        needle = self.search.text().strip().upper()
        name = self.table.item(row, 0).text().upper()
        self.table.setRowHidden(row, bool(needle) and needle not in name)
