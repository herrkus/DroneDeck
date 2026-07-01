"""params.py -- MAVLink parameter protocol (download + set) and the editor dialog.

  download  GCS -> PARAM_REQUEST_LIST
            veh -> PARAM_VALUE x N   (each carries param_count + param_index)
            (missing indices are re-requested individually with PARAM_REQUEST_READ)
  set       GCS -> PARAM_SET ; veh echoes PARAM_VALUE with the stored value

ArduPilot stores every parameter as REAL32, so the editor treats values as floats.
"""
from __future__ import annotations

import os

from PySide6.QtCore import QObject, Signal, QTimer, Qt
from PySide6.QtGui import QFont, QColor
from PySide6.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLineEdit, QPushButton,
                               QLabel, QTableWidget, QTableWidgetItem, QHeaderView, QProgressBar,
                               QFileDialog, QMessageBox)

import mavlink

TIMEOUT_MS = 1500
MAX_RETRIES = 6
SET_TIMEOUT_MS = 1000
SET_RETRIES = 5


class ParamManager(QObject):
    progress = Signal(str)
    finished = Signal(bool, str)
    param = Signal(str, float, int, int)    # name, value, index, count
    updated = Signal(str, float)            # name, value (any PARAM_VALUE)
    set_result = Signal(str, bool, str)     # PARAM_SET readback: name, confirmed, message
    download_progress = Signal(int, int)    # received, total

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
        self.type_of = {}                   # name -> MAV_PARAM_TYPE (for a correctly-typed PARAM_SET)
        self.pending = {}                   # name -> {value,type,tries} awaiting SET readback
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self._on_timeout)
        self.set_timer = QTimer(self)
        self.set_timer.setSingleShot(True)
        self.set_timer.timeout.connect(self._on_set_timeout)

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

    def request(self, names):
        """Request specific parameters by name (PARAM_REQUEST_READ) -- for setup
        pages that need a handful of params, not the whole 1000+ list. Values arrive
        via the `updated` signal and are stored in self.values / self.type_of."""
        if not self._ready():
            return
        link, tgt = self._link(), self._target()
        for name in names:
            link.request_param_read(tgt, param_id=name)

    def set(self, name, value):
        if not self._ready():
            self.finished.emit(False, "no vehicle connected")
            return
        value = float(value)
        ptype = self.type_of.get(name, mavlink.MAV_PARAM_TYPE_REAL32)
        self.pending[name] = {"value": value, "type": ptype, "tries": 1}
        self._link().set_param(self._target(), name, value, ptype)
        self.progress.emit(f"set {name} = {value:g} ...")
        if not self.set_timer.isActive():
            self.set_timer.start(SET_TIMEOUT_MS)

    @staticmethod
    def _values_match(a, b, ptype):
        # integer param types must land exactly; floats within a small tolerance
        if ptype in (1, 2, 3, 4, 5, 6, 7, 8):      # (U)INT8/16/32/64
            return round(a) == round(b)
        return abs(a - b) <= max(1e-4, abs(b) * 1e-3)

    def _on_set_timeout(self):
        # A PARAM_SET can be dropped on UDP/radio; re-send until the vehicle echoes
        # the value back (QGC does the same), then give up loudly after SET_RETRIES.
        if not self.pending:
            return
        if not self._ready():
            for name in list(self.pending):
                self.set_result.emit(name, False, f"{name}: set failed (disconnected)")
            self.pending.clear()
            return
        link, tgt = self._link(), self._target()
        for name in list(self.pending):
            p = self.pending[name]
            if p["tries"] >= SET_RETRIES:
                self.set_result.emit(name, False,
                                     f"{name}: set NOT confirmed after {SET_RETRIES} tries")
                self.pending.pop(name, None)
            else:
                p["tries"] += 1
                link.set_param(tgt, name, p["value"], p["type"])
                self.progress.emit(f"re-sending {name} = {p['value']:g} (try {p['tries']})")
        if self.pending:
            self.set_timer.start(SET_TIMEOUT_MS)

    def handle_messages(self, batch):
        for m in batch:
            if m.msgid == mavlink.PARAM_VALUE:
                self._on_value(m)

    def _on_value(self, m):
        name = m.fields.get("param_id", "")
        if not name:
            return
        ptype = int(m.fields.get("param_type", mavlink.MAV_PARAM_TYPE_REAL32))
        val = mavlink.param_decode(float(m.fields.get("param_value", 0.0)), ptype)
        idx = int(m.fields.get("param_index", 0))
        cnt = int(m.fields.get("param_count", 0))
        self.values[name] = val
        self.index_of[name] = idx
        self.type_of[name] = ptype
        self.updated.emit(name, val)
        # confirm a pending PARAM_SET once the vehicle echoes our value back
        if name in self.pending:
            if self._values_match(val, self.pending[name]["value"], self.type_of[name]):
                self.pending.pop(name)
                self.set_result.emit(name, True, f"{name} = {val:g} confirmed")
                if not self.pending:
                    self.set_timer.stop()
        if self.state == "download":
            self.expected = cnt
            self.received.add(idx)
            self.retries = 0
            self.param.emit(name, val, idx, cnt)
            self.download_progress.emit(len(self.received), cnt)
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
        self.btn_save = QPushButton("Save...")
        self.btn_save.setToolTip("Save all parameters to a .params file")
        self.btn_save.clicked.connect(self._save_file)
        self.btn_load = QPushButton("Load...")
        self.btn_load.setToolTip("Load a .params file and write changed values to the vehicle")
        self.btn_load.clicked.connect(self._load_file)
        self.btn_compare = QPushButton("Compare...")
        self.btn_compare.setToolTip("Compare a .params file against the vehicle -- shows "
                                    "differences without writing anything")
        self.btn_compare.clicked.connect(self._compare_file)
        top.addWidget(self.search, 1)
        top.addWidget(self.btn_refresh)
        top.addWidget(self.btn_write)
        top.addWidget(self.btn_save)
        top.addWidget(self.btn_load)
        top.addWidget(self.btn_compare)
        lay.addLayout(top)

        self.table = QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["Parameter", "Value"])
        self.table.verticalHeader().setVisible(False)
        self.table.setFont(QFont("DejaVu Sans Mono", 9))
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table.itemChanged.connect(self._item_changed)
        lay.addWidget(self.table, 1)

        self.pbar = QProgressBar()
        self.pbar.setTextVisible(True)
        self.pbar.hide()
        lay.addWidget(self.pbar)
        self.status = QLabel("no parameters loaded")
        self.status.setStyleSheet("color:#8a90a0;")
        lay.addWidget(self.status)

        self.mgr.param.connect(self._on_param)
        self.mgr.updated.connect(self._on_updated)
        self.mgr.progress.connect(self.status.setText)
        self.mgr.download_progress.connect(self._on_dl_progress)
        self.mgr.set_result.connect(self._on_set_result)
        self.mgr.finished.connect(self._on_finished)

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

    def _on_dl_progress(self, received, total):
        if total:
            self.pbar.setMaximum(total)
            self.pbar.setValue(received)
            self.pbar.setFormat("%v / %m parameters")
            self.pbar.show()

    def _on_set_result(self, name, ok, msg):
        self.status.setText(msg)
        if name in self.rows:
            vitem = self.table.item(self.rows[name], 1)
            if vitem:
                vitem.setForeground(QColor("#37d67a" if ok else "#ff6b6b"))

    def _on_finished(self, ok, msg):
        self.status.setText(msg)
        self.pbar.hide()

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

    # -- file save / load (QGC-compatible .params) ---------------------------
    def save_params_to(self, path):
        """Write all downloaded parameters to `path` in QGC's tab-separated .params format.
        Returns the number of parameters written."""
        try:
            sysid = int(self.mgr._target())
        except Exception:
            sysid = 1
        lines = ["# Onboard parameters saved by DroneDeck",
                 "# MAV ID\tCOMPONENT ID\tNAME\tVALUE\tTYPE"]
        for name in sorted(self.mgr.values):
            ptype = int(self.mgr.type_of.get(name, mavlink.MAV_PARAM_TYPE_REAL32))
            v = self.mgr.values[name]
            # integer types must be written exactly (a large bitmask would lose digits in %g);
            # real types get 9 sig figs, which round-trips a float32 exactly
            vtxt = f"{int(round(v))}" if ptype in (1, 2, 3, 4, 5, 6, 7, 8) else f"{v:.9g}"
            lines.append(f"{sysid}\t1\t{name}\t{vtxt}\t{ptype}")
        with open(path, "w") as f:
            f.write("\n".join(lines) + "\n")
        return len(self.mgr.values)

    @staticmethod
    def load_params_from(path):
        """Parse a .params file -> {name: float}. Accepts QGC tab format (sysid comp name value
        type) and a plain 'name,value' CSV; '#' comment lines and blanks are ignored."""
        out = {}
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = [p for p in (line.split("\t") if "\t" in line else line.split(","))
                         if p.strip() != ""]
                if len(parts) >= 5:                 # QGC: sysid comp NAME VALUE TYPE
                    name, val = parts[2].strip(), parts[3].strip()
                elif len(parts) == 2:               # NAME,VALUE
                    name, val = parts[0].strip(), parts[1].strip()
                else:
                    continue
                try:
                    out[name] = float(val)
                except ValueError:
                    continue
        return out

    def _save_file(self):
        if not self.mgr.values:
            self.status.setText("no parameters loaded -- Refresh first")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Save parameters",
                                              os.path.expanduser("~/vehicle.params"),
                                              "Parameters (*.params)")
        if not path:
            return
        if not path.endswith(".params"):
            path += ".params"
        n = self.save_params_to(path)
        self.status.setText(f"saved {n} parameters to {os.path.basename(path)}")

    @staticmethod
    def diff_params(current, type_of, loaded):
        """Compare a loaded {name: value} file against the vehicle's current values. Returns
        (changed, missing): `changed` is a sorted list of (name, vehicle_value, file_value) for
        params present on the vehicle whose value differs (type-aware, same test as a write would
        use); `missing` is a sorted list of names in the file the vehicle does not have. Pure --
        writes nothing."""
        changed, missing = [], []
        for name in sorted(loaded):
            fval = loaded[name]
            if name not in current:
                missing.append(name)
            elif not ParamManager._values_match(
                    current[name], fval, type_of.get(name, mavlink.MAV_PARAM_TYPE_REAL32)):
                changed.append((name, current[name], fval))
        return changed, missing

    def _compare_file(self):
        path, _ = QFileDialog.getOpenFileName(self, "Compare parameters", os.path.expanduser("~"),
                                              "Parameters (*.params);;All files (*)")
        if not path:
            return
        loaded = self.load_params_from(path)
        if not loaded:
            self.status.setText("no parameters found in that file")
            return
        if not self.mgr.values:
            self.status.setText(f"parsed {len(loaded)} params -- Refresh the vehicle to compare")
            return
        changed, missing = self.diff_params(self.mgr.values, self.mgr.type_of, loaded)
        base = os.path.basename(path)
        if not changed and not missing:
            QMessageBox.information(self, "Compare parameters",
                                   f"No differences: the vehicle matches {base} "
                                   f"({len(loaded)} params checked).")
            self.status.setText(f"compared {base}: identical")
            return
        lines = []
        if changed:
            lines.append(f"{len(changed)} parameter(s) differ (vehicle -> file):")
            for name, cur, fval in changed:
                lines.append(f"  {name}\t{cur:g}\t->\t{fval:g}")
        if missing:
            lines.append("")
            lines.append(f"{len(missing)} param(s) in file not on this vehicle:")
            lines.extend(f"  {n}" for n in missing)
        box = QMessageBox(self)
        box.setWindowTitle("Compare parameters")
        box.setIcon(QMessageBox.Information)
        box.setText(f"{base}: {len(changed)} differ, {len(missing)} missing on vehicle. "
                    "Nothing was written -- use Load to apply.")
        box.setDetailedText("\n".join(lines))
        box.exec()
        self.status.setText(f"compared {base}: {len(changed)} differ, {len(missing)} missing")

    def _load_file(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load parameters", os.path.expanduser("~"),
                                              "Parameters (*.params);;All files (*)")
        if not path:
            return
        loaded = self.load_params_from(path)
        if not loaded:
            self.status.setText("no parameters found in that file")
            return
        if not self.mgr._ready():
            self.status.setText(f"parsed {len(loaded)} params -- connect a vehicle to write them")
            return
        changed = 0
        for name, val in loaded.items():           # write only params the vehicle has that differ
            if name in self.mgr.values and not ParamManager._values_match(
                    self.mgr.values[name], val, self.mgr.type_of.get(name, mavlink.MAV_PARAM_TYPE_REAL32)):
                self.mgr.set(name, val)
                changed += 1
        self.status.setText(f"loaded {len(loaded)} params from file; writing {changed} changed")

    def _filter(self, _text):
        for row in range(self.table.rowCount()):
            self._apply_filter_row(row)

    def _apply_filter_row(self, row):
        needle = self.search.text().strip().upper()
        name = self.table.item(row, 0).text().upper()
        self.table.setRowHidden(row, bool(needle) and needle not in name)
