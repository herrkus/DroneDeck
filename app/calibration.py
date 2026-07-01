"""calibration.py -- vehicle setup: radio (RC) + sensor calibration.

Radio: reads RC_CHANNELS while the operator sweeps the sticks, captures per
channel min/max/trim, and writes RCn_MIN/MAX/TRIM parameters.

Sensors: sends MAV_CMD_PREFLIGHT_CALIBRATION (gyro/accel/level/compass) and
shows the STATUSTEXT prompts the firmware streams back. The GCS is a relay for
sensor cal -- the fitting math lives in the autopilot -- so this drives and
displays the workflow, which is exactly what is testable against the simulator.
"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal, QRectF
from PySide6.QtGui import QPainter, QColor, QPen, QFont
from PySide6.QtWidgets import (QDialog, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
                               QLabel, QTabWidget, QListWidget, QListWidgetItem,
                               QLineEdit, QFormLayout)

import mavlink

_MONO = QFont("DejaVu Sans Mono", 9)
SEV_COLOR = {0: "#ff5050", 1: "#ff5050", 2: "#ff6a3d", 3: "#ff6a3d",
             4: "#e0a030", 5: "#39c0d0", 6: "#c4c8d0", 7: "#7a8090"}


class RcBars(QWidget):
    """Per-channel PWM bars with captured min/max ticks."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(220)
        self.count = 0
        self.cur = {}
        self.lo = {}
        self.hi = {}

    def update_rc(self, count, values, capture):
        self.count = count
        for i, v in enumerate(values[:count], start=1):
            if v == 65535:
                continue
            self.cur[i] = v
            if capture:
                self.lo[i] = min(self.lo.get(i, v), v)
                self.hi[i] = max(self.hi.get(i, v), v)
        self.update()

    def reset(self):
        self.lo, self.hi = {}, {}
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(22, 24, 29))
        p.setFont(_MONO)
        n = max(1, self.count)
        rh = min(24, (self.height() - 8) / n)
        LO, HI = 900.0, 2100.0
        for i in range(1, self.count + 1):
            y = 4 + (i - 1) * rh
            track = QRectF(56, y + 3, self.width() - 116, rh - 6)
            p.setPen(QColor(150, 156, 166))
            p.drawText(QRectF(4, y, 50, rh), Qt.AlignVCenter | Qt.AlignLeft, f"CH{i}")
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(38, 42, 50))
            p.drawRoundedRect(track, 3, 3)

            def x_of(v):
                return track.left() + (max(LO, min(HI, v)) - LO) / (HI - LO) * track.width()

            if i in self.lo:
                p.setPen(QPen(QColor(90, 200, 230), 2))
                p.drawLine(x_of(self.lo[i]), track.top(), x_of(self.lo[i]), track.bottom())
                p.drawLine(x_of(self.hi[i]), track.top(), x_of(self.hi[i]), track.bottom())
            cur = self.cur.get(i)
            if cur is not None:
                cx = x_of(cur)
                p.setPen(Qt.NoPen)
                p.setBrush(QColor(120, 230, 140))
                p.drawRoundedRect(QRectF(track.left(), track.top(), cx - track.left(),
                                         track.height()), 3, 3)
                p.setPen(QColor(220, 224, 230))
                p.drawText(QRectF(track.right() + 4, y, 56, rh),
                           Qt.AlignVCenter | Qt.AlignLeft, str(cur))
        p.end()


class RcCalibrationWidget(QWidget):
    saveRequested = Signal(dict)          # {param_name: value}

    def __init__(self, parent=None):
        super().__init__(parent)
        self._capture = False
        v = QVBoxLayout(self)
        row = QHBoxLayout()
        self.btn_start = QPushButton("Start")
        self.btn_stop = QPushButton("Stop")
        self.btn_save = QPushButton("Save to params")
        self.btn_stop.setEnabled(False)
        self.btn_save.setEnabled(False)
        self.btn_start.clicked.connect(self._start)
        self.btn_stop.clicked.connect(self._stop)
        self.btn_save.clicked.connect(self._save)
        row.addWidget(self.btn_start)
        row.addWidget(self.btn_stop)
        row.addWidget(self.btn_save)
        row.addStretch(1)
        v.addLayout(row)
        self.hint = QLabel("Start, then move every stick and switch to its extremes, then Stop.")
        self.hint.setStyleSheet("color:#8fa3bf;")
        v.addWidget(self.hint)
        self.bars = RcBars()
        v.addWidget(self.bars, 1)

    def handle_rc(self, fields):
        count = int(fields.get("chancount", 0))
        vals = [int(fields.get(f"chan{i}_raw", 65535)) for i in range(1, 19)]
        self.bars.update_rc(count, vals, self._capture)

    def _start(self):
        self._capture = True
        self.bars.reset()
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_save.setEnabled(False)
        self.hint.setText("Capturing… move sticks/switches through their full range.")

    def _stop(self):
        self._capture = False
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.btn_save.setEnabled(bool(self.bars.lo))
        self.hint.setText("Captured. Review the min/max ticks, then Save to params.")

    def _save(self):
        params = {}
        for i in sorted(self.bars.lo):
            params[f"RC{i}_MIN"] = float(self.bars.lo[i])
            params[f"RC{i}_MAX"] = float(self.bars.hi[i])
            params[f"RC{i}_TRIM"] = float(self.bars.cur.get(i, (self.bars.lo[i] + self.bars.hi[i]) // 2))
        if params:
            self.saveRequested.emit(params)
            self.hint.setText(f"Wrote {len(params)} parameters for {len(self.bars.lo)} channels.")


class SensorCalibrationWidget(QWidget):
    calRequested = Signal(str)             # 'gyro' | 'accel' | 'level' | 'compass'

    def __init__(self, parent=None):
        super().__init__(parent)
        v = QVBoxLayout(self)
        row = QHBoxLayout()
        for label, kind in (("Gyro", "gyro"), ("Accel", "accel"),
                            ("Level horizon", "level"), ("Compass", "compass")):
            b = QPushButton(label)
            b.clicked.connect(lambda _=False, k=kind: self._request(k))
            row.addWidget(b)
        row.addStretch(1)
        v.addLayout(row)
        self.status = QLabel("Pick a calibration; follow the prompts from the vehicle.")
        self.status.setStyleSheet("color:#8fa3bf;")
        v.addWidget(self.status)
        self.log = QListWidget()
        self.log.setFont(_MONO)
        v.addWidget(self.log, 1)

    def _request(self, kind):
        self.log.clear()
        self.status.setText(f"{kind} calibration started…")
        self.calRequested.emit(kind)

    def add_status(self, severity, text):
        sev = mavlink.MAV_SEVERITY.get(severity, str(severity))
        item = QListWidgetItem(f"[{sev}] {text}")
        item.setForeground(QColor(SEV_COLOR.get(severity, "#c4c8d0")))
        self.log.addItem(item)
        self.log.scrollToBottom()


class ParamPage(QWidget):
    """Base Vehicle-Setup page: reads + writes a fixed set of parameters through the
    read-back-confirmed ParamManager. Autopilot-agnostic -- only the parameters the
    vehicle actually reports are shown. Subclasses set PARAMS and NOUN."""

    PARAMS = []                           # [(param_name, label), ...]
    NOUN = "parameters"

    def __init__(self, mgr, parent=None):
        super().__init__(parent)
        self.mgr = mgr
        self.rows = {}                    # name -> (QLabel, QLineEdit)
        lay = QVBoxLayout(self)
        info = QLabel(f"{self.NOUN.capitalize()} parameters the vehicle reports. Edit a value "
                      "and Write; each write is confirmed by read-back (green = ok, red = failed).")
        info.setWordWrap(True)
        info.setStyleSheet("color:#8a90a0;")
        lay.addWidget(info)
        self.form = QFormLayout()
        for name, label in self.PARAMS:
            lbl, edit = QLabel(label), QLineEdit()
            edit.setPlaceholderText("—")
            self.form.addRow(lbl, edit)
            lbl.hide(); edit.hide()
            self.rows[name] = (lbl, edit)
        lay.addLayout(self.form)
        lay.addStretch(1)
        row = QHBoxLayout()
        self.status = QLabel("press Refresh to load")
        self.status.setStyleSheet("color:#8a90a0;")
        btn_refresh = QPushButton("Refresh")
        btn_refresh.clicked.connect(self.refresh)
        btn_write = QPushButton("Write changed")
        btn_write.clicked.connect(self._write)
        row.addWidget(self.status, 1)
        row.addWidget(btn_refresh)
        row.addWidget(btn_write)
        lay.addLayout(row)
        self.mgr.updated.connect(self._on_value)
        self.mgr.set_result.connect(self._on_set_result)

    def refresh(self):
        self.status.setText(f"requesting {self.NOUN} parameters...")
        self.mgr.request([n for n, _ in self.PARAMS])

    def _on_value(self, name, val):
        if name in self.rows:
            lbl, edit = self.rows[name]
            lbl.show(); edit.show()
            if not edit.hasFocus():          # don't overwrite what the user is typing
                edit.setText(f"{val:g}")
                edit.setStyleSheet("")

    def _write(self):
        n = 0
        for name, (lbl, edit) in self.rows.items():
            if edit.isHidden() or not edit.text().strip():   # isHidden = explicit flag
                continue
            try:
                val = float(edit.text())
            except ValueError:
                edit.setStyleSheet("color:#ff6b6b;")
                continue
            cur = self.mgr.values.get(name)
            if cur is None or abs(val - cur) > 1e-9:
                self.mgr.set(name, val)
                n += 1
        self.status.setText(f"writing {n} parameter(s)..." if n else "no changes to write")

    def _on_set_result(self, name, ok, msg):
        if name in self.rows:
            self.status.setText(msg)
            self.rows[name][1].setStyleSheet("color:#37d67a;" if ok else "color:#ff6b6b;")


class SafetyWidget(ParamPage):
    NOUN = "safety"
    PARAMS = [
        ("COM_DL_LOSS_T",   "PX4: datalink-loss timeout (s)"),
        ("NAV_RCL_ACT",     "PX4: RC-loss action"),
        ("NAV_DLL_ACT",     "PX4: datalink-loss action"),
        ("COM_LOW_BAT_ACT", "PX4: low-battery action"),
        ("GF_ACTION",       "PX4: geofence breach action"),
        ("GF_MAX_HOR_DIST", "PX4: geofence max distance (m)"),
        ("RTL_RETURN_ALT",  "PX4: RTL return altitude (m)"),
        ("RTL_DESCEND_ALT", "PX4: RTL descend altitude (m)"),
        ("COM_DISARM_LAND", "PX4: auto-disarm after land (s)"),
        ("FS_THR_ENABLE",   "APM: throttle failsafe"),
        ("FS_BATT_ENABLE",  "APM: battery failsafe"),
        ("BATT_LOW_VOLT",   "APM: battery low voltage (V)"),
        ("FENCE_ENABLE",    "APM: geofence enable"),
        ("RTL_ALT",         "APM: RTL altitude (cm)"),
    ]


class PowerWidget(ParamPage):
    NOUN = "power / battery"
    PARAMS = [
        ("BAT1_N_CELLS",     "PX4: battery cell count"),
        ("BAT1_V_CHARGED",   "PX4: cell voltage full (V)"),
        ("BAT1_V_EMPTY",     "PX4: cell voltage empty (V)"),
        ("BAT1_CAPACITY",    "PX4: battery capacity (mAh)"),
        ("BAT1_V_LOAD_DROP", "PX4: voltage drop per A (V)"),
        ("BAT_N_CELLS",      "PX4: battery cell count"),
        ("BAT_V_CHARGED",    "PX4: cell voltage full (V)"),
        ("BAT_V_EMPTY",      "PX4: cell voltage empty (V)"),
        ("BAT_CAPACITY",     "PX4: battery capacity (mAh)"),
        ("BATT_MONITOR",     "APM: battery monitor type"),
        ("BATT_CAPACITY",    "APM: battery capacity (mAh)"),
        ("BATT_LOW_VOLT",    "APM: low battery voltage (V)"),
        ("BATT_CRT_VOLT",    "APM: critical voltage (V)"),
        ("BATT_NUM_CELLS",   "APM: cell count"),
    ]


class CalibrationDialog(QDialog):
    """Setup view: Radio + Sensors tabs, fed live from the link while open."""

    def __init__(self, link_getter, param_mgr=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Vehicle Setup — Calibration")
        self.resize(560, 460)
        self._link = link_getter
        self.radio = RcCalibrationWidget()
        self.sensor = SensorCalibrationWidget()
        tabs = QTabWidget()
        tabs.addTab(self.radio, "Radio")
        tabs.addTab(self.sensor, "Sensors")
        if param_mgr is not None:
            self.safety = SafetyWidget(param_mgr)
            tabs.addTab(self.safety, "Safety")
            self.power = PowerWidget(param_mgr)
            tabs.addTab(self.power, "Power")
            self.safety.refresh()          # auto-request params on open
            self.power.refresh()
        lay = QVBoxLayout(self)
        lay.addWidget(tabs)
        link = self._link()
        self._link_obj = link
        if link is not None:
            link.messages.connect(self._on_messages)

    def _on_messages(self, batch):
        for m in batch:
            if m.msgid == mavlink.RC_CHANNELS:
                self.radio.handle_rc(m.fields)
            elif m.msgid == mavlink.STATUSTEXT:
                self.sensor.add_status(int(m.fields.get("severity", 6)),
                                       m.fields.get("text", ""))

    def closeEvent(self, e):
        if self._link_obj is not None:
            try:
                self._link_obj.messages.disconnect(self._on_messages)
            except (RuntimeError, TypeError):
                pass
        super().closeEvent(e)
