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
                               QLineEdit, QFormLayout, QCheckBox, QSpinBox, QGridLayout)

import mavlink

_MONO = QFont("DejaVu Sans Mono", 9)
SEV_COLOR = {0: "#ff5050", 1: "#ff5050", 2: "#ff6a3d", 3: "#ff6a3d",
             4: "#e0a030", 5: "#39c0d0", 6: "#c4c8d0", 7: "#7a8090"}
# min PWM travel (us) for an RC channel to count as "moved" during calibration. A real stick/switch
# sweeps ~800-1000 us end to end; an untouched channel only jitters a few us. 200 us cleanly separates
# them and guards against writing a degenerate MIN==MAX==TRIM calibration.
RC_MIN_TRAVEL = 200


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
        skipped = []
        for i in sorted(self.bars.lo):
            lo, hi = int(self.bars.lo[i]), int(self.bars.hi[i])
            # Only write a channel that actually swept a usable range. An untouched channel sits at
            # a near-constant value, giving MIN==MAX==TRIM -- a zero-span RC calibration that on a
            # real vehicle means a dead/failsafe axis (and MIN==MAX invites divide-by-range bugs in
            # the autopilot's RC scaling). Require >=RC_MIN_TRAVEL us of travel; skip the rest.
            if hi - lo < RC_MIN_TRAVEL:
                skipped.append(i)
                continue
            trim = int(self.bars.cur.get(i, (lo + hi) // 2))
            trim = max(lo, min(hi, trim))                # keep TRIM within [MIN, MAX]
            params[f"RC{i}_MIN"] = float(lo)
            params[f"RC{i}_MAX"] = float(hi)
            params[f"RC{i}_TRIM"] = float(trim)
        if params:
            self.saveRequested.emit(params)
            msg = f"Wrote {len(params)} parameters for {len(params) // 3} channel(s)."
            if skipped:
                msg += f" Skipped unmoved channel(s): {', '.join(str(s) for s in skipped)}."
            self.hint.setText(msg)
        else:
            self.hint.setText("No channel swept a usable range -- move every stick and switch to its "
                              "extremes during capture, then Save.")


class SensorCalibrationWidget(QWidget):
    calRequested = Signal(str)             # 'gyro' | 'accel' | 'level' | 'compass'
    accelPosRequested = Signal(int)        # accel 6-position: 1 level..6 back (ACCELCAL_VEHICLE_POS)

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
        # accel 6-position row: click each orientation as the vehicle prompts for it (ArduPilot).
        # Disabled until an accel calibration is started.
        pos_row = QHBoxLayout()
        self._pos_btns = []
        for label, pos in (("Level", mavlink.ACCELCAL_POS_LEVEL), ("Left", mavlink.ACCELCAL_POS_LEFT),
                           ("Right", mavlink.ACCELCAL_POS_RIGHT), ("Nose Down", mavlink.ACCELCAL_POS_NOSEDOWN),
                           ("Nose Up", mavlink.ACCELCAL_POS_NOSEUP), ("Back", mavlink.ACCELCAL_POS_BACK)):
            b = QPushButton(label)
            b.setEnabled(False)
            b.clicked.connect(lambda _=False, p=pos: self.accelPosRequested.emit(p))
            pos_row.addWidget(b)
            self._pos_btns.append(b)
        pos_row.addStretch(1)
        v.addLayout(pos_row)
        self.status = QLabel("Pick a calibration; follow the prompts from the vehicle.")
        self.status.setStyleSheet("color:#8fa3bf;")
        v.addWidget(self.status)
        self.log = QListWidget()
        self.log.setFont(_MONO)
        v.addWidget(self.log, 1)

    def _request(self, kind):
        self.log.clear()
        accel = (kind == "accel")
        for b in self._pos_btns:            # position buttons are only for the accel 6-position dance
            b.setEnabled(accel)
        self.status.setText("Accel: place the vehicle in each orientation the log prompts for, then "
                            "click the matching button." if accel else f"{kind} calibration started…")
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
    LEGEND = ""                           # optional value cheat-sheet under the header

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
        if self.LEGEND:
            leg = QLabel(self.LEGEND)
            leg.setWordWrap(True)
            leg.setStyleSheet("color:#6a707c;")
            lay.addWidget(leg)
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


class FlightModesWidget(ParamPage):
    NOUN = "flight-mode"
    LEGEND = ("PX4 slot values -- 0 Manual  1 Altitude  2 Position  3 Mission  4 Hold  "
              "5 Return  6 Acro  7 Offboard  8 Stabilized  10 Takeoff  11 Land  12 Follow  "
              "14 Orbit  (-1 unassigned).")
    PARAMS = [
        ("COM_FLTMODE1",   "PX4: mode slot 1"),
        ("COM_FLTMODE2",   "PX4: mode slot 2"),
        ("COM_FLTMODE3",   "PX4: mode slot 3"),
        ("COM_FLTMODE4",   "PX4: mode slot 4"),
        ("COM_FLTMODE5",   "PX4: mode slot 5"),
        ("COM_FLTMODE6",   "PX4: mode slot 6"),
        ("RC_MAP_FLTMODE", "PX4: mode-selector RC channel"),
        ("RC_MAP_MODE_SW", "PX4: mode-switch RC channel"),
        ("FLTMODE1",       "APM: mode slot 1"),
        ("FLTMODE2",       "APM: mode slot 2"),
        ("FLTMODE3",       "APM: mode slot 3"),
        ("FLTMODE4",       "APM: mode slot 4"),
        ("FLTMODE5",       "APM: mode slot 5"),
        ("FLTMODE6",       "APM: mode slot 6"),
        ("FLTMODE_CH",     "APM: mode-selector RC channel"),
    ]


class TuningWidget(ParamPage):
    NOUN = "tuning / rate-gain"
    LEGEND = ("Rate-controller gains -- change in small steps and test in a safe, "
              "hover-capable mode. Higher P is snappier but can oscillate.")
    PARAMS = [
        ("MC_ROLLRATE_P",   "PX4: roll rate P"),
        ("MC_ROLLRATE_I",   "PX4: roll rate I"),
        ("MC_ROLLRATE_D",   "PX4: roll rate D"),
        ("MC_PITCHRATE_P",  "PX4: pitch rate P"),
        ("MC_PITCHRATE_I",  "PX4: pitch rate I"),
        ("MC_PITCHRATE_D",  "PX4: pitch rate D"),
        ("MC_YAWRATE_P",    "PX4: yaw rate P"),
        ("MC_YAWRATE_I",    "PX4: yaw rate I"),
        ("MC_YAWRATE_D",    "PX4: yaw rate D"),
        ("MC_ROLL_P",       "PX4: roll attitude P"),
        ("MC_PITCH_P",      "PX4: pitch attitude P"),
        ("MC_YAW_P",        "PX4: yaw attitude P"),
        ("MPC_XY_CRUISE",   "PX4: horizontal cruise speed (m/s)"),
        ("MPC_XY_VEL_MAX",  "PX4: max horizontal speed (m/s)"),
        ("MPC_TILTMAX_AIR", "PX4: max tilt in air (deg)"),
        ("ATC_RAT_RLL_P",   "APM: roll rate P"),
        ("ATC_RAT_RLL_I",   "APM: roll rate I"),
        ("ATC_RAT_RLL_D",   "APM: roll rate D"),
        ("ATC_RAT_PIT_P",   "APM: pitch rate P"),
        ("ATC_RAT_PIT_I",   "APM: pitch rate I"),
        ("ATC_RAT_PIT_D",   "APM: pitch rate D"),
        ("ATC_RAT_YAW_P",   "APM: yaw rate P"),
        ("ATC_RAT_YAW_I",   "APM: yaw rate I"),
        ("ATC_RAT_YAW_D",   "APM: yaw rate D"),
    ]


class AirframeWidget(ParamPage):
    NOUN = "airframe / vehicle-type"
    LEGEND = ("MAV_TYPE -- 1 fixed-wing  2 quad  4 heli  10 rover  13 hexa  14 octo  "
              "15 tri  19-20 VTOL tailsitter  21 VTOL tiltrotor  22 VTOL standard.  "
              "SYS_AUTOSTART is the PX4 airframe config ID (e.g. 4001 generic quad, "
              "13000-13050 VTOL).  VT_TYPE -- 0 tailsitter  1 tiltrotor  2 standard.")
    PARAMS = [
        ("SYS_AUTOSTART", "PX4: airframe config ID"),
        ("MAV_TYPE",      "PX4: MAVLink vehicle type"),
        ("VT_TYPE",       "PX4: VTOL type"),
        ("CA_AIRFRAME",   "PX4: control-allocation airframe"),
        ("SYS_HITL",      "PX4: hardware-in-the-loop mode"),
        ("FRAME_CLASS",   "APM: frame class"),
        ("FRAME_TYPE",    "APM: frame type"),
        ("Q_ENABLE",      "APM: quadplane enable"),
    ]


class MotorTestWidget(QWidget):
    """Vehicle Setup > Motors (QGroundControl-style): spin one motor, or all motors in sequence, at
    a low throttle for a few seconds to verify motor order and rotation direction before flight.
    Every spin button is disabled until the operator confirms the propellers are removed -- a spun
    prop is a serious hazard, so this widget refuses to command a motor without that acknowledgement.
    Emits motorTestRequested(motor, throttle_pct, duration_s, count); count=0 tests one motor."""
    motorTestRequested = Signal(int, float, float, int)   # motor(1-based), throttle%, seconds, count

    MAX_THROTTLE = 50            # cap the UI throttle: bench-testing motor order needs only a nudge

    def __init__(self, parent=None):
        super().__init__(parent)
        v = QVBoxLayout(self)
        self.safety = QCheckBox("Propellers are REMOVED — safe to spin motors")
        self.safety.setStyleSheet("color:#e0a030; font-weight:bold;")
        self.safety.toggled.connect(self._on_safety)
        v.addWidget(self.safety)

        form = QHBoxLayout()
        form.addWidget(QLabel("Motors:"))
        self.count = QSpinBox(); self.count.setRange(1, 12); self.count.setValue(4)
        self.count.valueChanged.connect(self._rebuild_buttons)
        form.addWidget(self.count)
        form.addWidget(QLabel("Throttle %:"))
        self.throttle = QSpinBox(); self.throttle.setRange(1, self.MAX_THROTTLE); self.throttle.setValue(8)
        form.addWidget(self.throttle)
        form.addWidget(QLabel("Seconds:"))
        self.duration = QSpinBox(); self.duration.setRange(1, 10); self.duration.setValue(2)
        form.addWidget(self.duration)
        form.addStretch(1)
        v.addLayout(form)

        self._grid = QGridLayout()
        self._motor_btns = []
        v.addLayout(self._grid)

        self.btn_all = QPushButton("Test ALL in sequence")
        self.btn_all.clicked.connect(self._test_all)
        v.addWidget(self.btn_all)

        self.hint = QLabel("Remove propellers, tick the box, then test each motor. Motor 1 should be "
                           "the one your autopilot calls motor 1 (check your airframe diagram).")
        self.hint.setWordWrap(True)
        self.hint.setStyleSheet("color:#8fa3bf;")
        v.addWidget(self.hint)
        v.addStretch(1)

        self._rebuild_buttons()
        self._on_safety(False)

    def _rebuild_buttons(self):
        for b in self._motor_btns:
            b.setParent(None)
        self._motor_btns = []
        for i in range(self.count.value()):
            b = QPushButton(f"Motor {i + 1}")
            b.clicked.connect(lambda _=False, n=i + 1: self._test_one(n))
            self._grid.addWidget(b, i // 4, i % 4)
            self._motor_btns.append(b)
        self._on_safety(self.safety.isChecked())

    def _on_safety(self, ok):
        for b in self._motor_btns:
            b.setEnabled(ok)
        self.btn_all.setEnabled(ok)

    def _test_one(self, motor):
        if not self.safety.isChecked():
            return
        self.motorTestRequested.emit(motor, float(self.throttle.value()),
                                     float(self.duration.value()), 0)
        self.hint.setText(f"Spinning motor {motor} at {self.throttle.value()}% for "
                          f"{self.duration.value()}s. Confirm it spins in the expected direction.")

    def _test_all(self):
        if not self.safety.isChecked():
            return
        n = self.count.value()
        self.motorTestRequested.emit(1, float(self.throttle.value()),
                                     float(self.duration.value()), n)
        self.hint.setText(f"Spinning all {n} motors in sequence at {self.throttle.value()}% -- watch "
                          f"the order matches your airframe's numbering.")


class CalibrationDialog(QDialog):
    """Setup view: Radio + Sensors tabs, fed live from the link while open."""

    def __init__(self, link_getter, param_mgr=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Vehicle Setup — Calibration")
        self.resize(560, 460)
        self._link = link_getter
        self.radio = RcCalibrationWidget()
        self.sensor = SensorCalibrationWidget()
        self.motors = MotorTestWidget()
        tabs = QTabWidget()
        tabs.addTab(self.radio, "Radio")
        tabs.addTab(self.sensor, "Sensors")
        tabs.addTab(self.motors, "Motors")
        if param_mgr is not None and hasattr(param_mgr, "updated"):
            self.safety = SafetyWidget(param_mgr)
            tabs.addTab(self.safety, "Safety")
            self.power = PowerWidget(param_mgr)
            tabs.addTab(self.power, "Power")
            self.flightmodes = FlightModesWidget(param_mgr)
            tabs.addTab(self.flightmodes, "Flight Modes")
            self.tuning = TuningWidget(param_mgr)
            tabs.addTab(self.tuning, "Tuning")
            self.airframe = AirframeWidget(param_mgr)
            tabs.addTab(self.airframe, "Airframe")
            self.safety.refresh()          # auto-request params on open
            self.power.refresh()
            self.flightmodes.refresh()
            self.tuning.refresh()
            self.airframe.refresh()
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
