"""panels.py -- text telemetry readouts grouped like a GCS sidebar."""
from __future__ import annotations

import math
import time


def _num(x, fmt, suffix=""):
    """Format a telemetry float, or '--' if it is NaN/Inf. Autopilots emit non-finite speeds,
    climb and vibration during init / before the estimator converges -- showing 'nan m/s' to a
    pilot is worse than showing nothing."""
    try:
        return format(x, fmt) + suffix if math.isfinite(x) else "--"
    except (TypeError, ValueError):
        return "--"

from PySide6.QtCore import Qt, QRectF, QPointF, Signal
from PySide6.QtGui import QFont, QColor, QPainter, QPen, QTextCursor
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QFormLayout, QGroupBox, QLabel,
                               QListWidget, QListWidgetItem, QTableWidget,
                               QTableWidgetItem, QHeaderView, QPushButton, QSlider,
                               QSpinBox, QHBoxLayout, QGridLayout, QProgressBar,
                               QAbstractItemView, QPlainTextEdit, QLineEdit, QMenu, QComboBox)

import mavlink

_MONO = QFont("DejaVu Sans Mono", 10)

# ESTIMATOR_STATUS flag bits -> (short label, is_risk). Risk bits are alarming when SET;
# capability bits are reassuring when set. Order matches the MAVLink bit order.
EKF_FLAG_BITS = [
    (mavlink.ESTIMATOR_ATTITUDE, "att", False),
    (mavlink.ESTIMATOR_VELOCITY_HORIZ, "vel-h", False),
    (mavlink.ESTIMATOR_VELOCITY_VERT, "vel-v", False),
    (mavlink.ESTIMATOR_POS_HORIZ_REL, "pos-h-rel", False),
    (mavlink.ESTIMATOR_POS_HORIZ_ABS, "pos-h-abs", False),
    (mavlink.ESTIMATOR_POS_VERT_ABS, "pos-v-abs", False),
    (mavlink.ESTIMATOR_POS_VERT_AGL, "pos-v-agl", False),
    (mavlink.ESTIMATOR_CONST_POS_MODE, "const-pos", True),
    (mavlink.ESTIMATOR_PRED_POS_HORIZ_REL, "pred-h-rel", False),
    (mavlink.ESTIMATOR_PRED_POS_HORIZ_ABS, "pred-h-abs", False),
    (mavlink.ESTIMATOR_GPS_GLITCH, "gps-glitch", True),
    (mavlink.ESTIMATOR_ACCEL_ERROR, "accel-err", True),
]


def ekf_flag_states(flags):
    """[(label, is_set, is_risk), ...] for the estimator status flag bits."""
    return [(label, bool(flags & bit), risk) for bit, label, risk in EKF_FLAG_BITS]


def ekf_flags_html(flags):
    """Render the flag set as coloured chips: risk-set red, capability-set green, unset dim."""
    out = []
    for label, on, risk in ekf_flag_states(flags):
        color = "#e05050" if (on and risk) else "#37d67a" if on else "#55607a"
        out.append(f'<span style="color:{color}">{label}</span>')
    return " ".join(out)


class TelemetryPanel(QWidget):
    groupsChanged = Signal()               # user toggled which groups are shown

    def __init__(self, parent=None):
        super().__init__(parent)
        self.v: dict[str, QLabel] = {}
        self.groups: dict[str, QGroupBox] = {}     # title -> box, for show/hide
        specs = {
            "LINK": [("link", "Status"), ("rate", "Msg rate"), ("counts", "OK / drop"),
                     ("loss", "Packet loss"), ("rc", "RC signal")],
            "FLIGHT": [("mode", "Mode"), ("armed", "State"), ("status", "System"), ("type", "Airframe")],
            "BATTERY": [("voltage", "Voltage"), ("current", "Current"), ("remaining", "Remaining")],
            "POSITION": [("lat", "Latitude"), ("lon", "Longitude"), ("alt_msl", "Alt MSL"),
                         ("alt_rel", "Alt rel"), ("rangefinder", "Rangefinder"), ("terrain", "Terrain AGL")],
            "MOTION": [("gspeed", "Ground spd"), ("aspeed", "Air spd"), ("climb", "Climb"),
                       ("throttle", "Throttle"), ("hdg", "Heading"), ("wind", "Wind"),
                       ("vibe", "Vibration")],
            "NAVIGATION": [("home_dist", "Dist to home"), ("home_alt", "Home alt"),
                           ("flight_time", "Flight time"), ("home_eta", "Home ETA"),
                           ("wp_num", "Waypoint"), ("wp_dist", "Dist to WP"),
                           ("wp_eta", "WP ETA"), ("rtl_time", "RTL time"),
                           ("mission_eta", "Mission ETA"), ("odometer", "Distance flown")],
            "GPS": [("fix", "Fix"), ("sats", "Satellites"), ("hdop", "HDOP"), ("pos_acc", "Pos acc"),
                    ("rtk", "RTK")],
        }
        # Wide-and-short: this panel lives in a wide bottom dock, so the groups flow
        # across columns instead of one tall stack (kills the old cramped scroll).
        columns = [["LINK", "GPS"], ["FLIGHT", "BATTERY"], ["POSITION", "NAVIGATION"], ["MOTION"]]
        outer = QHBoxLayout(self)
        outer.setContentsMargins(6, 6, 6, 6)
        outer.setSpacing(8)
        for col in columns:
            cv = QVBoxLayout()
            cv.setSpacing(6)
            for title in col:
                cv.addWidget(self._group(title, specs[title]))
            cv.addStretch(1)
            outer.addLayout(cv)
        outer.addStretch(1)

    def _group(self, title, rows):
        box = QGroupBox(title)
        form = QFormLayout(box)
        form.setLabelAlignment(Qt.AlignRight)
        form.setContentsMargins(10, 6, 10, 6)
        form.setVerticalSpacing(3)
        for key, label in rows:
            val = QLabel("--")
            val.setFont(_MONO)
            self.v[key] = val
            form.addRow(QLabel(label), val)
        self.groups[title] = box
        return box

    def contextMenuEvent(self, e):
        """Right-click to choose which telemetry groups are shown (QGC-style)."""
        menu = QMenu(self)
        menu.addAction("Telemetry groups").setEnabled(False)
        menu.addSeparator()
        for title, box in self.groups.items():
            act = menu.addAction(title)
            act.setCheckable(True)
            act.setChecked(not box.isHidden())
            act.toggled.connect(lambda on, b=box: (b.setVisible(on), self.groupsChanged.emit()))
        self._ctx_menu = menu                       # keep a ref so it isn't GC'd
        menu.exec(e.globalPos())

    def hidden_groups(self):
        return [t for t, b in self.groups.items() if b.isHidden()]

    def set_hidden_groups(self, titles):
        want = set(titles or ())
        for title, box in self.groups.items():
            box.setVisible(title not in want)

    def _set(self, key, text, color=None):
        lbl = self.v[key]
        lbl.setText(text)
        lbl.setStyleSheet(f"color:{color};" if color else "")

    def update_all(self, ve, link_state: str, rate: float, ok: int, drop: int, nav=None, loss=None):
        alive = ve.link_alive
        self._set("link", link_state, "#37d67a" if alive else "#e0a030")
        self._set("rate", f"{rate:5.1f} Hz")
        self._set("counts", f"{ok} / {drop}", "#e05050" if drop else None)
        if loss is None:
            self._set("loss", "--")
        else:
            self._set("loss", f"{loss:.1f} %",
                      "#37d67a" if loss < 2.0 else "#e0a030" if loss < 10.0 else "#e05050")
        if ve.rc_rssi is None:
            self._set("rc", "--")
        else:
            self._set("rc", f"{ve.rc_rssi} %",
                      "#37d67a" if ve.rc_rssi >= 60 else "#e0a030" if ve.rc_rssi >= 30 else "#e05050")

        self._set("mode", ve.mode, "#8fd0ff")
        self._set("armed", "ARMED" if ve.armed else "DISARMED",
                  "#e05050" if ve.armed else "#37d67a")
        self._set("status", {0: "Uninit", 1: "Boot", 2: "Calibrating", 3: "Standby",
                             4: "Active", 5: "Critical", 6: "Emergency", 7: "Poweroff",
                             8: "Terminate"}.get(ve.system_status, "--"),
                  "#e05050" if ve.system_status >= 5 else None)
        self._set("type", ve.type_text)

        vcolor = None
        if ve.voltage and ve.voltage < 10.5:
            vcolor = "#e05050"
        self._set("voltage", f"{ve.voltage:5.2f} V" if ve.voltage else "--", vcolor)
        self._set("current", f"{ve.current:5.1f} A" if ve.current else "--")
        rem = ve.battery_remaining
        self._set("remaining", f"{rem} %" if rem >= 0 else "--",
                  "#e05050" if 0 <= rem < 20 else None)

        if ve.have_position:
            self._set("lat", f"{ve.lat:.6f}")
            self._set("lon", f"{ve.lon:.6f}")
        else:
            self._set("lat", "--"); self._set("lon", "--")
        self._set("alt_msl", f"{ve.alt_msl:7.1f} m")
        self._set("alt_rel", f"{ve.alt_rel:7.1f} m")
        if ve.rangefinder_m is None:
            self._set("rangefinder", "--")
        else:
            # amber if below the sensor's own min range (unreliable), else green
            lo = ve.rangefinder_min_m or 0.0
            self._set("rangefinder", f"{ve.rangefinder_m:6.2f} m",
                      "#e0a030" if ve.rangefinder_m < lo else "#37d67a")
        if ve.terrain_agl_m is None:
            self._set("terrain", "--")
        elif ve.terrain_pending:                          # tiles still loading -> amber
            self._set("terrain", f"{ve.terrain_agl_m:6.1f} m  ({ve.terrain_pending} pend)", "#e0a030")
        else:
            self._set("terrain", f"{ve.terrain_agl_m:6.1f} m", "#37d67a")

        self._set("gspeed", _num(ve.groundspeed, "5.1f", " m/s"))
        self._set("aspeed", _num(ve.airspeed, "5.1f", " m/s"))
        self._set("climb", _num(ve.climb, "+5.1f", " m/s"))
        self._set("throttle", f"{ve.throttle} %")
        self._set("hdg", f"{ve.heading:5.1f} deg")
        if ve.have_wind:
            self._set("wind", f"{ve.wind_speed():4.1f} m/s from {ve.wind_dir():3.0f} deg")
        else:
            self._set("wind", "--")
        vpk = max(ve.vibration) if ve.have_vibration else None
        if vpk is not None and math.isfinite(vpk):
            # peak axis vibration; PX4 rule of thumb: <30 good, 30-60 caution, >60 bad
            self._set("vibe", f"{vpk:4.1f} m/s2",
                      "#37d67a" if vpk < 30 else "#e0a030" if vpk < 60 else "#e05050")
        else:
            self._set("vibe", "--")

        self._set("fix", ve.fix_text, "#37d67a" if ve.fix_type >= 3 else "#e0a030")
        self._set("sats", str(ve.satellites))
        if ve.eph is not None:
            hd = ve.eph / 100.0
            self._set("hdop", f"{hd:.2f}",
                      "#37d67a" if hd < 2 else "#e0a030" if hd < 5 else "#e05050")
        else:
            self._set("hdop", "--")
        if ve.pos_horiz_acc is not None:
            h, vv = ve.pos_horiz_acc, ve.pos_vert_acc or 0.0
            self._set("pos_acc", f"H {h:.1f} / V {vv:.1f} m",
                      "#37d67a" if h < 2 else "#e0a030" if h < 5 else "#e05050")
        else:
            self._set("pos_acc", "--")
        if ve.rtk_health is not None:                     # GPS_RTK reporting -> RTK receiver present
            healthy = ve.rtk_health > 0
            self._set("rtk", f"{'OK' if healthy else 'no fix'}  {ve.rtk_nsats or 0} sat  "
                             f"+/-{ve.rtk_accuracy_mm or 0} mm",
                      "#37d67a" if healthy else "#e0a030")
        else:
            self._set("rtk", "--")

        nav = nav or {}
        self._set("home_dist", nav.get("home_dist", "--"))
        self._set("home_alt", nav.get("home_alt", "--"))
        self._set("flight_time", nav.get("flight_time", "--"))
        self._set("home_eta", nav.get("home_eta", "--"))
        self._set("wp_num", nav.get("wp_num", "--"))
        self._set("wp_dist", nav.get("wp_dist", "--"))
        self._set("wp_eta", nav.get("wp_eta", "--"))
        self._set("rtl_time", nav.get("rtl_time", "--"))
        self._set("mission_eta", nav.get("mission_eta", "--"))
        self._set("odometer", nav.get("odometer", "--"))


class LogPanel(QWidget):
    """Onboard log browser: refresh the list, pick a log, download it with a
    progress bar. Emits intent signals wired to a LogManager in main."""

    refreshRequested = Signal()
    downloadRequested = Signal(int)        # log id

    def __init__(self, parent=None):
        super().__init__(parent)
        v = QVBoxLayout(self)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(4)
        row = QHBoxLayout()
        self.btn_refresh = QPushButton("Refresh list")
        self.btn_refresh.clicked.connect(self.refreshRequested.emit)
        self.btn_dl = QPushButton("Download")
        self.btn_dl.clicked.connect(self._download)
        row.addWidget(self.btn_refresh)
        row.addWidget(self.btn_dl)
        row.addStretch(1)
        v.addLayout(row)
        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["#", "Size", "Date (UTC)"])
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.setFont(_MONO)
        v.addWidget(self.table)
        self.bar = QProgressBar()
        self.bar.setTextVisible(True)
        v.addWidget(self.bar)
        self.status = QLabel("idle")
        self.status.setStyleSheet("color:#8fa3bf;")
        v.addWidget(self.status)

    def set_entries(self, entries):
        self.table.setRowCount(len(entries))
        for r, e in enumerate(entries):
            kib = e["size"] / 1024.0
            size = f"{kib:.1f} KiB" if kib < 1024 else f"{kib / 1024:.1f} MiB"
            utc = e.get("time_utc", 0)
            date = time.strftime("%Y-%m-%d %H:%M", time.gmtime(utc)) if utc else "--"
            id_item = QTableWidgetItem(str(e["id"]))
            id_item.setData(Qt.UserRole, e["id"])
            self.table.setItem(r, 0, id_item)
            self.table.setItem(r, 1, QTableWidgetItem(size))
            self.table.setItem(r, 2, QTableWidgetItem(date))
        self.status.setText(f"{len(entries)} log(s)" if entries else "no logs on vehicle")
        if entries:
            self.table.selectRow(0)

    def set_progress(self, got, total):
        self.bar.setMaximum(max(1, total))
        self.bar.setValue(got)
        self.status.setText(f"downloading... {got}/{total} bytes")

    def set_status(self, text):
        self.status.setText(text)

    def _download(self):
        items = self.table.selectedItems()
        if items:
            lid = self.table.item(items[0].row(), 0).data(Qt.UserRole)
            self.downloadRequested.emit(int(lid))


class CameraPanel(QWidget):
    """Camera trigger / video / trigger-distance and a gimbal pitch+yaw control.
    Emits intent signals; main wires them to the link's COMMAND_LONG helpers."""

    photoRequested = Signal()
    videoToggled = Signal(bool)
    triggerDistance = Signal(float)
    gimbalChanged = Signal(float, float)        # pitch, yaw degrees
    cameraMode = Signal(int)                    # 0 = photo/image, 1 = video
    cameraZoom = Signal(float)                  # +1 zoom in / -1 zoom out (one step)

    def __init__(self, parent=None):
        super().__init__(parent)
        g = QGridLayout(self)
        g.setContentsMargins(8, 6, 8, 6)
        g.setVerticalSpacing(6)

        self.btn_photo = QPushButton("Photo")
        self.btn_photo.clicked.connect(self.photoRequested.emit)
        self.btn_video = QPushButton("Video ●")
        self.btn_video.setCheckable(True)
        self.btn_video.toggled.connect(self._on_video)
        g.addWidget(self.btn_photo, 0, 0)
        g.addWidget(self.btn_video, 0, 1)

        g.addWidget(QLabel("Trigger dist"), 1, 0)
        trow = QHBoxLayout()
        self.trig = QSpinBox()
        self.trig.setRange(0, 1000)
        self.trig.setSuffix(" m")
        btn_trig = QPushButton("Set")
        btn_trig.clicked.connect(lambda: self.triggerDistance.emit(float(self.trig.value())))
        trow.addWidget(self.trig)
        trow.addWidget(btn_trig)
        g.addLayout(trow, 1, 1)

        self.pitch = QSlider(Qt.Horizontal)
        self.pitch.setRange(-90, 30)
        self.yaw = QSlider(Qt.Horizontal)
        self.yaw.setRange(-180, 180)
        self.lbl_pitch = QLabel("pitch   0°")
        self.lbl_yaw = QLabel("yaw   0°")
        self.lbl_pitch.setFont(_MONO)
        self.lbl_yaw.setFont(_MONO)
        for s in (self.pitch, self.yaw):
            s.valueChanged.connect(self._on_gimbal_label)
            s.sliderReleased.connect(self._emit_gimbal)
        g.addWidget(self.lbl_pitch, 2, 0)
        g.addWidget(self.pitch, 2, 1)
        g.addWidget(self.lbl_yaw, 3, 0)
        g.addWidget(self.yaw, 3, 1)
        btn_center = QPushButton("Center gimbal")
        btn_center.clicked.connect(self._center)
        g.addWidget(btn_center, 4, 0)
        # gimbal protocol: v1 DO_MOUNT_CONTROL (default, widest support) or v2 gimbal manager
        # (DO_GIMBAL_MANAGER_PITCHYAW -- what QGC uses for modern gimbals)
        self.gimbal_proto = QComboBox()
        self.gimbal_proto.addItems(["Mount v1", "Manager v2"])
        self.gimbal_proto.setToolTip("Gimbal protocol: v1 DO_MOUNT_CONTROL or v2 gimbal manager")
        g.addWidget(self.gimbal_proto, 4, 1)

        # camera mode (photo/video) + zoom -- SET_CAMERA_MODE / SET_CAMERA_ZOOM
        self.btn_mode = QPushButton("Mode: Photo")
        self.btn_mode.setCheckable(True)
        self.btn_mode.toggled.connect(self._on_mode)
        g.addWidget(self.btn_mode, 5, 0, 1, 2)
        zrow = QHBoxLayout()
        zrow.addWidget(QLabel("Zoom"))
        btn_zoom_out = QPushButton("−")
        btn_zoom_out.clicked.connect(lambda: self.cameraZoom.emit(-1.0))
        btn_zoom_in = QPushButton("+")
        btn_zoom_in.clicked.connect(lambda: self.cameraZoom.emit(1.0))
        zrow.addWidget(btn_zoom_out)
        zrow.addWidget(btn_zoom_in)
        g.addLayout(zrow, 6, 0, 1, 2)
        # live camera storage + capture state (STORAGE_INFORMATION / CAMERA_CAPTURE_STATUS)
        self.cam_status = QLabel("")
        self.cam_status.setFont(_MONO)
        self.cam_status.setWordWrap(True)
        g.addWidget(self.cam_status, 7, 0, 1, 2)
        g.setRowStretch(8, 1)

    def _on_video(self, on):
        self.btn_video.setText("Video ■" if on else "Video ●")
        self.btn_video.setStyleSheet("color:#e05050;" if on else "")
        self.videoToggled.emit(on)

    def _on_mode(self, video):
        self.btn_mode.setText("Mode: Video" if video else "Mode: Photo")
        self.cameraMode.emit(1 if video else 0)

    def _on_gimbal_label(self):
        self.lbl_pitch.setText(f"pitch {self.pitch.value():4d}°")
        self.lbl_yaw.setText(f"yaw {self.yaw.value():4d}°")

    def _emit_gimbal(self):
        self.gimbalChanged.emit(float(self.pitch.value()), float(self.yaw.value()))

    def _center(self):
        self.pitch.setValue(0)
        self.yaw.setValue(0)
        self._emit_gimbal()

    _CAM_MODE = {0: "PHOTO", 1: "VIDEO", 2: "SURVEY"}

    def update_status(self, ve):
        """Refresh the mode + storage + capture readout from the vehicle's camera telemetry (or blank)."""
        parts = []
        if ve is not None and ve.cam_mode is not None:
            parts.append(self._CAM_MODE.get(ve.cam_mode, f"mode {ve.cam_mode}"))
        if ve is not None and ve.storage_available_mb is not None:
            total = (ve.storage_total_mb or 0.0) / 1024.0
            avail = ve.storage_available_mb / 1024.0
            parts.append(f"SD {avail:.1f}/{total:.1f} GB")
        if ve is not None and ve.cam_recording is not None:
            parts.append(f"REC {int(ve.cam_recording_time_s or 0)}s" if ve.cam_recording else "not rec")
        if ve is not None and ve.cam_images_captured is not None:
            parts.append(f"img {ve.cam_images_captured}")
        self.cam_status.setText("   ".join(parts))


class MavlinkConsole(QWidget):
    """Interactive shell to the autopilot's nsh over SERIAL_CONTROL (PX4). Type a
    command, press Enter; output streams back as SERIAL_CONTROL replies."""
    send_bytes = Signal(bytes)          # command bytes -> SERIAL_CONTROL

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(3, 3, 3, 3)
        lay.setSpacing(3)
        self.out = QPlainTextEdit()
        self.out.setReadOnly(True)
        self.out.setFont(QFont("DejaVu Sans Mono", 9))
        self.out.setMaximumBlockCount(4000)
        self.out.setPlaceholderText("PX4 nsh console -- connect, then type e.g. 'ver', 'help', "
                                    "'listener sensor_accel' and press Enter.")
        lay.addWidget(self.out, 1)
        row = QHBoxLayout()
        self.inp = QLineEdit()
        self.inp.setFont(QFont("DejaVu Sans Mono", 9))
        self.inp.setPlaceholderText("command + Enter")
        self.inp.returnPressed.connect(self._send)
        btn = QPushButton("Send")
        btn.clicked.connect(self._send)
        row.addWidget(self.inp, 1)
        row.addWidget(btn)
        lay.addLayout(row)

    def _send(self):
        cmd = self.inp.text()
        self.inp.clear()
        self.send_bytes.emit((cmd + "\n").encode("utf-8", "replace"))

    def handle_messages(self, batch):
        chunk = bytearray()
        for m in batch:
            if m.msgid == mavlink.SERIAL_CONTROL:
                n = int(m.fields.get("count", 0))
                chunk += bytes(m.fields.get("data", b""))[:n]
        if chunk:
            text = chunk.decode("utf-8", "replace").replace("\r\n", "\n").replace("\r", "")
            self.out.moveCursor(QTextCursor.End)
            self.out.insertPlainText(text)
            self.out.moveCursor(QTextCursor.End)


class StatusStrip(QWidget):
    """QGroundControl-style top indicator bar: a row of colour-coded chips
    (armed/mode, GPS, battery, link, flight time, messages)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(38)
        self.chips = []                  # (label, dot_color, text_color[, fill_color])

    def set_chips(self, chips):
        self.chips = chips
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(18, 20, 26))
        p.setPen(QColor(40, 44, 52))
        p.drawLine(0, self.height() - 1, self.width(), self.height() - 1)
        h = self.height()
        x = 8.0
        for chip in self.chips:
            label, dot, txt = chip[0], chip[1], chip[2]
            fill = chip[3] if len(chip) > 3 else None
            if fill is not None:
                # bold filled badge (arm state) -- no dot, dark text on bright fill
                p.setFont(QFont("DejaVu Sans", 11, QFont.Bold))
                tw = p.fontMetrics().horizontalAdvance(label)
                w = tw + 24
                rect = QRectF(x, 5, w, h - 11)
                p.setPen(Qt.NoPen)
                p.setBrush(QColor(fill))
                p.drawRoundedRect(rect, 6, 6)
                p.setPen(QColor("#0d0f14"))
                p.drawText(rect, Qt.AlignCenter, label)
            else:
                p.setFont(QFont("DejaVu Sans", 10, QFont.Bold))
                tw = p.fontMetrics().horizontalAdvance(label)
                w = tw + 30
                rect = QRectF(x, 5, w, h - 11)
                p.setBrush(QColor(30, 33, 40))
                p.setPen(QPen(QColor(48, 52, 62), 1))
                p.drawRoundedRect(rect, 6, 6)
                p.setPen(Qt.NoPen)
                p.setBrush(QColor(dot))
                p.drawEllipse(QPointF(x + 14, h / 2.0), 5.0, 5.0)
                p.setPen(QColor(txt))
                p.drawText(QRectF(x + 24, 5, tw + 6, h - 11),
                           Qt.AlignVCenter | Qt.AlignLeft, label)
            x += w + 7
        p.end()


class HealthPanel(QWidget):
    """Compact sensor-health grid from the SYS_STATUS bitmasks (QGC style):
    green = present + healthy, amber = present but unhealthy, grey = absent."""

    GREEN = QColor("#37d67a")
    AMBER = QColor("#e0a030")
    GREY = QColor("#4a4f5a")

    def __init__(self, parent=None):
        super().__init__(parent)
        self.present = self.enabled = self.health = 0
        self.cols = 2
        rows = (len(mavlink.SENSOR_BITS) + self.cols - 1) // self.cols
        self._rh = 16
        self.setMinimumHeight(rows * self._rh + 8)

    def set_health(self, present, enabled, health):
        self.present, self.enabled, self.health = present, enabled, health
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        items = mavlink.SENSOR_BITS
        rows = (len(items) + self.cols - 1) // self.cols
        cw = self.width() / self.cols
        p.setFont(QFont("DejaVu Sans", 8))
        for idx, (bit, label) in enumerate(items):
            col, row = idx // rows, idx % rows
            x = col * cw + 8
            y = 4 + row * self._rh
            if not (self.present & bit):
                dot, txt = self.GREY, QColor("#6a6f7a")
            elif self.health & bit:
                dot, txt = self.GREEN, QColor("#c4c8d0")
            else:
                dot, txt = self.AMBER, QColor("#e8c070")
            p.setPen(Qt.NoPen)
            p.setBrush(dot)
            p.drawEllipse(QPointF(x + 4, y + self._rh / 2), 4, 4)
            p.setPen(txt)
            p.drawText(QRectF(x + 14, y, cw - 20, self._rh),
                       Qt.AlignVCenter | Qt.AlignLeft, label)
        p.end()


class MessageConsole(QListWidget):
    """Scrolling, severity-colored log of STATUSTEXT (and local notes)."""

    SEV_COLOR = {0: "#ff5050", 1: "#ff5050", 2: "#ff6a3d", 3: "#ff6a3d",
                 4: "#e0a030", 5: "#39c0d0", 6: "#c4c8d0", 7: "#7a8090"}

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFont(QFont("DejaVu Sans Mono", 9))
        self.setUniformItemSizes(True)
        self.setSelectionMode(QListWidget.NoSelection)
        self.threshold = 7                 # show severities <= threshold (7 = everything)

    def _add(self, item, severity):
        item.setData(Qt.UserRole, int(severity))
        self.addItem(item)
        item.setHidden(int(severity) > self.threshold)   # only sticks once in the list
        while self.count() > 300:
            self.takeItem(0)
        self.scrollToBottom()

    def add_message(self, severity, text):
        sev = mavlink.MAV_SEVERITY.get(severity, str(severity))
        item = QListWidgetItem(f"[{sev}] {text}")
        item.setForeground(QColor(self.SEV_COLOR.get(severity, "#c4c8d0")))
        self._add(item, severity)

    def add_note(self, text, color="#8fd0ff"):
        item = QListWidgetItem(text)
        item.setForeground(QColor(color))
        self._add(item, 0)                 # local notes always show (0 <= any threshold)

    def set_threshold(self, sev):
        """Hide STATUSTEXT lines less severe than `sev` (higher MAV_SEVERITY number)."""
        self.threshold = int(sev)
        for i in range(self.count()):
            it = self.item(i)
            s = it.data(Qt.UserRole)
            it.setHidden((s if s is not None else 0) > self.threshold)


class MavInspector(QWidget):
    """Live table of every received message type: rate (Hz) and last field values.

    Like QGroundControl's MAVLink Inspector. Counts are accumulated from the link
    stream; refresh() (called from the UI timer) recomputes per-type rates and
    repaints, throttled so a busy stream doesn't thrash the table."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.stats = {}                 # msgid -> dict(name, count, last_count, hz, fields)
        self._last_t = time.monotonic()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Message", "ID", "Hz", "Fields"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionMode(QTableWidget.NoSelection)
        self.table.setFont(QFont("DejaVu Sans Mono", 9))
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(3, QHeaderView.Stretch)
        lay.addWidget(self.table)

    def consume(self, batch):
        for m in batch:
            s = self.stats.get(m.msgid)
            if s is None:
                s = {"name": m.name, "count": 0, "last_count": 0, "hz": 0.0, "fields": {}}
                self.stats[m.msgid] = s
            s["count"] += 1
            s["fields"] = m.fields

    def refresh(self):
        now = time.monotonic()
        dt = now - self._last_t
        if dt < 0.25:                   # cap table repaint at ~4 Hz
            return
        self._last_t = now
        for s in self.stats.values():
            s["hz"] = (s["count"] - s["last_count"]) / dt
            s["last_count"] = s["count"]
        order = sorted(self.stats.keys())
        if self.table.rowCount() != len(order):
            self.table.setRowCount(len(order))
        for row, mid in enumerate(order):
            s = self.stats[mid]
            self._set(row, 0, s["name"])
            self._set(row, 1, str(mid))
            self._set(row, 2, f"{s['hz']:4.1f}")
            self._set(row, 3, self._fmt_fields(s["fields"]))

    def _set(self, row, col, text):
        it = self.table.item(row, col)
        if it is None:
            it = QTableWidgetItem(text)
            if col == 3:
                it.setForeground(QColor("#9aa4b2"))
            self.table.setItem(row, col, it)
        elif it.text() != text:
            it.setText(text)

    @staticmethod
    def _fmt_fields(fields):
        parts = []
        for k, v in fields.items():
            parts.append(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}")
        return "  ".join(parts)[:240]


class TrafficPanel(QWidget):
    """Textual ADSB traffic list (callsign / ICAO / altitude / distance / bearing), the
    tabular companion to the map targets -- like QGroundColor's ADSB view. Rows are built
    by the main window (which owns the geo helpers) and pushed in via set_rows()."""
    COLS = ["Callsign", "ICAO", "Alt", "Dist", "Brg"]

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        self.table = QTableWidget(0, len(self.COLS))
        self.table.setHorizontalHeaderLabels(self.COLS)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QAbstractItemView.NoSelection)
        self.table.verticalHeader().setVisible(False)
        self.table.setFont(_MONO)
        lay.addWidget(self.table)
        self.status = QLabel("no traffic")
        self.status.setStyleSheet("color:#8fa3bf;")
        lay.addWidget(self.status)

    def set_rows(self, rows):
        """rows: list of 5-tuples of display strings (callsign, icao, alt, dist, brg)."""
        self.table.setRowCount(len(rows))
        for r, cols in enumerate(rows):
            for c, val in enumerate(cols):
                self.table.setItem(r, c, QTableWidgetItem(str(val)))
        self.status.setText(f"{len(rows)} target{'' if len(rows) == 1 else 's'}"
                            if rows else "no traffic")


class _VibBars(QWidget):
    """Three horizontal vibration bars (x/y/z), coloured by severity."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(60)
        self.vals = (0.0, 0.0, 0.0)

    def set_values(self, vals):
        self.vals = vals
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(22, 24, 29))
        MAXV = 80.0
        for i, (lab, v) in enumerate(zip(("X", "Y", "Z"), self.vals)):
            y = 5 + i * 18
            track = QRectF(24, y, self.width() - 74, 12)
            p.setPen(QColor(150, 156, 166))
            p.drawText(QRectF(4, y - 2, 18, 16), Qt.AlignVCenter, lab)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(38, 42, 50))
            p.drawRoundedRect(track, 3, 3)
            # non-finite (a drone can stream NaN vibration): show nothing, NOT a full red bar --
            # frac would clamp NaN->1.0 and the colour fall through to red, a false critical alarm
            finite = math.isfinite(v)
            frac = max(0.0, min(1.0, v / MAXV)) if finite else 0.0
            col = (QColor(120, 210, 120) if v < 30 else
                   QColor(230, 180, 60) if v < 60 else QColor(230, 90, 70))
            p.setBrush(col)
            p.drawRoundedRect(QRectF(track.left(), track.top(), track.width() * frac,
                                     track.height()), 3, 3)
            p.setPen(QColor(220, 224, 230))
            p.drawText(QRectF(track.right() + 4, y - 2, 46, 16), Qt.AlignVCenter,
                       f"{v:.0f}" if finite else "--")
        p.end()


class _CellBars(QWidget):
    """One vertical bar per battery cell, filled by voltage over the LiPo 3.2-4.2 V range
    and coloured green (>=3.7) / amber (>=3.5) / red, with the reading beneath each bar."""
    LO, HI = 3.2, 4.2

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(48)
        self.cells = []

    def set_cells(self, cells):
        self.cells = list(cells or [])
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(22, 24, 29))
        if not self.cells:
            p.setPen(QColor(120, 126, 136))
            p.drawText(self.rect(), Qt.AlignCenter, "no cell data")
            p.end()
            return
        n = len(self.cells)
        gap = 4.0
        bw = max(6.0, (self.width() - (n + 1) * gap) / n)
        top, bot = 4.0, self.height() - 14.0
        H = max(1.0, bot - top)
        for i, v in enumerate(self.cells):
            x = gap + i * (bw + gap)
            finite = math.isfinite(v)
            frac = max(0.0, min(1.0, (v - self.LO) / (self.HI - self.LO))) if finite else 0.0
            col = (QColor(120, 210, 120) if v >= 3.7 else
                   QColor(230, 180, 60) if v >= 3.5 else QColor(230, 90, 70))
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(38, 42, 50))
            p.drawRoundedRect(QRectF(x, top, bw, H), 2, 2)
            fh = H * frac
            p.setBrush(col)
            p.drawRoundedRect(QRectF(x, bot - fh, bw, fh), 2, 2)
            p.setPen(QColor(210, 214, 220))
            p.setFont(QFont("DejaVu Sans", 7))
            p.drawText(QRectF(x - 2, bot + 1, bw + 4, 12), Qt.AlignCenter,
                       f"{v:.2f}" if finite else "--")
        p.end()


class _ServoBars(QWidget):
    """Eight actuator-output bars (servo1..8), PWM microseconds scaled over 1000-2000 us.
    Channels reading near zero are treated as inactive (drawn empty with a dash)."""
    LO, HI = 1000.0, 2000.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(52)
        self.vals = [0] * 8

    def set_values(self, vals):
        vals = list(vals)[:8]
        self.vals = vals + [0] * (8 - len(vals))
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(22, 24, 29))
        n = 8
        gap = 4.0
        bw = max(6.0, (self.width() - (n + 1) * gap) / n)
        top, bot = 4.0, self.height() - 14.0
        H = max(1.0, bot - top)
        for i, v in enumerate(self.vals):
            x = gap + i * (bw + gap)
            active = math.isfinite(v) and v > 0       # 0 / non-finite = channel not driven
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(38, 42, 50))
            p.drawRoundedRect(QRectF(x, top, bw, H), 2, 2)
            if active:
                frac = max(0.0, min(1.0, (v - self.LO) / (self.HI - self.LO)))
                fh = H * frac
                if fh > 0:
                    p.setBrush(QColor(90, 170, 230))
                    p.drawRoundedRect(QRectF(x, bot - fh, bw, fh), 2, 2)
            p.setPen(QColor(200, 205, 212))
            p.setFont(QFont("DejaVu Sans", 7))
            p.drawText(QRectF(x - 3, bot + 1, bw + 6, 12), Qt.AlignCenter, str(v) if active else "-")
        p.end()


class SystemsPanel(QWidget):
    """Detailed battery, vibration and altitude readouts, fed from BATTERY_STATUS,
    VIBRATION and ALTITUDE -- the QGC-style 'systems' instrument view."""

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)

        bat = QGroupBox("Battery")
        bf = QFormLayout(bat)
        self.b_volt = QLabel("--"); self.b_curr = QLabel("--"); self.b_used = QLabel("--")
        self.b_rem = QLabel("--"); self.b_temp = QLabel("--"); self.b_cells = QLabel("--")
        self.b_time = QLabel("--")
        bf.addRow("Voltage", self.b_volt)
        bf.addRow("Current", self.b_curr)
        bf.addRow("Consumed", self.b_used)
        bf.addRow("Remaining", self.b_rem)
        bf.addRow("Time left", self.b_time)
        bf.addRow("Temperature", self.b_temp)
        bf.addRow("Cells (V)", self.b_cells)
        self.cell_bars = _CellBars()
        bf.addRow(self.cell_bars)
        lay.addWidget(bat)

        vib = QGroupBox("Vibration")
        vv = QVBoxLayout(vib)
        self.vib_bars = _VibBars()
        vv.addWidget(self.vib_bars)
        self.vib_clip = QLabel("clip 0 / 0 / 0")
        self.vib_clip.setStyleSheet("color:#8fa3bf;")
        vv.addWidget(self.vib_clip)
        lay.addWidget(vib)

        ekf = QGroupBox("Estimator (EKF)")
        ef = QFormLayout(ekf)
        self.e_status = QLabel("--")
        self.e_vel = QLabel("--"); self.e_ph = QLabel("--")
        self.e_pv = QLabel("--"); self.e_comp = QLabel("--")
        ef.addRow("Status", self.e_status)
        ef.addRow("Velocity var", self.e_vel)
        ef.addRow("Pos horiz var", self.e_ph)
        ef.addRow("Pos vert var", self.e_pv)
        ef.addRow("Compass var", self.e_comp)
        self.e_flags = QLabel("--")
        self.e_flags.setWordWrap(True)
        self.e_flags.setTextFormat(Qt.RichText)
        ef.addRow("Flags", self.e_flags)
        lay.addWidget(ekf)

        gmb = QGroupBox("Gimbal")
        gf = QFormLayout(gmb)
        self.g_roll = QLabel("--"); self.g_pitch = QLabel("--"); self.g_yaw = QLabel("--")
        gf.addRow("Roll", self.g_roll)
        gf.addRow("Pitch", self.g_pitch)
        gf.addRow("Yaw", self.g_yaw)
        lay.addWidget(gmb)

        act = QGroupBox("Actuator outputs (us)")
        av = QVBoxLayout(act)
        self.servo_bars = _ServoBars()
        av.addWidget(self.servo_bars)
        lay.addWidget(act)

        escb = QGroupBox("ESC / motors")
        ef = QFormLayout(escb)
        ef.setContentsMargins(10, 6, 10, 6)
        ef.setVerticalSpacing(3)
        self._esc_rows = []                              # (label_widget, value_widget) per ESC
        for i in range(8):
            name = QLabel(f"M{i + 1}")
            val = QLabel("--")
            val.setFont(_MONO)
            ef.addRow(name, val)
            self._esc_rows.append((name, val))
        self._esc_group = escb
        lay.addWidget(escb)

        alt = QGroupBox("Altitude")
        af = QFormLayout(alt)
        self.a_amsl = QLabel("--"); self.a_rel = QLabel("--"); self.a_terr = QLabel("--")
        af.addRow("AMSL", self.a_amsl)
        af.addRow("Above home", self.a_rel)
        af.addRow("Above terrain", self.a_terr)
        lay.addWidget(alt)

        fst = QGroupBox("Flight state")
        ff = QFormLayout(fst)
        self.f_landed = QLabel("--")
        self.f_vtol = QLabel("--")
        self._f_vtol_label = QLabel("VTOL")
        ff.addRow("State", self.f_landed)
        ff.addRow(self._f_vtol_label, self.f_vtol)
        lay.addWidget(fst)

        radio = QGroupBox("Radio link")
        rf = QFormLayout(radio)
        self.r_rssi = QLabel("--"); self.r_remrssi = QLabel("--"); self.r_noise = QLabel("--")
        rf.addRow("RSSI (local)", self.r_rssi)
        rf.addRow("RSSI (remote)", self.r_remrssi)
        rf.addRow("Noise", self.r_noise)
        lay.addWidget(radio)

        # GPS integrity / security (GNSS_INTEGRITY, development dialect) -- hidden until received
        self._gi_group = QGroupBox("GPS integrity")
        gif = QFormLayout(self._gi_group)
        self.gi_jam = QLabel("--"); self.gi_spoof = QLabel("--")
        self.gi_raim = QLabel("--"); self.gi_sig = QLabel("--")
        gif.addRow("Jamming", self.gi_jam)
        gif.addRow("Spoofing", self.gi_spoof)
        gif.addRow("RAIM", self.gi_raim)
        gif.addRow("Signal", self.gi_sig)
        self._gi_group.setVisible(False)
        lay.addWidget(self._gi_group)
        lay.addStretch(1)

    def update_from(self, ve):
        if ve is None:
            return
        self.b_volt.setText(f"{ve.voltage:.2f} V")
        # colour pack voltage by the weakest cell -- but ONLY with genuine per-cell data (every
        # entry in the plausible <5 V Li-cell range). Some autopilots (incl. PX4 SITL) report the
        # whole pack voltage in voltages[0] (e.g. 16.2 V); treating that as a cell would show a
        # false green on a sagging pack, so leave it neutral instead.
        if ve.cells and max(ve.cells) < 5.0:
            lo = min(ve.cells)
            self.b_volt.setStyleSheet("color:#37d67a;" if lo >= 3.7 else
                                      "color:#e0a030;" if lo >= 3.5 else "color:#e05050;")
        else:
            self.b_volt.setStyleSheet("")
        self.b_curr.setText(f"{ve.current:.1f} A")
        self.b_used.setText("--" if ve.battery_consumed < 0 else f"{ve.battery_consumed} mAh")
        rem = ve.battery_remaining      # colour remaining % to match the status-strip chip
        self.b_rem.setText("--" if rem < 0 else f"{rem}%")
        self.b_rem.setStyleSheet("" if rem < 0 else
                                 ("color:#37d67a;" if rem >= 40 else
                                  "color:#e0a030;" if rem >= 20 else "color:#e05050;"))
        est = ve.battery_time_estimate()
        if est < 0:
            self.b_time.setText("--"); self.b_time.setStyleSheet("")
        else:
            s = int(est)
            self.b_time.setText(f"{s // 60}:{s % 60:02d}" if s < 3600
                                else f"{s // 3600}h{(s % 3600) // 60:02d}")
            self.b_time.setStyleSheet("color:#e05050;" if s < 120 else
                                      "color:#e0a030;" if s < 300 else "color:#37d67a;")
        self.b_temp.setText("--" if ve.battery_temp is None else f"{ve.battery_temp:.1f} C")
        if ve.have_ekf:
            def vcol(v):
                return "#37d67a" if v < 0.5 else "#e0a030" if v < 0.8 else "#e05050"
            for lbl, v in ((self.e_vel, ve.ekf_vel_var), (self.e_ph, ve.ekf_pos_horiz_var),
                           (self.e_pv, ve.ekf_pos_vert_var), (self.e_comp, ve.ekf_compass_var)):
                lbl.setText(f"{v:.2f}"); lbl.setStyleSheet(f"color:{vcol(v)};")
            ok = ve.ekf_ok()
            self.e_status.setText("OK" if ok else "CHECK")
            self.e_status.setStyleSheet("color:#37d67a;" if ok else "color:#e05050;")
            self.e_flags.setText(ekf_flags_html(ve.ekf_flags))
        else:
            for lbl in (self.e_vel, self.e_ph, self.e_pv, self.e_comp):
                lbl.setText("--"); lbl.setStyleSheet("")
            self.e_status.setText("--"); self.e_status.setStyleSheet("")
            self.e_flags.setText("--")
        if ve.have_gimbal:
            self.g_roll.setText(f"{ve.gimbal_roll:+.1f} deg")
            self.g_pitch.setText(f"{ve.gimbal_pitch:+.1f} deg")
            self.g_yaw.setText(f"{ve.gimbal_yaw:+.1f} deg")
        else:
            for lbl in (self.g_roll, self.g_pitch, self.g_yaw):
                lbl.setText("--")
        self.b_cells.setText("  ".join(f"{c:.2f}" for c in ve.cells) if ve.cells else "--")
        self.cell_bars.set_cells(ve.cells)
        self.servo_bars.set_values(ve.servo_raw)
        esc_active = [(ve.esc_rpm[i] != 0 or ve.esc_voltage[i] > 0.05) for i in range(8)]
        if ve.have_esc and any(esc_active):     # only meaningful once motors are turning
            self._esc_group.show()
            for i, (name, val) in enumerate(self._esc_rows):
                name.setVisible(esc_active[i])
                val.setVisible(esc_active[i])
                if esc_active[i]:
                    val.setText(f"{ve.esc_rpm[i]:>6d} rpm  {ve.esc_voltage[i]:4.1f} V  "
                                f"{ve.esc_current[i]:4.1f} A")
        else:
            self._esc_group.hide()
        if ve.have_ext_state:
            self.f_landed.setText(mavlink.MAV_LANDED_STATE_TEXT.get(ve.landed_state, "--"))
            col = ("#37d67a" if ve.landed_state == 2 else            # in air
                   "#e0a030" if ve.landed_state in (3, 4) else "")   # takeoff / landing
            self.f_landed.setStyleSheet(f"color:{col};" if col else "")
            vtol_on = ve.vtol_state > 0
            self._f_vtol_label.setVisible(vtol_on)
            self.f_vtol.setVisible(vtol_on)
            if vtol_on:
                self.f_vtol.setText(mavlink.MAV_VTOL_STATE_TEXT.get(ve.vtol_state, "--"))
        else:
            self.f_landed.setText("--")
            self.f_landed.setStyleSheet("")
            self._f_vtol_label.hide()
            self.f_vtol.hide()
        self.vib_bars.set_values(ve.vibration)
        self.vib_clip.setText("clip {} / {} / {}".format(*ve.clipping))
        self.a_amsl.setText(f"{ve.alt_msl:.1f} m")
        self.a_rel.setText(f"{ve.alt_rel:.1f} m")
        _t = ve.alt_terrain                      # NaN when PX4 has no terrain estimate
        self.a_terr.setText("--" if _t is None or _t != _t else f"{_t:.1f} m")
        self.r_rssi.setText("--" if ve.radio_rssi is None else str(ve.radio_rssi))
        self.r_remrssi.setText("--" if ve.radio_remrssi is None else str(ve.radio_remrssi))
        self.r_noise.setText("--" if ve.radio_noise is None else str(ve.radio_noise))

        if ve.have_gnss_integrity:
            self._gi_group.setVisible(True)

            def gcol(state, ok_vals):
                # 3 = detected/failed (red), 2 = mitigated (amber), ok_vals -> green, else neutral
                return ("#e05050" if state == 3 else "#e0a030" if state == 2
                        else "#37d67a" if state in ok_vals else "#8fa3bf")
            self.gi_jam.setText(mavlink.GPS_JAMMING_TEXT.get(ve.gps_jamming, "--"))
            self.gi_jam.setStyleSheet(f"color:{gcol(ve.gps_jamming, (1,))};")
            self.gi_spoof.setText(mavlink.GPS_SPOOFING_TEXT.get(ve.gps_spoofing, "--"))
            self.gi_spoof.setStyleSheet(f"color:{gcol(ve.gps_spoofing, (1,))};")
            self.gi_raim.setText(mavlink.GPS_RAIM_TEXT.get(ve.gps_raim, "--"))
            self.gi_raim.setStyleSheet(f"color:{gcol(ve.gps_raim, (2,))};")
            q = ve.gps_signal_quality
            self.gi_sig.setText("--" if q > 10 else f"{q}/10")
