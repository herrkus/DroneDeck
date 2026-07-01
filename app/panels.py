"""panels.py -- text telemetry readouts grouped like a GCS sidebar."""
from __future__ import annotations

import time

from PySide6.QtCore import Qt, QRectF, QPointF, Signal
from PySide6.QtGui import QFont, QColor, QPainter, QPen
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QFormLayout, QGroupBox, QLabel,
                               QListWidget, QListWidgetItem, QTableWidget,
                               QTableWidgetItem, QHeaderView, QPushButton, QSlider,
                               QSpinBox, QHBoxLayout, QGridLayout, QProgressBar,
                               QAbstractItemView)

import mavlink

_MONO = QFont("DejaVu Sans Mono", 10)


class TelemetryPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.v: dict[str, QLabel] = {}
        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(6)
        lay.addWidget(self._group("LINK", [
            ("link", "Status"), ("rate", "Msg rate"), ("counts", "OK / drop")]))
        lay.addWidget(self._group("FLIGHT", [
            ("mode", "Mode"), ("armed", "State"), ("status", "System"), ("type", "Airframe")]))
        lay.addWidget(self._group("BATTERY", [
            ("voltage", "Voltage"), ("current", "Current"), ("remaining", "Remaining")]))
        lay.addWidget(self._group("POSITION", [
            ("lat", "Latitude"), ("lon", "Longitude"),
            ("alt_msl", "Alt MSL"), ("alt_rel", "Alt rel")]))
        lay.addWidget(self._group("MOTION", [
            ("gspeed", "Ground spd"), ("aspeed", "Air spd"),
            ("climb", "Climb"), ("throttle", "Throttle"), ("hdg", "Heading")]))
        lay.addWidget(self._group("NAVIGATION", [
            ("home_dist", "Dist to home"), ("flight_time", "Flight time"),
            ("home_eta", "Home ETA"), ("wp_dist", "Dist to WP")]))
        lay.addWidget(self._group("GPS", [
            ("fix", "Fix"), ("sats", "Satellites")]))
        lay.addStretch(1)

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
        return box

    def _set(self, key, text, color=None):
        lbl = self.v[key]
        lbl.setText(text)
        lbl.setStyleSheet(f"color:{color};" if color else "")

    def update_all(self, ve, link_state: str, rate: float, ok: int, drop: int, nav=None):
        alive = ve.link_alive
        self._set("link", link_state, "#37d67a" if alive else "#e0a030")
        self._set("rate", f"{rate:5.1f} Hz")
        self._set("counts", f"{ok} / {drop}", "#e05050" if drop else None)

        self._set("mode", ve.mode, "#8fd0ff")
        self._set("armed", "ARMED" if ve.armed else "DISARMED",
                  "#e05050" if ve.armed else "#37d67a")
        self._set("status", {0: "Uninit", 1: "Boot", 2: "Calibrating", 3: "Standby",
                             4: "Active", 5: "Critical", 6: "Emergency"}.get(ve.system_status, "--"))
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

        self._set("gspeed", f"{ve.groundspeed:5.1f} m/s")
        self._set("aspeed", f"{ve.airspeed:5.1f} m/s")
        self._set("climb", f"{ve.climb:+5.1f} m/s")
        self._set("throttle", f"{ve.throttle} %")
        self._set("hdg", f"{ve.heading:5.1f} deg")

        self._set("fix", ve.fix_text, "#37d67a" if ve.fix_type >= 3 else "#e0a030")
        self._set("sats", str(ve.satellites))

        nav = nav or {}
        self._set("home_dist", nav.get("home_dist", "--"))
        self._set("flight_time", nav.get("flight_time", "--"))
        self._set("home_eta", nav.get("home_eta", "--"))
        self._set("wp_dist", nav.get("wp_dist", "--"))


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
        g.addWidget(btn_center, 4, 0, 1, 2)
        g.setRowStretch(5, 1)

    def _on_video(self, on):
        self.btn_video.setText("Video ■" if on else "Video ●")
        self.btn_video.setStyleSheet("color:#e05050;" if on else "")
        self.videoToggled.emit(on)

    def _on_gimbal_label(self):
        self.lbl_pitch.setText(f"pitch {self.pitch.value():4d}°")
        self.lbl_yaw.setText(f"yaw {self.yaw.value():4d}°")

    def _emit_gimbal(self):
        self.gimbalChanged.emit(float(self.pitch.value()), float(self.yaw.value()))

    def _center(self):
        self.pitch.setValue(0)
        self.yaw.setValue(0)
        self._emit_gimbal()


class StatusStrip(QWidget):
    """QGroundControl-style top indicator bar: a row of colour-coded chips
    (armed/mode, GPS, battery, link, flight time, messages)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(30)
        self.chips = []                  # list of (label, dot_color, text_color)

    def set_chips(self, chips):
        self.chips = chips
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(18, 20, 26))
        p.setPen(QColor(40, 44, 52))
        p.drawLine(0, self.height() - 1, self.width(), self.height() - 1)
        font = QFont("DejaVu Sans", 9, QFont.Bold)
        p.setFont(font)
        fm = p.fontMetrics()
        x = 8.0
        for label, dot, txt in self.chips:
            tw = fm.horizontalAdvance(label)
            w = tw + 28
            rect = QRectF(x, 4, w, self.height() - 8)
            p.setBrush(QColor(30, 33, 40))
            p.setPen(QPen(QColor(48, 52, 62), 1))
            p.drawRoundedRect(rect, 5, 5)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(dot))
            p.drawEllipse(QPointF(x + 13, self.height() / 2.0), 4.5, 4.5)
            p.setPen(QColor(txt))
            p.drawText(QRectF(x + 22, 4, tw + 4, self.height() - 8),
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

    def add_message(self, severity, text):
        sev = mavlink.MAV_SEVERITY.get(severity, str(severity))
        item = QListWidgetItem(f"[{sev}] {text}")
        item.setForeground(QColor(self.SEV_COLOR.get(severity, "#c4c8d0")))
        self.addItem(item)
        while self.count() > 300:
            self.takeItem(0)
        self.scrollToBottom()

    def add_note(self, text, color="#8fd0ff"):
        item = QListWidgetItem(text)
        item.setForeground(QColor(color))
        self.addItem(item)
        while self.count() > 300:
            self.takeItem(0)
        self.scrollToBottom()


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
            frac = max(0.0, min(1.0, v / MAXV))
            col = (QColor(120, 210, 120) if v < 30 else
                   QColor(230, 180, 60) if v < 60 else QColor(230, 90, 70))
            p.setBrush(col)
            p.drawRoundedRect(QRectF(track.left(), track.top(), track.width() * frac,
                                     track.height()), 3, 3)
            p.setPen(QColor(220, 224, 230))
            p.drawText(QRectF(track.right() + 4, y - 2, 46, 16), Qt.AlignVCenter, f"{v:.0f}")
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
        bf.addRow("Voltage", self.b_volt)
        bf.addRow("Current", self.b_curr)
        bf.addRow("Consumed", self.b_used)
        bf.addRow("Remaining", self.b_rem)
        bf.addRow("Temperature", self.b_temp)
        bf.addRow("Cells (V)", self.b_cells)
        lay.addWidget(bat)

        vib = QGroupBox("Vibration")
        vv = QVBoxLayout(vib)
        self.vib_bars = _VibBars()
        vv.addWidget(self.vib_bars)
        self.vib_clip = QLabel("clip 0 / 0 / 0")
        self.vib_clip.setStyleSheet("color:#8fa3bf;")
        vv.addWidget(self.vib_clip)
        lay.addWidget(vib)

        alt = QGroupBox("Altitude")
        af = QFormLayout(alt)
        self.a_amsl = QLabel("--"); self.a_rel = QLabel("--"); self.a_terr = QLabel("--")
        af.addRow("AMSL", self.a_amsl)
        af.addRow("Above home", self.a_rel)
        af.addRow("Above terrain", self.a_terr)
        lay.addWidget(alt)

        radio = QGroupBox("Radio link")
        rf = QFormLayout(radio)
        self.r_rssi = QLabel("--"); self.r_remrssi = QLabel("--"); self.r_noise = QLabel("--")
        rf.addRow("RSSI (local)", self.r_rssi)
        rf.addRow("RSSI (remote)", self.r_remrssi)
        rf.addRow("Noise", self.r_noise)
        lay.addWidget(radio)
        lay.addStretch(1)

    def update_from(self, ve):
        if ve is None:
            return
        self.b_volt.setText(f"{ve.voltage:.2f} V")
        self.b_curr.setText(f"{ve.current:.1f} A")
        self.b_used.setText("--" if ve.battery_consumed < 0 else f"{ve.battery_consumed} mAh")
        self.b_rem.setText("--" if ve.battery_remaining < 0 else f"{ve.battery_remaining}%")
        self.b_temp.setText("--" if ve.battery_temp is None else f"{ve.battery_temp:.1f} C")
        self.b_cells.setText("  ".join(f"{c:.2f}" for c in ve.cells) if ve.cells else "--")
        self.vib_bars.set_values(ve.vibration)
        self.vib_clip.setText("clip {} / {} / {}".format(*ve.clipping))
        self.a_amsl.setText(f"{ve.alt_msl:.1f} m")
        self.a_rel.setText(f"{ve.alt_rel:.1f} m")
        self.a_terr.setText("--" if ve.alt_terrain is None else f"{ve.alt_terrain:.1f} m")
        self.r_rssi.setText("--" if ve.radio_rssi is None else str(ve.radio_rssi))
        self.r_remrssi.setText("--" if ve.radio_remrssi is None else str(ve.radio_remrssi))
        self.r_noise.setText("--" if ve.radio_noise is None else str(ve.radio_noise))
