"""panels.py -- text telemetry readouts grouped like a GCS sidebar."""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QFormLayout, QGroupBox, QLabel)

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
            ("armed", "State"), ("status", "System"), ("type", "Airframe")]))
        lay.addWidget(self._group("BATTERY", [
            ("voltage", "Voltage"), ("current", "Current"), ("remaining", "Remaining")]))
        lay.addWidget(self._group("POSITION", [
            ("lat", "Latitude"), ("lon", "Longitude"),
            ("alt_msl", "Alt MSL"), ("alt_rel", "Alt rel")]))
        lay.addWidget(self._group("MOTION", [
            ("gspeed", "Ground spd"), ("aspeed", "Air spd"),
            ("climb", "Climb"), ("throttle", "Throttle"), ("hdg", "Heading")]))
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

    def update_all(self, ve, link_state: str, rate: float, ok: int, drop: int):
        alive = ve.link_alive
        self._set("link", link_state, "#37d67a" if alive else "#e0a030")
        self._set("rate", f"{rate:5.1f} Hz")
        self._set("counts", f"{ok} / {drop}", "#e05050" if drop else None)

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
