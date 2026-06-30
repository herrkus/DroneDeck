#!/usr/bin/env python3
"""main.py -- DroneDeck ground control station (entry point).

A QGroundControl-style GCS: live map, attitude/heading instruments and a
telemetry sidebar, fed by the native C++/assembly MAVLink core over a UDP link.
Run:  python3 app/main.py [udp_port]
"""
from __future__ import annotations
import os
import sys
import time

# Make sibling modules importable whether launched as a script or a module.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction, QFont
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QHBoxLayout,
                               QVBoxLayout, QSplitter, QToolBar, QLineEdit,
                               QPushButton, QLabel, QCheckBox, QMessageBox, QScrollArea,
                               QDockWidget)

import core
import mavlink
from vehicle import Vehicle
from link import UdpLink
from instruments import AttitudeIndicator, Compass
from mapview import MapView
from panels import TelemetryPanel, MessageConsole

DARK_QSS = """
QMainWindow, QWidget { background:#15171c; color:#d6d9df; }
QGroupBox { border:1px solid #2a2d36; border-radius:6px; margin-top:8px;
            font-weight:bold; color:#8fa3bf; }
QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 4px; }
QToolBar { background:#1b1e25; border-bottom:1px solid #2a2d36; spacing:6px; padding:4px; }
QPushButton { background:#262b35; border:1px solid #353b47; border-radius:5px;
              padding:5px 12px; color:#d6d9df; }
QPushButton:hover { background:#2f3542; }
QPushButton:disabled { color:#666; }
QLineEdit { background:#0f1115; border:1px solid #353b47; border-radius:5px; padding:4px; }
QStatusBar { background:#1b1e25; color:#8a90a0; }
QLabel { color:#c4c8d0; }
"""


class DroneDeck(QMainWindow):
    def __init__(self, port=14550):
        super().__init__()
        self.setWindowTitle("DroneDeck -- MAVLink Ground Control")
        self.resize(1240, 770)
        # Floor the size so panes can never be squeezed into each other.
        self.setMinimumSize(1060, 660)

        self.vehicle = Vehicle()
        self.link = UdpLink()
        self.default_port = port

        self._build_ui()
        self._wire()

        # 20 Hz UI refresh, decoupled from the message arrival rate.
        self._rate = 0.0
        self._last_count = 0
        self._last_t = time.monotonic()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(50)

        self._connect()           # start listening immediately

    # -- ui -------------------------------------------------------------------
    def _build_ui(self):
        tb = QToolBar("Main")
        tb.setMovable(False)
        self.addToolBar(tb)

        tb.addWidget(QLabel(" UDP port "))
        self.port_edit = QLineEdit(str(self.default_port))
        self.port_edit.setFixedWidth(70)
        tb.addWidget(self.port_edit)
        self.btn_conn = QPushButton("Disconnect")
        self.btn_conn.clicked.connect(self._toggle_conn)
        tb.addWidget(self.btn_conn)
        tb.addSeparator()

        self.btn_arm = QPushButton("Arm")
        self.btn_arm.clicked.connect(lambda: self._arm(True))
        self.btn_disarm = QPushButton("Disarm")
        self.btn_disarm.clicked.connect(lambda: self._arm(False))
        tb.addWidget(self.btn_arm)
        tb.addWidget(self.btn_disarm)
        tb.addSeparator()

        self.chk_follow = QCheckBox("Follow")
        self.chk_follow.setChecked(True)
        self.chk_follow.toggled.connect(self._set_follow)
        tb.addWidget(self.chk_follow)
        zin = QPushButton("+"); zin.setFixedWidth(32)
        zout = QPushButton("-"); zout.setFixedWidth(32)
        zin.clicked.connect(lambda: self.map.set_zoom(self.map.zoom + 1))
        zout.clicked.connect(lambda: self.map.set_zoom(self.map.zoom - 1))
        tb.addWidget(zout); tb.addWidget(zin)

        # central layout: map | (instruments over telemetry)
        self.map = MapView()
        right = QWidget()
        right.setMinimumWidth(360)
        rlay = QVBoxLayout(right)
        rlay.setContentsMargins(0, 0, 0, 0)
        rlay.setSpacing(0)
        inst = QWidget()
        inst.setFixedHeight(230)
        ilay = QHBoxLayout(inst)
        ilay.setContentsMargins(6, 6, 6, 0)
        ilay.setSpacing(10)
        self.adi = AttitudeIndicator()
        self.compass = Compass()
        ilay.addWidget(self.adi)
        ilay.addWidget(self.compass)
        rlay.addWidget(inst)
        self.panel = TelemetryPanel()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.panel)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFrameShape(QScrollArea.NoFrame)
        rlay.addWidget(scroll, 1)

        split = QSplitter(Qt.Horizontal)
        split.addWidget(self.map)
        split.addWidget(right)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 1)
        split.setSizes([820, 420])
        split.setChildrenCollapsible(False)   # neither pane can be crushed to zero
        split.setHandleWidth(4)
        self.setCentralWidget(split)

        # bottom message console (STATUSTEXT + command results)
        self.console = MessageConsole()
        self.console.setMinimumHeight(90)
        self.console.setMaximumHeight(170)
        dock = QDockWidget("Messages", self)
        dock.setObjectName("messages_dock")
        dock.setWidget(self.console)
        dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock)

        self.sb_info = QLabel("starting...")
        self.statusBar().addWidget(self.sb_info, 1)
        core_lbl = QLabel(f"core: {core.BACKEND} ")
        core_lbl.setFont(QFont("DejaVu Sans Mono", 8))
        self.statusBar().addPermanentWidget(core_lbl)

    CMD_NAMES = {400: "ARM/DISARM", 22: "TAKEOFF", 21: "LAND", 20: "RTL",
                 176: "SET MODE", 192: "REPOSITION", 193: "PAUSE/CONTINUE"}

    def _wire(self):
        self.link.messages.connect(self.vehicle.consume)
        self.link.info.connect(self._on_info)
        self.link.state.connect(self._on_state)
        self.vehicle.status_text.connect(self.console.add_message)
        self.vehicle.command_ack.connect(self._on_command_ack)

    def _on_command_ack(self, command, result):
        name = self.CMD_NAMES.get(command, f"CMD {command}")
        res = mavlink.MAV_RESULT.get(result, str(result))
        ok = (result == 0)
        line = f"{name}: {res}"
        self._on_info(line)
        self.console.add_note(line, "#37d67a" if ok else "#e05050")

    # -- actions --------------------------------------------------------------
    def _connect(self):
        try:
            port = int(self.port_edit.text())
        except ValueError:
            port = self.default_port
        self.link.open(port)

    def _toggle_conn(self):
        if self.link.is_open:
            self.link.close()
        else:
            self._connect()

    def _on_state(self, up):
        self.btn_conn.setText("Disconnect" if up else "Connect")

    def _on_info(self, msg):
        self.sb_info.setText(msg)

    def _arm(self, arm):
        if not (self.link.is_open and self.link.remote is not None):
            QMessageBox.information(self, "No vehicle", "No telemetry source connected yet.")
            return
        sysid = self.vehicle.sysid or 1
        self.link.arm(sysid, arm)
        self._on_info(f"sent {'ARM' if arm else 'DISARM'} to system {sysid}")

    def _set_follow(self, on):
        self.map.follow = on
        if on and self.vehicle.have_position:
            self.map.center = (self.vehicle.lat, self.vehicle.lon)

    # -- refresh --------------------------------------------------------------
    def _refresh(self):
        ve = self.vehicle
        self.adi.set_attitude(ve.roll, ve.pitch)
        self.compass.set_heading(ve.heading)
        if ve.have_position:
            self.map.update_vehicle(ve.lat, ve.lon, ve.heading, ve.home, ve.trail)

        now = time.monotonic()
        if now - self._last_t >= 1.0:
            self._rate = (ve.msg_count - self._last_count) / (now - self._last_t)
            self._last_count = ve.msg_count
            self._last_t = now

        ok, drop = self.link.parser.stats if self.link.parser else (0, 0)
        state = "connected" if self.link.is_open else "disconnected"
        if self.link.is_open and not ve.link_alive and self.link.remote is None:
            state = "listening"
        self.panel.update_all(ve, state, self._rate, ok, drop)

        connected = self.link.is_open and self.link.remote is not None
        self.btn_arm.setEnabled(connected)
        self.btn_disarm.setEnabled(connected)

    def closeEvent(self, e):
        self.link.close()
        super().accept() if False else super().closeEvent(e)


def main():
    port = 14550
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            pass
    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_QSS)
    win = DroneDeck(port)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
