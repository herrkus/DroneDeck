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
                               QDockWidget, QComboBox, QInputDialog, QListWidget)

import core
import mavlink
from vehicle import Vehicle
from link import UdpLink, TcpLink, SerialLink
from mission import MissionProtocol, MissionItem, survey_grid
from instruments import AttitudeIndicator, Compass
from mapview import MapView
from panels import TelemetryPanel, MessageConsole, MavInspector

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
        self.link = None
        self.default_port = port

        # mission planning state
        self.plan_mode = False
        self.mission_items = []                         # list[MissionItem]
        self.mission = MissionProtocol(lambda: self.link, self._sysid)

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

        tb.addWidget(QLabel(" Link "))
        self.transport_combo = QComboBox()
        self.transport_combo.addItems(["UDP", "TCP", "Serial"])
        tb.addWidget(self.transport_combo)
        self.link_edit = QLineEdit(str(self.default_port))
        self.link_edit.setFixedWidth(150)
        self.link_edit.setPlaceholderText("port")
        tb.addWidget(self.link_edit)
        self.transport_combo.currentIndexChanged.connect(self._on_transport)
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

        # second toolbar row: flight controls
        self.addToolBarBreak()
        tb2 = QToolBar("Flight")
        tb2.setMovable(False)
        self.addToolBar(tb2)
        tb2.addWidget(QLabel(" Mode "))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["STABILIZE", "ALT_HOLD", "LOITER", "POSHOLD", "GUIDED",
                                  "AUTO", "RTL", "SMART_RTL", "LAND", "BRAKE"])
        self.mode_combo.setCurrentText("LOITER")
        self.mode_combo.activated.connect(self._set_mode)
        tb2.addWidget(self.mode_combo)
        tb2.addSeparator()
        self._flight_btns = []
        for label, slot in (("Takeoff", self._takeoff), ("Land", self._land),
                            ("RTL", self._rtl), ("Pause", self._pause)):
            b = QPushButton(label)
            b.clicked.connect(slot)
            tb2.addWidget(b)
            self._flight_btns.append(b)
        tb2.addSeparator()
        self.map_hint = QLabel(" click map = Goto ")
        self.map_hint.setStyleSheet("color:#8a90a0;")
        tb2.addWidget(self.map_hint)

        # third toolbar row: mission planning
        self.addToolBarBreak()
        tb3 = QToolBar("Mission")
        tb3.setMovable(False)
        self.addToolBar(tb3)
        tb3.addWidget(QLabel(" Plan "))
        self.btn_plan = QPushButton("Plan mode")
        self.btn_plan.setCheckable(True)
        self.btn_plan.toggled.connect(self._toggle_plan)
        tb3.addWidget(self.btn_plan)
        self._mission_btns = []
        for label, slot in (("Survey", self._survey), ("Clear", self._clear_mission),
                            ("Upload", self._upload_mission), ("Download", self._download_mission)):
            b = QPushButton(label)
            b.clicked.connect(slot)
            tb3.addWidget(b)
            self._mission_btns.append((label, b))
        tb3.addSeparator()
        self.mission_status = QLabel("no mission")
        self.mission_status.setStyleSheet("color:#8a90a0;")
        tb3.addWidget(self.mission_status)

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

        # mission waypoint list, tabbed with Messages at the bottom
        self.mission_list = QListWidget()
        self.mission_list.setFont(QFont("DejaVu Sans Mono", 9))
        self.mission_list.setMinimumHeight(90)
        self.mission_list.setMaximumHeight(170)
        mdock = QDockWidget("Mission", self)
        mdock.setObjectName("mission_dock")
        mdock.setWidget(self.mission_list)
        mdock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, mdock)

        # MAVLink inspector (live message rates + fields), also tabbed at the bottom
        self.inspector = MavInspector()
        idock = QDockWidget("Inspector", self)
        idock.setObjectName("inspector_dock")
        idock.setWidget(self.inspector)
        idock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, idock)
        self.tabifyDockWidget(dock, mdock)
        self.tabifyDockWidget(mdock, idock)
        dock.raise_()

        self.sb_info = QLabel("starting...")
        self.statusBar().addWidget(self.sb_info, 1)
        core_lbl = QLabel(f"core: {core.BACKEND} ")
        core_lbl.setFont(QFont("DejaVu Sans Mono", 8))
        self.statusBar().addPermanentWidget(core_lbl)

    CMD_NAMES = {400: "ARM/DISARM", 22: "TAKEOFF", 21: "LAND", 20: "RTL",
                 176: "SET MODE", 192: "REPOSITION", 193: "PAUSE/CONTINUE"}

    def _wire(self):
        self.vehicle.status_text.connect(self.console.add_message)
        self.vehicle.command_ack.connect(self._on_command_ack)
        self.map.clicked.connect(self._on_map_click)
        self.mission.progress.connect(self._on_mission_progress)
        self.mission.finished.connect(self._on_mission_finished)
        self.mission.downloaded.connect(self._on_mission_downloaded)

    def _make_link(self):
        cls = {"TCP": TcpLink, "Serial": SerialLink}.get(self.transport_combo.currentText(), UdpLink)
        link = cls()
        link.messages.connect(self.vehicle.consume)
        link.messages.connect(self.mission.handle_messages)
        link.messages.connect(self.inspector.consume)
        link.info.connect(self._on_info)
        link.state.connect(self._on_state)
        return link

    def _on_transport(self):
        t = self.transport_combo.currentText()
        if t == "UDP":
            self.link_edit.setText("14550")
            self.link_edit.setPlaceholderText("port")
        elif t == "TCP":
            self.link_edit.setText("127.0.0.1:5760")
            self.link_edit.setPlaceholderText("host:port")
        else:
            ports = SerialLink.available_ports()
            self.link_edit.setText(f"{ports[0] if ports else '/dev/ttyACM0'}:57600")
            self.link_edit.setPlaceholderText("port:baud")

    def _on_command_ack(self, command, result):
        name = self.CMD_NAMES.get(command, f"CMD {command}")
        res = mavlink.MAV_RESULT.get(result, str(result))
        ok = (result == 0)
        line = f"{name}: {res}"
        self._on_info(line)
        self.console.add_note(line, "#37d67a" if ok else "#e05050")

    # -- actions --------------------------------------------------------------
    def _connect(self):
        if self.link is not None:
            self.link.close()
            self.link.deleteLater()
        self.link = self._make_link()
        t = self.transport_combo.currentText()
        p = self.link_edit.text().strip()
        try:
            if t == "TCP":
                host, _, port = p.partition(":")
                self.link.open(host=host or "127.0.0.1", port=int(port or 5760))
            elif t == "Serial":
                port, _, baud = p.partition(":")
                self.link.open(port=port, baud=int(baud or 57600))
            else:
                self.link.open(port=int(p or self.default_port))
        except ValueError:
            self._on_info(f"invalid {t} parameters: {p!r}")

    def _toggle_conn(self):
        if self.link is not None and self.link.is_open:
            self.link.close()
        else:
            self._connect()

    def _on_state(self, up):
        self.btn_conn.setText("Disconnect" if up else "Connect")

    def _on_info(self, msg):
        self.sb_info.setText(msg)

    def _has_vehicle(self):
        return self.link is not None and self.link.is_open and self.link.remote is not None

    def _sysid(self):
        return self.vehicle.sysid or 1

    def _arm(self, arm):
        if not self._has_vehicle():
            QMessageBox.information(self, "No vehicle", "No telemetry source connected yet.")
            return
        self.link.arm(self._sysid(), arm)
        self._on_info(f"sent {'ARM' if arm else 'DISARM'} to system {self._sysid()}")

    def _set_mode(self):
        if not self._has_vehicle():
            return
        name = self.mode_combo.currentText()
        num = mavlink.mode_number(self.vehicle.autopilot, self.vehicle.mav_type, name)
        if num is None:
            self._on_info(f"mode {name} not available for this vehicle")
            return
        self.link.set_mode(self._sysid(), num)
        self._on_info(f"set mode {name}")

    def _takeoff(self):
        if not self._has_vehicle():
            return
        alt, ok = QInputDialog.getDouble(self, "Takeoff", "Altitude (m):", 30.0, 1.0, 1000.0, 1)
        if ok:
            self.link.takeoff(self._sysid(), alt)
            self._on_info(f"takeoff to {alt:.0f} m")

    def _land(self):
        if self._has_vehicle():
            self.link.land(self._sysid())
            self._on_info("land")

    def _rtl(self):
        if self._has_vehicle():
            self.link.rtl(self._sysid())
            self._on_info("return to launch")

    def _pause(self):
        if self._has_vehicle():
            self.link.pause(self._sysid(), cont=False)
            self._on_info("pause / hold position")

    def _on_map_click(self, lat, lon):
        if self.plan_mode:
            self._add_waypoint(lat, lon)
            return
        if not self._has_vehicle():
            return
        alt = max(self.vehicle.alt_rel, 30.0)
        if QMessageBox.question(self, "Goto",
                                f"Fly to:\n{lat:.6f}, {lon:.6f}\nat {alt:.0f} m relative altitude?"
                                ) == QMessageBox.StandardButton.Yes:
            self.link.goto(self._sysid(), lat, lon, alt)
            self._on_info(f"goto {lat:.5f}, {lon:.5f} @ {alt:.0f} m")

    # -- mission planning -----------------------------------------------------
    def _toggle_plan(self, on):
        self.plan_mode = on
        self.map_hint.setText(" click map = add waypoint " if on else " click map = Goto ")
        self.btn_plan.setText("Plan mode ON" if on else "Plan mode")

    def _add_waypoint(self, lat, lon):
        alt = self.mission_items[-1].alt if self.mission_items else 50.0
        seq = len(self.mission_items)
        self.mission_items.append(MissionItem(seq, lat, lon, alt))
        self._refresh_mission_view()

    def _refresh_mission_view(self):
        self.mission_list.clear()
        for it in self.mission_items:
            self.mission_list.addItem(
                f"{it.seq:2d}  {it.cmd_name:9s} {it.lat:10.6f} {it.lon:11.6f}  {it.alt:5.0f} m")
        self.map.set_mission([(it.lat, it.lon) for it in self.mission_items])
        n = len(self.mission_items)
        self.mission_status.setText(f"{n} waypoint{'' if n == 1 else 's'}" if n else "no mission")

    def _renumber(self):
        for i, it in enumerate(self.mission_items):
            it.seq = i

    def _survey(self):
        if len(self.mission_items) < 2:
            QMessageBox.information(self, "Survey",
                                    "Place at least 2 waypoints to outline the area, then Survey.")
            return
        alt = self.mission_items[0].alt or 50.0
        grid = survey_grid([(it.lat, it.lon) for it in self.mission_items], spacing_m=35.0, alt=alt)
        if grid:
            self.mission_items = grid
            self._refresh_mission_view()
            self._on_info(f"survey grid: {len(grid)} waypoints")

    def _clear_mission(self):
        self.mission_items = []
        self._refresh_mission_view()
        if self._has_vehicle():
            self.mission.clear()

    def _upload_mission(self):
        if not self._has_vehicle():
            QMessageBox.information(self, "Upload", "No vehicle connected.")
            return
        if not self.mission_items:
            QMessageBox.information(self, "Upload", "No waypoints to upload.")
            return
        self._renumber()
        self.mission.upload(self.mission_items)

    def _download_mission(self):
        if not self._has_vehicle():
            QMessageBox.information(self, "Download", "No vehicle connected.")
            return
        self.mission.download()

    def _on_mission_progress(self, msg):
        self.mission_status.setText(msg)

    def _on_mission_finished(self, ok, msg):
        self.mission_status.setText(msg)
        self.console.add_note(f"mission: {msg}", "#4caf50" if ok else "#ff6b6b")

    def _on_mission_downloaded(self, items):
        self.mission_items = list(items)
        self._refresh_mission_view()

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

        link = self.link
        ok, drop = (link.parser.stats if (link and link.parser) else (0, 0))
        is_open = bool(link and link.is_open)
        state = "connected" if is_open else "disconnected"
        if is_open and not ve.link_alive and link.remote is None:
            state = "listening"
        self.panel.update_all(ve, state, self._rate, ok, drop)

        connected = self._has_vehicle()
        self.btn_arm.setEnabled(connected)
        self.btn_disarm.setEnabled(connected)
        self.mode_combo.setEnabled(connected)
        for b in self._flight_btns:
            b.setEnabled(connected)
        for label, b in self._mission_btns:
            b.setEnabled(connected if label in ("Upload", "Download") else True)

        self.inspector.refresh()

    def closeEvent(self, e):
        if self.link is not None:
            self.link.close()
        super().closeEvent(e)


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
