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
import math
import json

# Make sibling modules importable whether launched as a script or a module.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PySide6.QtCore import Qt, QTimer, QSettings
from PySide6.QtGui import QAction, QFont
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QHBoxLayout,
                               QVBoxLayout, QSplitter, QToolBar, QLineEdit,
                               QPushButton, QLabel, QCheckBox, QMessageBox, QScrollArea,
                               QDockWidget, QComboBox, QInputDialog, QListWidget,
                               QGroupBox, QTabWidget, QSpinBox)

import core
import mavlink
from vehicle import Vehicle
from link import UdpLink, TcpLink, SerialLink, ReplayLink
from mission import MissionProtocol, MissionItem, survey_grid
from params import ParamManager, ParamDialog
from tlog import TlogWriter
from logdownload import LogManager
from charts import ChartPanel
from joystick import VirtualJoystick
from video import VideoPane
from links_manager import LinksDialog
from calibration import CalibrationDialog
from instruments import AttitudeIndicator, Compass

LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
from mapview import MapView
from panels import (TelemetryPanel, MessageConsole, MavInspector, HealthPanel,
                    StatusStrip, CameraPanel, LogPanel, SystemsPanel)


def haversine(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres."""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(min(1.0, math.sqrt(a)))


def _fmt_dist(d):
    return f"{d:6.0f} m" if d < 10000 else f"{d / 1000:6.2f} km"


def _fmt_mmss(s):
    s = int(s)
    return f"{s // 60:02d}:{s % 60:02d}"


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
    def __init__(self, port=14550, replay_path=None):
        super().__init__()
        self.setWindowTitle("DroneDeck -- MAVLink Ground Control")
        self.resize(1240, 770)
        # Floor the size so panes can never be squeezed into each other.
        self.setMinimumSize(1060, 660)

        self.vehicle = Vehicle()
        self.vehicles = {}                 # sysid -> Vehicle (multi-vehicle)
        self.traffic = {}                  # ADSB: ICAO -> {lat, lon, heading, callsign, t}
        self.link = None
        self.default_port = port

        # settings persistence (only the real app opts in; tests stay deterministic)
        self._persist = False
        self.settings = QSettings("DroneDeck", "DroneDeck")
        self.link_configs = []                          # saved comm-link configs

        # flight-time tracking (since arm)
        self._arm_t0 = None
        self._flight_time = 0.0

        # mission planning state
        self.plan_mode = False
        self.plan_type = "Mission"                      # Mission | Fence | Rally
        self.mission_items = []                         # list[MissionItem]
        self.fence_inc = []                             # inclusion polygon [(lat, lon)]
        self.fence_exc = []                             # exclusion polygon [(lat, lon)]
        self.fence_circles = []                         # [{"lat","lon","radius","incl"}]
        self.fence_radius = 50                          # m, for new circles
        self.rally_pts = []                             # list[(lat, lon)]
        self._selecting = False
        self.mission = MissionProtocol(lambda: self.link, self._sysid)

        # parameter editor
        self.params = ParamManager(lambda: self.link, self._sysid)
        self.logs = LogManager(lambda: self.link, self._sysid, LOG_DIR)
        self._param_dialog = None

        # telemetry recording
        self._recorder = None
        self._last_log = ""

        self._build_ui()
        self._wire()

        # 20 Hz UI refresh, decoupled from the message arrival rate.
        self._rate = 0.0
        self._last_count = 0
        self._last_t = time.monotonic()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(50)

        # manual control (virtual joystick + keyboard)
        self.manual_on = False
        self._manual_keys = set()
        self.manual_timer = QTimer(self)
        self.manual_timer.timeout.connect(self._send_manual)

        if replay_path:           # launched on a .tlog -> start in replay mode
            self.transport_combo.setCurrentText("Replay")
            self.link_edit.setText(replay_path)
        self._connect()           # start listening (or replaying) immediately

    # -- ui -------------------------------------------------------------------
    def _build_ui(self):
        tb = QToolBar("Main")
        tb.setMovable(False)
        self.addToolBar(tb)

        tb.addWidget(QLabel(" Link "))
        self.transport_combo = QComboBox()
        self.transport_combo.addItems(["UDP", "TCP", "Serial", "Replay"])
        tb.addWidget(self.transport_combo)
        self.link_edit = QLineEdit(str(self.default_port))
        self.link_edit.setFixedWidth(150)
        self.link_edit.setPlaceholderText("port")
        tb.addWidget(self.link_edit)
        self.transport_combo.currentIndexChanged.connect(self._on_transport)
        self.btn_conn = QPushButton("Disconnect")
        self.btn_conn.clicked.connect(self._toggle_conn)
        tb.addWidget(self.btn_conn)
        self.btn_links = QPushButton("Links…")
        self.btn_links.clicked.connect(self._open_links)
        tb.addWidget(self.btn_links)
        self.btn_cal = QPushButton("Calibrate…")
        self.btn_cal.clicked.connect(self._open_calibration)
        tb.addWidget(self.btn_cal)
        tb.addSeparator()
        tb.addWidget(QLabel(" Vehicle "))
        self.vehicle_combo = QComboBox()
        self.vehicle_combo.setMinimumWidth(80)
        self.vehicle_combo.currentIndexChanged.connect(self._select_vehicle)
        tb.addWidget(self.vehicle_combo)
        tb.addSeparator()

        self.btn_arm = QPushButton("Arm")
        self.btn_arm.clicked.connect(lambda: self._arm(True))
        self.btn_disarm = QPushButton("Disarm")
        self.btn_disarm.clicked.connect(lambda: self._arm(False))
        tb.addWidget(self.btn_arm)
        tb.addWidget(self.btn_disarm)
        self.btn_params = QPushButton("Params")
        self.btn_params.clicked.connect(self._open_params)
        tb.addWidget(self.btn_params)
        self.btn_record = QPushButton("Record")
        self.btn_record.setCheckable(True)
        self.btn_record.toggled.connect(self._toggle_record)
        tb.addWidget(self.btn_record)
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
        self.btn_joystick = QPushButton("Joystick")
        self.btn_joystick.setCheckable(True)
        self.btn_joystick.toggled.connect(self._toggle_manual)
        tb2.addWidget(self.btn_joystick)
        self._flight_btns.append(self.btn_joystick)
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
        self.plan_type_combo = QComboBox()
        self.plan_type_combo.addItems(["Mission", "Fence incl", "Fence excl",
                                       "Circle incl", "Circle excl", "Rally"])
        self.plan_type_combo.currentTextChanged.connect(self._on_plan_type)
        tb3.addWidget(self.plan_type_combo)
        self.radius_spin = QSpinBox()
        self.radius_spin.setRange(5, 5000)
        self.radius_spin.setValue(self.fence_radius)
        self.radius_spin.setSuffix(" m")
        self.radius_spin.setToolTip("circle radius")
        self.radius_spin.valueChanged.connect(lambda v: setattr(self, "fence_radius", v))
        tb3.addWidget(self.radius_spin)
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
        inst.setFixedHeight(250)
        ilay = QHBoxLayout(inst)
        ilay.setContentsMargins(6, 6, 6, 0)
        ilay.setSpacing(10)
        self.adi = AttitudeIndicator()
        self.compass = Compass()
        ilay.addWidget(self.adi, 3)
        ilay.addWidget(self.compass, 2)
        rlay.addWidget(inst)

        self.health = HealthPanel()
        hbox = QGroupBox("SYSTEM HEALTH")
        hb = QVBoxLayout(hbox)
        hb.setContentsMargins(8, 4, 8, 6)
        hb.addWidget(self.health)
        rlay.addWidget(hbox)
        # full text readouts now live in a roomy "Telemetry" bottom-dock tab (below),
        # so the right column stays uncramped: just the PFD + system health + breathing room.
        self.panel = TelemetryPanel()
        rlay.addStretch(1)

        # left pane: Map / Video tabs (QGC-style swap)
        self.video_pane = VideoPane()
        self.left_tabs = QTabWidget()
        self.left_tabs.addTab(self.map, "Map")
        self.left_tabs.addTab(self.video_pane, "Video")
        split = QSplitter(Qt.Horizontal)
        split.addWidget(self.left_tabs)
        split.addWidget(right)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 1)
        split.setSizes([820, 420])
        split.setChildrenCollapsible(False)   # neither pane can be crushed to zero
        split.setHandleWidth(4)
        # user asked that the map / right-column divider be fixed, not draggable
        for _i in range(split.count()):
            _h = split.handle(_i)
            if _h is not None:
                _h.setEnabled(False)

        # QGC-style status strip above the split view
        self.status_strip = StatusStrip()
        central = QWidget()
        cv = QVBoxLayout(central)
        cv.setContentsMargins(0, 0, 0, 0)
        cv.setSpacing(0)
        cv.addWidget(self.status_strip)
        cv.addWidget(split, 1)
        self.setCentralWidget(central)

        # bottom message console (STATUSTEXT + command results)
        self.console = MessageConsole()
        self.console.setMinimumHeight(90)
        self.console.setMaximumHeight(170)
        self.msg_dock = dock = QDockWidget("Messages", self)
        dock.setObjectName("messages_dock")
        dock.setWidget(self.console)
        dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock)

        # mission waypoint list + edit buttons, tabbed with Messages at the bottom
        self.mission_list = QListWidget()
        self.mission_list.setFont(QFont("DejaVu Sans Mono", 9))
        self.mission_list.itemSelectionChanged.connect(self._wp_list_selected)
        self.mission_list.itemDoubleClicked.connect(self._wp_edit_alt)
        mwrap = QWidget()
        mv = QVBoxLayout(mwrap)
        mv.setContentsMargins(2, 2, 2, 2)
        mv.setSpacing(2)
        mrow = QHBoxLayout()
        for label, slot in (("Delete", self._wp_delete), ("Up", self._wp_up), ("Down", self._wp_down)):
            b = QPushButton(label)
            b.clicked.connect(slot)
            mrow.addWidget(b)
        hint = QLabel("double-click = edit alt, drag on map = move")
        hint.setStyleSheet("color:#8a90a0;")
        mrow.addWidget(hint)
        mrow.addStretch(1)
        mv.addLayout(mrow)
        mv.addWidget(self.mission_list)
        self.mission_stats = QLabel("no mission")
        self.mission_stats.setStyleSheet("color:#8fa3bf; padding:2px 4px;")
        self.mission_stats.setFont(QFont("DejaVu Sans Mono", 9))
        mv.addWidget(self.mission_stats)
        mdock = QDockWidget("Mission", self)
        mdock.setObjectName("mission_dock")
        mdock.setWidget(mwrap)
        mdock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, mdock)

        # MAVLink inspector (live message rates + fields), also tabbed at the bottom
        self.inspector = MavInspector()
        idock = QDockWidget("Inspector", self)
        idock.setObjectName("inspector_dock")
        idock.setWidget(self.inspector)
        idock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, idock)

        # live telemetry charts, also tabbed at the bottom
        self.charts = ChartPanel()
        cdock = QDockWidget("Charts", self)
        cdock.setObjectName("charts_dock")
        cdock.setWidget(self.charts)
        cdock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, cdock)

        # camera + gimbal control, tabbed at the bottom
        self.camera = CameraPanel()
        camdock = QDockWidget("Camera", self)
        camdock.setObjectName("camera_dock")
        camdock.setWidget(self.camera)
        camdock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, camdock)
        self.tabifyDockWidget(cdock, camdock)
        self.camera.photoRequested.connect(self._cam_photo)
        self.camera.videoToggled.connect(self._cam_video)
        self.camera.triggerDistance.connect(self._cam_trigdist)
        self.camera.gimbalChanged.connect(self._cam_gimbal)

        # onboard log download, tabbed at the bottom
        self.log_panel = LogPanel()
        ldock = QDockWidget("Logs", self)
        ldock.setObjectName("logs_dock")
        ldock.setWidget(self.log_panel)
        ldock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, ldock)
        self.tabifyDockWidget(camdock, ldock)
        self.log_panel.refreshRequested.connect(self._logs_refresh)
        self.log_panel.downloadRequested.connect(self._logs_download)
        self.logs.entries.connect(self.log_panel.set_entries)
        self.logs.progress.connect(self.log_panel.set_progress)
        self.logs.finished.connect(self._logs_finished)

        # detailed systems (battery / vibration / altitude), tabbed at the bottom
        self.systems = SystemsPanel()
        sysscroll = QScrollArea()
        sysscroll.setWidgetResizable(True)
        sysscroll.setWidget(self.systems)
        sysscroll.setFrameShape(QScrollArea.NoFrame)
        self.sys_dock = sdock = QDockWidget("Systems", self)
        sdock.setObjectName("systems_dock")
        sdock.setWidget(sysscroll)
        sdock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, sdock)
        self.tabifyDockWidget(ldock, sdock)

        # full telemetry readouts -- a roomy wide tab (was a cramped scroll in the column)
        tscroll = QScrollArea()
        tscroll.setWidgetResizable(True)
        tscroll.setWidget(self.panel)
        tscroll.setFrameShape(QScrollArea.NoFrame)
        tdock = QDockWidget("Telemetry", self)
        tdock.setObjectName("telemetry_dock")
        tdock.setWidget(tscroll)
        tdock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, tdock)
        self.tabifyDockWidget(sdock, tdock)

        # virtual joystick dock (hidden until the Joystick button is toggled)
        self.joystick = VirtualJoystick()
        self.jdock = QDockWidget("Manual Control", self)
        self.jdock.setObjectName("joystick_dock")
        self.jdock.setWidget(self.joystick)
        self.jdock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable
                               | QDockWidget.DockWidgetClosable)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.jdock)
        self.jdock.hide()

        self.tabifyDockWidget(dock, mdock)
        self.tabifyDockWidget(mdock, idock)
        self.tabifyDockWidget(idock, cdock)
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
        self.map.contextAction.connect(self._on_map_context)
        self.map.waypoint_selected.connect(self._wp_selected)
        self.map.waypoint_moved.connect(self._wp_moved)
        self.mission.progress.connect(self._on_mission_progress)
        self.mission.finished.connect(self._on_mission_finished)
        self.mission.downloaded.connect(self._on_mission_downloaded)

    def _make_link(self):
        cls = {"TCP": TcpLink, "Serial": SerialLink, "Replay": ReplayLink}.get(
            self.transport_combo.currentText(), UdpLink)
        link = cls()
        link.messages.connect(self._route)
        link.messages.connect(self.mission.handle_messages)
        link.messages.connect(self.params.handle_messages)
        link.messages.connect(self.logs.handle_messages)
        link.messages.connect(self.inspector.consume)
        link.info.connect(self._on_info)
        link.state.connect(self._on_state)
        link.recorder = self._recorder        # keep recording across reconnects
        return link

    def _on_transport(self):
        t = self.transport_combo.currentText()
        if t == "UDP":
            self.link_edit.setText("14550")
            self.link_edit.setPlaceholderText("port")
        elif t == "TCP":
            self.link_edit.setText("127.0.0.1:5760")
            self.link_edit.setPlaceholderText("host:port")
        elif t == "Replay":
            self.link_edit.setText(self._last_log)
            self.link_edit.setPlaceholderText("path/to/file.tlog[@speed]")
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
            elif t == "Replay":
                path, _, sp = p.partition("@")
                self.link.open(path=path.strip(), speed=float(sp or 1.0))
            else:
                self.link.open(port=int(p or self.default_port))
        except ValueError:
            self._on_info(f"invalid {t} parameters: {p!r}")

    def _toggle_conn(self):
        if self.link is not None and self.link.is_open:
            self.link.close()
        else:
            self._connect()

    def _open_links(self):
        dlg = LinksDialog(self.link_configs, self)
        dlg.connectRequested.connect(self._connect_saved)
        dlg.exec()
        self.link_configs = dlg.configs
        if self._persist:
            self.settings.setValue("links/configs", json.dumps(self.link_configs))
            self.settings.sync()

    def _connect_saved(self, cfg):
        self.transport_combo.setCurrentText(cfg.get("transport", "UDP"))
        self.link_edit.setText(cfg.get("target", ""))
        self._connect()
        self._on_info(f"connecting to '{cfg.get('name', '')}'")

    # -- calibration (radio + sensors) ----------------------------------------
    def _open_calibration(self):
        dlg = CalibrationDialog(lambda: self.link, self)
        dlg.radio.saveRequested.connect(self._cal_write_params)
        dlg.sensor.calRequested.connect(self._cal_sensor)
        dlg.exec()

    def _cal_write_params(self, params):
        if not self._has_vehicle():
            QMessageBox.information(self, "No vehicle", "Connect to a vehicle first.")
            return
        for name, value in params.items():
            self.params.set(name, value)
        self._on_info(f"wrote {len(params)} RC calibration parameters")

    def _cal_sensor(self, kind):
        if not self._has_vehicle():
            QMessageBox.information(self, "No vehicle", "Connect to a vehicle first.")
            return
        self.link.calibrate(self._sysid(), kind)
        self._on_info(f"requested {kind} calibration")

    # -- multi-vehicle routing + ADSB traffic ---------------------------------
    def _route(self, batch):
        adsb = [m for m in batch if m.msgid == mavlink.ADSB_VEHICLE]
        if adsb:
            self._update_traffic(adsb)
        bysys = {}
        for m in batch:
            if m.msgid == mavlink.ADSB_VEHICLE or m.sysid == 0:
                continue
            bysys.setdefault(m.sysid, []).append(m)
        for sysid, msgs in bysys.items():
            self._ensure_vehicle(sysid).consume(msgs)

    def _ensure_vehicle(self, sysid):
        veh = self.vehicles.get(sysid)
        if veh is not None:
            return veh
        if not self.vehicles:
            veh = self.vehicle                  # reuse the pre-wired primary vehicle
        else:
            veh = Vehicle()
            veh.status_text.connect(self.console.add_message)
            veh.command_ack.connect(self._on_command_ack)
        self.vehicles[sysid] = veh
        active_sid = next((s for s, v in self.vehicles.items() if v is self.vehicle), sysid)
        self.vehicle_combo.blockSignals(True)
        self.vehicle_combo.clear()
        for sid in sorted(self.vehicles):
            self.vehicle_combo.addItem(f"#{sid}", sid)
        idx = self.vehicle_combo.findData(active_sid)
        if idx >= 0:
            self.vehicle_combo.setCurrentIndex(idx)
        self.vehicle_combo.blockSignals(False)
        if len(self.vehicles) == 2:
            self._on_info("multiple vehicles detected -- use the Vehicle selector")
        return veh

    def _select_vehicle(self, idx):
        sid = self.vehicle_combo.itemData(idx)
        if sid in self.vehicles:
            self.vehicle = self.vehicles[sid]
            self._arm_t0 = None
            self._on_info(f"active vehicle: #{sid}")

    def _update_traffic(self, msgs):
        now = time.monotonic()
        for m in msgs:
            f = m.fields
            icao = int(f.get("ICAO_address", 0))
            self.traffic[icao] = {
                "lat": f.get("lat", 0) / 1e7, "lon": f.get("lon", 0) / 1e7,
                "heading": f.get("heading", 0) / 100.0,
                "callsign": (f.get("callsign", "") or "").strip(), "t": now}

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

    # -- camera + gimbal ------------------------------------------------------
    def _cam_photo(self):
        if self._has_vehicle():
            self.link.trigger_camera(self._sysid())
            self._on_info("camera: photo trigger sent")

    def _cam_video(self, on):
        if self._has_vehicle():
            self.link.video_capture(self._sysid(), on)
            self._on_info(f"camera: video {'start' if on else 'stop'} sent")

    def _cam_trigdist(self, metres):
        if self._has_vehicle():
            self.link.set_trigger_distance(self._sysid(), metres)
            self._on_info(f"camera: trigger distance {metres:.0f} m")

    def _cam_gimbal(self, pitch, yaw):
        if self._has_vehicle():
            self.link.set_gimbal(self._sysid(), pitch, yaw)

    # -- onboard logs ---------------------------------------------------------
    def _logs_refresh(self):
        if not self._has_vehicle():
            QMessageBox.information(self, "Logs", "No vehicle connected.")
            return
        self.log_panel.set_status("requesting log list...")
        self.logs.request_list()

    def _logs_download(self, log_id):
        if self._has_vehicle():
            self.logs.download(log_id)

    def _logs_finished(self, ok, path, msg):
        self.log_panel.set_status(msg)
        self.console.add_note(f"log: {msg}", "#4caf50" if ok else "#ff6b6b")

    # -- manual control -------------------------------------------------------
    def _toggle_manual(self, on):
        self.manual_on = on
        self._manual_keys.clear()
        self.joystick.set_keys(0, 0, 0, 0)
        if on:
            self.jdock.show()
            self.jdock.raise_()
            self.manual_timer.start(40)             # 25 Hz
            self._on_info("manual control ON -- WASD = throttle/yaw, arrows = pitch/roll")
        else:
            self.manual_timer.stop()
            self.jdock.hide()

    def _send_manual(self):
        if not (self.manual_on and self._has_vehicle()):
            return
        x, y, z, r = self.joystick.values()
        self.link.send_manual_control(self._sysid(), x, y, z, r)

    _MANUAL_KEYS = {Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D,
                    Qt.Key_Up, Qt.Key_Down, Qt.Key_Left, Qt.Key_Right}

    def _apply_manual_keys(self):
        k = self._manual_keys
        up = (1 if Qt.Key_W in k else 0) - (1 if Qt.Key_S in k else 0)
        yaw = (1 if Qt.Key_D in k else 0) - (1 if Qt.Key_A in k else 0)
        fwd = (1 if Qt.Key_Up in k else 0) - (1 if Qt.Key_Down in k else 0)
        right = (1 if Qt.Key_Right in k else 0) - (1 if Qt.Key_Left in k else 0)
        self.joystick.set_keys(fwd, right, up, yaw)

    def keyPressEvent(self, e):
        if self.manual_on and not e.isAutoRepeat() and e.key() in self._MANUAL_KEYS:
            self._manual_keys.add(e.key())
            self._apply_manual_keys()
            e.accept()
            return
        super().keyPressEvent(e)

    def keyReleaseEvent(self, e):
        if self.manual_on and not e.isAutoRepeat() and e.key() in self._MANUAL_KEYS:
            self._manual_keys.discard(e.key())
            self._apply_manual_keys()
            e.accept()
            return
        super().keyReleaseEvent(e)

    def _toggle_record(self, on):
        if on:
            try:
                os.makedirs(LOG_DIR, exist_ok=True)
                path = os.path.join(LOG_DIR, time.strftime("dronedeck-%Y%m%d-%H%M%S.tlog"))
                self._recorder = TlogWriter(path)
            except OSError as e:
                self._on_info(f"record failed: {e}")
                self.btn_record.setChecked(False)
                return
            self._last_log = path
            if self.link is not None:
                self.link.recorder = self._recorder
            self.btn_record.setText("Recording")
            self._on_info(f"recording to {os.path.basename(path)}")
        else:
            if self.link is not None:
                self.link.recorder = None
            n = self._recorder.count if self._recorder else 0
            if self._recorder:
                self._recorder.close()
            self._recorder = None
            self.btn_record.setText("Record")
            self._on_info(f"recording stopped ({n} frames) -> {os.path.basename(self._last_log)}")

    def _open_params(self):
        if self._param_dialog is None:
            self._param_dialog = ParamDialog(self.params, self)
        self._param_dialog.show()
        self._param_dialog.raise_()
        self._param_dialog.activateWindow()
        if self._has_vehicle() and not self.params.values:
            self.params.download()

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

    def _on_map_context(self, action, lat, lon):
        if not self._has_vehicle():
            QMessageBox.information(self, "No vehicle", "Connect to a vehicle first.")
            return
        alt = max(self.vehicle.alt_rel, 30.0)
        sysid = self._sysid()
        if action == "goto":
            self.link.goto(sysid, lat, lon, alt)
            self._on_info(f"goto {lat:.5f}, {lon:.5f} @ {alt:.0f} m")
        elif action == "orbit":
            self.link.orbit(sysid, lat, lon, 50.0, alt)
            self._on_info(f"orbit {lat:.5f}, {lon:.5f} r=50 m @ {alt:.0f} m")
        elif action == "roi":
            self.link.set_roi(sysid, lat, lon, alt)
            self._on_info(f"ROI {lat:.5f}, {lon:.5f}")
        elif action == "sethome":
            self.link.set_home(sysid, lat, lon, alt)
            self._on_info(f"set home {lat:.5f}, {lon:.5f}")

    # -- mission planning -----------------------------------------------------
    def _toggle_plan(self, on):
        self.plan_mode = on
        self.map_hint.setText(" click map = add waypoint " if on else " click map = Goto ")
        self.btn_plan.setText("Plan mode ON" if on else "Plan mode")

    def _on_plan_type(self, text):
        self.plan_type = text

    def _is_fence(self):
        return self.plan_type in ("Fence incl", "Fence excl", "Circle incl", "Circle excl")

    def _redraw_fence(self):
        self.map.set_fence_shapes(self.fence_inc, self.fence_exc, self.fence_circles)

    def _add_waypoint(self, lat, lon):
        if self.plan_type == "Fence incl":
            self.fence_inc.append((lat, lon))
            self._redraw_fence()
            self.mission_status.setText(f"inclusion fence: {len(self.fence_inc)} vertices")
        elif self.plan_type == "Fence excl":
            self.fence_exc.append((lat, lon))
            self._redraw_fence()
            self.mission_status.setText(f"exclusion fence: {len(self.fence_exc)} vertices")
        elif self.plan_type in ("Circle incl", "Circle excl"):
            incl = self.plan_type == "Circle incl"
            self.fence_circles.append({"lat": lat, "lon": lon,
                                       "radius": float(self.fence_radius), "incl": incl})
            self._redraw_fence()
            self.mission_status.setText(f"{'incl' if incl else 'excl'} circle r={self.fence_radius}m "
                                        f"({len(self.fence_circles)} total)")
        elif self.plan_type == "Rally":
            self.rally_pts.append((lat, lon))
            self.map.set_rally(self.rally_pts)
            self.mission_status.setText(f"rally: {len(self.rally_pts)} points")
        else:
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
        self.mission_stats.setText(self._mission_stats_text())

    def _mission_stats_text(self):
        wps = [it for it in self.mission_items
               if not (abs(it.lat) < 1e-6 and abs(it.lon) < 1e-6)]
        if not wps:
            return "no mission"
        pts = ([self.vehicle.home] if self.vehicle.home else []) + [(it.lat, it.lon) for it in wps]
        dist = sum(haversine(a[0], a[1], b[0], b[1]) for a, b in zip(pts, pts[1:]))
        CRUISE = 10.0       # m/s assumed cruise speed for the estimate
        max_alt = max(it.alt for it in wps)
        dtxt = f"{dist:.0f} m" if dist < 1000 else f"{dist / 1000:.2f} km"
        return f"{len(wps)} WP · {dtxt} · ~{_fmt_mmss(dist / CRUISE)} · max {max_alt:.0f} m"

    def _renumber(self):
        for i, it in enumerate(self.mission_items):
            it.seq = i

    def _update_wp_row(self, idx):
        it = self.mission_items[idx]
        item = self.mission_list.item(idx)
        if item:
            item.setText(f"{it.seq:2d}  {it.cmd_name:9s} {it.lat:10.6f} {it.lon:11.6f}  {it.alt:5.0f} m")
        self.mission_stats.setText(self._mission_stats_text())

    def _wp_selected(self, idx):
        self._selecting = True
        self.mission_list.setCurrentRow(idx)
        self._selecting = False

    def _wp_list_selected(self):
        if not self._selecting:
            self.map.set_selected(self.mission_list.currentRow())

    def _wp_moved(self, idx, lat, lon):
        if 0 <= idx < len(self.mission_items):
            self.mission_items[idx].lat = lat
            self.mission_items[idx].lon = lon
            self._update_wp_row(idx)

    def _wp_delete(self):
        i = self.mission_list.currentRow()
        if 0 <= i < len(self.mission_items):
            del self.mission_items[i]
            self._renumber()
            self._refresh_mission_view()
            self.mission_list.setCurrentRow(min(i, len(self.mission_items) - 1))

    def _wp_up(self):
        i = self.mission_list.currentRow()
        if 1 <= i < len(self.mission_items):
            self.mission_items[i - 1], self.mission_items[i] = self.mission_items[i], self.mission_items[i - 1]
            self._renumber()
            self._refresh_mission_view()
            self.mission_list.setCurrentRow(i - 1)

    def _wp_down(self):
        i = self.mission_list.currentRow()
        if 0 <= i < len(self.mission_items) - 1:
            self.mission_items[i + 1], self.mission_items[i] = self.mission_items[i], self.mission_items[i + 1]
            self._renumber()
            self._refresh_mission_view()
            self.mission_list.setCurrentRow(i + 1)

    def _wp_edit_alt(self, _item=None):
        i = self.mission_list.currentRow()
        if 0 <= i < len(self.mission_items):
            alt, ok = QInputDialog.getDouble(self, "Waypoint altitude",
                                             f"Altitude for WP {i} (m):",
                                             self.mission_items[i].alt, 0.0, 2000.0, 1)
            if ok:
                self.mission_items[i].alt = alt
                self._update_wp_row(i)

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

    def _build_fence_items(self):
        """Whole geofence (inclusion/exclusion polygons + circles) as MISSION_ITEM_INTs."""
        items = []
        seq = 0
        for la, lo in self.fence_inc:
            items.append(MissionItem(seq, la, lo, 0.0,
                                     command=mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                                     param1=float(len(self.fence_inc))))
            seq += 1
        for la, lo in self.fence_exc:
            items.append(MissionItem(seq, la, lo, 0.0,
                                     command=mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                                     param1=float(len(self.fence_exc))))
            seq += 1
        for c in self.fence_circles:
            cmd = (mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION if c["incl"]
                   else mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION)
            items.append(MissionItem(seq, c["lat"], c["lon"], 0.0, command=cmd,
                                     param1=float(c["radius"])))
            seq += 1
        return items

    def _clear_mission(self):
        if self._is_fence():
            self.fence_inc, self.fence_exc, self.fence_circles = [], [], []
            self._redraw_fence()
            mt = mavlink.MAV_MISSION_TYPE_FENCE
        elif self.plan_type == "Rally":
            self.rally_pts = []
            self.map.set_rally([])
            mt = mavlink.MAV_MISSION_TYPE_RALLY
        else:
            self.mission_items = []
            self._refresh_mission_view()
            mt = mavlink.MAV_MISSION_TYPE_MISSION
        if self._has_vehicle():
            self.mission.clear(mt)

    def _upload_mission(self):
        if not self._has_vehicle():
            QMessageBox.information(self, "Upload", "No vehicle connected.")
            return
        if self._is_fence():
            if len(self.fence_inc) in (1, 2) or len(self.fence_exc) in (1, 2):
                QMessageBox.information(self, "Upload", "A fence polygon needs at least 3 vertices.")
                return
            items = self._build_fence_items()
            if not items:
                QMessageBox.information(self, "Upload", "No fence shapes to upload.")
                return
            self.mission.upload(items, mavlink.MAV_MISSION_TYPE_FENCE)
        elif self.plan_type == "Rally":
            if not self.rally_pts:
                QMessageBox.information(self, "Upload", "No rally points to upload.")
                return
            items = [MissionItem(i, la, lo, 50.0, command=mavlink.MAV_CMD_NAV_RALLY_POINT)
                     for i, (la, lo) in enumerate(self.rally_pts)]
            self.mission.upload(items, mavlink.MAV_MISSION_TYPE_RALLY)
        else:
            if not self.mission_items:
                QMessageBox.information(self, "Upload", "No waypoints to upload.")
                return
            self._renumber()
            self.mission.upload(self.mission_items, mavlink.MAV_MISSION_TYPE_MISSION)

    def _download_mission(self):
        if not self._has_vehicle():
            QMessageBox.information(self, "Download", "No vehicle connected.")
            return
        mt = (mavlink.MAV_MISSION_TYPE_FENCE if self._is_fence()
              else mavlink.MAV_MISSION_TYPE_RALLY if self.plan_type == "Rally"
              else mavlink.MAV_MISSION_TYPE_MISSION)
        self.mission.download(mt)

    def _on_mission_progress(self, msg):
        self.mission_status.setText(msg)

    def _on_mission_finished(self, ok, msg):
        self.mission_status.setText(msg)
        self.console.add_note(f"mission: {msg}", "#4caf50" if ok else "#ff6b6b")

    def _on_mission_downloaded(self, items):
        mt = self.mission.mtype
        if mt == mavlink.MAV_MISSION_TYPE_FENCE:
            self.fence_inc, self.fence_exc, self.fence_circles = [], [], []
            for it in items:
                if it.command == mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                    self.fence_inc.append((it.lat, it.lon))
                elif it.command == mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    self.fence_exc.append((it.lat, it.lon))
                elif it.command in (mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
                                    mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION):
                    self.fence_circles.append({
                        "lat": it.lat, "lon": it.lon, "radius": float(it.param1),
                        "incl": it.command == mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION})
            self._redraw_fence()
        elif mt == mavlink.MAV_MISSION_TYPE_RALLY:
            self.rally_pts = [(it.lat, it.lon) for it in items]
            self.map.set_rally(self.rally_pts)
        else:
            self.mission_items = list(items)
            self._refresh_mission_view()

    def _update_status_strip(self, ve, is_open, drop):
        GREEN, AMBER, RED = "#37d67a", "#e0a030", "#e05050"
        TEAL, GREY, LIGHT = "#39c0d0", "#9aa0ac", "#d6d9df"
        chips = []
        if ve.link_alive:
            armed = ve.armed
            # arm state as its own bold filled badge (most safety-critical); mode separate
            chips.append(("ARMED" if armed else "DISARMED", RED if armed else GREEN,
                          LIGHT, RED if armed else GREEN))
            chips.append((ve.mode, TEAL, LIGHT))
            chips.append((f"GPS {ve.fix_text} · {ve.satellites}",
                          GREEN if ve.fix_type >= 3 else AMBER, LIGHT))
            rem = ve.battery_remaining
            bcol = GREEN if (rem < 0 or rem >= 40) else (AMBER if rem >= 20 else RED)
            blabel = f"{ve.voltage:.1f}V" + (f" · {rem}%" if rem >= 0 else "")
            chips.append((blabel, bcol, bcol if (0 <= rem < 20) else LIGHT))
            chips.append((f"FLT {_fmt_mmss(self._flight_time)}", GREY, LIGHT))
        else:
            chips.append(("NO TELEMETRY", AMBER if is_open else GREY, AMBER if is_open else GREY))
        link_col = RED if drop else (TEAL if is_open else GREY)
        chips.append((f"{self._rate:.0f} Hz" + (f" · {drop} drop" if drop else ""),
                      link_col, RED if drop else LIGHT))
        chips.append((f"MSG {len(ve.messages)}", GREY, LIGHT))
        self.status_strip.set_chips(chips)

    def _set_follow(self, on):
        self.map.follow = on
        if on and self.vehicle.have_position:
            self.map.center = (self.vehicle.lat, self.vehicle.lon)

    # -- refresh --------------------------------------------------------------
    def _refresh(self):
        ve = self.vehicle
        self.adi.set_data(ve.roll, ve.pitch, ve.airspeed or ve.groundspeed,
                          ve.alt_rel, ve.heading, ve.climb)
        self.compass.set_heading(ve.heading)
        self.health.set_health(ve.sensors_present, ve.sensors_enabled, ve.sensors_health)
        self.systems.update_from(ve)
        if ve.have_position:
            self.map.update_vehicle(ve.lat, ve.lon, ve.heading, ve.home, ve.trail)

        # other vehicles + ADSB traffic on the map (traffic expires after 10 s)
        now_t = time.monotonic()
        self.traffic = {k: v for k, v in self.traffic.items() if now_t - v["t"] < 10.0}
        self.map.set_traffic([{"lat": v["lat"], "lon": v["lon"], "heading": v["heading"],
                               "callsign": v["callsign"]} for v in self.traffic.values()])
        self.map.set_others([(v.lat, v.lon, v.heading) for s, v in self.vehicles.items()
                             if v is not ve and v.have_position])

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

        # navigation readouts
        if ve.armed:
            if self._arm_t0 is None:
                self._arm_t0 = now
            self._flight_time = now - self._arm_t0
        else:
            self._arm_t0 = None
        nav = {"flight_time": _fmt_mmss(self._flight_time)}
        if ve.have_position and ve.home:
            d = haversine(ve.lat, ve.lon, ve.home[0], ve.home[1])
            nav["home_dist"] = _fmt_dist(d)
            nav["home_eta"] = _fmt_mmss(d / ve.groundspeed) if ve.groundspeed > 0.4 else "--"
        if ve.have_position and self.mission_items:
            wps = [it for it in self.mission_items
                   if not (abs(it.lat) < 1e-6 and abs(it.lon) < 1e-6)]
            if wps:
                nav["wp_dist"] = _fmt_dist(min(haversine(ve.lat, ve.lon, it.lat, it.lon)
                                               for it in wps))
        self.panel.update_all(ve, state, self._rate, ok, drop, nav)
        self._update_status_strip(ve, is_open, drop)

        connected = self._has_vehicle()
        self.btn_arm.setEnabled(connected)
        self.btn_disarm.setEnabled(connected)
        self.mode_combo.setEnabled(connected)
        for b in self._flight_btns:
            b.setEnabled(connected)
        for label, b in self._mission_btns:
            b.setEnabled(connected if label in ("Upload", "Download") else True)

        self.inspector.refresh()
        self.charts.sample(ve)

    # -- settings persistence -------------------------------------------------
    def _size_bottom_docks(self):
        # bottom row (Messages / Systems / Telemetry ...) ~= 24% of the window height;
        # the rest goes to the map + PFD. Both bottom columns are sized together.
        h = max(self.height(), 700)
        try:
            self.resizeDocks([self.msg_dock, self.sys_dock],
                             [int(h * 0.24), int(h * 0.24)], Qt.Vertical)
        except Exception:
            pass

    def load_settings(self):
        s = self.settings
        geo = s.value("win/geometry")
        if geo is not None:
            self.restoreGeometry(geo)
        st = s.value("win/state")
        if st is not None:
            self.restoreState(st)
        # keep the bottom message/systems row compact so the map + instruments get the
        # height; deferred so it runs after the window is shown (not undone by restore).
        QTimer.singleShot(0, self._size_bottom_docks)
        tr = s.value("link/transport")
        if tr in ("UDP", "TCP", "Serial", "Replay"):
            self.transport_combo.setCurrentText(tr)
        tgt = s.value("link/target")
        if tgt:
            self.link_edit.setText(str(tgt))
        try:
            lat, lon = s.value("map/lat", type=float), s.value("map/lon", type=float)
            if lat and lon:
                self.map.center = (lat, lon)
            z = s.value("map/zoom", type=int)
            if z:
                self.map.set_zoom(z)
        except (TypeError, ValueError):
            pass
        self.chk_follow.setChecked(s.value("map/follow", True, type=bool))
        raw = s.value("links/configs")
        if raw:
            try:
                self.link_configs = json.loads(raw)
            except (ValueError, TypeError):
                self.link_configs = []

    def save_settings(self):
        s = self.settings
        s.setValue("win/geometry", self.saveGeometry())
        s.setValue("win/state", self.saveState())
        s.setValue("link/transport", self.transport_combo.currentText())
        s.setValue("link/target", self.link_edit.text())
        s.setValue("map/lat", float(self.map.center[0]))
        s.setValue("map/lon", float(self.map.center[1]))
        s.setValue("map/zoom", int(self.map.zoom))
        s.setValue("map/follow", bool(self.map.follow))
        s.setValue("links/configs", json.dumps(self.link_configs))
        s.sync()

    def closeEvent(self, e):
        if self._persist:
            try:
                self.save_settings()
            except Exception:
                pass
        if self.link is not None:
            self.link.close()
        super().closeEvent(e)


def main():
    port = 14550
    replay_path = None
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if arg.isdigit():
            port = int(arg)
        elif os.path.isfile(arg):          # a .tlog (or any capture) -> replay it
            replay_path = os.path.abspath(arg)
    app = QApplication(sys.argv)
    app.setApplicationName("DroneDeck")
    app.setApplicationDisplayName("DroneDeck")
    app.setDesktopFileName("DroneDeck")     # Wayland app_id -> stable Hyprland window class
    app.setStyleSheet(DARK_QSS)
    win = DroneDeck(port, replay_path=replay_path)
    win._persist = True
    win.load_settings()
    if os.environ.get("HYPRLAND_INSTANCE_SIGNATURE"):
        win.show()               # run.sh maximizes it via hyprctl under Hyprland
    else:
        win.showMaximized()      # other WMs/DEs: open filling the screen directly
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
