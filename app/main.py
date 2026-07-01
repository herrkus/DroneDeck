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
from PySide6.QtGui import QAction, QFont, QColor
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QHBoxLayout,
                               QVBoxLayout, QSplitter, QToolBar, QLineEdit,
                               QPushButton, QLabel, QCheckBox, QMessageBox, QScrollArea,
                               QDockWidget, QComboBox, QInputDialog, QListWidget,
                               QGroupBox, QTabWidget, QSpinBox, QDialog, QFormLayout,
                               QDoubleSpinBox, QDialogButtonBox, QFileDialog, QMenu)

import core
import mavlink
from vehicle import Vehicle
from link import UdpLink, TcpLink, SerialLink, ReplayLink
from mission import (MissionProtocol, MissionItem, survey_grid, corridor_scan,
                     structure_scan, fence_from_mission, validate_mission)
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
                    StatusStrip, CameraPanel, LogPanel, SystemsPanel, MavlinkConsole)


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
/* draggable divider between the map and the bottom panels -- grab it to make the
   map smaller / the message board bigger; highlights blue on hover */
QMainWindow::separator { background:#2a2d36; height:6px; width:6px; }
QMainWindow::separator:hover { background:#3d7fb5; }
QSplitter::handle:horizontal { width:4px; }
QSplitter::handle:hover { background:#3d7fb5; }
"""


class WaypointEditor(QDialog):
    """Edit one mission item's command + altitude + the params that matter for it."""

    CMDS = [("Waypoint", 16), ("Takeoff", 22), ("Loiter (time)", 19),
            ("Loiter (unlim)", 17), ("Land", 21), ("Return to launch", 20),
            ("ROI (point camera)", 195), ("Clear ROI", 197),
            ("Change speed", 178), ("Jump to WP", 177), ("Land start", 189)]

    def __init__(self, item, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Edit WP {item.seq}")
        form = QFormLayout(self)
        self.cmd = QComboBox()
        for name, cid in self.CMDS:
            self.cmd.addItem(name, cid)
        idx = next((i for i, (_, c) in enumerate(self.CMDS) if c == item.command), -1)
        if idx < 0:                                   # unknown command -> keep it as an option
            self.cmd.addItem(item.cmd_name, item.command)
            idx = self.cmd.count() - 1
        self.cmd.setCurrentIndex(idx)

        def dspin(lo, hi, val, suf="", dec=1):
            s = QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setDecimals(dec)
            s.setValue(val)
            if suf:
                s.setSuffix(suf)
            return s

        self.alt = dspin(-500, 10000, item.alt, " m")
        self.p1 = dspin(-1e6, 1e6, item.param1)       # hold / loiter time (s)
        self.p3 = dspin(-1e6, 1e6, item.param3)       # loiter radius (m)
        self.p4 = dspin(-360, 360, item.param4, " deg")   # yaw
        self.spd = dspin(0, 100, item.param2 if item.command == 178 else 5.0, " m/s")
        self.jump_to = QSpinBox()
        self.jump_to.setRange(0, 999)
        self.jump_to.setValue(int(item.param1) if item.command == 177 else 0)
        self.jump_rep = QSpinBox()
        self.jump_rep.setRange(-1, 999)
        self.jump_rep.setSpecialValueText("forever")      # shown when value == -1
        self.jump_rep.setValue(int(item.param2) if item.command == 177 else 1)
        self.altmode = QComboBox()
        self.altmode.addItem("Relative (home)", mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
        self.altmode.addItem("AMSL", mavlink.MAV_FRAME_GLOBAL_INT)
        self.altmode.setCurrentIndex(1 if item.frame == mavlink.MAV_FRAME_GLOBAL_INT else 0)
        form.addRow("Command", self.cmd)
        form.addRow("Altitude", self.alt)
        form.addRow("Altitude mode", self.altmode)
        form.addRow("Hold / loiter time (s)", self.p1)
        form.addRow("Loiter radius (m)", self.p3)
        form.addRow("Yaw", self.p4)
        form.addRow("Speed", self.spd)
        form.addRow("Jump to WP #", self.jump_to)
        form.addRow("Repeat count", self.jump_rep)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        form.addRow(bb)
        self.cmd.currentIndexChanged.connect(self._sync_fields)
        self._sync_fields()

    def _sync_fields(self):
        """Grey out the fields that don't apply to the chosen command (QGC-style)."""
        cmd = self.cmd.currentData()
        has_pos = cmd not in (20, 178, 177, 197, 189)  # RTL/speed/jump/clear-ROI/land-start: no pos
        is_loiter = cmd in (17, 19)
        is_jump = cmd == 177
        self.alt.setEnabled(has_pos)
        self.altmode.setEnabled(has_pos)             # AMSL/relative only for georeferenced items
        self.p1.setEnabled(is_loiter)
        self.p3.setEnabled(is_loiter)
        self.p4.setEnabled(has_pos)
        self.spd.setEnabled(cmd == 178)
        self.jump_to.setEnabled(is_jump)
        self.jump_rep.setEnabled(is_jump)

    def apply_to(self, item):
        cmd = self.cmd.currentData()
        item.command = cmd
        if cmd == 178:                                # DO_CHANGE_SPEED
            item.param1 = 1.0                         # 1 = ground speed
            item.param2 = self.spd.value()
            item.param3 = -1.0                        # throttle: no change
            item.param4 = 0.0
            item.alt = 0.0
        elif cmd == 177:                              # DO_JUMP
            item.param1 = float(self.jump_to.value())   # target waypoint seq
            item.param2 = float(self.jump_rep.value())  # repeat count (-1 = forever)
            item.param3 = 0.0
            item.param4 = 0.0
            item.alt = 0.0
        elif cmd in (197, 189):                       # DO_SET_ROI_NONE / DO_LAND_START markers
            item.param1 = item.param2 = item.param3 = item.param4 = 0.0
            item.alt = 0.0
        else:
            item.alt = self.alt.value()
            item.param1 = self.p1.value()
            item.param3 = self.p3.value()
            item.param4 = self.p4.value()
        # RTL / DO_CHANGE_SPEED / DO_JUMP / clear-ROI carry no position and must use the
        # MISSION frame (2); PX4 rejects them with a global frame. Georeferenced items take
        # the chosen altitude mode: relative-to-home (6) or AMSL (5).
        item.frame = 2 if cmd in (20, 178, 177, 197, 189) else self.altmode.currentData()


class DroneDeck(QMainWindow):
    def __init__(self, port=14550, replay_path=None):
        super().__init__()
        self.setWindowTitle("DroneDeck -- MAVLink Ground Control")
        self.resize(1240, 770)
        # Floor the size so panes can never be squeezed into each other.
        self.setMinimumSize(1060, 660)
        self._bottom_sized = False   # compact the bottom row once, after the WM maximizes

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
        self._failsafe_prev = {}                # sysid -> last MAV_STATE (failsafe edge detect)

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
        zin.setToolTip("Zoom in  (+)")
        zout.setToolTip("Zoom out  (-)")
        tb.addWidget(zout); tb.addWidget(zin)
        self.map_provider = QComboBox()
        self.map_provider.blockSignals(True)
        self.map_provider.addItems(["Street", "Satellite", "Topo"])
        self.map_provider.blockSignals(False)
        self.map_provider.currentTextChanged.connect(lambda n: self.map.set_provider(n))
        self.map_provider.setToolTip("Map imagery: street / satellite / topographic")
        tb.addWidget(self.map_provider)
        tb.addSeparator()
        self.btn_help = QPushButton("?")
        self.btn_help.setFixedWidth(30)
        self.btn_help.clicked.connect(self._show_help)
        tb.addWidget(self.btn_help)

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
        # _needs: "armed" buttons only make sense while the vehicle is armed/flying;
        # the rest just need a live connection (Takeoff arms-if-needed itself).
        needs = {"Land": "armed", "RTL": "armed", "Pause": "armed",
                 "Alt": "armed", "Speed": "armed"}
        for label, slot, tip in (
                ("Takeoff", self._takeoff, "Arm if needed and climb to a set altitude  (Ctrl+T)"),
                ("Land", self._land, "Land at the current position  (Ctrl+L)"),
                ("RTL", self._rtl, "Return to launch and land  (Ctrl+R)"),
                ("Pause", self._pause, "Hold / loiter in place  (Ctrl+Space)"),
                ("Alt", self._change_alt, "Fly to a new altitude at the current position"),
                ("Speed", self._change_speed, "Set the cruise / ground speed (m/s)")):
            b = QPushButton(label)
            b.clicked.connect(slot)
            b.setToolTip(tip)
            b._needs = needs.get(label, "conn")
            tb2.addWidget(b)
            self._flight_btns.append(b)
        # VTOL transition -- only shown for VTOL airframes (MAV_TYPE 19-22)
        self.btn_vtol = QPushButton("VTOL")
        self.btn_vtol.setToolTip("Command a VTOL transition (only while flying)")
        self._vtol_menu = QMenu(self.btn_vtol)
        self._vtol_menu.addAction("Transition to Fixed-wing").triggered.connect(
            lambda: self._vtol_transition(mavlink.MAV_VTOL_STATE_FW))
        self._vtol_menu.addAction("Transition to Multirotor").triggered.connect(
            lambda: self._vtol_transition(mavlink.MAV_VTOL_STATE_MC))
        self.btn_vtol.setMenu(self._vtol_menu)
        self.btn_vtol.setVisible(False)
        tb2.addWidget(self.btn_vtol)
        tb2.addSeparator()
        self.btn_joystick = QPushButton("Joystick")
        self.btn_joystick.setCheckable(True)
        self.btn_joystick.toggled.connect(self._toggle_manual)
        self.btn_joystick._needs = "conn"
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
        for label, slot in (("Survey", self._survey), ("Corridor", self._corridor),
                            ("Structure", self._structure), ("Clear", self._clear_mission),
                            ("Upload", self._upload_mission), ("Download", self._download_mission)):
            b = QPushButton(label)
            b.clicked.connect(slot)
            tb3.addWidget(b)
            self._mission_btns.append((label, b))
        tb3.addSeparator()
        self.mission_status = QLabel("no mission")
        self.mission_status.setStyleSheet("color:#8a90a0;")
        tb3.addWidget(self.mission_status)

        self._setup_usability()   # tooltips + keyboard shortcuts (all toolbar widgets exist now)

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
        self.panel.groupsChanged.connect(self._save_telem_groups)
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
        # prominent link-loss banner (hidden until heartbeats go stale mid-session)
        self.link_banner = QLabel("")
        self.link_banner.setAlignment(Qt.AlignCenter)
        self.link_banner.setStyleSheet(
            "background:#c02020; color:white; font-weight:bold; padding:6px; font-size:13px;")
        self.link_banner.hide()
        # transient notification toast (e.g. command rejections), auto-hides
        self.notice_banner = QLabel("")
        self.notice_banner.setAlignment(Qt.AlignCenter)
        self.notice_banner.setWordWrap(True)
        self.notice_banner.hide()
        self._notice_timer = QTimer(self)
        self._notice_timer.setSingleShot(True)
        self._notice_timer.timeout.connect(self.notice_banner.hide)
        central = QWidget()
        cv = QVBoxLayout(central)
        cv.setContentsMargins(0, 0, 0, 0)
        cv.setSpacing(0)
        cv.addWidget(self.link_banner)
        cv.addWidget(self.notice_banner)
        cv.addWidget(self.status_strip)
        cv.addWidget(split, 1)
        self.setCentralWidget(central)

        # bottom message console (STATUSTEXT + command results) with a severity filter
        self.console = MessageConsole()
        self.console.setMinimumHeight(70)
        # no maximum height -- drag the map/messages divider to grow the board freely
        msg_wrap = QWidget()
        mcl = QVBoxLayout(msg_wrap)
        mcl.setContentsMargins(2, 2, 2, 2)
        mcl.setSpacing(2)
        frow = QHBoxLayout()
        frow.addWidget(QLabel("Show:"))
        self.msg_filter = QComboBox()
        for label, thr in (("All", 7), ("Info", 6), ("Warnings", 4), ("Errors", 3)):
            self.msg_filter.addItem(label, thr)
        self.msg_filter.currentIndexChanged.connect(
            lambda _=0: self.console.set_threshold(self.msg_filter.currentData()))
        frow.addWidget(self.msg_filter)
        frow.addStretch(1)
        mcl.addLayout(frow)
        mcl.addWidget(self.console, 1)
        self.msg_dock = dock = QDockWidget("Messages", self)
        dock.setObjectName("messages_dock")
        dock.setWidget(msg_wrap)
        dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock)

        # mission waypoint list + edit buttons, tabbed with Messages at the bottom
        self.mission_list = QListWidget()
        self.mission_list.setFont(QFont("DejaVu Sans Mono", 9))
        self.mission_list.itemSelectionChanged.connect(self._wp_list_selected)
        self.mission_list.itemDoubleClicked.connect(self._wp_edit)
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

        # MAVLink / nsh console (PX4 shell over SERIAL_CONTROL), tabbed at the bottom
        self.shell = MavlinkConsole()
        shdock = QDockWidget("Console", self)
        shdock.setObjectName("console_dock")
        shdock.setWidget(self.shell)
        shdock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, shdock)
        self.tabifyDockWidget(ldock, shdock)
        self.shell.send_bytes.connect(self._shell_send)

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

        self._build_menu()
        self._default_state = self.saveState()   # snapshot the default dock layout for Reset

    def _build_menu(self):
        filem = self.menuBar().addMenu("&File")
        filem.addAction("Open Plan...").triggered.connect(self._open_plan)
        filem.addAction("Save Plan...").triggered.connect(self._save_plan)
        self._view_menu = view = self.menuBar().addMenu("&View")
        self._act_reset = act_reset = view.addAction("Reset Layout")
        act_reset.setShortcut("Ctrl+Shift+L")
        act_reset.triggered.connect(self._reset_layout)
        view.addSeparator()
        self._panels_menu = panels = view.addMenu("Panels")   # show/hide any dock (QGC-style)
        for d in self.findChildren(QDockWidget):
            panels.addAction(d.toggleViewAction())
        self._tools_menu = tools = self.menuBar().addMenu("&Tools")
        act_analyze = tools.addAction("Analyze Log...")
        act_analyze.triggered.connect(self._open_analyze)
        act_fence = tools.addAction("Geofence from Mission")
        act_fence.triggered.connect(self._fence_from_mission)

    def _open_analyze(self):
        from analyze import AnalyzeDialog
        AnalyzeDialog(self, LOG_DIR).exec()

    def _save_telem_groups(self):
        if self._persist:
            self.settings.setValue("telem/hidden", json.dumps(self.panel.hidden_groups()))
            self.settings.sync()

    def _open_plan(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open mission plan", LOG_DIR,
                                              "QGC Plan (*.plan);;All files (*)")
        if not path:
            return
        try:
            import planfile
            items, (inc, exc, circles) = planfile.read_plan(path)
        except Exception as e:
            QMessageBox.warning(self, "Open Plan", f"Could not read plan:\n{e}")
            return
        if not (items or inc or exc or circles):
            self._on_info("plan has no mission items or fence")
            return
        if items:
            self.mission_items = items
            self._renumber()
            self._refresh_mission_view()
            self.map.center = (items[0].lat, items[0].lon)
        self.fence_inc, self.fence_exc, self.fence_circles = inc, exc, circles
        self.map.set_fence_shapes(self.fence_inc, self.fence_exc, self.fence_circles)
        self.map.update()
        extra = " + fence" if (inc or exc or circles) else ""
        self._on_info(f"loaded {len(items)} waypoints{extra} from {os.path.basename(path)}")

    def _save_plan(self):
        if not (self.mission_items or self.fence_inc or self.fence_exc or self.fence_circles):
            QMessageBox.information(self, "Save Plan", "Nothing to save (no waypoints or fence).")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Save mission plan",
                                              os.path.join(LOG_DIR, "mission.plan"),
                                              "QGC Plan (*.plan)")
        if not path:
            return
        if not path.endswith(".plan"):
            path += ".plan"
        try:
            import planfile
            self._renumber()
            planfile.save_plan(path, self.mission_items,
                               fence=(self.fence_inc, self.fence_exc, self.fence_circles),
                               home=self.vehicle.home if self.vehicle.home else None)
        except Exception as e:
            QMessageBox.warning(self, "Save Plan", f"Could not save plan:\n{e}")
            return
        self._on_info(f"saved {len(self.mission_items)} waypoints to {os.path.basename(path)}")

    def _fence_from_mission(self):
        """Build an inclusion geofence (convex hull + margin) around the planned mission
        and switch to Fence-incl mode so Upload sends it."""
        pts = [(it.lat, it.lon) for it in self.mission_items
               if not (abs(it.lat) < 1e-6 and abs(it.lon) < 1e-6)]
        if len(pts) < 2:
            QMessageBox.information(self, "Geofence from Mission",
                                    "Plan at least 2 mission waypoints first.")
            return
        fence = fence_from_mission(pts, margin_m=float(self.fence_radius))
        if len(fence) < 3:
            return
        self.fence_inc = fence
        self.plan_type = "Fence incl"
        self.plan_type_combo.setCurrentText("Fence incl")
        self.map.set_fence_shapes(self.fence_inc, self.fence_exc, self.fence_circles)
        self._on_info(f"geofence: {len(fence)}-vertex inclusion polygon "
                      f"({self.fence_radius} m margin) -- press Upload to send")

    def _reset_layout(self):
        """Restore the default dock arrangement + the compact bottom row."""
        if getattr(self, "_default_state", None) is not None:
            self.restoreState(self._default_state)
        self._bottom_sized = False
        self._size_bottom_docks()
        QTimer.singleShot(0, self._size_bottom_docks)
        self._on_info("layout reset to default")

    CMD_NAMES = {400: "ARM/DISARM", 22: "TAKEOFF", 21: "LAND", 20: "RTL",
                 176: "SET MODE", 192: "REPOSITION", 193: "PAUSE/CONTINUE"}

    def _wire(self):
        self.vehicle.status_text.connect(self.console.add_message)
        self.vehicle.command_ack.connect(self._on_command_ack)
        self.map.clicked.connect(self._on_map_click)
        self.map.contextAction.connect(self._on_map_context)
        self.map.wpAction.connect(self._on_wp_action)
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
        link.messages.connect(self.shell.handle_messages)
        link.info.connect(self._on_info)
        link.state.connect(self._on_state)
        link.recorder = self._recorder        # keep recording across reconnects
        return link

    def _shell_send(self, data):
        if self._has_vehicle():
            self.link.send_serial_control(data)

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
        # A rejected safety-critical command (arm, takeoff, ...) is easy to miss in the
        # console -- pop a prominent toast with the autopilot's own reason (the most
        # recent warning/error STATUSTEXT, e.g. "Arming denied: GPS not ready").
        critical = {mavlink.MAV_CMD_COMPONENT_ARM_DISARM, mavlink.MAV_CMD_NAV_TAKEOFF,
                    mavlink.MAV_CMD_NAV_LAND, mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                    mavlink.MAV_CMD_DO_SET_MODE}
        if not ok and command in critical:
            reason = next((txt for sev, txt in reversed(self.vehicle.messages[-12:])
                           if sev <= 4), "")
            self._notify(f"{name} REJECTED: {res}" + (f"  --  {reason}" if reason else ""),
                         "#c02020")

    def _notify(self, text, color="#e0a030", ms=7000):
        """Show a transient, auto-hiding notification banner (a toast)."""
        self.notice_banner.setStyleSheet(
            f"background:{color}; color:white; font-weight:bold; padding:6px; font-size:13px;")
        self.notice_banner.setText(text)
        self.notice_banner.show()
        self._notice_timer.start(ms)

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
        dlg = CalibrationDialog(lambda: self.link, self.params, self)
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
            is_new = sysid not in self.vehicles
            veh = self._ensure_vehicle(sysid)
            veh.consume(msgs)
            if is_new and self.link is not None:
                # A real drone streams little until asked -- request telemetry now that
                # we know it exists (and its autopilot, for the right request dialect).
                self.link.request_data_streams(sysid, veh.autopilot)
                self._on_info(f"vehicle #{sysid} detected -- requesting telemetry streams")
        self._sync_mode_combo()   # keep the mode selector matched to the autopilot

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

    def _update_button_states(self):
        """Grey out actions that can't be used right now: everything needs a live
        link; Arm only when disarmed, Disarm only when armed, and the flight actions
        tagged _needs=='armed' (Land/RTL/Pause/Alt/Speed) only while armed."""
        ve = self.vehicle
        connected = self._has_vehicle()
        armed = connected and ve.armed
        self.btn_arm.setEnabled(connected and not armed)
        self.btn_disarm.setEnabled(connected and armed)
        self.mode_combo.setEnabled(connected)
        for b in self._flight_btns:
            b.setEnabled(connected and (armed if getattr(b, "_needs", "conn") == "armed" else True))
        # VTOL transition button: shown only for VTOL airframes, active only while armed
        is_vtol = connected and ve.mav_type in (19, 20, 21, 22)
        self.btn_vtol.setVisible(is_vtol)
        self.btn_vtol.setEnabled(is_vtol and armed)
        for label, b in self._mission_btns:
            b.setEnabled(connected if label in ("Upload", "Download") else True)

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
        params = mavlink.mode_command(self.vehicle.autopilot, self.vehicle.mav_type, name)
        if params is None:
            self._on_info(f"mode {name} not available for this vehicle")
            return
        self.link.set_mode(self._sysid(), params[0], params[1])
        self._on_info(f"set mode {name}")

    def _sync_mode_combo(self):
        # Show the mode list matching the connected autopilot (PX4 vs ArduPilot) so the
        # selector never offers modes the vehicle can't accept. Rebuilds only on change.
        key = (int(self.vehicle.autopilot), int(self.vehicle.mav_type))
        if key == getattr(self, "_mode_combo_key", None):
            return
        self._mode_combo_key = key
        self.mode_combo.blockSignals(True)
        self.mode_combo.clear()
        self.mode_combo.addItems(mavlink.mode_names(self.vehicle.autopilot, self.vehicle.mav_type))
        idx = self.mode_combo.findText(self.vehicle.mode)
        if idx >= 0:
            self.mode_combo.setCurrentIndex(idx)
        self.mode_combo.blockSignals(False)

    def _takeoff(self):
        if not self._has_vehicle():
            return
        alt, ok = QInputDialog.getDouble(self, "Takeoff", "Altitude (m):", 30.0, 1.0, 1000.0, 1)
        if ok:
            self.link.takeoff(self._sysid(), alt, self.vehicle.lat, self.vehicle.lon)
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

    def _change_alt(self):
        if not self._has_vehicle():
            return
        cur = round(self.vehicle.alt_rel) or 30
        alt, ok = QInputDialog.getDouble(self, "Change Altitude",
                                         "Altitude above home (m):", float(cur), 1.0, 1000.0, 1)
        if ok:
            self.link.change_altitude(self._sysid(), self.vehicle.lat, self.vehicle.lon, alt)
            self._on_info(f"change altitude to {alt:.0f} m")

    def _change_speed(self):
        if not self._has_vehicle():
            return
        cur = round(self.vehicle.groundspeed, 1) or 5.0
        spd, ok = QInputDialog.getDouble(self, "Change Speed",
                                         "Ground speed (m/s):", float(cur), 0.5, 100.0, 1)
        if ok:
            self.link.change_speed(self._sysid(), spd)
            self._on_info(f"change speed to {spd:.1f} m/s")

    def _vtol_transition(self, state):
        if not self._has_vehicle():
            return
        name = "fixed-wing" if state == mavlink.MAV_VTOL_STATE_FW else "multirotor"
        self.link.vtol_transition(self._sysid(), state)
        self._on_info(f"VTOL transition to {name} requested")

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
        if action == "add_roi":                        # a mission item, not a live command
            alt = self.mission_items[-1].alt if self.mission_items else 50.0
            self.mission_items.append(MissionItem(len(self.mission_items), lat, lon, alt,
                                                  command=195))
            self._refresh_mission_view()
            self._on_info(f"added ROI waypoint at {lat:.5f}, {lon:.5f}")
            return
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

    def _wp_row_text(self, it):
        extra = ""
        if it.command in (19, 17):                    # LOITER_TIME / LOITER_UNLIM
            if it.param1:
                extra = f"  {it.param1:.0f}s"
            elif it.param3:
                extra = f"  r{it.param3:.0f}"
        elif it.command == 178:                       # DO_CHANGE_SPEED
            extra = f"  {it.param2:.1f} m/s"
        elif it.command == 177:                       # DO_JUMP
            rep = "inf" if it.param2 < 0 else f"{it.param2:.0f}"
            extra = f"  -> WP{it.param1:.0f} x{rep}"
        amode = " MSL" if it.frame == mavlink.MAV_FRAME_GLOBAL_INT else ""
        return (f"{it.seq:2d}  {it.cmd_name:9s} {it.lat:10.6f} {it.lon:11.6f}"
                f"  {it.alt:5.0f} m{amode}{extra}")

    def _refresh_mission_view(self):
        self.mission_list.clear()
        for it in self.mission_items:
            self.mission_list.addItem(self._wp_row_text(it))
        self.map.set_mission([(it.lat, it.lon) for it in self.mission_items])
        n = len(self.mission_items)
        self.mission_status.setText(f"{n} waypoint{'' if n == 1 else 's'}" if n else "no mission")
        self.mission_stats.setText(self._mission_stats_text())
        self._hl_wp = -2                      # force the current-wp highlight to re-apply

    def _highlight_current_wp(self, cur):
        if cur == getattr(self, "_hl_wp", -2):
            return
        self._hl_wp = cur
        for r in range(self.mission_list.count()):
            it = self.mission_list.item(r)
            if it is not None:
                it.setForeground(QColor("#37d67a") if r == cur else QColor("#d6d9df"))

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
            item.setText(self._wp_row_text(it))
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

    def _wp_edit(self, _item=None):
        i = self.mission_list.currentRow()
        if not (0 <= i < len(self.mission_items)):
            return
        dlg = WaypointEditor(self.mission_items[i], self)
        if dlg.exec() == QDialog.Accepted:
            dlg.apply_to(self.mission_items[i])
            self._update_wp_row(i)

    def _on_wp_action(self, action, idx):
        """Right-click-a-waypoint actions from the map: edit / insert-before / delete."""
        if not (0 <= idx < len(self.mission_items)):
            return
        if action == "edit":
            self.mission_list.setCurrentRow(idx)
            self._wp_edit()
        elif action == "delete":
            del self.mission_items[idx]
            self._renumber()
            self._refresh_mission_view()
            self.mission_list.setCurrentRow(min(idx, len(self.mission_items) - 1))
            self._on_info(f"deleted WP {idx}")
        elif action == "insert_before":
            cur = self.mission_items[idx]
            if idx > 0:                            # drop the new point on the mid of the leg
                prev = self.mission_items[idx - 1]
                la, lo = (prev.lat + cur.lat) / 2.0, (prev.lon + cur.lon) / 2.0
            else:
                la, lo = cur.lat, cur.lon
            self.mission_items.insert(idx, MissionItem(idx, la, lo, cur.alt))
            self._renumber()
            self._refresh_mission_view()
            self.mission_list.setCurrentRow(idx)
            self._on_info(f"inserted WP before {idx + 1}")

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

    def _corridor(self):
        if len(self.mission_items) < 2:
            QMessageBox.information(self, "Corridor Scan",
                                    "Place at least 2 waypoints to draw the corridor "
                                    "centerline, then Corridor.")
            return
        from PySide6.QtWidgets import QDialog, QFormLayout, QDialogButtonBox
        dlg = QDialog(self)
        dlg.setWindowTitle("Corridor Scan")
        form = QFormLayout(dlg)

        def spin(lo, hi, val, suf=" m"):
            s = QSpinBox()
            s.setRange(lo, hi)
            s.setValue(val)
            s.setSuffix(suf)
            return s

        w_spin = spin(5, 2000, 60)
        s_spin = spin(2, 500, 30)
        a_spin = spin(2, 1000, int(self.mission_items[0].alt or 50))
        form.addRow("Corridor width", w_spin)
        form.addRow("Pass spacing", s_spin)
        form.addRow("Altitude", a_spin)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept)
        bb.rejected.connect(dlg.reject)
        form.addRow(bb)
        if dlg.exec() != QDialog.Accepted:
            return
        items = corridor_scan([(it.lat, it.lon) for it in self.mission_items],
                              width_m=float(w_spin.value()), spacing_m=float(s_spin.value()),
                              alt=float(a_spin.value()))
        if items:
            self.mission_items = items
            self._refresh_mission_view()
            self._on_info(f"corridor scan: {len(items)} waypoints "
                          f"({w_spin.value()}m wide, {s_spin.value()}m spacing)")

    def _structure(self):
        if not self.mission_items:
            QMessageBox.information(self, "Structure Scan",
                                    "Place a waypoint at the structure (or a polygon "
                                    "around it), then Structure.")
            return
        from PySide6.QtWidgets import QDialog, QFormLayout, QDialogButtonBox
        dlg = QDialog(self)
        dlg.setWindowTitle("Structure Scan")
        form = QFormLayout(dlg)

        def spin(lo, hi, val, suf=" m"):
            s = QSpinBox()
            s.setRange(lo, hi)
            s.setValue(val)
            s.setSuffix(suf)
            return s

        r_spin = spin(2, 1000, 30)
        n_spin = spin(1, 30, 4, "")
        h_spin = spin(1, 100, 8)
        a_spin = spin(2, 500, int(self.mission_items[0].alt or 15))
        p_spin = spin(3, 72, 16, "")
        form.addRow("Standoff radius", r_spin)
        form.addRow("Layers", n_spin)
        form.addRow("Layer height", h_spin)
        form.addRow("Base altitude", a_spin)
        form.addRow("Points / layer", p_spin)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept)
        bb.rejected.connect(dlg.reject)
        form.addRow(bb)
        if dlg.exec() != QDialog.Accepted:
            return
        items = structure_scan([(it.lat, it.lon) for it in self.mission_items],
                               radius_m=float(r_spin.value()), layers=n_spin.value(),
                               layer_height_m=float(h_spin.value()), base_alt=float(a_spin.value()),
                               n_points=p_spin.value())
        if items:
            self.mission_items = items
            self._refresh_mission_view()
            self._on_info(f"structure scan: {len(items)} waypoints "
                          f"({n_spin.value()} layers x {p_spin.value()} pts, "
                          f"{r_spin.value()}m standoff)")

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
            warns = validate_mission(self.mission_items)
            for w in warns:                              # advisory, non-blocking (QGC-style)
                self.console.add_note(f"mission check: {w}", "#e0a030")
            if warns:
                self._notify(f"Mission check: {len(warns)} potential issue(s) -- see Messages",
                             "#e0a030")
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
            if not armed:
                ready, reasons = ve.preflight_status()
                if ready:
                    chips.append(("READY TO ARM", GREEN, GREEN))
                else:
                    first = reasons[0] + (f"  +{len(reasons) - 1}" if len(reasons) > 1 else "")
                    chips.append((f"NOT READY: {first}", AMBER, AMBER))
                self.status_strip.setToolTip(
                    "Preflight: " + ("ready to arm" if ready else "; ".join(reasons)))
            else:
                self.status_strip.setToolTip("")
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

    def _update_link_banner(self, ve, is_open):
        # Prominent alert only when we HAD telemetry and it went stale (mid-session link
        # loss) -- the most safety-critical case. Startup "no telemetry yet" stays quiet.
        if is_open and ve.last_heartbeat and not ve.link_alive:
            secs = int(time.monotonic() - ve.last_heartbeat)
            self.link_banner.setText(f"COMMUNICATION LOST  --  no heartbeat for {secs} s")
            self.link_banner.show()
        else:
            self.link_banner.hide()

    def _set_follow(self, on):
        self.map.follow = on
        if on and self.vehicle.have_position:
            self.map.center = (self.vehicle.lat, self.vehicle.lon)

    # -- refresh --------------------------------------------------------------
    def _check_failsafe(self, ve):
        """Raise a one-shot console note + toast when a vehicle transitions INTO a
        Critical/Emergency (or worse) MAV_STATE. Fires once per entry into the failsafe
        band, not on every telemetry update, and re-arms after the vehicle recovers."""
        sid = ve.sysid
        if not sid:
            return
        prev = self._failsafe_prev.get(sid)
        cur = ve.system_status
        self._failsafe_prev[sid] = cur
        if cur >= 5 and (prev is None or prev < 5):
            name = {5: "CRITICAL", 6: "EMERGENCY", 7: "POWEROFF",
                    8: "FLIGHT TERMINATION"}.get(cur, f"MAV_STATE {cur}")
            self.console.add_note(f"FAILSAFE: vehicle {sid} entered {name}", "#e05050")
            self._notify(f"FAILSAFE: {name}", "#e05050")

    def _refresh(self):
        ve = self.vehicle
        self._check_failsafe(ve)
        self.adi.set_data(ve.roll, ve.pitch, ve.airspeed or ve.groundspeed,
                          ve.alt_rel, ve.heading, ve.climb)
        self.compass.set_heading(ve.heading)
        self.health.set_health(ve.sensors_present, ve.sensors_enabled, ve.sensors_health)
        self.systems.update_from(ve)
        if ve.have_position:
            self.map.update_vehicle(ve.lat, ve.lon, ve.heading, ve.home, ve.trail)
        self.map.set_current_wp(ve.current_wp)
        self._highlight_current_wp(ve.current_wp)

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
        self._update_link_banner(ve, is_open)

        self._update_button_states()

        self.inspector.refresh()
        self.charts.sample(ve)

    # -- settings persistence -------------------------------------------------
    def _setup_usability(self):
        # Tooltips: hover any control to learn what it does (and its shortcut).
        self.btn_conn.setToolTip("Connect / disconnect the current link  (Ctrl+K)")
        self.btn_links.setToolTip("Manage saved comm links (UDP / TCP / serial)")
        self.btn_cal.setToolTip("Vehicle setup: radio + sensor calibration")
        self.btn_arm.setToolTip("Arm the vehicle  (Ctrl+Shift+A)")
        self.btn_disarm.setToolTip("Disarm the vehicle  (Ctrl+Shift+D)")
        self.btn_params.setToolTip("Download, search and edit parameters  (Ctrl+P)")
        self.btn_record.setToolTip("Record telemetry to a .tlog file for replay")
        self.btn_help.setToolTip("Keyboard shortcuts & quick help  (F1)")
        self.chk_follow.setToolTip("Keep the map centred on the vehicle  (F)")
        self.transport_combo.setToolTip("Link type: UDP / TCP / Serial / Replay a .tlog")
        self.link_edit.setToolTip("UDP port, TCP host:port, serial port:baud, or .tlog path")
        self.vehicle_combo.setToolTip("Select which vehicle to control")
        self.mode_combo.setToolTip("Set the flight mode")
        self.btn_joystick.setToolTip("On-screen thumbsticks + WASD/arrows stream MANUAL_CONTROL  (J)")
        self.btn_plan.setToolTip("Plan mode: click the map to add mission waypoints")
        # Keyboard shortcuts for the common actions.
        from PySide6.QtGui import QKeySequence, QShortcut
        def sc(seq, fn):
            s = QShortcut(QKeySequence(seq), self)
            s.activated.connect(fn)
        sc("Ctrl+K", self._toggle_conn)
        sc("Ctrl+Shift+A", lambda: self._arm(True))
        sc("Ctrl+Shift+D", lambda: self._arm(False))
        sc("Ctrl+T", self._takeoff)
        sc("Ctrl+L", self._land)
        sc("Ctrl+R", self._rtl)
        sc("Ctrl+Space", self._pause)
        sc("Ctrl+P", self._open_params)
        sc("F", self.chk_follow.toggle)
        sc("J", self.btn_joystick.toggle)
        sc("+", lambda: self.map.set_zoom(self.map.zoom + 1))
        sc("=", lambda: self.map.set_zoom(self.map.zoom + 1))
        sc("-", lambda: self.map.set_zoom(self.map.zoom - 1))
        sc("F1", self._show_help)

    def _show_help(self):
        html = (
            "<h3>DroneDeck &mdash; Quick Help</h3>"
            "<p><b>Connect:</b> choose a Link type (UDP / TCP / Serial), set the target, "
            "then Connect. A drone over Wi-Fi/UDP uses port 14550; a telemetry radio uses "
            "Serial, e.g. <tt>/dev/ttyUSB0:57600</tt>.</p>"
            "<p><b>Fly:</b> pick a Mode, Arm, then Takeoff. Click the map to Goto. "
            "Use Land / RTL / Pause as needed.</p>"
            "<table cellpadding='4'>"
            "<tr><td><b>Ctrl+K</b></td><td>connect / disconnect</td>"
            "<td><b>Ctrl+Shift+A/D</b></td><td>arm / disarm</td></tr>"
            "<tr><td><b>Ctrl+T</b></td><td>takeoff</td><td><b>Ctrl+L</b></td><td>land</td></tr>"
            "<tr><td><b>Ctrl+R</b></td><td>return to launch</td>"
            "<td><b>Ctrl+Space</b></td><td>pause / hold</td></tr>"
            "<tr><td><b>F</b></td><td>follow vehicle</td><td><b>J</b></td><td>joystick</td></tr>"
            "<tr><td><b>+ / &minus;</b></td><td>zoom map</td>"
            "<td><b>Ctrl+P</b></td><td>parameters</td></tr>"
            "<tr><td><b>F1</b></td><td>this help</td><td></td><td></td></tr>"
            "</table>")
        QMessageBox.information(self, "DroneDeck Help", html)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        # The window is shown small and the WM maximizes it a moment later, so wait for
        # the real (tall) size before compacting the bottom row -- sizing against the
        # pre-maximize height lets the docks balloon when the window then grows. Runs
        # once; afterwards the dividers are the user's to drag freely.
        if not self._bottom_sized and self.height() > 850:
            self._bottom_sized = True
            self._size_bottom_docks()

    def _size_bottom_docks(self):
        # Clean startup layout: a compact bottom row (~24% of the window height) so the
        # map + PFD get the space, with Messages (left) and Systems (right) as the active
        # tabs. Runs every launch so the app always opens in this known-good arrangement
        # (the dividers stay draggable during the session for a temporary taller board).
        h = max(self.height(), 700)
        try:
            self.resizeDocks([self.msg_dock, self.sys_dock],
                             [int(h * 0.24), int(h * 0.24)], Qt.Vertical)
            self.msg_dock.raise_()   # Messages up on the left
            self.sys_dock.raise_()   # Systems up on the right
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
        # Always open in the clean, compact layout (map big, Messages + Systems tabs up)
        # so the app looks the same every launch -- no stale/offset bottom row. The
        # dividers stay draggable during the session for a temporary bigger board.
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
            prov = s.value("map/provider")
            if prov in ("Street", "Satellite", "Topo"):
                self.map.set_provider(prov)
                self.map_provider.setCurrentText(prov)
        except (TypeError, ValueError):
            pass
        self.chk_follow.setChecked(s.value("map/follow", True, type=bool))
        raw = s.value("links/configs")
        if raw:
            try:
                self.link_configs = json.loads(raw)
            except (ValueError, TypeError):
                self.link_configs = []
        hidden = s.value("telem/hidden")
        if hidden:
            try:
                self.panel.set_hidden_groups(json.loads(hidden))
            except (ValueError, TypeError):
                pass

    def save_settings(self):
        s = self.settings
        s.setValue("win/geometry", self.saveGeometry())
        s.setValue("win/state", self.saveState())
        s.setValue("link/transport", self.transport_combo.currentText())
        s.setValue("link/target", self.link_edit.text())
        s.setValue("map/lat", float(self.map.center[0]))
        s.setValue("map/lon", float(self.map.center[1]))
        s.setValue("map/zoom", int(self.map.zoom))
        s.setValue("map/provider", self.map.provider)
        s.setValue("map/follow", bool(self.map.follow))
        s.setValue("links/configs", json.dumps(self.link_configs))
        s.setValue("telem/hidden", json.dumps(self.panel.hidden_groups()))
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
