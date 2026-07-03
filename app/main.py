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
                               QDoubleSpinBox, QDialogButtonBox, QFileDialog, QMenu,
                               QProgressBar)

import core
import mavlink
from vehicle import Vehicle
from link import UdpLink, TcpLink, SerialLink, ReplayLink
from mission import (MissionProtocol, MissionItem, survey_grid, corridor_scan,
                     structure_scan, fence_from_mission, validate_mission,
                     autopilot_mission_warnings, px4_unsupported_cmds)
from params import ParamManager, ParamDialog
from tlog import TlogWriter
from logdownload import LogManager
from charts import ChartPanel
from joystick import VirtualJoystick, HwJoystick, list_joysticks
from video import VideoPane
from links_manager import LinksDialog
from calibration import CalibrationDialog
from instruments import AttitudeIndicator, Compass

LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
APP_VERSION = "1.0"
from mapview import MapView
from panels import (TelemetryPanel, MessageConsole, MavInspector, HealthPanel,
                    StatusStrip, CameraPanel, LogPanel, SystemsPanel, MavlinkConsole,
                    TrafficPanel)


def haversine(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres."""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(min(1.0, math.sqrt(a)))


def bearing(lat1, lon1, lat2, lon2):
    """Initial great-circle bearing from point 1 to point 2, degrees 0-360 (0 = north)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    b = math.degrees(math.atan2(y, x)) % 360.0
    # a tiny negative angle (e.g. bearing to a pole) yields -eps % 360 == 360.0 in float, which
    # violates the documented [0, 360) range and would break a consumer like int(b / 45); pin it to 0
    return b if b < 360.0 else 0.0


def trail_to_gpx(points, name="DroneDeck flight track"):
    """Serialize a flown track [(lat, lon), ...] to a GPX 1.1 document (string) -- the standard
    GPS-track format any mapping tool (Google Earth, QGIS, ...) can open."""
    from xml.sax.saxutils import escape
    head = ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<gpx version="1.1" creator="DroneDeck" '
            'xmlns="http://www.topografix.com/GPX/1/1">\n'
            f'  <trk><name>{escape(str(name))}</name><trkseg>\n')
    # skip non-finite points: f"{nan:.7f}" == 'nan', which is not valid GPX decimal and makes mapping
    # tools reject the whole file. The trail is normally int-derived (finite), but a replayed/edited
    # source could carry NaN/Inf -- never emit a malformed track.
    body = "".join(f'    <trkpt lat="{la:.7f}" lon="{lo:.7f}"></trkpt>\n'
                   for la, lo in points if math.isfinite(la) and math.isfinite(lo))
    return head + body + '  </trkseg></trk>\n</gpx>\n'


def _parse_port(text, default):
    """Parse a user-typed TCP/UDP port to a valid 1..65535 int, or raise ValueError. A port outside
    that range reaches Qt's socket bind/connect as an out-of-uint16 value and raises OverflowError
    (not ValueError), which would escape _connect's handler and crash the Connect action."""
    n = int(text) if str(text).strip() else default
    if not 0 < n <= 65535:
        raise ValueError(f"port {n} out of range 1-65535")
    return n


def _point_in_poly(pt, poly):
    """Ray-casting point-in-polygon. pt=(lat, lon), poly=[(lat, lon), ...]. Treats lat/lon
    as planar, which is fine over geofence-sized areas."""
    n = len(poly)
    if n < 3:
        return False
    x, y = pt[1], pt[0]                 # lon = x, lat = y
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i][1], poly[i][0]
        xj, yj = poly[j][1], poly[j][0]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


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

    CMDS = [("Waypoint", 16), ("Spline waypoint", 82), ("Takeoff", 22), ("Loiter (time)", 19),
            ("Loiter (unlim)", 17), ("Loiter (turns)", 18), ("Loiter to alt", 31), ("Delay", 93),
            ("Land", 21), ("VTOL takeoff", 84), ("VTOL land", 85), ("Return to launch", 20),
            ("ROI (point camera)", 195), ("Clear ROI", 197),
            ("Change speed", 178), ("Jump to WP", 177), ("Land start", 189),
            ("Set servo", 183), ("Condition: Yaw", 115), ("Camera trig dist", 206)]

    def __init__(self, item, parent=None, autopilot=None):
        super().__init__(parent)
        self.setWindowTitle(f"Edit WP {item.seq}")
        form = QFormLayout(self)
        self.cmd = QComboBox()
        # On PX4, hide the commands its firmware rejects (QGC does the same) so the user can't build a
        # mission that fails to upload -- but always keep THIS item's own command visible, so an item
        # loaded from an ArduPilot .plan can still be seen/edited on a PX4 link.
        hide = px4_unsupported_cmds() if autopilot == mavlink.MAV_AUTOPILOT_PX4 else set()
        palette = [(name, cid) for name, cid in self.CMDS
                   if cid not in hide or cid == item.command]
        for name, cid in palette:
            self.cmd.addItem(name, cid)
        idx = next((i for i, (_, c) in enumerate(palette) if c == item.command), -1)
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
        self.servo_ch = QSpinBox()
        self.servo_ch.setRange(1, 16)
        self.servo_ch.setValue(int(item.param1) if item.command == 183 else 5)
        self.servo_pwm = QSpinBox()
        self.servo_pwm.setRange(800, 2200)
        self.servo_pwm.setSuffix(" us")
        self.servo_pwm.setValue(int(item.param2) if item.command == 183 else 1500)
        self.altmode = QComboBox()
        self.altmode.addItem("Relative (home)", mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
        self.altmode.addItem("AMSL", mavlink.MAV_FRAME_GLOBAL_INT)
        self.altmode.addItem("Terrain (AGL)", mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT)
        _ai = self.altmode.findData(item.frame)
        self.altmode.setCurrentIndex(_ai if _ai >= 0 else 0)
        form.addRow("Command", self.cmd)
        form.addRow("Altitude", self.alt)
        form.addRow("Altitude mode", self.altmode)
        self._p1_label = QLabel("Hold / loiter time (s)")   # relabelled per command (QGC-style)
        form.addRow(self._p1_label, self.p1)
        form.addRow("Loiter radius (m)", self.p3)
        form.addRow("Yaw", self.p4)
        form.addRow("Speed", self.spd)
        form.addRow("Jump to WP #", self.jump_to)
        form.addRow("Repeat count", self.jump_rep)
        form.addRow("Servo channel", self.servo_ch)
        form.addRow("Servo PWM", self.servo_pwm)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        form.addRow(bb)
        self.cmd.currentIndexChanged.connect(self._sync_fields)
        self._sync_fields()

    def _sync_fields(self):
        """Grey out the fields that don't apply to the chosen command (QGC-style)."""
        cmd = self.cmd.currentData()
        # RTL/speed/jump/clear-ROI/land-start/delay/set-servo/yaw/cam-trigg carry no position
        has_pos = cmd not in (20, 178, 177, 197, 189, 93, 183, 115, 206)
        is_loiter = cmd in (17, 19, 18)              # unlim / time / turns
        is_delay = cmd == 93
        is_jump = cmd == 177
        is_servo = cmd == 183
        is_yaw = cmd == 115                          # CONDITION_YAW: only the heading matters
        is_trigg = cmd == 206                        # DO_SET_CAM_TRIGG_DIST: only the distance
        is_loiter_alt = cmd == 31                    # NAV_LOITER_TO_ALT: circle to a target altitude
        self.alt.setEnabled(has_pos)
        self.altmode.setEnabled(has_pos)             # AMSL/relative only for georeferenced items
        self.p1.setEnabled(is_loiter or is_delay or is_trigg)   # loiter time/turns, delay s, or trig m
        self.p3.setEnabled(is_loiter or is_loiter_alt)          # loiter radius (also for loiter-to-alt)
        self.p4.setEnabled((has_pos and not is_loiter_alt) or is_yaw)   # Yaw: not used by loiter-to-alt
        self.spd.setEnabled(cmd == 178)
        self.jump_to.setEnabled(is_jump)
        self.jump_rep.setEnabled(is_jump)
        self.servo_ch.setEnabled(is_servo)
        self.servo_pwm.setEnabled(is_servo)
        # the multi-purpose p1 field means different things per command -- relabel it like QGC does
        self._p1_label.setText("Loiter turns" if cmd == 18 else
                               "Delay (s)" if is_delay else
                               "Trigger dist (m)" if is_trigg else
                               "Hold / loiter time (s)")

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
        elif cmd == 93:                               # NAV_DELAY: hold at the current point
            item.param1 = self.p1.value()             # delay seconds (>=0; -1 would use hh/mm/ss)
            item.param2 = item.param3 = item.param4 = 0.0
            item.alt = 0.0
        elif cmd == 183:                              # DO_SET_SERVO: drive a servo/actuator channel
            item.param1 = float(self.servo_ch.value())    # servo output channel (1-16)
            item.param2 = float(self.servo_pwm.value())    # PWM microseconds (typ. 1000-2000)
            item.param3 = item.param4 = 0.0
            item.alt = 0.0
        elif cmd == 115:                              # CONDITION_YAW: point the nose to a heading
            item.param1 = float(self.p4.value()) % 360.0   # target angle (deg), reuses the Yaw field
            item.param2 = 0.0                         # yaw rate: 0 = autopilot default
            item.param3 = 0.0                         # direction: 0 = shortest way round
            item.param4 = 0.0                         # 0 = absolute heading (1 would be relative)
            item.alt = 0.0
        elif cmd == 206:                              # DO_SET_CAM_TRIGG_DIST: shoot every N metres
            item.param1 = max(0.0, float(self.p1.value()))  # trigger distance m (0 = stop), reuses p1
            item.param2 = 0.0                         # shutter integration time: 0 = autopilot default
            item.param3 = 0.0                         # 0 = do not fire one immediately on receipt
            item.param4 = 0.0
            item.alt = 0.0
        elif cmd == 31:                               # NAV_LOITER_TO_ALT: circle here until at alt
            item.alt = self.alt.value()               # target altitude (the point of the item)
            item.param1 = 0.0                         # 0 = heading not required at loiter exit
            item.param2 = float(self.p3.value())      # loiter radius (m) -- note: radius is param2 here
            item.param3 = 0.0
            item.param4 = 0.0                         # xtrack (0); NOT a yaw
        else:
            item.alt = self.alt.value()
            item.param1 = self.p1.value()             # loiter time (17/19) or turns (18)
            item.param3 = self.p3.value()
            item.param4 = self.p4.value()
        # RTL / DO_CHANGE_SPEED / DO_JUMP / clear-ROI / land-start / delay carry no position and must
        # use the MISSION frame (2); PX4 rejects them with a global frame. Georeferenced items take
        # the chosen altitude mode: relative-to-home (6) or AMSL (5).
        item.frame = 2 if cmd in (20, 178, 177, 197, 189, 93, 183, 115, 206) else self.altmode.currentData()


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
        self._fwd_target = None             # (host, port) for MAVLink forwarding, persisted across links
        self.default_port = port

        # settings persistence (only the real app opts in; tests stay deterministic)
        self._persist = False
        self.settings = QSettings("DroneDeck", "DroneDeck")
        self.link_configs = []                          # saved comm-link configs

        # flight-time tracking (since arm)
        self._arm_t0 = None
        self._flight_time = 0.0
        self._failsafe_prev = {}                # sysid -> last MAV_STATE (failsafe edge detect)
        self._fence_breached = {}               # geofence breach edge detect, keyed by sysid
        self._batt_band = {}                    # sysid -> last battery band (low-batt edge detect)

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
        self.params = ParamManager(lambda: self.link, self._sysid,
                                   lambda: self.vehicle.autopilot)
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

        # manual control (virtual joystick + keyboard, or a real hardware gamepad)
        self.manual_on = False
        self._manual_keys = set()
        self.hw_joystick = None            # HwJoystick when a real device is selected, else virtual
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
        self.btn_estop = QPushButton("Emergency Stop")
        self.btn_estop.setStyleSheet(
            "QPushButton { background:#7a1414; color:#fff; font-weight:bold; }"
            "QPushButton:hover { background:#a01a1a; }"
            "QPushButton:disabled { background:#3a2222; color:#886; }")
        self.btn_estop.clicked.connect(self._emergency_stop)
        tb.addWidget(self.btn_arm)
        tb.addWidget(self.btn_disarm)
        tb.addWidget(self.btn_estop)
        self.btn_params = QPushButton("Params")
        self.btn_params.clicked.connect(self._open_params)
        tb.addWidget(self.btn_params)
        self.btn_record = QPushButton("Record")
        self.btn_record.setCheckable(True)
        self.btn_record.toggled.connect(self._toggle_record)
        tb.addWidget(self.btn_record)
        tb.addSeparator()

        self.btn_center = QPushButton("Center")
        self.btn_center.clicked.connect(self._center_on_vehicle)
        tb.addWidget(self.btn_center)
        self.btn_fit = QPushButton("Fit")
        self.btn_fit.clicked.connect(self._fit_map)
        tb.addWidget(self.btn_fit)
        self.btn_ruler = QPushButton("Ruler")
        self.btn_ruler.setCheckable(True)
        self.btn_ruler.toggled.connect(self._toggle_ruler)
        tb.addWidget(self.btn_ruler)
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
                 "Alt": "armed", "Speed": "armed", "Head": "armed"}
        for label, slot, tip in (
                ("Takeoff", self._takeoff, "Arm if needed and climb to a set altitude  (Ctrl+T)"),
                ("Land", self._land, "Land at the current position  (Ctrl+L)"),
                ("RTL", self._rtl, "Return to launch and land  (Ctrl+R)"),
                ("Pause", self._pause, "Hold / loiter in place  (Ctrl+Space)"),
                ("Alt", self._change_alt, "Fly to a new altitude at the current position"),
                ("Speed", self._change_speed, "Set the cruise / ground speed (m/s)"),
                ("Head", self._change_heading, "Point the nose to a compass heading (deg)")):
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
        # payload gripper (delivery drones) -- release / grab
        self.btn_payload = QPushButton("Payload")
        self.btn_payload.setToolTip("Release or grab the payload gripper (DO_GRIPPER)")
        self._payload_menu = QMenu(self.btn_payload)
        self._payload_menu.addAction("Release payload").triggered.connect(
            lambda: self._gripper(mavlink.GRIPPER_ACTION_RELEASE))
        self._payload_menu.addAction("Grab payload").triggered.connect(
            lambda: self._gripper(mavlink.GRIPPER_ACTION_GRAB))
        self._payload_menu.addSeparator()
        self._payload_menu.addAction("Winch (lower / raise)...").triggered.connect(self._winch)
        self._payload_menu.addAction("Winch relax").triggered.connect(
            lambda: self._winch_relax())
        self.btn_payload.setMenu(self._payload_menu)
        self.btn_payload._needs = "conn"
        tb2.addWidget(self.btn_payload)
        self._flight_btns.append(self.btn_payload)
        tb2.addSeparator()
        self.btn_joystick = QPushButton("Joystick")
        self.btn_joystick.setCheckable(True)
        self.btn_joystick.toggled.connect(self._toggle_manual)
        self.btn_joystick._needs = "conn"
        tb2.addWidget(self.btn_joystick)
        self._flight_btns.append(self.btn_joystick)
        # manual-control source: on-screen virtual pad, or a real hardware gamepad/stick
        self.joy_source = QComboBox()
        self.joy_source.setToolTip("Manual control input device")
        self.joy_source.currentIndexChanged.connect(self._select_joystick)
        tb2.addWidget(self.joy_source)
        self._refresh_joysticks()
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
        self.mission_progress = QProgressBar()
        self.mission_progress.setMaximumWidth(130)
        self.mission_progress.setMaximumHeight(14)
        self.mission_progress.setTextVisible(True)
        self.mission_progress.hide()          # shown only during a transfer
        tb3.addWidget(self.mission_progress)

        self._setup_usability()   # tooltips + keyboard shortcuts (all toolbar widgets exist now)

        # central layout: map | (instruments over telemetry)
        self.map = MapView()
        self.map.followChanged.connect(self._on_map_follow_changed)
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
        self._msg_unread = 0        # STATUSTEXT arrived while the Messages tab wasn't visible
        self._msg_worst = 99        # worst (lowest) unread severity; 99 = none
        self._takeoff_alt = 25.0    # remembered takeoff altitude (persisted across runs)
        self._ruler_a = None        # first point of the map measure tool, or None
        self._stream_reqs = {}      # sysid -> {tries, last_t}: bounded re-request of telemetry streams
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
        self.btn_msg_clear = QPushButton("Clear")
        self.btn_msg_clear.setToolTip("Clear the message log")
        self.btn_msg_clear.clicked.connect(self._clear_messages)
        frow.addWidget(self.btn_msg_clear)
        mcl.addLayout(frow)
        mcl.addWidget(self.console, 1)
        self.msg_dock = dock = QDockWidget("Messages", self)
        dock.setObjectName("messages_dock")
        dock.setWidget(msg_wrap)
        dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock)
        dock.visibilityChanged.connect(self._on_msg_visibility)   # clear unread when viewed

        # mission waypoint list + edit buttons, tabbed with Messages at the bottom
        self.mission_list = QListWidget()
        self.mission_list.setFont(QFont("DejaVu Sans Mono", 9))
        self.mission_list.itemSelectionChanged.connect(self._wp_list_selected)
        self.mission_list.itemDoubleClicked.connect(self._wp_edit)
        self.mission_list.setContextMenuPolicy(Qt.CustomContextMenu)
        self.mission_list.customContextMenuRequested.connect(self._wp_context_menu)
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
        self.camera.cameraMode.connect(self._cam_mode)
        self.camera.cameraZoom.connect(self._cam_zoom)

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

        # ADSB traffic list (the tabular companion to the map targets)
        self.traffic_panel = TrafficPanel()
        tdock = QDockWidget("Traffic", self)
        tdock.setObjectName("traffic_dock")
        tdock.setWidget(self.traffic_panel)
        tdock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, tdock)
        self.tabifyDockWidget(sdock, tdock)

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
        act_geotag = tools.addAction("GeoTag Images...")
        act_geotag.setToolTip("Write GPS EXIF into survey photos from a flight log (CAMERA_FEEDBACK)")
        act_geotag.triggered.connect(self._open_geotag)
        act_fence = tools.addAction("Geofence from Mission")
        act_fence.triggered.connect(self._fence_from_mission)
        act_fence_on = tools.addAction("Enable Geofence")
        act_fence_on.setToolTip("Turn on geofence enforcement (DO_FENCE_ENABLE; ArduPilot)")
        act_fence_on.triggered.connect(lambda: self._fence_enable(True))
        act_fence_off = tools.addAction("Disable Geofence")
        act_fence_off.triggered.connect(lambda: self._fence_enable(False))
        act_vinfo = tools.addAction("Vehicle Info...")
        act_vinfo.triggered.connect(self._show_vehicle_info)
        act_ftp = tools.addAction("Vehicle Files (FTP)...")
        act_ftp.setToolTip("Browse + download the vehicle's filesystem over MAVLink FTP (PX4 logs, params)")
        act_ftp.triggered.connect(self._open_ftp)
        act_fwd = tools.addAction("MAVLink Forwarding...")
        act_fwd.setToolTip("Re-broadcast received telemetry to a 2nd UDP endpoint (companion, 2nd GCS)")
        act_fwd.triggered.connect(self._open_forward)
        act_preflight = tools.addAction("Preflight Check...")
        act_preflight.setToolTip("Arming readiness: GPS / estimator / home / battery / sensor health")
        act_preflight.triggered.connect(self._open_preflight)
        tools.addAction("Export Track (GPX)...").triggered.connect(self._export_track)
        tools.addSeparator()
        act_chute = tools.addAction("Deploy Parachute (emergency)")
        act_chute.setToolTip("Emergency: deploy the parachute now (irreversible)")
        act_chute.triggered.connect(self._deploy_parachute)
        self._help_menu = helpm = self.menuBar().addMenu("&Help")
        helpm.addAction("Quick Help").triggered.connect(self._show_help)
        helpm.addAction("About DroneDeck...").triggered.connect(self._show_about)

    def _open_analyze(self):
        from analyze import AnalyzeDialog
        AnalyzeDialog(self, LOG_DIR).exec()

    def _open_ftp(self):
        if not self._has_vehicle():
            QMessageBox.information(self, "No vehicle",
                                   "Connect to a vehicle first to browse its files over MAVLink FTP.")
            return
        from ftpbrowser import FtpBrowserDialog
        os.makedirs(LOG_DIR, exist_ok=True)
        FtpBrowserDialog(lambda: self.link, self._sysid, LOG_DIR, self).exec()

    def _open_geotag(self):
        from geotagdialog import GeotagDialog
        GeotagDialog(LOG_DIR, self).exec()

    def _open_preflight(self):
        from preflight import PreflightDialog
        PreflightDialog(lambda: self.vehicle if self._has_vehicle() else None, self).exec()

    def _open_forward(self):
        from PySide6.QtWidgets import QInputDialog
        cur = f"{self._fwd_target[0]}:{self._fwd_target[1]}" if self._fwd_target else "127.0.0.1:14551"
        text, ok = QInputDialog.getText(self, "MAVLink Forwarding",
                                        "Forward received telemetry to (host:port), or blank to stop:",
                                        text=cur)
        if not ok:
            return
        text = text.strip()
        if not text:
            self._fwd_target = None
            if self.link is not None:
                self.link.clear_forward()
            self._on_info("MAVLink forwarding stopped")
            return
        host, _, port_s = text.partition(":")
        port = _parse_port(port_s, 0)
        if not port:
            self._on_info(f"invalid forward target: {text!r}")
            return
        self._fwd_target = (host or "127.0.0.1", port)
        if self.link is not None:
            self.link.set_forward(*self._fwd_target)
        self._on_info(f"forwarding telemetry to {self._fwd_target[0]}:{self._fwd_target[1]}")

    def _export_track(self):
        trail = list(self.vehicle.trail) if self.vehicle else []
        if len(trail) < 2:
            QMessageBox.information(self, "Export Track", "No flight track yet -- the track is "
                                    "recorded from the vehicle's position as it flies.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Export track",
                                              os.path.join(LOG_DIR, "track.gpx"),
                                              "GPX track (*.gpx)")
        if not path:
            return
        if not path.endswith(".gpx"):
            path += ".gpx"
        with open(path, "w") as f:
            f.write(trail_to_gpx(trail))
        self._on_info(f"exported {len(trail)}-point track to {os.path.basename(path)}")

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

    def _fence_enable(self, on):
        """Turn geofence enforcement on/off at runtime (DO_FENCE_ENABLE)."""
        if not self._has_vehicle():
            return
        self.link.fence_enable(self._sysid(), on)
        self._on_info(f"geofence {'ENABLED' if on else 'DISABLED'}")

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
        self._wire_vehicle(self.vehicle)
        self.map.clicked.connect(self._on_map_click)
        self.map.contextAction.connect(self._on_map_context)
        self.map.wpAction.connect(self._on_wp_action)
        self.map.waypoint_selected.connect(self._wp_selected)
        self.map.waypoint_moved.connect(self._wp_moved)
        self.mission.progress.connect(self._on_mission_progress)
        self.mission.progress_n.connect(self._on_mission_progress_n)
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
        link.command_unacked.connect(self._on_command_unacked)
        link.recorder = self._recorder        # keep recording across reconnects
        if self._fwd_target:                  # keep MAVLink forwarding across reconnects too
            link.set_forward(*self._fwd_target)
        return link

    def _on_command_unacked(self, command):
        # a safety-critical command (arm/mode/land/rtl) was resent ACK_MAX_TRIES times with no
        # COMMAND_ACK -- warn loudly; on a real RF link this means it may not have been received.
        self._notify(f"No acknowledgement for command {int(command)} after retries -- "
                     f"the link may be lossy; it may not have been received", "#e07030")

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
        # attribute the ACK to the vehicle that emitted it (sender() = the Vehicle, direct
        # connection) -- in a multi-vehicle session a rejection from a NON-selected vehicle used
        # to pop an unattributed toast with a reason scraped from the WRONG vehicle's log.
        src = self.sender()
        if not isinstance(src, Vehicle):
            src = self.vehicle
        tag = f"#{src.sysid} " if (src is not self.vehicle and src.sysid) else ""
        name = self.CMD_NAMES.get(command, f"CMD {command}")
        res = mavlink.MAV_RESULT.get(result, str(result))
        line = f"{tag}{name}: {res}"
        self._on_info(line)
        # MAV_RESULT_IN_PROGRESS is NOT a final result: the command was accepted and is still
        # executing (the vehicle may send it repeatedly, then a final ACCEPTED/FAILED -- e.g. a slow
        # arm sequence, takeoff, or calibration). Treating it as "not ACCEPTED == failure" used to
        # log it red and fire a false "REJECTED" toast for a command that was proceeding fine. Show
        # it as neutral progress and wait for the real result.
        if result == mavlink.MAV_RESULT_IN_PROGRESS:
            self.console.add_note(line, "#39c0d0")
            return
        ok = (result == mavlink.MAV_RESULT_ACCEPTED)
        self.console.add_note(line, "#37d67a" if ok else "#e05050")
        # A rejected safety-critical command (arm, takeoff, ...) is easy to miss in the
        # console -- pop a prominent toast with the autopilot's own reason (the most
        # recent warning/error STATUSTEXT, e.g. "Arming denied: GPS not ready").
        critical = {mavlink.MAV_CMD_COMPONENT_ARM_DISARM, mavlink.MAV_CMD_NAV_TAKEOFF,
                    mavlink.MAV_CMD_NAV_LAND, mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                    mavlink.MAV_CMD_DO_SET_MODE}
        if not ok and command in critical:
            reason = next((txt for sev, txt in reversed(src.messages[-12:])
                           if sev <= 4), "")
            self._notify(f"{tag}{name} REJECTED: {res}" + (f"  --  {reason}" if reason else ""),
                         "#c02020")

    def _on_mission_reached(self, seq):
        """MISSION_ITEM_REACHED -- note each waypoint the vehicle completes (QGC-style)."""
        self.console.add_note(f"reached waypoint {seq}", "#8fd0ff")
        self._on_info(f"reached waypoint {seq}")

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
        self._reset_vehicles()      # fresh link -> fresh vehicles (no stale sysid/home/params)
        self.link = self._make_link()
        t = self.transport_combo.currentText()
        p = self.link_edit.text().strip()
        try:
            if t == "TCP":
                host, _, port = p.partition(":")
                self.link.open(host=host or "127.0.0.1", port=_parse_port(port, 5760))
            elif t == "Serial":
                port, _, baud = p.partition(":")
                self.link.open(port=port, baud=int(baud or 57600))
            elif t == "Replay":
                path, _, sp = p.partition("@")
                self.link.open(path=path.strip(), speed=float(sp or 1.0))
            else:
                self.link.open(port=_parse_port(p, self.default_port))
        except (ValueError, OverflowError):
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
        dlg.sensor.accelPosRequested.connect(self._cal_accel_pos)
        dlg.sensor.compassAccept.connect(self._cal_mag_accept)
        dlg.sensor.compassCancel.connect(self._cal_mag_cancel)
        dlg.motors.motorTestRequested.connect(self._motor_test)
        dlg.exec()

    def _motor_test(self, motor, throttle_pct, duration_s, count):
        if not self._has_vehicle():
            QMessageBox.information(self, "No vehicle", "Connect to a vehicle first.")
            return
        self.link.motor_test(self._sysid(), motor, throttle_pct, duration_s, count)
        what = f"all {count} motors in sequence" if count else f"motor {motor}"
        self._on_info(f"motor test: {what} at {throttle_pct:.0f}% for {duration_s:.0f}s")

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
        if kind == "compass":
            self.link.start_mag_cal(self._sysid())     # ArduPilot onboard mag cal (DO_START_MAG_CAL)
        else:
            self.link.calibrate(self._sysid(), kind)
        self._on_info(f"requested {kind} calibration")

    def _cal_mag_accept(self):
        if self._has_vehicle():
            self.link.accept_mag_cal(self._sysid())
            self._on_info("compass cal accepted")

    def _cal_mag_cancel(self):
        if self._has_vehicle():
            self.link.cancel_mag_cal(self._sysid())
            self._on_info("compass cal cancelled")

    def _cal_accel_pos(self, position):
        """Advance the accel 6-position calibration (ACCELCAL_VEHICLE_POS)."""
        if self._has_vehicle():
            self.link.accel_cal_position(self._sysid(), position)
            self._on_info(f"accel cal position {position}")

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
            # A neighbouring GCS / lone peripheral is not a vehicle: don't create one for a sysid
            # whose only traffic is autopilot-INVALID heartbeats (QGC applies the same rule). A real
            # vehicle still gets created from its first telemetry even if its heartbeat is late.
            if is_new and all(m.msgid == mavlink.HEARTBEAT and
                              int(m.fields.get("autopilot", 0)) == mavlink.MAV_AUTOPILOT_INVALID
                              for m in msgs):
                continue
            veh = self._ensure_vehicle(sysid)
            veh.consume(msgs)
            if self.link is not None:
                self._maybe_request_streams(sysid, veh, is_new)
        self._sync_mode_combo()   # keep the mode selector matched to the autopilot

    STREAM_MAX_TRIES = 5          # cap re-requests so a silent vehicle can't be spammed forever
    STREAM_RETRY_S = 2.0          # wait between stream re-requests

    def _maybe_request_streams(self, sysid, veh, is_new):
        """Request telemetry streams on first detection, and -- because a real drone streams
        little until asked and a single request can be dropped on a lossy radio link -- re-request
        (bounded) until high-rate telemetry actually arrives. The retry also covers the case where
        the first packet from a vehicle wasn't its HEARTBEAT, so its autopilot type (hence the
        right SET_MESSAGE_INTERVAL dialect) wasn't known on the first request."""
        st = self._stream_reqs.get(sysid)
        if is_new or st is None:
            self.link.request_data_streams(sysid, veh.autopilot)
            self._stream_reqs[sysid] = {"tries": 1, "last_t": time.monotonic()}
            self._on_info(f"vehicle #{sysid} detected -- requesting telemetry streams")
            return
        if veh.have_attitude or veh.have_position:
            return                              # telemetry is flowing -- nothing more to do
        if st["tries"] >= self.STREAM_MAX_TRIES:
            return                              # gave up (vehicle silent) -- do not spam
        now = time.monotonic()
        if now - st["last_t"] >= self.STREAM_RETRY_S:
            self.link.request_data_streams(sysid, veh.autopilot)
            st["tries"] += 1
            st["last_t"] = now
            self._on_info(f"vehicle #{sysid}: no telemetry yet -- re-requesting streams "
                          f"(try {st['tries']}/{self.STREAM_MAX_TRIES})")

    def _wire_vehicle(self, veh):
        """Connect a Vehicle's signals to the window. Factored so the primary vehicle and any
        later per-sysid vehicle are wired identically (and so a reconnect can rebuild cleanly)."""
        veh.status_text.connect(self.console.add_message)
        veh.status_text.connect(self._on_new_message)
        veh.command_ack.connect(self._on_command_ack)
        veh.mission_reached.connect(self._on_mission_reached)

    def _reset_vehicles(self):
        """Drop all per-vehicle state on a fresh connect. Without this, switching from one drone to
        another (SITL sysid 1 -> a real drone sysid 2, or reconnecting at a new field) left the old
        dead vehicle active -- commands went to the wrong sysid, the HUD showed NO TELEMETRY while
        data flowed, and a stale home corrupted the RTL reference. A brand-new primary Vehicle also
        clears attitude/battery/home/trail so nothing bleeds across the reconnect."""
        self.vehicle = Vehicle()
        self._wire_vehicle(self.vehicle)
        self.vehicles = {}
        self.traffic = {}
        self._fence_breached = {}
        self._failsafe_prev = {}
        self._batt_band = {}
        self._stream_reqs = {}
        self.params.reset()
        self.vehicle_combo.blockSignals(True)
        self.vehicle_combo.clear()
        self.vehicle_combo.blockSignals(False)

    def _ensure_vehicle(self, sysid):
        veh = self.vehicles.get(sysid)
        if veh is not None:
            return veh
        if not self.vehicles:
            veh = self.vehicle                  # reuse the pre-wired primary vehicle
        else:
            veh = Vehicle()
            self._wire_vehicle(veh)
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
        if sid in self.vehicles and self.vehicles[sid] is not self.vehicle:
            self.vehicle = self.vehicles[sid]
            self._arm_t0 = None
            self.params.reset()     # the param table/manager is global -> clear #A's params so a
            #                         re-open re-downloads #B's (never write A's values to B)
            self._on_info(f"active vehicle: #{sid} -- re-open Params to load its parameters")

    TRAFFIC_TTL = 60.0        # seconds an ADSB target lingers after its last report
    TRAFFIC_MAX = 2000        # hard ceiling on tracked targets (far above any real airspace)

    def _update_traffic(self, msgs):
        now = time.monotonic()
        for m in msgs:
            f = m.fields
            icao = int(f.get("ICAO_address", 0))
            self.traffic[icao] = {
                "lat": f.get("lat", 0) / 1e7, "lon": f.get("lon", 0) / 1e7,
                "heading": f.get("heading", 0) / 100.0,
                "alt": f.get("altitude", 0) / 1000.0,      # mm ASL -> m
                "callsign": (f.get("callsign", "") or "").strip(), "t": now}
        # Expire stale targets + hard-cap. ADSB aircraft fly out of range, and a busy sky (or a
        # noisy/buggy/hostile source) can present unbounded distinct ICAOs; without this both the
        # dict and the per-refresh sorted table rebuild grow without bound over a long flight.
        stale = [k for k, v in self.traffic.items() if now - v["t"] > self.TRAFFIC_TTL]
        for k in stale:
            del self.traffic[k]
        if len(self.traffic) > self.TRAFFIC_MAX:
            excess = len(self.traffic) - self.TRAFFIC_MAX
            for k, _v in sorted(self.traffic.items(), key=lambda kv: kv[1]["t"])[:excess]:
                del self.traffic[k]

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
        self.btn_estop.setEnabled(connected)     # always reachable while connected
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

    def _confirm(self, title, text):
        """Yes/No guard for destructive flight actions; defaults to No (safe)."""
        return QMessageBox.question(
            self, title, text,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        ) == QMessageBox.StandardButton.Yes

    def _emergency_stop(self):
        if not self._has_vehicle():
            QMessageBox.information(self, "Emergency Stop", "No vehicle connected.")
            return
        if not self._confirm("EMERGENCY STOP",
                             "Force-disarm NOW?\n\nThis cuts ALL motors immediately -- if the "
                             "aircraft is airborne it WILL fall.\n\nUse only to prevent worse harm."):
            return
        self.link.force_disarm(self._sysid())
        self._on_info(f"EMERGENCY STOP -- force-disarm sent to system {self._sysid()}")

    def _arm(self, arm):
        if not self._has_vehicle():
            QMessageBox.information(self, "No vehicle", "No telemetry source connected yet.")
            return
        if arm:
            if not self._confirm("Arm", "Arm the vehicle?\nPropellers may spin up immediately."):
                return
        elif self.vehicle.armed:
            if not self._confirm("Disarm",
                                 "The vehicle is ARMED.\n\nDisarming now cuts the motors -- if it "
                                 "is airborne this WILL crash it.\n\nDisarm anyway?"):
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

    def _cam_mode(self, mode):
        if self._has_vehicle():
            self.link.set_camera_mode(self._sysid(), mode)
            self._on_info(f"camera: {'video' if mode else 'photo'} mode sent")

    def _cam_zoom(self, step):
        if self._has_vehicle():
            self.link.camera_zoom(self._sysid(), step)
            self._on_info(f"camera: zoom {'in' if step > 0 else 'out'} sent")

    def _cam_trigdist(self, metres):
        if self._has_vehicle():
            self.link.set_trigger_distance(self._sysid(), metres)
            self._on_info(f"camera: trigger distance {metres:.0f} m")

    def _cam_gimbal(self, pitch, yaw):
        if self._has_vehicle():
            if self.camera.gimbal_proto.currentIndex() == 1:     # Manager v2
                self.link.set_gimbal_v2(self._sysid(), pitch, yaw)
            else:                                                # Mount v1 (default)
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
        if self.hw_joystick is not None:
            self.hw_joystick.poll()               # drain real-device events into its axis/button state
            if not self.hw_joystick.is_open:      # unplugged mid-flight -> fall back to the pad
                self._fall_back_to_virtual()
                x, y, z, r = self.joystick.values()
            else:
                for b in self.hw_joystick.take_presses():   # gamepad buttons -> mapped actions
                    self._joy_button(b)
                x, y, z, r = self.hw_joystick.values()
        else:
            x, y, z, r = self.joystick.values()
        self.link.send_manual_control(self._sysid(), x, y, z, r)

    # default gamepad button -> action (Xbox layout: A/B/X/Y). Fired immediately on press like a
    # transmitter switch (no modal confirm mid-flight); the link calls are ACK-confirmed + guarded.
    JOY_BUTTONS = {0: "arm", 1: "disarm", 2: "rtl", 3: "land"}

    def _joy_button(self, idx):
        act = self.JOY_BUTTONS.get(idx)
        if not act or not self._has_vehicle():
            return
        sysid = self._sysid()
        if act == "arm":
            self.link.arm(sysid, True)
        elif act == "disarm":
            self.link.arm(sysid, False)
        elif act == "rtl":
            self.link.rtl(sysid)
        elif act == "land":
            self.link.land(sysid)
        self._on_info(f"joystick button {idx}: {act.upper()}")

    def _refresh_joysticks(self):
        """Populate the input-source dropdown: the on-screen pad plus any real /dev/input/jsN."""
        self.joy_source.blockSignals(True)
        self.joy_source.clear()
        self.joy_source.addItem("Virtual pad", None)
        for path in list_joysticks():
            self.joy_source.addItem(f"HW {path.rsplit('/', 1)[-1]}", path)
        self.joy_source.blockSignals(False)

    def _select_joystick(self, _idx):
        """Switch the manual-control source to the chosen device (or the virtual pad)."""
        path = self.joy_source.currentData()
        if self.hw_joystick is not None:
            self.hw_joystick.close()
            self.hw_joystick = None
        if path:
            js = HwJoystick(path)
            if js.is_open:
                self.hw_joystick = js
                self._on_info(f"manual input: hardware {path}")
            else:
                self._notify(f"Could not open {path} -- using the virtual pad", "#e0a030")
                self.joy_source.setCurrentIndex(0)
        else:
            self._on_info("manual input: virtual pad")

    def _fall_back_to_virtual(self):
        if self.hw_joystick is not None:
            self.hw_joystick.close()
            self.hw_joystick = None
        self.joy_source.setCurrentIndex(0)
        self._notify("Joystick disconnected -- reverted to the virtual pad", "#e0a030")

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
        alt, ok = QInputDialog.getDouble(self, "Takeoff", "Altitude (m):",
                                         self._takeoff_alt, 1.0, 1000.0, 1)
        if ok:
            self._takeoff_alt = alt          # remember it for next time (and persist on close)
            self.link.takeoff(self._sysid(), alt, self.vehicle.lat, self.vehicle.lon)
            self._on_info(f"takeoff to {alt:.0f} m")

    def _land(self):
        if self._has_vehicle():
            if not self._confirm("Land", "Command the vehicle to LAND at its current position?"):
                return
            self.link.land(self._sysid())
            self._on_info("land")

    def _rtl(self):
        if self._has_vehicle():
            if not self._confirm("Return to Launch",
                                 "Return to launch?\nThe vehicle will fly to home and land."):
                return
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

    def _change_heading(self):
        if not self._has_vehicle():
            return
        cur = round(self.vehicle.heading) % 360
        hdg, ok = QInputDialog.getInt(self, "Change Heading",
                                      "Point the nose to (deg, 0=N 90=E):", int(cur), 0, 359, 1)
        if ok:
            # hold current position + altitude, only rotate to the new heading (DO_REPOSITION yaw)
            self.link.change_heading(self._sysid(), self.vehicle.lat, self.vehicle.lon,
                                     self.vehicle.alt_rel, float(hdg))
            self._on_info(f"change heading to {hdg}deg")

    def _vtol_transition(self, state):
        if not self._has_vehicle():
            return
        name = "fixed-wing" if state == mavlink.MAV_VTOL_STATE_FW else "multirotor"
        self.link.vtol_transition(self._sysid(), state)
        self._on_info(f"VTOL transition to {name} requested")

    def _gripper(self, action):
        if not self._has_vehicle():
            return
        self.link.gripper(self._sysid(), action)
        rel = action == mavlink.GRIPPER_ACTION_RELEASE
        self._on_info(f"payload: {'release' if rel else 'grab'} sent")

    def _deploy_parachute(self):
        if not self._has_vehicle():
            QMessageBox.information(self, "Parachute", "No vehicle connected.")
            return
        box = QMessageBox(QMessageBox.Warning, "Deploy parachute",
                          "Deploy the parachute NOW?\n\nThis is an emergency action -- it ends the "
                          "flight and cannot be undone.",
                          QMessageBox.Yes | QMessageBox.Cancel, self)
        box.setDefaultButton(QMessageBox.Cancel)      # never the destructive option by default
        if box.exec() == QMessageBox.Yes:
            self.link.deploy_parachute(self._sysid())
            self._on_info("PARACHUTE DEPLOY sent")

    def _winch(self):
        if not self._has_vehicle():
            return
        length, ok = QInputDialog.getDouble(self, "Winch",
                                            "Cable length -- lower + / raise - (m):",
                                            0.0, -100.0, 100.0, 1)
        if ok and length != 0.0:
            self.link.winch(self._sysid(), mavlink.WINCH_LENGTH_CONTROL, length=length, rate=1.0)
            self._on_info(f"winch {length:+.1f} m")

    def _winch_relax(self):
        if not self._has_vehicle():
            return
        self.link.winch(self._sysid(), mavlink.WINCH_RELAXED)
        self._on_info("winch relaxed (free spool)")

    def _on_map_click(self, lat, lon):
        if self.btn_ruler.isChecked():          # measure tool takes priority over goto/plan
            self._ruler_click(lat, lon)
            return
        if self.plan_mode:
            self._add_waypoint(lat, lon)
            return
        if not self._has_vehicle():
            return
        alt = max(self.vehicle.alt_rel, 30.0)
        if QMessageBox.question(self, "Goto",
                                f"Fly to:\n{lat:.6f}, {lon:.6f}\nat {alt:.0f} m relative altitude?"
                                ) == QMessageBox.StandardButton.Yes:
            self._guided_goto(lat, lon, alt)
            self._on_info(f"goto {lat:.5f}, {lon:.5f} @ {alt:.0f} m")

    def _guided_goto(self, lat, lon, alt):
        """Autopilot-correct guided 'fly to': PX4 ignores a one-shot SET_POSITION_TARGET outside an
        OFFBOARD stream, so it gets DO_REPOSITION(+CHANGE_MODE) -- the primitive QGC uses and the
        one change-altitude already live-verified. ArduPilot keeps the guided position target."""
        if self.vehicle.autopilot == mavlink.MAV_AUTOPILOT_PX4:
            self.link.reposition(self._sysid(), lat, lon, alt)
        else:
            self.link.goto(self._sysid(), lat, lon, alt)

    def _on_map_context(self, action, lat, lon):
        if action == "add_roi":                        # a mission item, not a live command
            alt = self.mission_items[-1].alt if self.mission_items else 50.0
            self.mission_items.append(MissionItem(len(self.mission_items), lat, lon, alt,
                                                  command=195))
            self._refresh_mission_view()
            self._on_info(f"added ROI waypoint at {lat:.5f}, {lon:.5f}")
            return
        if action == "clear_trail":                # view action -- no vehicle command needed
            self.vehicle.clear_trail()
            self.map.update()
            self._on_info("flight trail cleared")
            return
        if action == "copy_coords":                # view action -- copy the point to the clipboard
            text = f"{lat:.6f}, {lon:.6f}"
            QApplication.clipboard().setText(text)
            self._on_info(f"copied {text} to clipboard")
            return
        if not self._has_vehicle():
            QMessageBox.information(self, "No vehicle", "Connect to a vehicle first.")
            return
        alt = max(self.vehicle.alt_rel, 30.0)
        sysid = self._sysid()
        if action == "goto":
            self._guided_goto(lat, lon, alt)
            self._on_info(f"goto {lat:.5f}, {lon:.5f} @ {alt:.0f} m")
        elif action == "orbit":
            self.link.orbit(sysid, lat, lon, 50.0, alt)
            self._on_info(f"orbit {lat:.5f}, {lon:.5f} r=50 m @ {alt:.0f} m")
        elif action == "roi":
            self.link.set_roi(sysid, lat, lon, alt)
            self._on_info(f"ROI {lat:.5f}, {lon:.5f}")
        elif action == "sethome":
            # home altitude = ground elevation AMSL, not the vehicle's relative alt: use the
            # authoritative HOME_POSITION elevation when known, else derive ground level from
            # the vehicle's own AMSL minus its height above home.
            ve = self.vehicle
            alt_amsl = ve.home_alt if ve.have_home_position else (ve.alt_msl - ve.alt_rel)
            self.link.set_home(sysid, lat, lon, alt_amsl)
            self._on_info(f"set home {lat:.5f}, {lon:.5f} @ {alt_amsl:.0f} m AMSL")

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
        amode = (" MSL" if it.frame == mavlink.MAV_FRAME_GLOBAL_INT else
                 " AGL" if it.frame == mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT else "")
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
        dlg = WaypointEditor(self.mission_items[i], self, autopilot=self.vehicle.autopilot)
        if dlg.exec() == QDialog.Accepted:
            dlg.apply_to(self.mission_items[i])
            self._update_wp_row(i)

    def _wp_context_menu(self, pos):
        """Right-click a mission-list row: edit it, or make it the vehicle's active waypoint."""
        row = self.mission_list.currentRow()
        has_row = 0 <= row < len(self.mission_items)
        menu = QMenu(self.mission_list)
        menu.addAction("Edit...").triggered.connect(self._wp_edit)
        act = menu.addAction(f"Set as current waypoint  (#{row})" if has_row
                             else "Set as current waypoint")
        act.setEnabled(has_row)
        act.triggered.connect(lambda: self._set_current_wp(row))
        menu.exec(self.mission_list.mapToGlobal(pos))

    def _set_current_wp(self, seq):
        """MISSION_SET_CURRENT: jump the flying vehicle's active mission item to `seq` (skip ahead
        to it, or restart the mission from it). The vehicle echoes MISSION_CURRENT, refreshing the
        map's green target ring + the list highlight."""
        if not self._has_vehicle():
            QMessageBox.information(self, "No vehicle", "Connect to a vehicle first.")
            return
        if not (0 <= seq < len(self.mission_items)):
            return
        self.link.set_current_wp(self._sysid(), seq)
        self._on_info(f"set current waypoint -> #{seq}")

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
            warns += autopilot_mission_warnings(self.mission_items, self.vehicle.autopilot)
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

    def _on_mission_progress_n(self, done, total):
        if total > 0:
            self.mission_progress.setRange(0, total)
            self.mission_progress.setValue(done)
            self.mission_progress.setFormat(f"%v/{total}")
            self.mission_progress.show()

    def _on_mission_finished(self, ok, msg):
        self.mission_status.setText(msg)
        self.console.add_note(f"mission: {msg}", "#4caf50" if ok else "#ff6b6b")
        self.mission_progress.hide()          # transfer over -- status label carries the result

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

    def _update_status_strip(self, ve, is_open, drop, loss=None):
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
            # flight-phase chip -- only when meaningful (armed, or airborne/transitioning),
            # so a disarmed-on-ground vehicle isn't cluttered by a redundant "ON GROUND"
            if ve.have_ext_state and (armed or ve.landed_state in (2, 3, 4)):
                fs = {1: ("ON GROUND", GREY, LIGHT), 2: ("FLYING", GREEN, GREEN),
                      3: ("TAKING OFF", AMBER, AMBER),
                      4: ("LANDING", AMBER, AMBER)}.get(ve.landed_state)
                if fs:
                    chips.append(fs)
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
        lp = loss or 0.0
        link_txt = f"{self._rate:.0f} Hz"
        if drop:
            link_txt += f" · {drop} drop"
        if lp >= 2.0:                                    # surface a degrading radio link
            link_txt += f" · {lp:.0f}% loss"
        bad = drop or lp >= 10.0
        link_col = RED if bad else (AMBER if lp >= 2.0 else (TEAL if is_open else GREY))
        chips.append((link_txt, link_col, RED if bad else (AMBER if lp >= 2.0 else LIGHT)))
        if self._msg_unread:
            mcol = RED if self._msg_worst <= 3 else AMBER if self._msg_worst == 4 else LIGHT
            chips.append((f"MSG {len(ve.messages)} (+{self._msg_unread})", mcol, mcol))
        else:
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

    def _center_on_vehicle(self):
        """One-shot recenter on the vehicle (QGC 'center on vehicle'); no follow change."""
        if not self.map.center_on_vehicle():
            self.statusBar().showMessage("No position fix yet", 2000)

    def _fit_map(self):
        """Zoom + centre the map to frame the mission, vehicle and home together."""
        pts = [(it.lat, it.lon) for it in self.mission_items]
        ve = self.vehicle
        if ve.have_position:
            pts.append((ve.lat, ve.lon))
        if ve.home:
            pts.append((ve.home[0], ve.home[1]))
        if self.map.fit_bounds(pts):
            self.map.set_follow(False)      # a fixed frame -- detach follow so it isn't overridden
        else:
            self.statusBar().showMessage("Nothing to fit (no mission, vehicle or home)", 2500)

    def _toggle_ruler(self, on):
        """Enable/disable the map measure tool; clears any partial measurement when turned off."""
        if on:
            self.statusBar().showMessage("Ruler: click two points to measure distance & bearing", 0)
        else:
            self._ruler_a = None
            self.map.set_ruler(None)
            self.statusBar().clearMessage()

    def _ruler_click(self, lat, lon):
        """Handle a map click while the ruler is active: first sets A, second measures A->B."""
        if self._ruler_a is None:
            self._ruler_a = (lat, lon)
            self.map.set_ruler(self._ruler_a)
            self.statusBar().showMessage("Ruler: click the second point", 0)
        else:
            a, b = self._ruler_a, (lat, lon)
            d = haversine(a[0], a[1], b[0], b[1])
            brg = bearing(a[0], a[1], b[0], b[1])
            dtxt = f"{d:.0f} m" if d < 1000 else f"{d / 1000:.2f} km"
            label = f"{dtxt}  {brg:.0f}°"
            self.map.set_ruler(a, b, label)
            self.statusBar().showMessage(f"Ruler: {label}  (click to start a new measurement)", 0)
            self._ruler_a = None            # next click starts a fresh A->B

    def _on_new_message(self, sev, _text):
        """Count a STATUSTEXT as unread unless the Messages tab is the one on screen."""
        if not self.msg_dock.isVisible():
            self._msg_unread += 1
            self._msg_worst = min(self._msg_worst, int(sev))
            self._update_msg_badge()

    def _on_msg_visibility(self, visible):
        """The Messages dock became the active/visible tab -> everything is now read."""
        if visible and self._msg_unread:
            self._msg_unread = 0
            self._msg_worst = 99
            self._update_msg_badge()

    def _update_msg_badge(self):
        """Reflect the unread count on the dock's tab title (blank when all read)."""
        self.msg_dock.setWindowTitle("Messages" if not self._msg_unread
                                     else f"Messages ({self._msg_unread})")

    def _clear_messages(self):
        """Empty the message log and reset the unread badge."""
        self.console.clear()
        self._msg_unread = 0
        self._msg_worst = 99
        self._update_msg_badge()

    def _on_map_follow_changed(self, on):
        """Keep the Follow checkbox in sync when the map auto-detaches on pan / re-attaches on
        double-click. Block signals so this doesn't re-enter _set_follow."""
        if self.chk_follow.isChecked() != on:
            self.chk_follow.blockSignals(True)
            self.chk_follow.setChecked(on)
            self.chk_follow.blockSignals(False)

    # -- refresh --------------------------------------------------------------
    def _refresh_traffic(self, ve):
        """Rebuild the ADSB traffic table from self.traffic, with distance + bearing from
        the active vehicle when its position is known."""
        rows = []
        for icao, t in sorted(self.traffic.items(),
                              key=lambda kv: kv[1].get("callsign") or f"{kv[0]:06X}"):
            alt = t.get("alt")
            alt_s = f"{alt:.0f} m" if alt is not None else "--"
            dist_s = brg_s = "--"
            if ve.have_position and (t.get("lat") or t.get("lon")):
                dist_s = _fmt_dist(haversine(ve.lat, ve.lon, t["lat"], t["lon"])).strip()
                brg_s = f"{bearing(ve.lat, ve.lon, t['lat'], t['lon']):.0f}"
            rows.append((t.get("callsign") or "--", f"{icao:06X}", alt_s, dist_s, brg_s))
        self.traffic_panel.set_rows(rows)

    def _check_geofence(self, ve):
        """One-shot warning when the vehicle crosses OUT of an inclusion fence or INTO an
        exclusion fence (polygon or circle). Edge-detected: fires once on breach and re-arms
        when the vehicle returns to safe airspace."""
        if not ve.have_position:
            return
        breaches = []
        if self.fence_inc and not _point_in_poly((ve.lat, ve.lon), self.fence_inc):
            breaches.append("left inclusion area")
        if self.fence_exc and _point_in_poly((ve.lat, ve.lon), self.fence_exc):
            breaches.append("entered exclusion area")
        for c in self.fence_circles:
            d = haversine(ve.lat, ve.lon, c["lat"], c["lon"])
            if c.get("incl", True) and d > c["radius"]:
                breaches.append(f"left inclusion circle ({d:.0f}>{c['radius']:.0f} m)")
            elif not c.get("incl", True) and d < c["radius"]:
                breaches.append(f"entered exclusion circle ({d:.0f}<{c['radius']:.0f} m)")
        now = bool(breaches)
        # edge flag keyed per sysid: with several vehicles checked each refresh, one shared flag
        # would flip between them and either re-fire every tick or swallow a second breach.
        was = self._fence_breached.get(ve.sysid, False)
        if now and not was:
            tag = f"vehicle #{ve.sysid}: " if len(self.vehicles) > 1 else ""
            self.console.add_note("GEOFENCE BREACH: " + tag + "; ".join(breaches), "#e05050")
            self._notify(f"GEOFENCE BREACH{' #' + str(ve.sysid) if tag else ''}", "#e05050")
        self._fence_breached[ve.sysid] = now

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

    # GCS-side low-battery advisory thresholds (percent remaining). These sit ABOVE a typical
    # autopilot failsafe so the pilot is prompted to return/land first; HYST stops a value jittering
    # around a threshold from re-firing the warning.
    BATT_LOW_PCT = 30
    BATT_CRIT_PCT = 15
    BATT_HYST_PCT = 3

    def _check_battery(self, ve):
        """Edge-triggered GCS-side low-battery annunciation (QGroundControl-style): warn ONCE when a
        vehicle's remaining battery drops past the LOW then CRITICAL threshold, so the operator is
        prompted to return/land BEFORE the vehicle's own failsafe triggers. Re-arms (with hysteresis)
        after a recharge/battery swap. Advisory + independent of _check_failsafe; uses the fuel-gauge
        percent (battery_remaining), skipping vehicles with no estimate (-1)."""
        sid = ve.sysid
        rem = ve.battery_remaining
        if not sid or rem < 0:                     # -1 = no battery sensor / not reported
            return
        prev = self._batt_band.get(sid, 0)         # 0 OK, 1 LOW, 2 CRITICAL
        if rem < self.BATT_CRIT_PCT:
            band = 2
        elif rem < self.BATT_LOW_PCT:
            band = 1
        else:
            band = 0
        # hysteresis: only step DOWN to a healthier band once clearly above that boundary, so a
        # reading jittering around a threshold doesn't re-arm + re-fire the warning repeatedly.
        if band < prev:
            if prev >= 2 and rem < self.BATT_CRIT_PCT + self.BATT_HYST_PCT:
                band = prev
            elif prev >= 1 and rem < self.BATT_LOW_PCT + self.BATT_HYST_PCT:
                band = prev
        self._batt_band[sid] = band
        if band <= prev:                           # unchanged or (hysteresis-gated) recovery
            return
        tag = f"vehicle {sid} " if len(self.vehicles) > 1 else ""
        if band == 2:
            self.console.add_note(f"CRITICAL BATTERY: {tag}{rem}% -- land now", "#e05050")
            self._notify(f"CRITICAL BATTERY {rem}%", "#e05050")
        else:
            self.console.add_note(f"LOW BATTERY: {tag}{rem}% -- return soon", "#e0a030")
            self._notify(f"LOW BATTERY {rem}%", "#e0a030")

    def _refresh(self):
        ve = self.vehicle
        # failsafe + geofence watch EVERY tracked vehicle (state is keyed per-sysid) -- a
        # non-selected vehicle entering CRITICAL or breaching the fence must still alert.
        for v in (list(self.vehicles.values()) or [ve]):
            self._check_failsafe(v)
            self._check_geofence(v)
            self._check_battery(v)
        self.adi.set_data(ve.roll, ve.pitch, ve.airspeed or ve.groundspeed,
                          ve.alt_rel, ve.heading, ve.climb)
        self.compass.set_heading(ve.heading)
        self.compass.set_wind(ve.wind_speed(), ve.wind_dir(), ve.have_wind)
        # home bug: bearing to launch, but hidden within ~10 m of home where the bearing is just
        # GPS-noise jitter (a spinning H is worse than none) -- matches QGC hiding it near home
        if ve.have_position and ve.home and \
                haversine(ve.lat, ve.lon, ve.home[0], ve.home[1]) > 10.0:
            self.compass.set_home_bearing(bearing(ve.lat, ve.lon, ve.home[0], ve.home[1]))
        else:
            self.compass.set_home_bearing(0.0, have=False)
        self.health.set_health(ve.sensors_present, ve.sensors_enabled, ve.sensors_health)
        self.systems.update_from(ve)
        if ve.have_position:
            self.map.update_vehicle(ve.lat, ve.lon, ve.heading, ve.home, ve.trail)
        self.map.set_current_wp(ve.current_wp)
        self._highlight_current_wp(ve.current_wp)

        # other vehicles + ADSB traffic on the map (traffic expires after 10 s)
        now_t = time.monotonic()
        # expire by the ONE documented TTL -- the old hardcoded 10 s here overrode TRAFFIC_TTL=60,
        # blinking real ADSB targets (sporadic reception at range) off the map 6x too fast.
        self.traffic = {k: v for k, v in self.traffic.items() if now_t - v["t"] < self.TRAFFIC_TTL}
        self.map.set_traffic([{"lat": v["lat"], "lon": v["lon"], "heading": v["heading"],
                               "callsign": v["callsign"]} for v in self.traffic.values()])
        self._refresh_traffic(ve)
        self.map.set_others([(v.lat, v.lon, v.heading) for s, v in self.vehicles.items()
                             if v is not ve and v.have_position])

        now = time.monotonic()
        if now - self._last_t >= 1.0:
            self._rate = (ve.msg_count - self._last_count) / (now - self._last_t)
            self._last_count = ve.msg_count
            self._last_t = now

        link = self.link
        ok, drop = (link.parser.stats if (link and link.parser) else (0, 0))
        loss = (link.parser.loss if (link and link.parser) else None)
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
        nav = {"flight_time": _fmt_mmss(self._flight_time),
               "odometer": _fmt_dist(ve.distance_traveled).strip()}
        if ve.have_position and ve.home:
            d = haversine(ve.lat, ve.lon, ve.home[0], ve.home[1])
            nav["home_dist"] = _fmt_dist(d)
            nav["home_eta"] = _fmt_mmss(d / ve.groundspeed) if ve.groundspeed > 0.4 else "--"
        if ve.home_alt is not None:
            nav["home_alt"] = f"{ve.home_alt:.0f} m"
        if ve.have_time_estimate:                    # autopilot's own estimates (>=0 valid, -1 n/a)
            if ve.eta_safe_return >= 0:
                nav["rtl_time"] = _fmt_mmss(ve.eta_safe_return)
            if ve.eta_mission_end >= 0:
                nav["mission_eta"] = _fmt_mmss(ve.eta_mission_end)
        if ve.have_position and self.mission_items:
            n = len(self.mission_items)
            cw = ve.current_wp
            act = self.mission_items[cw] if 0 <= cw < n else None
            if act is not None and not (abs(act.lat) < 1e-6 and abs(act.lon) < 1e-6):
                # actively flying a mission: distance + ETA to the current waypoint
                d_act = haversine(ve.lat, ve.lon, act.lat, act.lon)
                nav["wp_num"] = f"{cw + 1} / {n}"
                nav["wp_dist"] = _fmt_dist(d_act)
                nav["wp_eta"] = _fmt_mmss(d_act / ve.groundspeed) if ve.groundspeed > 0.4 else "--"
            else:
                # not on an active georeferenced waypoint: distance to the nearest, for planning
                wps = [it for it in self.mission_items
                       if not (abs(it.lat) < 1e-6 and abs(it.lon) < 1e-6)]
                if wps:
                    nav["wp_dist"] = _fmt_dist(min(haversine(ve.lat, ve.lon, it.lat, it.lon)
                                                   for it in wps))
                if n and 0 <= cw < n:
                    nav["wp_num"] = f"{cw + 1} / {n}"
        self.panel.update_all(ve, state, self._rate, ok, drop, nav, loss=loss)
        self.camera.update_status(ve)
        self._update_status_strip(ve, is_open, drop, loss)
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
        self.btn_estop.setToolTip("Emergency Stop: force-disarm / cut all motors immediately")
        self.btn_params.setToolTip("Download, search and edit parameters  (Ctrl+P)")
        self.btn_record.setToolTip("Record telemetry to a .tlog file for replay")
        self.btn_help.setToolTip("Keyboard shortcuts & quick help  (F1)")
        self.chk_follow.setToolTip("Keep the map centred on the vehicle  (F)")
        self.btn_center.setToolTip("Recentre the map on the vehicle once  (C)")
        self.btn_fit.setToolTip("Zoom to frame the mission, vehicle and home")
        self.btn_ruler.setToolTip("Measure tool: click two points for distance & bearing")
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
        sc("C", self._center_on_vehicle)
        sc("J", self.btn_joystick.toggle)
        sc("+", lambda: self.map.set_zoom(self.map.zoom + 1))
        sc("=", lambda: self.map.set_zoom(self.map.zoom + 1))
        sc("-", lambda: self.map.set_zoom(self.map.zoom - 1))
        sc("F1", self._show_help)

    def _show_vehicle_info(self):
        ve = self.vehicle
        if not self._has_vehicle():
            QMessageBox.information(self, "Vehicle Info", "No vehicle connected.")
            return
        if not ve.have_autopilot_version:
            if self.link:
                self.link.send_command_long(self._sysid(), mavlink.MAV_CMD_REQUEST_MESSAGE,
                                            [float(mavlink.AUTOPILOT_VERSION), 0, 0, 0, 0, 0, 0])
            QMessageBox.information(self, "Vehicle Info",
                                    "Firmware info not received yet -- requested it now; "
                                    "reopen this in a moment.")
            return
        ap = {3: "ArduPilot", 12: "PX4"}.get(ve.autopilot, f"autopilot #{ve.autopilot}")
        comps = ve.active_components()
        comps_html = ", ".join(f"{name} (#{cid})" for cid, name in comps) or "(only the autopilot seen)"
        caps = mavlink.capability_names(ve.capabilities)
        caps_html = "".join(f"<li>{c}</li>" for c in caps) or "<li>(none reported)</li>"
        html = (f"<h3>Vehicle {self._sysid()} &mdash; {ap}</h3>"
                "<table cellspacing=6>"
                f"<tr><td><b>Firmware</b></td><td>{ve.fw_version or '--'}</td></tr>"
                f"<tr><td><b>Git hash</b></td><td>{ve.fw_git or '--'}</td></tr>"
                f"<tr><td><b>Board version</b></td><td>{ve.board_version}</td></tr>"
                f"<tr><td><b>Vendor / Product</b></td>"
                f"<td>0x{ve.vendor_id:04x} / 0x{ve.product_id:04x}</td></tr>"
                f"<tr><td><b>MAVLink</b></td>"
                f"<td>{(self.link.mavlink_version_str if self.link else None) or 'unknown'}</td></tr>"
                f"<tr><td><b>Components</b></td><td>{comps_html}</td></tr>"
                "</table>"
                f"<p><b>Capabilities</b> (0x{ve.capabilities:x}):</p><ul>{caps_html}</ul>")
        QMessageBox.information(self, "Vehicle Info", html)

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
            "<tr><td><b>C</b></td><td>centre on vehicle</td><td></td><td></td></tr>"
            "<tr><td><b>+ / &minus;</b></td><td>zoom map</td>"
            "<td><b>Ctrl+P</b></td><td>parameters</td></tr>"
            "<tr><td><b>F1</b></td><td>this help</td><td></td><td></td></tr>"
            "</table>")
        QMessageBox.information(self, "DroneDeck Help", html)

    def _about_html(self):
        ve = self.vehicle
        veh_html = ""
        if ve is not None and ve.have_autopilot_version:
            ap = {3: "ArduPilot", 12: "PX4"}.get(ve.autopilot, f"autopilot #{ve.autopilot}")
            veh_html = (f"<tr><td><b>Vehicle</b></td>"
                        f"<td>{ap} &mdash; {ve.fw_version or 'firmware --'}</td></tr>")
        mav = (self.link.mavlink_version_str if self.link else None) or "not connected"
        return (
            "<h2>DroneDeck</h2>"
            f"<p>MAVLink ground control station &mdash; version {APP_VERSION}</p>"
            "<table cellspacing=6>"
            "<tr><td><b>Built with</b></td><td>Python (PySide6) + C++ + x86-64 assembly</td></tr>"
            "<tr><td><b>Protocol</b></td><td>MAVLink v1 / v2, MISSION_INT mission protocol</td></tr>"
            f"<tr><td><b>Link MAVLink</b></td><td>{mav}</td></tr>"
            f"{veh_html}"
            "</table>"
            "<p style='color:#8a90a0'>Bring your own drone &mdash; no vehicle models bundled.</p>")

    def _show_about(self):
        QMessageBox.about(self, "About DroneDeck", self._about_html())

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
            if lat and lon and math.isfinite(lat) and math.isfinite(lon):
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
        # validate the persisted takeoff altitude against the takeoff dialog's own 1..1000 m range:
        # a corrupt setting coerces to nan/inf (type=float doesn't raise on 'nan') or to 0.0 (junk
        # string) -- neither should seed the dialog default or be re-saved
        ta = s.value("flight/takeoff_alt", 25.0, type=float)
        self._takeoff_alt = ta if (math.isfinite(ta) and 1.0 <= ta <= 1000.0) else 25.0
        raw = s.value("links/configs")
        if raw:
            try:
                loaded = json.loads(raw)
            except (ValueError, TypeError):
                loaded = []
            # keep only well-formed entries: the Links dialog does dict(c) + c['name'] and would
            # otherwise raise on every launch on a corrupt-but-valid-JSON value (e.g. "[1,2]").
            self.link_configs = [c for c in loaded
                                 if isinstance(c, dict) and {"name", "transport", "target"} <= c.keys()] \
                if isinstance(loaded, list) else []
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
        s.setValue("flight/takeoff_alt", float(self._takeoff_alt))
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
