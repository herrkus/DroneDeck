"""preflight.py -- arming-readiness (preflight) checks + live dialog.

QGC gates arming on the vehicle's own preflight checks and shows why it will/won't arm. This mirrors the
common conditions a GCS can see from telemetry -- GPS, estimator, home, battery, sensor health, armed
state -- so the operator gets a single glanceable "READY / NOT READY" before flight. Pure evaluation
(preflight_checks) is separated from the dialog so it is fully testable.
"""
import mavlink

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QDialog, QVBoxLayout, QLabel, QTableWidget, QTableWidgetItem,
                               QHeaderView, QPushButton, QHBoxLayout)
from PySide6.QtGui import QColor

_STATUS_COLOR = {"pass": "#37d67a", "warn": "#e0a030", "fail": "#e05050", "unknown": "#8a90a0"}


def preflight_checks(ve):
    """Evaluate arming readiness from a Vehicle's live telemetry.

    Returns a list of (name, status, detail) where status is 'pass' | 'warn' | 'fail' | 'unknown'.
    'unknown' means that telemetry has not arrived yet (not a failure)."""
    checks = []

    if ve.fix_type >= 3 and ve.satellites >= 6:
        checks.append(("GPS", "pass", f"{ve.fix_type}D fix, {ve.satellites} sats"))
    elif ve.fix_type == 0 and ve.satellites == 0:
        checks.append(("GPS", "unknown", "no GPS telemetry"))
    else:
        checks.append(("GPS", "fail", f"fix {ve.fix_type}, {ve.satellites} sats (need 3D + >=6)"))

    if ve.ekf_flags == 0:
        checks.append(("Estimator", "unknown", "no estimator status"))
    elif ve.ekf_ok():
        checks.append(("Estimator", "pass", f"converged (var {ve.ekf_variance_max():.2f})"))
    else:
        checks.append(("Estimator", "fail", f"not converged (var {ve.ekf_variance_max():.2f})"))

    checks.append(("Home", "pass", "set") if ve.home is not None
                  else ("Home", "fail", "not set (needs a position fix)"))

    if ve.battery_remaining is not None and ve.battery_remaining >= 0:
        if ve.battery_remaining >= 25:
            checks.append(("Battery", "pass", f"{ve.battery_remaining}%"))
        elif ve.battery_remaining >= 15:
            checks.append(("Battery", "warn", f"{ve.battery_remaining}% (low)"))
        else:
            checks.append(("Battery", "fail", f"{ve.battery_remaining}% (critical)"))
    elif ve.voltage > 0:
        checks.append(("Battery", "pass", f"{ve.voltage:.1f} V"))
    else:
        checks.append(("Battery", "unknown", "no battery telemetry"))

    if ve.sensors_present == 0:
        checks.append(("Sensors", "unknown", "no SYS_STATUS"))
    else:
        bad = [name for bit, name in mavlink.SENSOR_BITS
               if (ve.sensors_enabled & bit) and not (ve.sensors_health & bit)]
        checks.append(("Sensors", "pass", "all healthy") if not bad
                      else ("Sensors", "fail", "unhealthy: " + ", ".join(bad)))

    checks.append(("Armed", "warn", "vehicle is ARMED") if ve.armed
                  else ("Armed", "pass", "disarmed"))
    return checks


def is_ready(checks):
    """Ready to fly only if nothing failed (warnings and not-yet-known items don't block)."""
    return not any(status == "fail" for _n, status, _d in checks)


class PreflightDialog(QDialog):
    """Live arming-readiness readout, refreshed from the vehicle while open."""

    def __init__(self, vehicle_getter, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Preflight Check")
        self.resize(420, 320)
        self._vehicle = vehicle_getter
        self.banner = QLabel("")
        self.banner.setAlignment(Qt.AlignCenter)
        self.banner.setStyleSheet("font-size:16px; font-weight:bold; padding:8px;")
        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["Check", "Status", "Detail"])
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        btn = QPushButton("Close")
        btn.clicked.connect(self.close)
        row = QHBoxLayout()
        row.addStretch(1)
        row.addWidget(btn)
        lay = QVBoxLayout(self)
        lay.addWidget(self.banner)
        lay.addWidget(self.table, 1)
        lay.addLayout(row)

        self._ve = self._vehicle()
        if self._ve is not None:
            self._ve.updated.connect(self.refresh)
        self.refresh()

    def refresh(self):
        ve = self._vehicle()
        if ve is None:
            self.banner.setText("NO VEHICLE")
            self.banner.setStyleSheet("font-size:16px; font-weight:bold; padding:8px; color:#8a90a0;")
            self.table.setRowCount(0)
            return
        checks = preflight_checks(ve)
        ready = is_ready(checks)
        self.banner.setText("READY TO ARM" if ready else "NOT READY")
        self.banner.setStyleSheet("font-size:16px; font-weight:bold; padding:8px; color:"
                                  + ("#37d67a" if ready else "#e05050") + ";")
        self.table.setRowCount(len(checks))
        for r, (name, status, detail) in enumerate(checks):
            self.table.setItem(r, 0, QTableWidgetItem(name))
            sitem = QTableWidgetItem(status.upper())
            sitem.setForeground(QColor(_STATUS_COLOR.get(status, "#d6dae2")))
            self.table.setItem(r, 1, sitem)
            self.table.setItem(r, 2, QTableWidgetItem(detail))

    def closeEvent(self, e):
        if self._ve is not None:
            try:
                self._ve.updated.disconnect(self.refresh)
            except (RuntimeError, TypeError):
                pass
        super().closeEvent(e)
