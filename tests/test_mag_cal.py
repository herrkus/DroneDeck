#!/usr/bin/env python3
"""test_mag_cal.py -- compass onboard-calibration handshake (1:1 QGC parity).

Completes calibration gap #1: the compass half. DroneDeck could not run ArduPilot's onboard mag cal
(DO_START_MAG_CAL) nor show its live progress (MAG_CAL_PROGRESS) / result (MAG_CAL_REPORT) nor
accept/cancel it. Added both message parsers (CRC 92/36; PROGRESS decodes only the 17-byte prefix and
skips the completion_mask[10] array), vehicle state, the three commands, and a progress bar + Accept/
Cancel in the sensor-cal widget. Verifies decode parity + exact values (incl. the realistic 27-byte
PROGRESS frame whose trailing mask must be ignored), vehicle state, command encodings, and the widget
show/enable/emit behaviour + main routing."""
import os
import sys
import struct

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import mavlink
import core
from vehicle import Vehicle
from link import Link

fail = []

# realistic full 27-byte PROGRESS payload: 17-byte decoded prefix + 10-byte completion_mask to skip
prog_prefix = struct.pack("<fffBBBBB", 0.1, 0.2, 0.3, 1, 1, 2, 1, 47)
prog_payload = prog_prefix + bytes([0xFF] * 10)
prog_frame = mavlink.frame(mavlink.MAG_CAL_PROGRESS, prog_payload, 7, 1, 1, crc_fn=core.crc_extra)
prog_exp = {"direction_x": 0.1, "direction_y": 0.2, "direction_z": 0.3, "compass_id": 1,
            "cal_mask": 1, "cal_status": 2, "attempt": 1, "completion_pct": 47}

rep_payload = struct.pack("<ffffffffffBBBB", 12.5, 1, 2, 3, 0.9, 0.8, 0.7, 0.1, 0.2, 0.3, 1, 1, 4, 1)
rep_frame = mavlink.frame(mavlink.MAG_CAL_REPORT, rep_payload, 7, 1, 1, crc_fn=core.crc_extra)
rep_exp = {"fitness": 12.5, "ofs_x": 1, "ofs_y": 2, "ofs_z": 3, "diag_x": 0.9, "diag_y": 0.8,
           "diag_z": 0.7, "offdiag_x": 0.1, "offdiag_y": 0.2, "offdiag_z": 0.3,
           "compass_id": 1, "cal_mask": 1, "cal_status": 4, "autosaved": 1}

# 1) both backends decode identically with exact values; completion_mask is absent -------------------
for frame, exp, name in ((prog_frame, prog_exp, "MAG_CAL_PROGRESS"), (rep_frame, rep_exp, "MAG_CAL_REPORT")):
    nat, pyp = core.Parser().feed(frame), mavlink.PyParser().feed(frame)
    if len(nat) != 1 or len(pyp) != 1:
        fail.append(f"{name}: decode count native={len(nat)} py={len(pyp)}")
        continue
    for k, v in exp.items():
        a, b = nat[0].fields.get(k), pyp[0].fields.get(k)
        if a is None or b is None or abs(a - v) > 1e-3 or abs(b - v) > 1e-3:
            fail.append(f"{name}.{k}: native={a} py={b} expected={v}")
    if "completion_mask" in nat[0].fields or "completion_mask" in pyp[0].fields:
        fail.append(f"{name}: completion_mask should be skipped, not decoded")

# 2) vehicle state tracks progress then report -------------------------------------------------------
ve = Vehicle()
ve.consume(core.Parser().feed(prog_frame))
if ve.mag_cal_pct != 47 or ve.mag_cal_status != 2 or ve.mag_cal_done is not False:
    fail.append(f"progress state: pct={ve.mag_cal_pct} status={ve.mag_cal_status} done={ve.mag_cal_done}")
ve.consume(core.Parser().feed(rep_frame))
if (ve.mag_cal_pct != 100 or ve.mag_cal_status != 4 or ve.mag_cal_done is not True
        or abs((ve.mag_cal_fitness or 0) - 12.5) > 1e-3):
    fail.append(f"report state: pct={ve.mag_cal_pct} status={ve.mag_cal_status} "
                f"fitness={ve.mag_cal_fitness} done={ve.mag_cal_done}")

# 3) the three commands encode correctly -------------------------------------------------------------
class CaptureLink(Link):
    def __init__(self):
        super().__init__()
        self.sent = []
        self._open = True
        self.remote = True

    def _write(self, data):
        self.sent.append(data)


lk = CaptureLink()
lk.start_mag_cal(1)
lk.accept_mag_cal(1)
lk.cancel_mag_cal(1)
cmds = [core.Parser().feed(d)[0].fields for d in lk.sent]
if int(cmds[0]["command"]) != mavlink.MAV_CMD_DO_START_MAG_CAL or abs(cmds[0]["param2"] - 1) > 1e-6 or abs(cmds[0]["param3"] - 1) > 1e-6:
    fail.append(f"start_mag_cal wrong: {cmds[0].get('command')} p2={cmds[0].get('param2')} p3={cmds[0].get('param3')}")
if int(cmds[1]["command"]) != mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL:
    fail.append(f"accept_mag_cal wrong: {cmds[1].get('command')}")
if int(cmds[2]["command"]) != mavlink.MAV_CMD_DO_CANCEL_MAG_CAL:
    fail.append(f"cancel_mag_cal wrong: {cmds[2].get('command')}")

# 4) the sensor-cal widget: progress row shows only for compass; Accept gated on success -------------
from PySide6.QtWidgets import QApplication
app = QApplication.instance() or QApplication([])
from calibration import SensorCalibrationWidget
w = SensorCalibrationWidget()
# isHidden() reflects the widget's own visibility flag (isVisible() is False in headless tests because
# the top-level is never shown), so it is what tells us whether the row is meant to be showing.
if not w.mag_row.isHidden():
    fail.append("compass progress row should start hidden")
w._request("compass")
if w.mag_row.isHidden() or w.btn_mag_accept.isEnabled() or not w.btn_mag_cancel.isEnabled():
    fail.append("compass cal should show the row, disable Accept, enable Cancel")
w.set_mag_progress(47, 2)
if w.mag_bar.value() != 47:
    fail.append(f"progress bar should be 47, got {w.mag_bar.value()}")
w.set_mag_report(4, 12.5)                                  # status 4 = success
if not w.btn_mag_accept.isEnabled() or w.mag_bar.value() != 100:
    fail.append("a successful report should enable Accept and fill the bar")
w.set_mag_report(5, 99.0)                                  # status 5 = failed
if w.btn_mag_accept.isEnabled():
    fail.append("a failed report must NOT enable Accept")
got = []
w.compassAccept.connect(lambda: got.append("accept"))
w.compassCancel.connect(lambda: got.append("cancel"))
w.btn_mag_accept.setEnabled(True)
w.btn_mag_accept.click()
w.btn_mag_cancel.click()
if got != ["accept", "cancel"]:
    fail.append(f"accept/cancel buttons should emit, got {got}")
w._request("gyro")
if not w.mag_row.isHidden():
    fail.append("a non-compass calibration should hide the compass row")

# 5) main routes compass -> start_mag_cal and accept/cancel -> their commands, all vehicle-guarded ---
import main as m
win = m.DroneDeck(16450)
win._persist = False
calls = []
win.link.start_mag_cal = lambda s: calls.append("start")
win.link.calibrate = lambda s, k: calls.append(("cal", k))
win.link.accept_mag_cal = lambda s: calls.append("accept")
win.link.cancel_mag_cal = lambda s: calls.append("cancel")
win._sysid = lambda: 1
win._has_vehicle = lambda: True
win._cal_sensor("compass")
win._cal_sensor("gyro")
win._cal_mag_accept()
win._cal_mag_cancel()
if calls != ["start", ("cal", "gyro"), "accept", "cancel"]:
    fail.append(f"main routing wrong: {calls}")
calls.clear()
win._has_vehicle = lambda: False
win._cal_mag_accept()
win._cal_mag_cancel()
if calls:
    fail.append("no-vehicle: accept/cancel must not send")

print("MAG_CAL FAILED: " + "; ".join(fail) if fail else
      "MAG_CAL PASSED (MAG_CAL_PROGRESS/REPORT decode native==python w/ exact values, completion_mask "
      "skipped; vehicle tracks pct/status/fitness/done; DO_START/ACCEPT/CANCEL_MAG_CAL encode; widget "
      "shows progress for compass only + gates Accept on success + emits; main routes + guards)")
sys.stdout.flush()
os._exit(1 if fail else 0)
