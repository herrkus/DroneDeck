#!/usr/bin/env python3
"""test_rc_calibration.py -- RC calibration captures min/max and writes params.

Runs the simulator (which streams RC_CHANNELS with each channel sweeping its
full range), captures for a couple of seconds through the real calibration
widget, then checks the captured extremes are sane and that Save emits the
RCn_MIN/MAX/TRIM parameter set.
"""
import os
import sys
import subprocess

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
SIM = os.path.join(ROOT, "sim", "simulator.py")

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer
import main as appmain
from calibration import CalibrationDialog

app = QApplication([])
win = appmain.DroneDeck(14550)
dlg = CalibrationDialog(lambda: win.link, parent=win)
sim = subprocess.Popen([sys.executable, SIM, "--target", "127.0.0.1:14550"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

snap = {}
def start():
    dlg.radio._start()

def stop_and_grab():
    dlg.radio._stop()
    snap["lo"] = dict(dlg.radio.bars.lo)
    snap["hi"] = dict(dlg.radio.bars.hi)
    params = {}
    dlg.radio.saveRequested.connect(lambda p: params.update(p))
    dlg.radio._save()
    snap["params"] = params
    app.quit()

QTimer.singleShot(900, start)         # let the link come up + first RC arrive
QTimer.singleShot(3600, stop_and_grab)
app.exec()

sim.terminate()
try:
    sim.wait(timeout=2)
except Exception:
    sim.kill()

lo, hi, params = snap.get("lo", {}), snap.get("hi", {}), snap.get("params", {})
print("channels captured:", len(lo),
      "| RC1:", lo.get(1), "->", hi.get(1),
      "| params:", len(params))

fail = []
if len(lo) < 6:
    fail.append(f"only {len(lo)} channels captured")
for ch in lo:
    if hi[ch] - lo[ch] < 300:
        fail.append(f"ch{ch} range too small ({hi[ch] - lo[ch]})")
        break
    if not (lo[ch] < 1500 < hi[ch]):
        fail.append(f"ch{ch} did not sweep across centre ({lo[ch]}..{hi[ch]})")
        break
for key in ("RC1_MIN", "RC1_MAX", "RC1_TRIM"):
    if key not in params:
        fail.append(f"missing param {key}")
if params.get("RC1_MIN", 0) >= params.get("RC1_MAX", 1):
    fail.append("RC1_MIN >= RC1_MAX")
if len(params) != 3 * len(lo):
    fail.append(f"expected {3 * len(lo)} params, got {len(params)}")

# -- audit batch 10: a channel that never moved must NOT be written as MIN==MAX==TRIM --------------
# (degenerate cal = a dead/failsafe axis + divide-by-range risk on a real vehicle). Inject a
# synthetic capture: ch1 swept a full range, ch2 sat still. Only ch1 must be written.
dlg.radio.bars.lo = {1: 1000, 2: 1498}
dlg.radio.bars.hi = {1: 2000, 2: 1502}
dlg.radio.bars.cur = {1: 1500, 2: 1500}
degen = {}
dlg.radio.saveRequested.connect(lambda p: degen.update(p))
dlg.radio._save()
if "RC1_MIN" not in degen:
    fail.append("moved channel 1 was not written")
if "RC2_MIN" in degen:
    fail.append("unmoved channel 2 was written (degenerate MIN==MAX==TRIM)")
if degen.get("RC1_MIN", 0) >= degen.get("RC1_MAX", 1):
    fail.append("degenerate-guard: RC1_MIN >= RC1_MAX")

print("RC CAL FAILED: " + "; ".join(fail) if fail else
      "RC CAL PASSED (+ batch 10: unmoved channel skipped, not written as zero-span)")
sys.exit(1 if fail else 0)
