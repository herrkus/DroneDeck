#!/usr/bin/env python3
"""test_params.py -- parameter download + set round-trip vs the simulator.

Downloads the full parameter set from the test vehicle, then sets one value and
confirms the vehicle echoes the new value back.
"""
import os
import sys
import subprocess

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APP = os.path.join(ROOT, "app")
SIM = os.path.join(ROOT, "sim", "simulator.py")
sys.path.insert(0, APP)

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer

from link import UdpLink
from params import ParamManager
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # for _ports
from _ports import free_udp_port
PORT = free_udp_port()

app = QApplication([])
link = UdpLink()
link.open(port=PORT)
sim = subprocess.Popen([sys.executable, SIM, "--target", f"127.0.0.1:{PORT}"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

pm = ParamManager(lambda: link, lambda: 1)
link.messages.connect(pm.handle_messages)
steps = []
pm.finished.connect(lambda ok, msg: steps.append((ok, msg)))

QTimer.singleShot(1000, lambda: pm.download() if link.remote is not None else None)
QTimer.singleShot(2600, lambda: pm.set("RTL_ALT", 2500.0))
QTimer.singleShot(3600, app.quit)
app.exec()

sim.terminate()
try:
    sim.wait(timeout=2)
except Exception:
    sim.kill()
link.close()

print("protocol steps:")
for ok, msg in steps:
    print(f"  {'ok ' if ok else 'ERR'} {msg}")
print(f"downloaded {len(pm.values)} parameters; sample: "
      f"WPNAV_SPEED={pm.values.get('WPNAV_SPEED')}, RTL_ALT={pm.values.get('RTL_ALT')}")

fail = []
if len(pm.values) != 14:
    fail.append(f"expected 14 params, got {len(pm.values)}")
if not steps or not steps[0][0]:
    fail.append("download did not finish OK")
if pm.values.get("WPNAV_SPEED") != 500.0:
    fail.append(f"WPNAV_SPEED wrong: {pm.values.get('WPNAV_SPEED')}")
if pm.values.get("RTL_ALT") != 2500.0:
    fail.append(f"set not echoed: RTL_ALT={pm.values.get('RTL_ALT')}")

print("PARAMS FAILED: " + "; ".join(fail) if fail else "PARAMS PASSED")
sys.exit(1 if fail else 0)
