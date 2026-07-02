#!/usr/bin/env python3
"""test_logs.py -- onboard log list + download vs the simulator.

Lists the simulator's fake logs, downloads one, and verifies the saved file has
the right size and the exact bytes the simulator generated.
"""
import os
import sys
import tempfile
import subprocess

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # for _ports
from _ports import free_udp_port
PORT = free_udp_port()
sys.path.insert(0, os.path.join(ROOT, "sim"))
SIM = os.path.join(ROOT, "sim", "simulator.py")

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer

from link import UdpLink
from logdownload import LogManager
import simulator as simmod          # for the reference byte pattern

app = QApplication([])
link = UdpLink()
link.open(port=PORT)
sim = subprocess.Popen([sys.executable, SIM, "--target", f"127.0.0.1:{PORT}"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

tmp = tempfile.mkdtemp(prefix="dronedeck-logs-")
mgr = LogManager(lambda: link, lambda: 1, tmp)
link.messages.connect(mgr.handle_messages)

result = {}
mgr.entries.connect(lambda e: result.__setitem__("entries", e))
mgr.finished.connect(lambda ok, path, msg: result.update(ok=ok, path=path, msg=msg))

DL_ID = 2
QTimer.singleShot(1200, mgr.request_list)
QTimer.singleShot(2200, lambda: mgr.download(DL_ID))
QTimer.singleShot(4200, app.quit)
app.exec()

sim.terminate()
try:
    sim.wait(timeout=2)
except Exception:
    sim.kill()
link.close()

entries = result.get("entries", [])
print("entries:", entries)
print("download:", result.get("ok"), result.get("msg"))

fail = []
sizes = {e["id"]: e["size"] for e in entries}
if len(entries) != len(simmod.SIM_LOGS):
    fail.append(f"listed {len(entries)} logs, expected {len(simmod.SIM_LOGS)}")
for lid, sz, _ in simmod.SIM_LOGS:
    if sizes.get(lid) != sz:
        fail.append(f"log {lid} size {sizes.get(lid)} != {sz}")

path = result.get("path", "")
exp_size = dict((l[0], l[1]) for l in simmod.SIM_LOGS)[DL_ID]
if not result.get("ok") or not path or not os.path.exists(path):
    fail.append("download did not finish / no file")
else:
    blob = open(path, "rb").read()
    expect = bytes(simmod._logbyte(DL_ID, k) for k in range(exp_size))
    if len(blob) != exp_size:
        fail.append(f"file is {len(blob)} bytes, expected {exp_size}")
    elif blob != expect:
        fail.append("downloaded bytes do not match the simulator's log content")

print("LOGS FAILED: " + "; ".join(fail) if fail else "LOGS PASSED")
sys.exit(1 if fail else 0)
