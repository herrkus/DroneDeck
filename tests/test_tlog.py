#!/usr/bin/env python3
"""test_tlog.py -- record a live session to a .tlog, then replay it.

Phase 1: record ~1.5 s of simulator telemetry to a .tlog via the link recorder.
Phase 2: feed that file through ReplayLink and confirm the frames decode back.
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

from link import UdpLink, ReplayLink
from tlog import TlogWriter, read_tlog
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # for _ports
from _ports import free_udp_port
PORT = free_udp_port()

SCRATCH = os.environ.get("SCRATCH", "/tmp")
REC = os.path.join(SCRATCH, "dronedeck_test.tlog")

app = QApplication([])

# ---- phase 1: record -------------------------------------------------------
link = UdpLink()
link.open(port=PORT)
sim = subprocess.Popen([sys.executable, SIM, "--target", f"127.0.0.1:{PORT}"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
rec = TlogWriter(REC)

QTimer.singleShot(800, lambda: setattr(link, "recorder", rec))
QTimer.singleShot(2300, lambda: (setattr(link, "recorder", None), rec.close()))
QTimer.singleShot(2400, app.quit)
app.exec()

sim.terminate()
try:
    sim.wait(timeout=2)
except Exception:
    sim.kill()
link.close()

records = read_tlog(REC)

# ---- phase 2: replay -------------------------------------------------------
rl = ReplayLink()
got = {"n": 0, "names": set()}
rl.messages.connect(lambda b: (got.__setitem__("n", got["n"] + len(b)),
                               [got["names"].add(m.name) for m in b]))
done = {"v": False}
rl.info.connect(lambda s: done.__setitem__("v", done["v"] or "complete" in s))
rl.open(path=REC, speed=25.0)
QTimer.singleShot(2500, app.quit)
app.exec()
rl.close()

print(f"recorded {rec.count} frames, read_tlog returned {len(records)}")
print(f"replay decoded {got['n']} messages; types: {sorted(got['names'])}")

fail = []
if rec.count < 30:
    fail.append(f"too few frames recorded ({rec.count})")
if len(records) != rec.count:
    fail.append(f"read_tlog count {len(records)} != written {rec.count}")
if got["n"] < 20:
    fail.append(f"replay decoded too few messages ({got['n']})")
if "HEARTBEAT" not in got["names"] or "ATTITUDE" not in got["names"]:
    fail.append("replay missing expected message types")
if not done["v"]:
    fail.append("replay did not finish")

print("TLOG FAILED: " + "; ".join(fail) if fail else "TLOG PASSED")
sys.exit(1 if fail else 0)
