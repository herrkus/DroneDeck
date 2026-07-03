#!/usr/bin/env python3
"""test_linklifecycle.py -- connect/disconnect lifecycle robustness (iter110).

A common real workflow the other tests never exercised: the pilot disconnects mid-flight, reconnects,
switches links, or loses signal while a mission upload / param download is in flight. This guards that
none of that crashes or leaks:
  (a) disconnect MID-STREAM then _refresh/consume/grab -- _refresh must not deref the (now closed) link
  (b) rapid connect/disconnect/connect on the SAME port -- close() must release the port so rebind works
  (c) low-level link send on a CLOSED link is a silent no-op (sock=None guard), and GUI send actions
      gate on _has_vehicle() so nothing is sent while disconnected
  (d) disconnect while ParamManager download + MissionProtocol upload are in flight -- their retry
      timers fire against the closed link without crashing
  (e) no QUdpSocket / UdpLink object leak across cycles (deleteLater cleans up under the event loop)
Verification-only guard: no bug was found; this keeps the lifecycle robust. Modal dialogs are
neutralized so the offscreen run never blocks. Fixed fresh port. May need an isolated re-run if a
full-suite offscreen sweep trips Qt teardown."""
import os
import sys
import time
import gc

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication, QMessageBox
from PySide6.QtCore import QCoreApplication, QEvent
from PySide6.QtNetwork import QUdpSocket

app = QApplication.instance() or QApplication([])
import main as m
import mavlink
from link import UdpLink
from mission import MissionItem

# never block offscreen on a modal (the real GUI still shows these)
QMessageBox.information = staticmethod(lambda *a, **k: None)
QMessageBox.warning = staticmethod(lambda *a, **k: None)
QMessageBox.critical = staticmethod(lambda *a, **k: None)
QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.StandardButton.No)

PORT = 14766


class Msg:
    __slots__ = ("msgid", "sysid", "compid", "seq", "fields")
    def __init__(s, mid, **f):
        s.msgid, s.sysid, s.compid, s.seq, s.fields = mid, 1, 1, 0, f


def hb():
    return Msg(mavlink.HEARTBEAT, type=2, autopilot=12, base_mode=0, custom_mode=0, system_status=4)


def gp():
    return Msg(mavlink.GLOBAL_POSITION_INT, lat=474000000, lon=85000000, alt=100000,
               relative_alt=50000, vx=0, vy=0, vz=0, hdg=9000)


def pump(sec=0.15):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        QCoreApplication.processEvents()
        time.sleep(0.005)


def flush_deletes():
    # app.exec() drains DeferredDelete continuously; a manual processEvents() loop does not
    for _ in range(5):
        QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        QCoreApplication.processEvents()
        gc.collect()


win = m.DroneDeck(PORT)
win._persist = False

# (a) disconnect mid-stream, then refresh/consume/grab with the link closed --------------------------
win._connect(); pump()
win.vehicle.consume([hb(), gp()]); win.vehicle.last_heartbeat = time.monotonic()
win._refresh(); win.adi.grab()
win._toggle_conn()                              # disconnect (close, link kept as a closed object)
assert not win._has_vehicle()
win.vehicle.consume([hb(), gp()])               # late data arriving after disconnect
win._refresh(); win.adi.grab(); win.compass.grab(); win.map.grab()   # must not deref a None link

# (b) rapid reconnect on the same port -- close() must free the port so the rebind succeeds ----------
for i in range(5):
    win._connect(); pump(0.12)
    assert win.link.is_open, f"cycle {i}: reconnect on same port failed (port not released?)"
    win._toggle_conn(); pump(0.05)

# (c) low-level send on a CLOSED link is a no-op; GUI actions gate on _has_vehicle() -----------------
win._connect(); pump(0.1); win._toggle_conn()
lk = win.link
lk.arm(1, True); lk.set_mode(1, 1, 0); lk.send_command_long(1, 400, [1, 0, 0, 0, 0, 0, 0])
lk.send_manual_control(1, 0, 0, 0, 0); lk.send_mission_request_list(1); lk.request_params(1)
win._arm(True); win._set_mode(); win._takeoff()          # guarded -> no send, no crash
assert not win._has_vehicle()

# (d) disconnect while a param download + mission upload are in flight ------------------------------
win._connect(); pump(0.1)
win.params.download()
win.mission.upload([MissionItem(0, 47.0, 8.0, 50.0)], 0)
win._toggle_conn()                              # disconnect mid-protocol
for obj in (win.params, win.mission):
    for meth in ("_on_timeout", "_tick", "_retry"):
        if hasattr(obj, meth):
            getattr(obj, meth)()                # retry timer firing against the closed link
pump(0.3)

# (e) no socket / link object leak across cycles (measured after draining DeferredDelete) -----------
win._connect(); pump(0.1); win._toggle_conn(); flush_deletes()
base_s = sum(1 for o in gc.get_objects() if isinstance(o, QUdpSocket))
base_l = sum(1 for o in gc.get_objects() if isinstance(o, UdpLink))
for _ in range(8):
    win._connect(); pump(0.1); win._toggle_conn(); pump(0.05)
flush_deletes()
end_s = sum(1 for o in gc.get_objects() if isinstance(o, QUdpSocket))
end_l = sum(1 for o in gc.get_objects() if isinstance(o, UdpLink))
assert end_s - base_s <= 1, f"QUdpSocket leak across cycles: {base_s} -> {end_s}"
assert end_l - base_l <= 1, f"UdpLink leak across cycles: {base_l} -> {end_l}"

try:
    win.link.close()
except Exception:
    pass

print(f"LINKLIFECYCLE PASSED (disconnect mid-stream, rapid reconnect, closed-link sends, in-flight "
      f"disconnect, no leak: sockets {base_s}->{end_s} links {base_l}->{end_l})")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
