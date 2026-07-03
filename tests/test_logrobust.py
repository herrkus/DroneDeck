#!/usr/bin/env python3
"""test_logrobust.py -- LogManager download robustness on a lossy link (audit batch 10).

test_logs.py already covers the happy path against the simulator. This drives the adversarial cases
that only bite on a real radio, through a headless mock link (no sim, no socket, no port):
  * a DROPPED mid-stream chunk must not be counted as progress (no silent zero-filled hole) and must
    be re-requested from the first missing byte on timeout;
  * a dropped FINAL chunk must not hang "downloading" forever -- retries then a clean failure;
  * starting a second download while one is in flight must close the first file handle (no fh leak /
    corrupt file), not silently abandon it;
  * a short chunk sitting AFTER a gap must not end the download early.
No real link, no arming."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

import tempfile
from PySide6.QtCore import QCoreApplication
import mavlink
from logdownload import LogManager, CHUNK, DL_MAX_RETRIES

app = QCoreApplication.instance() or QCoreApplication([])


class MockLink:
    is_open = True
    remote = ("127.0.0.1", 14550)

    def __init__(self):
        self.data_reqs = []          # (log_id, ofs) for each request_log_data
        self.ended = 0

    def request_log_list(self, *a):
        pass

    def request_log_data(self, tgt, lid, ofs, count):
        self.data_reqs.append((lid, ofs))

    def log_request_end(self, *a):
        self.ended += 1


def data_msg(lid, ofs, count, payload=None):
    return type("M", (), {"msgid": mavlink.LOG_DATA, "fields": {
        "id": lid, "ofs": ofs, "count": count, "data": payload or bytes(count)}})()


def new_mgr(size):
    link = MockLink()
    d = tempfile.mkdtemp(prefix="dronedeck-logrobust-")
    mgr = LogManager(lambda: link, lambda: 1, d)
    mgr._entries = {5: {"id": 5, "size": size, "time_utc": 0}, 6: {"id": 6, "size": CHUNK, "time_utc": 0}}
    fin = []
    mgr.finished.connect(lambda ok, path, msg: fin.append((ok, path, msg)))
    return mgr, link, fin


fail = []


def check(name, cond, extra=""):
    if not cond:
        fail.append(f"{name} ({extra})")


# 1) dropped MID-stream chunk: gap is not progress, and is re-requested from the hole --------------
mgr, link, fin = new_mgr(3 * CHUNK)
mgr.download(5)
fh1 = mgr.fh
mgr._handle(data_msg(5, 0, CHUNK))                 # chunk 0 ok
mgr._handle(data_msg(5, 2 * CHUNK, CHUNK))         # chunk 2 arrives, chunk 1 dropped
check("gap not counted as progress", mgr.dl_got == CHUNK, f"dl_got={mgr.dl_got}")
check("still downloading (not falsely complete)", mgr.state == "downloading", mgr.state)
link.data_reqs.clear()
mgr._on_dl_timeout()
check("timeout re-requests from first missing byte", link.data_reqs and link.data_reqs[-1] == (5, CHUNK),
      f"{link.data_reqs}")
# the gap fill then completes it
mgr._handle(data_msg(5, CHUNK, CHUNK))             # fill chunk 1 -> contiguous through 2*CHUNK
mgr._handle(data_msg(5, 2 * CHUNK, CHUNK))         # chunk 2 again -> reaches size
check("completes after gap filled", fin and fin[-1][0] is True and mgr.state == "idle", fin[-1] if fin else None)

# 2) a short chunk sitting AFTER a gap must NOT end the download (only end when contiguous) ---------
mgr, link, fin = new_mgr(3 * CHUNK)
mgr.download(5)
mgr._handle(data_msg(5, 0, CHUNK))                 # chunk 0
mgr._handle(data_msg(5, 2 * CHUNK, CHUNK - 30))    # short final-looking chunk, but chunk 1 missing
check("short chunk after gap does not finish", mgr.state == "downloading" and not fin,
      f"state={mgr.state} fin={fin}")

# 3) dropped FINAL chunk: retries then a clean failure, no infinite hang, fh closed ----------------
mgr, link, fin = new_mgr(2 * CHUNK)
mgr.download(5)
fh = mgr.fh
mgr._handle(data_msg(5, 0, CHUNK))                 # only the first chunk ever arrives
for _ in range(DL_MAX_RETRIES + 1):
    mgr._on_dl_timeout()
check("stuck download fails cleanly", fin and fin[-1][0] is False, fin[-1] if fin else None)
check("failure closed the file handle", fh.closed and mgr.fh is None, f"closed={fh.closed}")
check("failure returns to idle", mgr.state == "idle", mgr.state)
check("failure message reports partial bytes", fin and "incomplete" in fin[-1][2], fin[-1] if fin else None)

# 4) busy guard: a second download closes the first fh (no leak / no corrupt half-file) ------------
mgr, link, fin = new_mgr(3 * CHUNK)
mgr.download(5)
first_fh = mgr.fh
mgr.download(6)                                     # start another before the first finished
check("busy guard closed prior fh", first_fh.closed, f"closed={first_fh.closed}")
check("busy guard switched target", mgr.dl_id == 6 and mgr.fh is not None and not mgr.fh.closed, mgr.dl_id)

print("LOGROBUST FAILED: " + "; ".join(fail) if fail else
      "LOGROBUST PASSED (dropped chunk re-requested, no silent hole; stuck download fails cleanly; "
      "busy guard closes prior fh; short-chunk-after-gap doesn't finish early)")
sys.stdout.flush()
os._exit(1 if fail else 0)
