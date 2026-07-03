#!/usr/bin/env python3
"""test_streamreq.py -- telemetry stream request with bounded retry. On first detection the GCS
requests streams; because a real drone streams little until asked and a single request can be
dropped on a lossy link, it re-requests (throttled) until high-rate telemetry (ATTITUDE) arrives,
capped at STREAM_MAX_TRIES so a silent vehicle is never spammed forever. A (re)connect clears the
retry state so streams are requested afresh. Pure logic with a recording link."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import main as m


class RecLink:
    def __init__(self):
        self.calls = []
    def request_data_streams(self, sysid, autopilot):
        self.calls.append((sysid, autopilot))
    def close(self):
        pass
    def deleteLater(self):
        pass


class FakeVeh:
    def __init__(self):
        self.have_attitude = False
        self.have_position = False
        self.autopilot = 12          # PX4


app = QApplication([])
win = m.DroneDeck(14599)
rec = RecLink()
win.link = rec
veh = FakeVeh()

# first detection -> exactly one request, tries=1, autopilot forwarded
win._maybe_request_streams(5, veh, is_new=True)
assert rec.calls == [(5, 12)], rec.calls
assert win._stream_reqs[5]["tries"] == 1

# immediate re-call while telemetry is still absent -> throttled, no new request
win._maybe_request_streams(5, veh, is_new=False)
assert len(rec.calls) == 1, "retry must be throttled by STREAM_RETRY_S"

# force the retry interval to have elapsed -> re-requests, up to the cap
for expect in (2, 3, 4, 5):
    win._stream_reqs[5]["last_t"] -= 10.0
    win._maybe_request_streams(5, veh, is_new=False)
    assert win._stream_reqs[5]["tries"] == expect, (expect, win._stream_reqs[5]["tries"])
assert len(rec.calls) == 5

# cap reached: further attempts do nothing (no spamming a silent vehicle)
win._stream_reqs[5]["last_t"] -= 10.0
win._maybe_request_streams(5, veh, is_new=False)
assert len(rec.calls) == 5, "must not exceed STREAM_MAX_TRIES"

# once telemetry flows, retries stop even below the cap
veh.have_attitude = True
win._stream_reqs.setdefault(6, {"tries": 1, "last_t": 0.0})
before = len(rec.calls)
win._maybe_request_streams(6, veh, is_new=False)
assert len(rec.calls) == before, "attitude flowing -> no re-request"

# a vehicle already streaming when first seen: one request, then nothing
veh2 = FakeVeh(); veh2.have_attitude = True
win._maybe_request_streams(7, veh2, is_new=True)
assert len(rec.calls) == before + 1
win._stream_reqs[7]["last_t"] -= 10.0
win._maybe_request_streams(7, veh2, is_new=False)
assert len(rec.calls) == before + 1

# a (re)connect clears the retry bookkeeping so streams are re-requested next detection
win.link = None
win._stream_reqs[99] = {"tries": 3, "last_t": 0.0}
win._connect()
assert win._stream_reqs == {}, "reconnect must clear stream retry state"

print("STREAMREQ PASSED")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
