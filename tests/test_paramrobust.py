#!/usr/bin/env python3
"""test_paramrobust.py -- ParamManager robustness against adversarial / buggy autopilot behaviour
(a download/set state machine parallel to MissionProtocol). Drives it through a headless mock link:
normal download, gaps + timeout re-request, duplicate indices, an OUT-OF-RANGE param_index (the
MAVLink by-name convention sends index 65535), param_count changing mid-download, and PARAM_SET
confirm / never-echoes / wrong-echo. Must never hang/crash/corrupt or complete a download prematurely.
No real link, no arming."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtCore import QCoreApplication
import mavlink
from params import ParamManager, SET_RETRIES, MAX_RETRIES

app = QCoreApplication.instance() or QCoreApplication([])


class Msg:
    __slots__ = ("msgid", "sysid", "compid", "seq", "fields")
    def __init__(self, msgid, **fields):
        self.msgid, self.sysid, self.compid, self.seq, self.fields = msgid, 1, 1, 0, fields


class MockLink:
    def __init__(self):
        self.is_open = True
        self.remote = ("127.0.0.1", 14550)
        self.sent = []
    def request_params(self, tgt): self.sent.append(("list",))
    def request_param_read(self, tgt, param_id="", index=-1): self.sent.append(("read", param_id, index))
    def set_param(self, tgt, param_id, value, ptype=0, bytewise=True): self.sent.append(("set", param_id, value))


def new_mgr():
    link = MockLink()
    mgr = ParamManager(lambda: link, lambda: 1)
    fin, sr = [], []
    mgr.finished.connect(lambda ok, msg: fin.append((ok, msg)))
    mgr.set_result.connect(lambda name, ok, msg: sr.append((name, ok, msg)))
    return mgr, link, fin, sr


def pv(name, value, idx, cnt, ptype=mavlink.MAV_PARAM_TYPE_REAL32):
    return Msg(mavlink.PARAM_VALUE, param_id=name, param_value=float(value),
               param_type=ptype, param_index=idx, param_count=cnt)


def reads(link):
    return [a[2] for a in link.sent if a[0] == "read"]      # requested indices


# 1) normal download ------------------------------------------------------------------
mgr, link, fin, sr = new_mgr()
mgr.download()
for i, nm in enumerate(("A", "B", "C")):
    mgr.handle_messages([pv(nm, i + 1, i, 3)])
assert fin[-1] == (True, "3 parameters"), fin
assert set(mgr.values) == {"A", "B", "C"} and mgr.state == "idle"

# 2) GAP + timeout re-request ---------------------------------------------------------
mgr, link, fin, sr = new_mgr()
mgr.download()
mgr.handle_messages([pv("A", 1, 0, 3), pv("C", 3, 2, 3)])    # missing index 1
assert not fin, "completed with a gap"
mgr._on_timeout()
assert 1 in reads(link), f"did not re-request the missing index: {reads(link)}"
mgr.handle_messages([pv("B", 2, 1, 3)])
assert fin[-1][0] is True and set(mgr.values) == {"A", "B", "C"}

# 3) DUPLICATE index must not double-count or mis-complete -----------------------------
mgr, link, fin, sr = new_mgr()
mgr.download()
mgr.handle_messages([pv("A", 1, 0, 2), pv("A", 1, 0, 2)])    # duplicate index 0
assert not fin, "duplicate index falsely completed the download"
mgr.handle_messages([pv("B", 2, 1, 2)])
assert fin[-1][0] is True and len(mgr.values) == 2

# 4) OUT-OF-RANGE index (65535 = by-name convention) must NOT complete prematurely ----
mgr, link, fin, sr = new_mgr()
mgr.download()
mgr.handle_messages([pv("A", 1, 0, 3), pv("B", 2, 1, 3), pv("X", 9, 65535, 3)])
assert not fin, "out-of-range param_index completed the download prematurely (real index 2 missing)"
mgr.handle_messages([pv("C", 3, 2, 3)])
assert fin[-1] == (True, "3 parameters"), fin
assert {"A", "B", "C"}.issubset(set(mgr.values))

# 5) param_count changing mid-download -> adopts the new count, does not crash --------
mgr, link, fin, sr = new_mgr()
mgr.download()
mgr.handle_messages([pv("A", 1, 0, 3), pv("B", 2, 1, 5)])    # count jumps 3 -> 5
assert mgr.expected == 5 and not fin

# 6) PARAM_SET confirmed by echo ------------------------------------------------------
mgr, link, fin, sr = new_mgr()
mgr.set("MPC_X", 10.0)
assert any(a[0] == "set" for a in link.sent)
mgr.handle_messages([pv("MPC_X", 10.0, 7, 100)])
assert sr[-1][0] == "MPC_X" and sr[-1][1] is True and "MPC_X" not in mgr.pending

# 7) PARAM_SET never echoes -> retries to the cap then gives up cleanly ---------------
mgr, link, fin, sr = new_mgr()
mgr.set("MPC_Y", 3.0)
for _ in range(SET_RETRIES + 2):
    if "MPC_Y" not in mgr.pending:
        break
    mgr._on_set_timeout()
assert sr and sr[-1][0] == "MPC_Y" and sr[-1][1] is False and "MPC_Y" not in mgr.pending
n_sets = sum(1 for a in link.sent if a[0] == "set")
assert n_sets <= SET_RETRIES, f"re-sent more than SET_RETRIES times: {n_sets}"

# 8) PARAM_SET echoed with the WRONG value must NOT be treated as confirmed -----------
mgr, link, fin, sr = new_mgr()
mgr.set("MPC_Z", 10.0)
mgr.handle_messages([pv("MPC_Z", 5.0, 3, 100)])              # wrong value echoed
assert "MPC_Z" in mgr.pending, "wrong-value echo falsely confirmed the set"
assert not any(ok for (_, ok, _) in sr), sr

print("PARAMROBUST PASSED (8 adversarial scenarios)")
