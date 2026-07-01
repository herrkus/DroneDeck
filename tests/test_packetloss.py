#!/usr/bin/env python3
"""test_packetloss.py -- link packet loss from MAVLink seq gaps, computed in the PARSER over every
valid frame (decoded or not). The subtle bug this guards: the GCS decodes only a subset of msgids,
so if loss were measured on decoded messages alone, all the undecoded traffic would look like loss.
Here interleaved unknown-msgid frames must NOT inflate loss; genuine seq gaps must. Both the native
C++ core and the pure-Python parser are checked and must agree. No link/Qt UI needed."""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

import struct
import mavlink
import core

HB = mavlink.HEARTBEAT
UNK = 99
assert UNK not in mavlink._WIRE, "test needs an undecoded msgid"


def build(specs, sysid=1, compid=1):
    """specs: list of ('hb'|'unk', seq). 'hb' -> a real HEARTBEAT frame; 'unk' -> an undecoded
    msgid (valid framing, arbitrary CRC -- the parser length-skips it)."""
    out = b""
    for kind, seq in specs:
        if kind == "hb":
            out += mavlink.frame(HB, b"\x00" * 9, seq, sysid, compid)
        else:
            pl = b"\x11\x22\x33"
            head = bytes((0xFE, len(pl), seq & 0xFF, sysid & 0xFF, compid & 0xFF, UNK & 0xFF))
            out += head + pl + b"\x00\x00"
    return out


def native_loss(data):
    p = core.Parser()
    p.feed(data)
    return p.loss


def py_loss(data):
    pp = mavlink.PyParser()
    pp.feed(data)
    tot = pp.frames + pp.lost
    return (100.0 * pp.lost / tot) if tot else 0.0


def both(data):
    n, p = native_loss(data), py_loss(data)
    assert abs(n - p) < 0.01, f"native {n} vs python {p} disagree"
    return n


# clean known stream -> 0% loss
assert both(build([("hb", i & 0xFF) for i in range(30)])) == 0.0

# THE KEY CASE: undecoded msgids interleaved with contiguous seqs -> still 0% (not counted as loss)
interleaved = build([("hb", 0), ("unk", 1), ("hb", 2), ("unk", 3), ("hb", 4), ("unk", 5), ("hb", 6)])
assert both(interleaved) == 0.0, "undecoded frames must not be counted as packet loss"

# a genuine gap in the seq stream IS loss: seqs 0,1,2,5,6 -> 2 missing between 2 and 5
gap = build([("hb", 0), ("hb", 1), ("unk", 2), ("hb", 5), ("hb", 6)])
lp = both(gap)                                   # 5 frames + 2 lost -> 2/7 = 28.57%
assert 28.0 < lp < 29.5, lp

# a sender reboot (seq resets after a high value) is a discontinuity, not ~200 losses -> ~0%
reboot = build([("hb", 200), ("hb", 201), ("hb", 202), ("hb", 0), ("hb", 1), ("hb", 2)])
assert both(reboot) == 0.0, reboot

# seq wrap 254,255,0,1 is contiguous -> no false loss
assert both(build([("hb", s) for s in (254, 255, 0, 1, 2)])) == 0.0

print("PACKETLOSS PASSED")
