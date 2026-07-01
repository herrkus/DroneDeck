#!/usr/bin/env python3
"""test_parserfuzz.py -- parser robustness against a hostile byte stream (real radio links corrupt,
drop, and truncate bytes). Feeds the streaming MAVLink parser -- both the native C++/asm core and
the pure-Python PyParser -- four hostile patterns and asserts it never crashes, never false-decodes
garbage, catches bit-flips via CRC, and always re-syncs to the next valid frame:
  (a) pure random garbage        -> no crash, ~0 spurious decodes
  (b) valid frames + garbage     -> every valid frame still decodes (byte-scan re-sync)
  (c) a frame split across feeds  -> completes and decodes once whole
  (d) bit-flipped payload         -> CRC rejects it, and the next clean frame still decodes
No link, no arming -- frames are crafted with mavlink.frame()."""
import os
import sys
import random
from collections import Counter

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

import mavlink
import core

random.seed(0xD1CE)

# candidate msgids -> keep only those that build and decode cleanly in isolation, so the test is
# robust to exactly which messages the dialect wires.
CANDIDATES = [mavlink.HEARTBEAT, mavlink.SYS_STATUS, mavlink.ATTITUDE,
              mavlink.GLOBAL_POSITION_INT, mavlink.VFR_HUD, mavlink.GPS_RAW_INT, mavlink.VIBRATION]


def valid_frame(msgid, seq=0, sysid=1, compid=1):
    full = mavlink._WIRE[msgid][2]
    return mavlink.frame(msgid, b"\x00" * full, seq, sysid, compid)


MSGIDS = []
for mid in CANDIDATES:
    if mid in mavlink._WIRE and mid in mavlink.CRC_EXTRA:
        got = core.Parser().feed(valid_frame(mid))
        if any(m.msgid == mid for m in got):
            MSGIDS.append(mid)
assert len(MSGIDS) >= 4, f"too few decodable msgids for the fuzz corpus: {MSGIDS}"


def garbage(n):
    return bytes(random.randrange(256) for _ in range(n))


def run_fuzz(label, mk):
    # (a) pure garbage -> no crash; a 16-bit CRC makes an accidental valid frame astronomically
    #     unlikely, so essentially nothing should decode
    p = mk()
    decoded = 0
    for _ in range(80):
        decoded += len(p.feed(garbage(256)))
    assert decoded <= 3, f"{label}: {decoded} false decodes from pure garbage"

    # (b) valid frames interleaved with random garbage, fed in random-sized chunks so frames span
    #     feed boundaries -> every valid frame must still decode (re-sync + cross-feed buffering)
    p = mk()
    expected, stream = [], b""
    for i in range(200):
        stream += garbage(random.randrange(0, 40))
        mid = MSGIDS[i % len(MSGIDS)]
        stream += valid_frame(mid, seq=i & 0xFF)
        expected.append(mid)
    got = []
    i = 0
    while i < len(stream):
        k = random.randrange(1, 64)
        got += [m.msgid for m in p.feed(stream[i:i + k])]
        i += k
    ec, gc = Counter(expected), Counter(got)
    for mid, c in ec.items():
        assert gc[mid] >= c, f"{label}: msgid {mid} decoded {gc[mid]}/{c} -- re-sync lost frames"

    # (c) a single frame split across two feeds -> no crash on the partial, decodes once complete
    p = mk()
    fr = valid_frame(mavlink.HEARTBEAT, seq=7)
    p.feed(fr[:len(fr) // 2])                    # partial: must not crash or spuriously emit
    tail = p.feed(fr[len(fr) // 2:])
    assert any(m.msgid == mavlink.HEARTBEAT for m in tail), f"{label}: split frame never completed"

    # (d) a bit-flipped payload must be rejected by CRC, and a following clean frame must still
    #     decode (a bad CRC must not desync the stream)
    p = mk()
    bad = bytearray(valid_frame(mavlink.ATTITUDE, seq=3))
    bad[8] ^= 0x40                               # flip a bit inside the payload
    got = p.feed(bytes(bad))
    assert all(m.msgid != mavlink.ATTITUDE for m in got), f"{label}: bit-flip slipped past CRC"
    after = p.feed(valid_frame(mavlink.HEARTBEAT, seq=4))
    assert any(m.msgid == mavlink.HEARTBEAT for m in after), f"{label}: no re-sync after a bad CRC"


run_fuzz(f"core.Parser(native={core.NATIVE})", core.Parser)
run_fuzz("PyParser", mavlink.PyParser)
print(f"PARSERFUZZ PASSED (corpus={len(MSGIDS)} msgids, native={core.NATIVE})")
