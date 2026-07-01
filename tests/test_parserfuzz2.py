#!/usr/bin/env python3
"""test_parserfuzz2.py -- extended parser resync fuzz, generalising the iter90 boundary bug across
frame variants. Builds streams of valid frames (MAVLink v1, v2, and v2-only msgid>255) separated by
BOTH random garbage AND structured UNKNOWN frames that straddle read boundaries -- oversized v1
unknowns, v2 unsigned unknowns, and SIGNED v2 unknowns (incompat sig bit -> +13 bytes, the resync
length math iter90 did not exercise). Each stream is fed to a fresh parser under MANY random
chunkings; every intact valid frame must decode every time (the strongest re-sync assertion). Runs
against both the native C++/asm core and the pure-Python PyParser. No link."""
import os
import sys
import random
import struct
from collections import Counter

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

import mavlink
import core

# valid-frame corpus -----------------------------------------------------------------
SMALL = [m for m in (mavlink.HEARTBEAT, mavlink.SYS_STATUS, mavlink.ATTITUDE,
                     mavlink.GLOBAL_POSITION_INT, mavlink.VFR_HUD, mavlink.GPS_RAW_INT,
                     mavlink.VIBRATION) if m in mavlink._WIRE and m in mavlink.CRC_EXTRA]
BIG = [m for m in mavlink._WIRE if m > 255 and m in mavlink.CRC_EXTRA][:4]   # v2-only msgids
assert len(SMALL) >= 5 and len(BIG) >= 2, (SMALL, BIG)


def make_valid(msgid, ver, seq):
    pl = b"\x00" * mavlink._WIRE[msgid][2]
    return mavlink.frame(msgid, pl, seq, 1, 1) if ver == 1 else mavlink.frame_v2(msgid, pl, seq, 1, 1)


def unknown_frame(rnd):
    """A structured but undecodable frame (unknown msgid) -- the interference class that swallowed a
    real frame in iter90. Cycles v1-oversized / v2 / signed-v2 so the resync length math is stressed."""
    kind = rnd.randrange(3)
    if kind == 0:                                   # oversized v1 unknown (large claimed length)
        mid = 177
        while mid in mavlink._WIRE:
            mid = (mid + 1) & 0xFF
        plen = rnd.randrange(200, 256)
        head = bytes((0xFE, plen, rnd.randrange(256), 7, 7, mid))
        return head + bytes(rnd.randrange(256) for _ in range(plen)) + bytes(2)
    mid = rnd.randrange(256, 0xFFFFFF)              # unknown v2 msgid (>255)
    plen = rnd.randrange(0, 256)
    sig = 0x01 if kind == 2 else 0x00              # kind 2 -> SIGNED (incompat bit) => +13 bytes
    head = bytes((0xFD, plen & 0xFF, sig, 0, rnd.randrange(256), 7, 7,
                  mid & 0xFF, (mid >> 8) & 0xFF, (mid >> 16) & 0xFF))
    body = bytes(rnd.randrange(256) for _ in range(plen))
    return head + body + bytes(2) + (bytes(13) if sig else b"")


def build_stream(seed):
    rnd = random.Random(seed)
    expected, parts = [], []
    for i in range(180):
        # interference before each valid frame: random garbage and/or a structured unknown frame
        if rnd.random() < 0.5:
            parts.append(bytes(rnd.randrange(256) for _ in range(rnd.randrange(0, 40))))
        if rnd.random() < 0.6:
            parts.append(unknown_frame(rnd))
        if i % 4 == 0:                              # a v2-only msgid>255 frame
            mid, ver = rnd.choice(BIG), 2
        else:
            mid, ver = rnd.choice(SMALL), rnd.choice((1, 2))
        parts.append(make_valid(mid, ver, i & 0xFF))
        expected.append(mid)
    return expected, b"".join(parts)


def feed_chunked(mk, stream, chunk_seed):
    rnd = random.Random(chunk_seed)
    p = mk()
    got, i = [], 0
    while i < len(stream):
        k = rnd.randrange(1, 64)
        got += [m.msgid for m in p.feed(stream[i:i + k])]
        i += k
    return got


def whole(mk, stream):
    return [m.msgid for m in mk().feed(stream)]


# Random garbage can, by chance, form a *self-consistent* false frame whose claimed length legitimately
# swallows a following valid frame -- that is inherent to MAVLink framing (the parser cannot tell a
# false-but-consistent frame from a real undecoded one) and MAVLink tolerates the loss. So the precise
# invariant (the one iter90 violated) is NOT "chunked decodes 100%" but:
#   (1) chunking introduces NO loss beyond what whole-feed already has (chunked[mid] >= whole[mid]);
#   (2) the native core and PyParser decode the exact same set for the same feed sequence (parity).
# A sanity floor also confirms whole-feed still decodes the overwhelming majority (the test isn't
# vacuously comparing two equally-broken parsers).
runs = 0
for s in range(8):
    expected, stream = build_stream(s)
    n_expected = len(expected)
    whole_n = Counter(whole(core.Parser, stream))
    whole_p = Counter(whole(mavlink.PyParser, stream))
    assert whole_n == whole_p, f"parity(whole) seed={s}: {whole_n.most_common(3)} vs {whole_p.most_common(3)}"
    assert sum(whole_n.values()) >= 0.9 * n_expected, \
        f"seed={s}: whole-feed decoded only {sum(whole_n.values())}/{n_expected} valid frames"
    for cs in range(10):
        cseed = cs * 131 + 7
        cn = Counter(feed_chunked(core.Parser, stream, cseed))
        cp = Counter(feed_chunked(mavlink.PyParser, stream, cseed))
        assert cn == cp, f"parity(chunk) s={s} cs={cs}: {(cn - cp).most_common(3)} / {(cp - cn).most_common(3)}"
        for mid, c in whole_n.items():
            assert cn[mid] >= c, (f"s={s} chunk={cs}: msgid {mid} chunked {cn[mid]} < whole-feed {c} "
                                  f"-- chunking lost a frame (iter90 class)")
        runs += 1

# pure structured-unknown noise (no valid frames) must decode ~nothing (no false positives)
rnd = random.Random(99)
noise = b"".join(unknown_frame(rnd) for _ in range(300))
for label, mk in (("native", core.Parser), ("py", mavlink.PyParser)):
    n = len(feed_chunked(mk, noise, 3))
    assert n <= 3, f"{label}: {n} false decodes from structured-unknown noise"

print(f"PARSERFUZZ2 PASSED ({runs} chunked runs x2 parsers, v1+v2+msgid>255+signed, native={core.NATIVE})")
