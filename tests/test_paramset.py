#!/usr/bin/env python3
"""test_paramset.py -- param SET/encode robustness (iter109 fix).

The param table lets a user type any value into a cell; _item_changed only rejects non-numeric text
via float(), so "nan"/"inf"/"1e20" all pass and get written. For an INTEGER param type the old
param_encode did struct.pack(fmt, int(round(value))) with no guard, so a Write of such a value crashed
(NaN -> ValueError, inf -> OverflowError, 1e20 -> struct.error 'i' overflow). PX4 exposes many INT
params (MAV_SYS_ID, BAT1_N_CELLS, ...), so this was reachable from the GUI. param_encode now maps
non-finite to 0 and clamps to the field's range. This locks that in and confirms normal values still
round-trip exactly through encode->decode (the PX4 bytewise int<->float slot)."""
import os
import sys
import math

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

import mavlink

REAL32 = mavlink.MAV_PARAM_TYPE_REAL32
INT_TYPES = list(mavlink._PARAM_INT_FMT.keys())     # 1..8 (uint8..int64)

# 1) no int param type crashes on a non-finite / out-of-range value, via encode AND the full set frame
for pt in INT_TYPES:
    for bad in (float("nan"), float("inf"), float("-inf"), 1e20, -1e20, 1e300):
        # returns a float32 bit-pattern (may itself be a NaN/inf pattern -- that's the PX4 bytewise
        # slot, fine); the point is it does not raise.
        v = mavlink.param_encode(bad, pt)
        assert isinstance(v, float), (pt, bad, v)
        frame = mavlink.enc_param_set("MC_PITCH_P", bad, pt)
        assert len(frame) == 23, (pt, bad, len(frame))                   # <fBB16sB = 4+1+1+16+1
# real32 passes non-finite through untouched (a real param may legitimately be any float)
assert math.isnan(mavlink.param_encode(float("nan"), REAL32))
assert math.isinf(mavlink.param_encode(float("inf"), REAL32))

# 2) normal integer values round-trip EXACTLY (encode -> decode is identity in range)
for pt in INT_TYPES:
    for v in (0.0, 1.0, 5.0, 42.0, 100.0):
        dec = mavlink.param_decode(mavlink.param_encode(v, pt), pt)
        assert abs(dec - v) < 1e-6, (pt, v, dec)

# 3) out-of-range integers clamp to the field bounds (no wrap, no crash)
assert mavlink.param_decode(mavlink.param_encode(300.0, 1), 1) == 255.0      # uint8  -> 255
assert mavlink.param_decode(mavlink.param_encode(-5.0, 1), 1) == 0.0         # uint8  -> 0
assert mavlink.param_decode(mavlink.param_encode(99999.0, 4), 4) == 32767.0  # int16  -> 32767
assert mavlink.param_decode(mavlink.param_encode(-99999.0, 4), 4) == -32768.0
assert mavlink.param_decode(mavlink.param_encode(1e20, 6), 6) == 2147483647.0  # int32 -> max

# 3b) ArduPilot / spec-default C-CAST encoding (iter113): APM reports typed int params whose
# param_value is the CAST number, NOT bytewise bits. Decoding must return the value as-is and
# encoding must send a plain float -- applying bytewise here turned RC5_MIN=1100 into -32768.
assert mavlink.param_bytewise(12) is True                   # PX4 -> bytewise
assert mavlink.param_bytewise(3) is False                   # ArduPilot -> C-cast
assert mavlink.param_bytewise(0) is False                   # generic/unknown -> spec default (cast)
for name, val, pt in (("RC5_MIN", 1100.0, 4), ("SYSID_THISMAV", 1.0, 2), ("WPNAV_SPEED", 500.0, 6)):
    assert mavlink.param_decode(val, pt, bytewise=False) == val, (name, val, pt)
    assert mavlink.param_encode(val, pt, bytewise=False) == val
# cast encode still rounds + clamps (the iter109 hardening applies in both encodings)
assert mavlink.param_encode(99999.0, 4, bytewise=False) == 32767.0
assert mavlink.param_encode(float("nan"), 6, bytewise=False) == 0.0
assert math.isnan(mavlink.param_encode(float("nan"), REAL32, bytewise=False))   # real32 untouched
# and the full PARAM_SET frame carries the cast value for APM
frame = mavlink.enc_param_set("RC5_MIN", 1100.0, 4, bytewise=False)
import struct as _s
assert _s.unpack("<f", frame[:4])[0] == 1100.0, "cast PARAM_SET must carry the plain value"
frame = mavlink.enc_param_set("MC_PITCH_P", 4.0, 6, bytewise=True)              # PX4 unchanged
assert mavlink.param_decode(_s.unpack("<f", frame[:4])[0], 6, bytewise=True) == 4.0

# 4) _int_range spans exactly match each struct format
import struct
for pt, (fmt, _sz) in mavlink._PARAM_INT_FMT.items():
    lo, hi = mavlink._int_range(fmt)
    struct.pack(fmt, lo); struct.pack(fmt, hi)                   # bounds are packable
    for over in (lo - 1, hi + 1):
        try:
            struct.pack(fmt, over); raise AssertionError(f"{fmt} accepted {over}")
        except struct.error:
            pass                                                # confirms lo/hi are the true edges

print("PARAMSET PASSED (int-param encode: non-finite/out-of-range safe, normal values round-trip)")
