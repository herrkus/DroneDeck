#!/usr/bin/env python3
"""test_newmsgfuzz.py -- adversarial robustness of the iter147-161 message paths (real-drone hardening).

The features added this wave (RTK, terrain, camera protocol, CAMERA_FEEDBACK, FTP) were proven against
PX4 SITL, but a real radio link is noisier and adversarial: truncated payloads, wrong lengths, all-0xFF,
NaN/Inf floats, garbage content. The older parsers have fuzz tests; the new ones did not. This hammers
every new decode path + handler + panel and asserts: (1) BOTH parsers survive malformed frames for each
new msgid and stay in LOCKSTEP (native decode-count == python decode-count -- a split would desync a
real link); (2) a non-finite TERRAIN_REPORT is dropped end-to-end (handler keeps last-good, the panel
never renders 'inf'/'nan'); (3) ftp.decode survives truncated/garbage payloads."""
import os
import sys
import struct
import random

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import mavlink
import core
import ftp
from vehicle import Vehicle

fail = []
random.seed(1234567)                                 # deterministic (no Date/rand nondeterminism)

NEW_MSGS = [mavlink.GPS_RTK, mavlink.GPS2_RTK, mavlink.TERRAIN_REPORT, mavlink.CAMERA_SETTINGS,
            mavlink.CAMERA_IMAGE_CAPTURED, mavlink.CAMERA_FEEDBACK, mavlink.FILE_TRANSFER_PROTOCOL]


def build(msgid, payload):
    """Frame `payload` for `msgid` picking v1/v2 by id (msgid>255 requires v2)."""
    fn = mavlink.frame_v2 if msgid > 255 else mavlink.frame
    return fn(msgid, payload, random.randint(0, 255), 1, 1, crc_fn=core.crc_extra)


# 1) both parsers survive malformed frames for every new msgid, in lockstep --------------------------
print("native backend live:", core.NATIVE)
for msgid in NEW_MSGS:
    base = mavlink._WIRE[msgid][2]
    # a MAVLink payload is <= 255 bytes; framing beyond that is invalid input, not an app path
    lengths = sorted(n for n in {0, 1, base // 2, max(0, base - 1), base, base + 5, base + 60, 255} if n <= 255)
    variants = [b"\x00", b"\xff", None]              # None -> random bytes
    for ln in lengths:
        for v in variants:
            payload = bytes(random.randint(0, 255) for _ in range(ln)) if v is None else v * ln
            try:
                frame = build(msgid, payload)
            except Exception as e:
                fail.append(f"framing {mavlink.MSG_NAME.get(msgid)} len={ln} raised {type(e).__name__}: {e}")
                continue
            try:
                nat = core.Parser().feed(frame)
                pyp = mavlink.PyParser().feed(frame)
            except Exception as e:
                fail.append(f"decode {mavlink.MSG_NAME.get(msgid)} len={ln} raised {type(e).__name__}: {e}")
                continue
            if len(nat) != len(pyp):
                fail.append(f"lockstep split {mavlink.MSG_NAME.get(msgid)} len={ln}: native={len(nat)} py={len(pyp)}")
            # feeding garbage to the vehicle handlers must never raise
            try:
                Vehicle().consume(nat)
            except Exception as e:
                fail.append(f"handler {mavlink.MSG_NAME.get(msgid)} len={ln} raised {type(e).__name__}: {e}")

# 2) a non-finite TERRAIN_REPORT is dropped end-to-end (handler + panel) -----------------------------
from PySide6.QtWidgets import QApplication
from panels import TelemetryPanel
app = QApplication.instance() or QApplication([])
tp = TelemetryPanel()

ve = Vehicle()
good = struct.pack("<iiffHHH", 473980000, 85460000, 500.0, 120.0, 100, 0, 12)
ve.consume(core.Parser().feed(mavlink.frame(mavlink.TERRAIN_REPORT, good, 7, 1, 1, crc_fn=core.crc_extra)))
if abs((ve.terrain_agl_m or 0) - 120.0) > 1e-3:
    fail.append(f"good terrain report not stored: {ve.terrain_agl_m}")

for bad in (float("nan"), float("inf"), float("-inf")):
    pl = struct.pack("<iiffHHH", 473980000, 85460000, bad, bad, 100, 0, 12)
    ve.consume(core.Parser().feed(mavlink.frame(mavlink.TERRAIN_REPORT, pl, 7, 1, 1, crc_fn=core.crc_extra)))
    if ve.terrain_agl_m != 120.0:                    # must KEEP last-good, not overwrite with garbage
        fail.append(f"non-finite terrain ({bad}) overwrote last-good AGL -> {ve.terrain_agl_m}")
    tp.update_all(ve, "UDP", 10.0, 100, 0)
    t = tp.v["terrain"].text().lower()
    if "inf" in t or "nan" in t:
        fail.append(f"panel terrain rendered non-finite text: '{tp.v['terrain'].text()}'")

# a totally fresh vehicle fed ONLY a non-finite report shows '--', never a number ------------------
fresh = Vehicle()
pl = struct.pack("<iiffHHH", 0, 0, float("nan"), float("nan"), 0, 0, 0)
fresh.consume(core.Parser().feed(mavlink.frame(mavlink.TERRAIN_REPORT, pl, 7, 1, 1, crc_fn=core.crc_extra)))
tp.update_all(fresh, "UDP", 10.0, 100, 0)
if fresh.terrain_agl_m is not None or tp.v["terrain"].text() != "--":
    fail.append(f"fresh vehicle + only non-finite terrain should stay '--', got agl={fresh.terrain_agl_m} '{tp.v['terrain'].text()}'")

# 3) ftp.decode survives truncated / oversize / garbage payloads ------------------------------------
for ln in (0, 1, 5, 11, 12, 13, 100, 251, 300):
    for v in (b"\x00", b"\xff", None):
        p = bytes(random.randint(0, 255) for _ in range(ln)) if v is None else v * ln
        try:
            d = ftp.decode(p)
            if not isinstance(d, dict):
                fail.append(f"ftp.decode(len={ln}) returned {type(d).__name__}, want dict")
        except Exception as e:
            fail.append(f"ftp.decode(len={ln}) raised {type(e).__name__}: {e}")

print("NEWMSGFUZZ FAILED: " + "; ".join(fail[:8]) if fail else
      "NEWMSGFUZZ PASSED (both parsers survive malformed RTK/terrain/camera/feedback/FTP frames in "
      "lockstep + handlers never raise; non-finite TERRAIN_REPORT dropped end-to-end, panel never shows "
      "inf/nan; ftp.decode survives truncated/garbage payloads)")
sys.stdout.flush()
os._exit(1 if fail else 0)
