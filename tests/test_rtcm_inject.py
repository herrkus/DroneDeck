#!/usr/bin/env python3
"""test_rtcm_inject.py -- RTK correction injection GPS_RTCM_DATA(233) (iter148, QGC parity).

Completes RTK support (iter147 parsed RTK status; this sends the corrections). QGC streams RTCM3 from
an NTRIP caster / base-station receiver to the vehicle as GPS_RTCM_DATA, fragmenting messages >=180 B
into up to 4 fragments. link.inject_rtcm replicates that framing exactly: the flags byte carries a
fragmented bit (LSB), a 2-bit fragment id (bits 1-2) and a 5-bit per-message sequence id (bits 3-7);
data[] is 180 B zero-padded with a valid-length field. Verifies: a short message -> one unfragmented
frame; a long one -> correctly-ordered fragments that reassemble to the original bytes; the sequence id
increments once per whole message (not per fragment) and wraps at 32; empty input sends nothing; and a
message over the 4-fragment (720 B) protocol cap is truncated, not silently corrupted."""
import os
import sys
import struct

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import mavlink
import core
from link import Link

fail = []


class CaptureLink(Link):
    def __init__(self):
        super().__init__()
        self.sent = []
        self._open = True
        self.remote = True

    def _write(self, data):
        self.sent.append(data)


def decode_rtcm(frame):
    """Pull (msgid, fragmented, fragment_id, seq, length, data) out of a v1 GPS_RTCM_DATA frame."""
    assert frame[0] == 0xFE, "expected a v1 frame"
    plen = frame[1]
    msgid = frame[5]
    payload = frame[6:6 + plen]
    flags, length = payload[0], payload[1]
    data = payload[2:2 + length]
    return msgid, (flags & 0x01), (flags >> 1) & 0x03, (flags >> 3) & 0x1F, length, bytes(data)


# confirm each emitted frame is a complete v1 frame AND its trailing CRC matches a fresh core.crc_extra
# recomputation over len+seq+sysid+compid+msgid+payload with the GPS_RTCM_DATA seed (=35)
def crc_ok(frame):
    if mavlink.frame_total(frame, 0) != len(frame):
        return False
    plen = frame[1]
    want = core.crc_extra(frame[1:6 + plen], mavlink.CRC_EXTRA[mavlink.GPS_RTCM_DATA])
    got = struct.unpack("<H", frame[6 + plen:6 + plen + 2])[0]
    return want == got


# 1) short message (< 180 B) -> a single unfragmented frame, seq 0 ----------------------------------
lk = CaptureLink()
msg = bytes((i * 7) & 0xFF for i in range(50))
n = lk.inject_rtcm(msg)
if n != 1 or len(lk.sent) != 1:
    fail.append(f"short msg should send 1 frame, got n={n} sent={len(lk.sent)}")
else:
    mid, frag, fid, seq, length, data = decode_rtcm(lk.sent[0])
    if mid != mavlink.GPS_RTCM_DATA or frag != 0 or fid != 0 or seq != 0 or length != 50 or data != msg:
        fail.append(f"short frame wrong: mid={mid} frag={frag} fid={fid} seq={seq} len={length} data_ok={data == msg}")
    if not crc_ok(lk.sent[0]):
        fail.append("short frame failed structural/CRC check")

# 2) long message (400 B) -> 3 fragments (180,180,40), same seq (=1 now), reassembling to original ---
big = bytes((i * 3 + 1) & 0xFF for i in range(400))
before = len(lk.sent)
n = lk.inject_rtcm(big)
frames = lk.sent[before:]
if n != 3 or len(frames) != 3:
    fail.append(f"400 B should send 3 fragments, got n={n} frames={len(frames)}")
else:
    reassembled = b""
    for expect_fid, fr in enumerate(frames):
        mid, frag, fid, seq, length, data = decode_rtcm(fr)
        if frag != 1 or fid != expect_fid or seq != 1:
            fail.append(f"frag {expect_fid}: fragmented={frag} fid={fid} seq={seq} (want 1/{expect_fid}/1)")
        if not crc_ok(fr):
            fail.append(f"frag {expect_fid} failed structural/CRC check")
        reassembled += data
    if reassembled != big:
        fail.append(f"reassembled {len(reassembled)} B != original 400 B")
    lens = [decode_rtcm(f)[4] for f in frames]
    if lens != [180, 180, 40]:
        fail.append(f"fragment lengths {lens}, want [180,180,40]")

# 3) sequence id increments once per message (0 then 1) and wraps at 32 ------------------------------
lk2 = CaptureLink()
for _ in range(33):                                    # 33 messages -> seq 0..31,0
    lk2.inject_rtcm(b"\x01\x02\x03")
seqs = [decode_rtcm(f)[3] for f in lk2.sent]
if seqs[:3] != [0, 1, 2] or seqs[31] != 31 or seqs[32] != 0:
    fail.append(f"seq should count 0..31 then wrap to 0: got [..{seqs[30]},{seqs[31]},{seqs[32]}]")

# 4) empty input sends nothing and does not burn a sequence id --------------------------------------
lk3 = CaptureLink()
if lk3.inject_rtcm(b"") != 0 or lk3.inject_rtcm(None) != 0 or lk3.sent:
    fail.append("empty RTCM should send 0 frames")
if lk3._rtcm_seq != 0:
    fail.append(f"empty RTCM must not advance seq, got {lk3._rtcm_seq}")

# 5) oversize (> 720 B) is capped at 4 fragments and logs, not silently mangled ----------------------
lk4 = CaptureLink()
huge = bytes(range(256)) * 4                            # 1024 B -> 6 fragments, capped to 4
n = lk4.inject_rtcm(huge)
if n != 4 or len(lk4.sent) != 4:
    fail.append(f"1024 B should cap at 4 fragments, got n={n} sent={len(lk4.sent)}")
else:
    carried = b"".join(decode_rtcm(f)[5] for f in lk4.sent)
    if carried != huge[:720]:
        fail.append("capped injection should carry exactly the first 720 B in order")

print("RTCM_INJECT FAILED: " + "; ".join(fail) if fail else
      "RTCM_INJECT PASSED (short->1 unfragmented frame; 400 B->3 ordered fragments reassembling to the "
      "original; seq increments per-message + wraps at 32; empty sends nothing; >720 B capped at 4 "
      "fragments; every frame CRC-valid)")
sys.exit(1 if fail else 0)
