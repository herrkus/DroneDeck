"""tlog.py -- telemetry log recording and reading.

The on-disk format matches MAVProxy / QGroundControl `.tlog` files: each record
is an 8-byte big-endian timestamp in MICROSECONDS since the Unix epoch, followed
by one raw MAVLink frame. So a DroneDeck recording opens in those tools too.
"""
from __future__ import annotations
import struct

import mavlink


class TlogWriter:
    """Accumulates received bytes, splits them into frames, and writes each frame
    prefixed with the supplied timestamp."""

    def __init__(self, path):
        self.path = path
        self._f = open(path, "wb")
        self._buf = bytearray()
        self.count = 0

    def write(self, data, t_us):
        self._buf += data
        frames, leftover = mavlink.split_frames(self._buf)
        for fr in frames:
            self._f.write(struct.pack(">Q", t_us & 0xFFFFFFFFFFFFFFFF))
            self._f.write(fr)
            self.count += 1
        self._buf = bytearray(leftover[-2048:])     # bound junk/partial growth
        if frames:
            self._f.flush()

    def close(self):
        try:
            self._f.close()
        except OSError:
            pass


def replay_schedule(records, max_gap_us=10_000_000):
    """Virtual playback offsets (microseconds from start) derived from a .tlog's raw timestamps,
    made monotonic with each inter-frame gap clamped to [0, max_gap_us]. Recorded timestamps can be
    non-monotonic or corrupt (a truncated/edited log can hold a backwards jump or an absurd value
    like 1e18 us ~= 31000 years); scheduling playback directly off `t_us - t0` would then let a
    single bad frame never come 'due', stalling replay forever. Clamping makes a bad timestamp cost
    at most one `max_gap_us` hiccup while preserving the original cadence of well-formed logs."""
    sched = []
    virt = 0
    prev = records[0][0] if records else 0
    for t_us, _fr in records:
        dt = t_us - prev
        if dt < 0 or dt > max_gap_us:
            dt = 0 if dt < 0 else max_gap_us
        virt += dt
        sched.append(virt)
        prev = t_us
    return sched


# Cap the in-memory parse. A .tlog grows ~unbounded with flight time; f.read() on a multi-GB log
# would exhaust RAM and freeze the GUI thread. 512 MB is hours of dense telemetry -- far beyond any
# real session -- and anything larger is parsed only up to this prefix (last whole frame within it).
MAX_TLOG_BYTES = 512 * 1024 * 1024


def read_tlog(path, max_bytes=MAX_TLOG_BYTES):
    """Parse a .tlog into a list of (timestamp_us, frame_bytes), in file order.

    Reads at most max_bytes so a runaway/huge log can't OOM or freeze the caller. If the file is
    larger, sets read_tlog.truncated = True and parses only the prefix (up to the last whole frame).
    """
    read_tlog.truncated = False
    with open(path, "rb") as f:
        data = f.read(max_bytes + 1)                 # one extra byte reveals an over-cap file
    if len(data) > max_bytes:
        read_tlog.truncated = True
        data = data[:max_bytes]
    records = []
    n = len(data)
    i = 0
    while i + 8 <= n:
        t_us = int.from_bytes(data[i:i + 8], "big")
        j = i + 8
        if j >= n or data[j] not in (0xFE, 0xFD):
            break                                    # not a frame boundary -> stop
        total = mavlink.frame_total(data, j)
        if total is None or j + total > n:
            break                                    # frame runs past the (capped) data -> stop clean
        records.append((t_us, bytes(data[j:j + total])))
        i = j + total
    return records


read_tlog.truncated = False
