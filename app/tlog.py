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


def read_tlog(path):
    """Parse a .tlog into a list of (timestamp_us, frame_bytes), in file order."""
    with open(path, "rb") as f:
        data = f.read()
    records = []
    n = len(data)
    i = 0
    while i + 8 <= n:
        t_us = int.from_bytes(data[i:i + 8], "big")
        j = i + 8
        if j >= n or data[j] not in (0xFE, 0xFD):
            break                                    # not a frame boundary -> stop
        total = mavlink.frame_total(data, j)
        if total is None:
            break
        records.append((t_us, bytes(data[j:j + total])))
        i = j + total
    return records
