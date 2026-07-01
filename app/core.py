"""core.py -- access to the MAVLink engine.

Prefers the native C++/assembly core (libdronecore.so) for the hot receive path
and the assembly CRC. If that library can't be loaded -- a different OS/arch, or
a checkout that hasn't been built -- it transparently falls back to the pure
Python implementation in mavlink.py, so the GUI runs anywhere PySide6 does.

`core.NATIVE` / `core.BACKEND` report which path is live.
"""
from __future__ import annotations
import ctypes
import os
import sys

import mavlink
from mavlink import Message            # unified message type for both paths

_HERE = os.path.dirname(os.path.abspath(__file__))
_LIB = os.path.normpath(os.path.join(_HERE, "..", "core", "libdronecore.so"))


class Decoded(ctypes.Structure):
    # Must match `struct Decoded` in core/dronecore.cpp byte-for-byte.
    _fields_ = [
        ("msgid", ctypes.c_uint32),
        ("sysid", ctypes.c_uint8),
        ("compid", ctypes.c_uint8),
        ("seq", ctypes.c_uint8),
        ("nfields", ctypes.c_uint8),
        ("f", ctypes.c_double * 24),
        ("text", ctypes.c_ubyte * 96),       # NUL-terminated string, or LOG_DATA blob
    ]


def _load():
    if os.environ.get("DRONEDECK_FORCE_PYTHON"):
        return None
    if not os.path.exists(_LIB):
        return None
    try:
        lib = ctypes.CDLL(_LIB)
    except OSError:
        return None
    u8p = ctypes.POINTER(ctypes.c_uint8)
    lib.mav_new.restype = ctypes.c_void_p
    lib.mav_free.argtypes = [ctypes.c_void_p]
    lib.mav_feed.argtypes = [ctypes.c_void_p, u8p, ctypes.c_size_t]
    lib.mav_feed.restype = ctypes.c_int
    lib.mav_pop.argtypes = [ctypes.c_void_p, ctypes.POINTER(Decoded)]
    lib.mav_pop.restype = ctypes.c_int
    lib.mav_count_ok.argtypes = [ctypes.c_void_p]
    lib.mav_count_ok.restype = ctypes.c_ulong
    lib.mav_count_drop.argtypes = [ctypes.c_void_p]
    lib.mav_count_drop.restype = ctypes.c_ulong
    lib.mav_count_frames.argtypes = [ctypes.c_void_p]
    lib.mav_count_frames.restype = ctypes.c_ulong
    lib.mav_count_lost.argtypes = [ctypes.c_void_p]
    lib.mav_count_lost.restype = ctypes.c_ulong
    lib.mav_crc.argtypes = [u8p, ctypes.c_size_t]
    lib.mav_crc.restype = ctypes.c_uint16
    lib.mav_crc_extra.argtypes = [u8p, ctypes.c_size_t, ctypes.c_uint8]
    lib.mav_crc_extra.restype = ctypes.c_uint16
    lib.mav_encode_heartbeat.argtypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8, u8p, ctypes.c_int]
    lib.mav_encode_heartbeat.restype = ctypes.c_int
    lib.mav_encode_command_long.argtypes = [
        ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8,
        ctypes.c_uint16, ctypes.POINTER(ctypes.c_float), u8p, ctypes.c_int]
    lib.mav_encode_command_long.restype = ctypes.c_int
    lib.mav_encode_command_int.argtypes = [
        ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8,
        ctypes.c_uint8, ctypes.c_uint16, ctypes.POINTER(ctypes.c_float),
        ctypes.c_int32, ctypes.c_int32, ctypes.c_float, u8p, ctypes.c_int]
    lib.mav_encode_command_int.restype = ctypes.c_int
    return lib


_lib = _load()
NATIVE = _lib is not None
BACKEND = "x86-64 asm slice-by-8 + C++ parser" if NATIVE else "pure-Python fallback"
if not NATIVE:
    print("[core] native libdronecore.so not loaded -- using pure-Python fallback "
          "(run ./build.sh for the optimized core)", file=sys.stderr)


class Parser:
    """Streaming parser; native when available, else pure Python."""

    def __init__(self):
        if NATIVE:
            self._p = _lib.mav_new()
            self._py = None
        else:
            self._p = None
            self._py = mavlink.PyParser()

    def __del__(self):
        try:
            if self._p:
                _lib.mav_free(self._p)
                self._p = None
        except Exception:
            pass

    def feed(self, data: bytes) -> list:
        if not data:
            return []
        if self._py is not None:
            return self._py.feed(data)
        buf = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
        n = _lib.mav_feed(self._p, buf, len(data))
        out = []
        d = Decoded()
        for _ in range(n):
            if not _lib.mav_pop(self._p, ctypes.byref(d)):
                break
            names = mavlink.FIELDS.get(d.msgid, [])
            fields = {names[i]: d.f[i] for i in range(min(d.nfields, len(names)))}
            if d.msgid == mavlink.STATUSTEXT:
                fields["text"] = bytes(d.text).split(b"\x00")[0].decode("utf-8", "replace")
            elif d.msgid in (mavlink.PARAM_VALUE, mavlink.PARAM_SET, mavlink.PARAM_REQUEST_READ):
                fields["param_id"] = bytes(d.text).split(b"\x00")[0].decode("utf-8", "replace")
            elif d.msgid in (mavlink.LOG_DATA, mavlink.SERIAL_CONTROL):
                fields["data"] = bytes(d.text)[:int(fields.get("count", 0))]
            elif d.msgid == mavlink.ADSB_VEHICLE:
                fields["callsign"] = bytes(d.text).split(b"\x00")[0].decode("utf-8", "replace")
            out.append(Message(d.msgid, d.sysid, d.compid, d.seq, fields))
        return out

    @property
    def stats(self) -> tuple[int, int]:
        if self._py is not None:
            return self._py.ok, self._py.drop
        return int(_lib.mav_count_ok(self._p)), int(_lib.mav_count_drop(self._p))

    @property
    def loss(self) -> float:
        """Rolling link packet-loss percentage from MAVLink seq gaps (0.0 on a clean link)."""
        if self._py is not None:
            f, l = self._py.frames, self._py.lost
        else:
            f = int(_lib.mav_count_frames(self._p))
            l = int(_lib.mav_count_lost(self._p))
        return (100.0 * l / (f + l)) if (f + l) else 0.0


# --- CRC + encoders (native, with pure-Python fallback) ----------------------
def crc(data: bytes) -> int:
    if NATIVE:
        buf = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
        return int(_lib.mav_crc(buf, len(data)))
    return mavlink.crc16(data)


def crc_extra(data: bytes, extra: int) -> int:
    if NATIVE:
        buf = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
        return int(_lib.mav_crc_extra(buf, len(data), extra & 0xFF))
    return mavlink.crc16_mcrf4xx(data, extra)


def encode_heartbeat(sysid: int, compid: int, seq: int) -> bytes:
    if NATIVE:
        out = (ctypes.c_uint8 * 64)()
        n = _lib.mav_encode_heartbeat(sysid, compid, seq & 0xFF, out, 64)
        return bytes(out[:n]) if n > 0 else b""
    pl = mavlink.enc_heartbeat(mav_type=6, autopilot=8, base_mode=0,
                               custom_mode=0, system_status=4)
    return mavlink.frame(mavlink.HEARTBEAT, pl, seq, sysid, compid)


def encode_command_long(sysid, compid, seq, tgt_sys, tgt_comp, command, params7) -> bytes:
    if NATIVE:
        arr = (ctypes.c_float * 7)(*[float(x) for x in (list(params7) + [0] * 7)[:7]])
        out = (ctypes.c_uint8 * 64)()
        n = _lib.mav_encode_command_long(sysid, compid, seq & 0xFF, tgt_sys, tgt_comp,
                                         command, arr, out, 64)
        return bytes(out[:n]) if n > 0 else b""
    pl = mavlink.enc_command_long(command, params7, tgt_sys, tgt_comp)
    return mavlink.frame(mavlink.COMMAND_LONG, pl, seq, sysid, compid)


def encode_command_int(sysid, compid, seq, tgt_sys, tgt_comp, frame, command,
                       params4, x, y, z) -> bytes:
    if NATIVE:
        arr = (ctypes.c_float * 4)(*[float(v) for v in (list(params4) + [0] * 4)[:4]])
        out = (ctypes.c_uint8 * 64)()
        n = _lib.mav_encode_command_int(sysid, compid, seq & 0xFF, tgt_sys, tgt_comp,
                                        frame, command, arr, int(x), int(y), float(z), out, 64)
        return bytes(out[:n]) if n > 0 else b""
    pl = mavlink.enc_command_int(command, params4, x, y, z, tgt_sys, tgt_comp, frame)
    return mavlink.frame(mavlink.COMMAND_INT, pl, seq, sysid, compid)
