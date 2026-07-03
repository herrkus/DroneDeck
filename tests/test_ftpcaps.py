#!/usr/bin/env python3
"""test_ftpcaps.py -- FTP client resource caps + O(n) buffering (iter163, real-drone hardening).

MAVLink FTP is driven by the autopilot's responses. A rogue/buggy FC (or a corrupt filesize field) could
stream a 'file' without end or list a directory forever -- unbounded, that exhausts GCS memory. And the
read buffer was `bytes` so `buffer += chunk` was O(n) per chunk => O(n^2) for a big file (a real 64 MB
log took minutes). Fixes: buffer is a bytearray (in-place extend, O(n) total), a read aborts past
MAX_READ and a listing past MAX_ENTRIES -- both finish with an EXPLICIT error, never silently. Verifies:
a never-ending read/list terminate at the caps with an error; a normal read still completes and returns
bytes (not bytearray); the O(n^2) regression is gone (a large read completes fast)."""
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import ftp

fail = []


def open_read(c, filesize_bytes=b""):
    c.read_file("/f")
    c.handle({"seq": c._expect, "opcode": ftp.OP_ACK, "session": 1, "data": filesize_bytes})


# 1) a never-ending read aborts at MAX_READ with an explicit error ----------------------------------
c = ftp.FtpClient(lambda *a: None)
c.MAX_READ = 10000                                   # small override for a fast, deterministic test
open_read(c)                                         # no filesize -> would read forever
guard = 0
while not c.done and guard < 100000:
    c.handle({"seq": c._expect, "opcode": ftp.OP_ACK, "session": 1, "data": b"\x00" * 239})
    guard += 1
if not c.done or not c.error or "limit" not in c.error:
    fail.append(f"runaway read not capped: done={c.done} err={c.error!r}")
if len(c.buffer) > c.MAX_READ + 239:                 # bounded (at most one chunk over)
    fail.append(f"read buffer overran cap: {len(c.buffer)} > {c.MAX_READ}+chunk")

# 2) a never-ending listing aborts at MAX_ENTRIES with an explicit error -----------------------------
c2 = ftp.FtpClient(lambda *a: None)
c2.MAX_ENTRIES = 50
c2.list_directory("/d")
guard = 0
while not c2.done and guard < 100000:
    c2.handle({"seq": c2._expect, "opcode": ftp.OP_ACK, "data": b"F" + b"file\t10" + b"\x00"})
    guard += 1
if not c2.done or not c2.error or "truncated" not in c2.error:
    fail.append(f"runaway listing not capped: done={c2.done} err={c2.error!r}")
if len(c2.entries) > c2.MAX_ENTRIES + 5:
    fail.append(f"listing overran cap: {len(c2.entries)} > {c2.MAX_ENTRIES}")

# 3) a normal small read still completes and returns bytes (not bytearray) ---------------------------
c3 = ftp.FtpClient(lambda *a: None)
open_read(c3, filesize_bytes=(6).to_bytes(4, "little"))   # filesize = 6 bytes
c3.handle({"seq": c3._expect, "opcode": ftp.OP_ACK, "session": 1, "data": b"ABCDEF"})
if not c3.done or c3.error:
    fail.append(f"normal read did not complete cleanly: done={c3.done} err={c3.error!r}")
if c3.result != b"ABCDEF":
    fail.append(f"normal read data wrong: {c3.result!r}")
if not isinstance(c3.result, bytes) or isinstance(c3.result, bytearray):
    fail.append(f"read result must be immutable bytes, got {type(c3.result).__name__}")

# 4) the O(n^2) regression is gone: a big read (up to the real 64 MB cap) completes fast -------------
c4 = ftp.FtpClient(lambda *a: None)                  # real MAX_READ (64 MB)
open_read(c4)
t0 = time.time()
guard = 0
while not c4.done and guard < 400000:
    c4.handle({"seq": c4._expect, "opcode": ftp.OP_ACK, "session": 1, "data": b"\x00" * 239})
    guard += 1
dt = time.time() - t0
if not c4.done:
    fail.append("64 MB runaway read never terminated")
if dt > 10.0:                                        # O(n^2) would take minutes; O(n) is well under a second
    fail.append(f"large read too slow ({dt:.1f}s) -- O(n^2) buffering regressed")

print("FTPCAPS FAILED: " + "; ".join(fail) if fail else
      f"FTPCAPS PASSED (runaway read capped at MAX_READ + listing at MAX_ENTRIES, both with explicit "
      f"errors; normal read returns immutable bytes; 64 MB read O(n) in {dt:.2f}s, no O(n^2) regression)")
sys.stdout.flush()
os._exit(1 if fail else 0)
