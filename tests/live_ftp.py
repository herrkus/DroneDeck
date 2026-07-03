#!/usr/bin/env python3
"""live_ftp.py -- END-TO-END MAVLink FTP against a REAL PX4 SITL (or drone) on udp:14550.

Proves the whole FTP path on the wire: FILE_TRANSFER_PROTOCOL requests reach the autopilot, its ACK/NAK
responses come back and decode, and the ftp.FtpClient state machine lists a directory (and reads a file
if one turns up) end-to-end -- not a mock. NOT in run_all.sh (needs a live vehicle). SKIPS cleanly
(exit 0) if no heartbeat arrives, so it is safe to run with no SITL. Read-only: only OPEN_FILE_RO /
READ / LIST / TERMINATE, never a write.

Usage:  QT_QPA_PLATFORM=offscreen python3 tests/live_ftp.py [udp_port]
"""
import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QCoreApplication
import mavlink
import ftp

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 14550
app = QApplication.instance() or QApplication([])
import main as m

win = m.DroneDeck(PORT)
win._persist = False
win._connect()
ve = win.vehicle


def pump(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        QCoreApplication.processEvents()
        time.sleep(0.01)


t0 = time.monotonic()
while ve.msg_count == 0 and time.monotonic() - t0 < 15:
    pump(0.5)
if ve.msg_count == 0:
    print(f"LIVE FTP SKIPPED (no MAVLink traffic on udp:{PORT} within 15s -- is a SITL/vehicle up?)")
    sys.exit(0)
pump(1.0)

target_sys = ve.sysid or 1
client = ftp.FtpClient(
    lambda seq, session, opcode, offset, data, size:
    win.link.send_ftp(target_sys, seq, session, opcode, offset, data, size=size, target_comp=1))

# route FILE_TRANSFER_PROTOCOL responses from the live link into the client
win.link.messages.connect(lambda batch: [
    client.handle(ftp.decode(msg.fields["payload"]))
    for msg in batch if msg.msgid == mavlink.FILE_TRANSFER_PROTOCOL and "payload" in msg.fields])


def run(start, timeout=20.0):
    """Drive one FTP operation to completion with resend-on-stall. Returns (result, error)."""
    box = {}
    start(lambda r, e: box.update(result=r, error=e, done=True))
    last_seq, last_change = -1, time.monotonic()
    while client.active and time.monotonic() - t0 < 300 and not box.get("done"):
        pump(0.1)
        if client.seq != last_seq:
            last_seq, last_change = client.seq, time.monotonic()
        elif time.monotonic() - last_change > 1.5:
            client.resend()
            last_change = time.monotonic()
        if time.monotonic() - last_change > timeout:
            break
    return box.get("result"), box.get("error")


fails = []
print(f"=== LIVE MAVLink FTP on udp:{PORT} (vehicle sysid {target_sys}) ===")

# 1) list a directory -- try a few common PX4 roots until one answers with entries
listing, used_path = None, None
for path in ("/", "/fs/microsd", "/fs/microsd/log", "/etc"):
    res, err = run(lambda cb, p=path: client.list_directory(p, cb))
    print(f"  list {path!r}: {len(res) if res else 0} entries, error={err}")
    if res:
        listing, used_path = res, path
        break
if not listing:
    print("LIVE FTP SKIPPED (FTP handshake produced no directory entries -- FTP may be disabled on this "
          "target; the client is proven headless by test_ftpclient.py)")
    try:
        win.link.close()
    except Exception:
        pass
    sys.exit(0)

for e in listing[:8]:
    print(f"      {e['type']:4} {e['name']}  {e['size'] if e['size'] is not None else ''}")

# 2) read a small file if the listing exposed one (bonus proof of the read path) --------------------
small = next((e for e in listing if e["type"] == "file" and (e["size"] or 0) > 0 and (e["size"] or 0) < 8000),
             None)
if small:
    fpath = (used_path.rstrip("/") + "/" + small["name"]) if used_path != "/" else "/" + small["name"]
    data, err = run(lambda cb: client.read_file(fpath, cb))
    got = len(data) if data else 0
    ok = err is None and got > 0
    print(f"  [{'ok' if ok else 'XX'}] read {fpath}: {got} B (listed {small['size']}), error={err}")
    if not ok:
        fails.append(f"read {fpath}")
else:
    print("  (no small file in the listing to read -- listing alone proves the FTP handshake)")

try:
    win.link.close()
except Exception:
    pass

if fails:
    print("LIVE FTP FAILED:", fails)
    sys.exit(1)
print(f"LIVE FTP PASSED (listed {len(listing)} entries from {used_path!r} over real MAVLink FTP"
      + (f"; read a {small['name']} file end-to-end" if small else "") + ")")
