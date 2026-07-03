#!/usr/bin/env python3
"""test_ftpbrowser.py -- MAVLink FTP file-browser dialog (iter153).

Step 3 of MAVLink FTP: the FtpBrowserDialog that lets a user navigate the vehicle's filesystem and
download a file. Driven here through a FakeLink that answers requests from a fake PX4-style FS and emits
responses DEFERRED (QTimer.singleShot) so the real Qt event-loop path -- link.messages -> client.handle
-> next request -- is exercised, not a synchronous shortcut. Verifies: the dialog lists its start
directory and renders dirs+files (+ '..'); double-click navigates into a subdir and Up returns; a
download reads the file over FTP and writes the exact bytes to disk."""
import os
import sys
import struct
import tempfile
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QObject, Signal, QTimer, QCoreApplication, Qt
import mavlink
import ftp
from ftpbrowser import FtpBrowserDialog

fail = []
app = QApplication.instance() or QApplication([])


class Server:
    FS = {
        "/fs/microsd/log": [b"Flog001.ulg\t500", b"Dsess"],
        "/fs/microsd/log/sess": [b"Flog002.ulg\t110"],
    }
    FILES = {
        "/fs/microsd/log/log001.ulg": bytes((i * 3) & 0xFF for i in range(500)),
        "/fs/microsd/log/sess/log002.ulg": b"hello world " * 9 + b"!!",   # 110 B
    }

    def __init__(self):
        self._cur = None

    def respond(self, req):
        seq, session, opcode, offset, data, size = req
        rseq = (seq + 1) & 0xFFFF
        path = data.decode("utf-8", "replace")

        def ack(off, d):
            return dict(seq=rseq, session=session, opcode=ftp.OP_ACK, offset=off, data=d)

        def nak(err):
            return dict(seq=rseq, session=session, opcode=ftp.OP_NAK, offset=offset, data=bytes([err]))

        if opcode == ftp.OP_LIST_DIRECTORY:
            entries = self.FS.get(path)
            if entries is None:
                return nak(ftp.ERR_FILE_NOT_FOUND)
            return ack(offset, entries[offset] + b"\x00") if offset < len(entries) else nak(ftp.ERR_EOF)
        if opcode == ftp.OP_OPEN_FILE_RO:
            f = self.FILES.get(path)
            if f is None:
                return nak(ftp.ERR_FILE_NOT_FOUND)
            self._cur = f
            return dict(seq=rseq, session=9, opcode=ftp.OP_ACK, offset=0, data=struct.pack("<I", len(f)))
        if opcode == ftp.OP_READ_FILE:
            f = self._cur or b""
            return ack(offset, f[offset:offset + size]) if offset < len(f) else nak(ftp.ERR_EOF)
        if opcode == ftp.OP_TERMINATE_SESSION:
            return ack(0, b"")
        return nak(ftp.ERR_UNKNOWN_COMMAND)


class FakeLink(QObject):
    messages = Signal(list)

    def __init__(self, server):
        super().__init__()
        self.server = server

    def send_ftp(self, target_sys, seq, session, opcode, offset=0, data=b"", size=None, target_comp=1):
        data = bytes(data)[:239]
        req = (seq, session, opcode, offset, data, size if size is not None else len(data))
        resp = self.server.respond(req)
        if resp is not None:
            payload = ftp.encode(resp["seq"], resp["session"], resp["opcode"], resp["offset"], resp["data"])
            msg = mavlink.Message(mavlink.FILE_TRANSFER_PROTOCOL, target_sys, 1, 0, {"payload": payload})
            QTimer.singleShot(0, lambda: self.messages.emit([msg]))


def pump_until(cond, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        QCoreApplication.processEvents()
        if cond():
            return True
        time.sleep(0.005)
    return False


link = FakeLink(Server())
tmp = tempfile.mkdtemp(prefix="ftpbrowser_")
dlg = FtpBrowserDialog(lambda: link, lambda: 1, tmp)

# 1) start dir listed + rendered (dirs, files, and '..') --------------------------------------------
if not pump_until(lambda: not dlg._busy and dlg._path == "/fs/microsd/log"):
    fail.append(f"start listing did not complete (path={dlg._path} busy={dlg._busy})")
else:
    names = [dlg.list.item(i).text() for i in range(dlg.list.count())]
    if "[..]" not in names or "[sess]" not in names or not any("log001.ulg" in n for n in names):
        fail.append(f"listing render wrong: {names}")
    if not any("500 B" in n for n in names):
        fail.append(f"file size not shown: {names}")

# 2) download a file over FTP -> exact bytes on disk ------------------------------------------------
file_entry = next((e for e in dlg._entries if e["type"] == "file"), None)
if not file_entry:
    fail.append("no file entry to download")
else:
    save_path = os.path.join(tmp, "got.ulg")
    dlg._do_download(file_entry, save_path)
    if not pump_until(lambda: not dlg._busy and os.path.exists(save_path)):
        fail.append("download did not finish")
    else:
        got = open(save_path, "rb").read()
        if got != Server.FILES["/fs/microsd/log/log001.ulg"]:
            fail.append(f"downloaded bytes wrong: {len(got)} B, want 500")

# 3) navigate into a subdir, then back up -----------------------------------------------------------
dlg._list("/fs/microsd/log/sess")
if not pump_until(lambda: not dlg._busy and dlg._path == "/fs/microsd/log/sess"):
    fail.append("navigate into subdir failed")
elif not any("log002.ulg" in dlg.list.item(i).text() for i in range(dlg.list.count())):
    fail.append("subdir listing missing log002.ulg")
dlg._go_up()
if not pump_until(lambda: not dlg._busy and dlg._path == "/fs/microsd/log"):
    fail.append("go up failed")

# 4) path join / parent helpers ---------------------------------------------------------------------
if dlg._join("x.ulg") != "/fs/microsd/log/x.ulg" or dlg._parent() != "/fs/microsd":
    fail.append(f"path helpers wrong: join={dlg._join('x.ulg')} parent={dlg._parent()}")

print("FTPBROWSER FAILED: " + "; ".join(fail) if fail else
      "FTPBROWSER PASSED (dialog lists its start dir + renders dirs/files/'..' over the real event-loop "
      "path; download reads a file over FTP and writes exact bytes; subdir navigate + Up work)")
sys.stdout.flush()
os._exit(1 if fail else 0)
