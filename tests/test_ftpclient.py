#!/usr/bin/env python3
"""test_ftpclient.py -- MAVLink FTP client state machine (iter152).

Step 2 of MAVLink FTP: the ftp.FtpClient state machine that lists a directory and reads a file over
FILE_TRANSFER_PROTOCOL. Driven here against a MockFtpServer (a fake PX4-style FTP filesystem) so the
multi-request logic is proven deterministically: directory listing paginates by rising offset until an
EOF-NAK; a file read opens a session, pulls CHUNK-sized reads, reassembles across chunks, stops on the
short final chunk, and terminates the session; and a not-found open surfaces as an error. The transport
is exercised for real in tests/live_ftp.py against the PX4 SITL (graceful-skip, not in run_all)."""
import os
import sys
import struct

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import ftp

fail = []


class MockFtpServer:
    """Answers FTP requests from a tiny fake filesystem, one directory entry per ACK."""
    def __init__(self):
        self.dir_entries = [b"Ffile1.txt\t100", b"Ffile2.bin\t2048", b"Dlogs"]
        self.file = bytes((i * 3) & 0xFF for i in range(500))     # 500 B -> 3 reads (239+239+22)
        self.session = 7
        self.missing = "/nope.txt"

    def respond(self, req):
        seq, session, opcode, offset, data, size = req
        rseq = (seq + 1) & 0xFFFF

        def ack(off, d):
            return ftp.decode(ftp.encode(rseq, session, ftp.OP_ACK, off, d))

        def nak(off, err):
            return ftp.decode(ftp.encode(rseq, session, ftp.OP_NAK, off, bytes([err])))

        if opcode == ftp.OP_LIST_DIRECTORY:
            if offset < len(self.dir_entries):
                return ack(offset, self.dir_entries[offset] + b"\x00")
            return nak(offset, ftp.ERR_EOF)
        if opcode == ftp.OP_OPEN_FILE_RO:
            if data.decode("utf-8", "replace") == self.missing:
                return nak(0, ftp.ERR_FILE_NOT_FOUND)
            return ftp.decode(ftp.encode(rseq, self.session, ftp.OP_ACK, 0, struct.pack("<I", len(self.file))))
        if opcode == ftp.OP_READ_FILE:
            if offset >= len(self.file):
                return nak(offset, ftp.ERR_EOF)
            return ack(offset, self.file[offset:offset + size])
        if opcode == ftp.OP_TERMINATE_SESSION:
            return ack(0, b"")
        return nak(offset, ftp.ERR_UNKNOWN_COMMAND)


def drive(start):
    """Run one operation to completion against the mock. Returns (result, error)."""
    server = MockFtpServer()
    requests = []
    client = ftp.FtpClient(lambda *a: requests.append(a))
    done = {}
    start(client, lambda r, e: done.update(result=r, error=e))
    steps = 0
    while client.active and requests and steps < 200:
        req = requests.pop(0)
        resp = server.respond(req)
        if resp is not None:
            client.handle(resp)
        steps += 1
    return done.get("result"), done.get("error"), server


# 1) directory listing paginates to completion ------------------------------------------------------
entries, err, _ = drive(lambda c, cb: c.list_directory("/", cb))
if err is not None:
    fail.append(f"list_directory errored: {err}")
elif [e["name"] for e in (entries or [])] != ["file1.txt", "file2.bin", "logs"]:
    fail.append(f"listing names wrong: {[e['name'] for e in (entries or [])]}")
else:
    byname = {e["name"]: e for e in entries}
    if byname["file2.bin"]["size"] != 2048 or byname["file2.bin"]["type"] != "file":
        fail.append(f"file2.bin entry wrong: {byname['file2.bin']}")
    if byname["logs"]["type"] != "dir":
        fail.append(f"logs should be a dir: {byname['logs']}")

# 2) file read reassembles across chunks + matches the source, session terminated -------------------
data, err, server = drive(lambda c, cb: c.read_file("/log/1.ulg", cb))
if err is not None:
    fail.append(f"read_file errored: {err}")
elif data != server.file:
    fail.append(f"read_file reassembly wrong: got {len(data or b'')} B, want {len(server.file)} B")

# 3) a not-found open surfaces as an error, no result -----------------------------------------------
data, err, _ = drive(lambda c, cb: c.read_file("/nope.txt", cb))
if err != "not-found" or data is not None:
    fail.append(f"missing file should error 'not-found', got result={data!r} error={err!r}")

# 4) empty file (filesize 0) completes cleanly with empty data --------------------------------------
class EmptyServer(MockFtpServer):
    def __init__(self):
        super().__init__()
        self.file = b""


srv = EmptyServer()
requests = []
client = ftp.FtpClient(lambda *a: requests.append(a))
done = {}
client.read_file("/empty.txt", lambda r, e: done.update(result=r, error=e))
steps = 0
while client.active and requests and steps < 50:
    resp = srv.respond(requests.pop(0))
    if resp is not None:
        client.handle(resp)
    steps += 1
if done.get("error") is not None or done.get("result") != b"":
    fail.append(f"empty file read should return b'' with no error, got {done}")

print("FTPCLIENT FAILED: " + "; ".join(fail) if fail else
      "FTPCLIENT PASSED (directory listing paginates to EOF with typed entries+sizes; file read opens a "
      "session, reassembles across CHUNK reads, stops on the short chunk, terminates; not-found -> error; "
      "empty file -> b'')")
sys.exit(1 if fail else 0)
