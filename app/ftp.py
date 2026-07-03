"""ftp.py -- MAVLink FTP (FILE_TRANSFER_PROTOCOL) packet codec + opcodes.

MAVLink FTP is how PX4 (and ArduPilot) expose their filesystem to a GCS -- log download, parameter
files, mission/geofence files. Each FILE_TRANSFER_PROTOCOL(110) message carries a fixed 251-byte FTP
packet: a 12-byte header (seq_number, session, opcode, size, req_opcode, burst_complete, padding,
offset) followed by up to 239 data bytes. This module only encodes request packets and decodes response
packets; the stateful client that opens sessions, retries, and reassembles reads is layered on top
(the link sends what this builds and feeds back what the parser decodes).
"""
import struct

# request opcodes
OP_NONE = 0
OP_TERMINATE_SESSION = 1
OP_RESET_SESSIONS = 2
OP_LIST_DIRECTORY = 3
OP_OPEN_FILE_RO = 4
OP_READ_FILE = 5
OP_CREATE_FILE = 6
OP_WRITE_FILE = 7
OP_REMOVE_FILE = 8
OP_CREATE_DIRECTORY = 9
OP_REMOVE_DIRECTORY = 10
OP_OPEN_FILE_WO = 11
OP_TRUNCATE_FILE = 12
OP_RENAME = 13
OP_CALC_FILE_CRC32 = 14
OP_BURST_READ_FILE = 15
# response opcodes
OP_ACK = 128
OP_NAK = 129

# NAK error codes (data[0] of a NAK)
ERR_NONE = 0
ERR_FAIL = 1
ERR_FAIL_ERRNO = 2          # data[1] carries the errno
ERR_INVALID_DATA_SIZE = 3
ERR_INVALID_SESSION = 4
ERR_NO_SESSIONS_AVAILABLE = 5
ERR_EOF = 6                 # normal end of a directory listing / file read
ERR_UNKNOWN_COMMAND = 7
ERR_FILE_EXISTS = 8
ERR_FILE_PROTECTED = 9
ERR_FILE_NOT_FOUND = 10

ERR_NAMES = {
    ERR_NONE: "none", ERR_FAIL: "fail", ERR_FAIL_ERRNO: "errno", ERR_INVALID_DATA_SIZE: "bad-size",
    ERR_INVALID_SESSION: "bad-session", ERR_NO_SESSIONS_AVAILABLE: "no-sessions", ERR_EOF: "eof",
    ERR_UNKNOWN_COMMAND: "unknown-cmd", ERR_FILE_EXISTS: "exists", ERR_FILE_PROTECTED: "protected",
    ERR_FILE_NOT_FOUND: "not-found",
}

_HDR = "<HBBBBBBI"          # seq, session, opcode, size, req_opcode, burst_complete, padding, offset
HDR_LEN = 12
DATA_MAX = 239
PKT_LEN = 251               # fixed FTP packet size on the wire (12-byte header + 239 data)


def encode(seq, session, opcode, offset=0, data=b"", req_opcode=0, burst_complete=0, size=None):
    """Build a 251-byte FTP packet. `data` is truncated to 239 bytes. `size` defaults to len(data)
    (write/list/open) but a reader passes it explicitly to request that many bytes with empty data."""
    data = bytes(data)[:DATA_MAX]
    if size is None:
        size = len(data)
    hdr = struct.pack(_HDR, seq & 0xFFFF, session & 0xFF, opcode & 0xFF, size & 0xFF,
                      req_opcode & 0xFF, burst_complete & 0xFF, 0, offset & 0xFFFFFFFF)
    return hdr + data + b"\x00" * (DATA_MAX - len(data))       # 12 + 239 = 251


def decode(payload):
    """Parse a 251-byte FTP packet (zero-padded if short) into a dict; `data` is the valid `size` bytes."""
    payload = bytes(payload)[:PKT_LEN]
    if len(payload) < HDR_LEN:
        payload = payload + b"\x00" * (HDR_LEN - len(payload))
    seq, session, opcode, size, req_opcode, burst, _pad, offset = struct.unpack(_HDR, payload[:HDR_LEN])
    return {"seq": seq, "session": session, "opcode": opcode, "size": size, "req_opcode": req_opcode,
            "burst_complete": burst, "offset": offset, "data": payload[HDR_LEN:HDR_LEN + size]}


class FtpClient:
    """Minimal MAVLink FTP client state machine: list a directory or read a file. Transport-agnostic --
    it calls send(seq, session, opcode, offset, data, size) to emit a request and is driven by handle()
    with each decoded FTP response packet; completion fires on_complete(result, error). One operation at
    a time. Reads pull CHUNK bytes per request until a short chunk / filesize / EOF-NAK; directory
    listings request rising offsets until an empty ACK or EOF-NAK."""
    CHUNK = DATA_MAX     # request up to 239 data bytes per read
    MAX_READ = 64 * 1024 * 1024   # abort a read past 64 MB -- a rogue/buggy autopilot (or a corrupt
                                  # filesize) must not exhaust GCS memory; real config/param files are tiny
    MAX_ENTRIES = 16384           # abort a listing past 16k entries -- a misbehaving FC must not loop forever

    def __init__(self, send):
        self._send = send
        self.seq = 0             # monotonic across operations so a stale response can't match a new one
        self._expect = -1        # seq the current outstanding request expects back (req seq + 1)
        self.reset()

    def reset(self):
        self._expect = -1        # NB: self.seq is intentionally NOT reset (keeps responses unambiguous)
        self.session = 0
        self.op = None
        self.path = ""
        self.offset = 0
        self.buffer = bytearray()    # bytearray, not bytes: += is in-place O(chunk), so a big read is O(n) not O(n^2)
        self.entries = []
        self.filesize = None
        self.done = False
        self.error = None
        self.result = None
        self.on_complete = None
        self._last = None
        self._opening = False

    @property
    def active(self):
        return self.op is not None and not self.done

    def _issue(self, opcode, offset=0, data=b"", size=None, session=None):
        s = self.session if session is None else session
        self._last = (opcode, offset, bytes(data), size, s)
        self._expect = (self.seq + 1) & 0xFFFF          # the ACK/NAK for THIS request
        self._send(self.seq, s, opcode, offset, bytes(data), size)
        self.seq = (self.seq + 1) & 0xFFFF

    def resend(self):
        """Re-issue the last request (call on a response timeout); reads are idempotent per offset."""
        if self._last and not self.done:
            opcode, offset, data, size, s = self._last
            self._expect = (self.seq + 1) & 0xFFFF      # now expect the resend's response
            self._send(self.seq, s, opcode, offset, data, size)
            self.seq = (self.seq + 1) & 0xFFFF

    def list_directory(self, path, on_complete=None):
        self.reset()
        self.op, self.path, self.on_complete = "list", path, on_complete
        self._issue(OP_LIST_DIRECTORY, offset=0, data=path.encode("utf-8"))

    def read_file(self, path, on_complete=None):
        self.reset()
        self.op, self.path, self.on_complete, self._opening = "read", path, on_complete, True
        self._issue(OP_OPEN_FILE_RO, offset=0, data=path.encode("utf-8"))

    def _finish(self, result=None, error=None):
        self.done, self.result, self.error = True, result, error
        if self.on_complete:
            self.on_complete(result, error)

    def _terminate(self):
        if self.session:
            self._issue(OP_TERMINATE_SESSION, session=self.session)

    def handle(self, pkt):
        if not self.active:
            return
        if pkt.get("seq") != self._expect:      # stale/duplicate/foreign response -> ignore
            return
        op = pkt.get("opcode")
        if op == OP_NAK:
            err = nak_error(pkt)
            if err == ERR_EOF:                       # normal end of listing / read
                if self.op == "list":
                    self._finish(self.entries)
                else:
                    data = bytes(self.buffer)
                    self._terminate()
                    self._finish(data)
            else:
                self._finish(error=ERR_NAMES.get(err, f"nak {err}"))
            return
        if op != OP_ACK:
            return
        if self.op == "list":
            self._list_ack(pkt)
        else:
            self._read_ack(pkt)

    def _list_ack(self, pkt):
        n = 0
        for rec in (pkt.get("data") or b"").split(b"\x00"):
            if not rec:
                continue
            n += 1
            t = chr(rec[0])
            rest = rec[1:].decode("utf-8", "replace")
            if t == "F":
                name, _, sz = rest.partition("\t")
                try:
                    size = int(sz)
                except ValueError:
                    size = None
                self.entries.append({"type": "file", "name": name, "size": size})
            elif t == "D":
                self.entries.append({"type": "dir", "name": rest, "size": None})
            # 'S' (skip) or anything else: still counted toward the offset, not listed
        if n == 0:
            self._finish(self.entries)               # empty ACK -> end of listing
        elif len(self.entries) > self.MAX_ENTRIES:   # misbehaving FC listing without end
            self._finish(self.entries, error=f"listing truncated at {self.MAX_ENTRIES} entries")
        else:
            self.offset += n
            self._issue(OP_LIST_DIRECTORY, offset=self.offset, data=self.path.encode("utf-8"))

    def _read_ack(self, pkt):
        if self._opening:                            # this ACK answered OPEN_FILE_RO
            self._opening = False
            self.session = pkt.get("session", 0)
            d = pkt.get("data") or b""
            self.filesize = struct.unpack("<I", d[:4])[0] if len(d) >= 4 else None
            self.offset, self.buffer = 0, bytearray()
            self._issue(OP_READ_FILE, offset=0, data=b"", size=self.CHUNK, session=self.session)
            return
        chunk = pkt.get("data") or b""
        self.buffer += chunk
        self.offset += len(chunk)
        if len(self.buffer) > self.MAX_READ:             # rogue/buggy FC streaming without end
            self._terminate()
            self._finish(error=f"file exceeds {self.MAX_READ} byte limit (aborted at {len(self.buffer)} bytes)")
            return
        if (not chunk) or (self.filesize is not None and self.offset >= self.filesize):
            data = bytes(self.buffer)
            self._terminate()
            self._finish(data)
        else:
            self._issue(OP_READ_FILE, offset=self.offset, data=b"", size=self.CHUNK, session=self.session)


def is_ack(pkt):
    return pkt.get("opcode") == OP_ACK


def nak_error(pkt):
    """Return the NAK error code, or None if the packet is not a NAK."""
    if pkt.get("opcode") != OP_NAK:
        return None
    d = pkt.get("data") or b""
    return d[0] if d else ERR_NONE
