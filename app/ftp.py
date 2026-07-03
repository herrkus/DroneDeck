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


def encode(seq, session, opcode, offset=0, data=b"", req_opcode=0, burst_complete=0):
    """Build a 251-byte FTP packet. `data` is truncated to 239 bytes; `size` is set from its length."""
    data = bytes(data)[:DATA_MAX]
    hdr = struct.pack(_HDR, seq & 0xFFFF, session & 0xFF, opcode & 0xFF, len(data) & 0xFF,
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


def is_ack(pkt):
    return pkt.get("opcode") == OP_ACK


def nak_error(pkt):
    """Return the NAK error code, or None if the packet is not a NAK."""
    if pkt.get("opcode") != OP_NAK:
        return None
    d = pkt.get("data") or b""
    return d[0] if d else ERR_NONE
