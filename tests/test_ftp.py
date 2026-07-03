#!/usr/bin/env python3
"""test_ftp.py -- MAVLink FTP wire + codec foundation FILE_TRANSFER_PROTOCOL(110) (iter151).

MAVLink FTP is how PX4/ArduPilot expose their filesystem (log download, param files). Step 1 lays the
foundation: carry FILE_TRANSFER_PROTOCOL through BOTH parsers (the native Decoded.text buffer was grown
96->256 to hold the 251-byte FTP payload; selftest mirror + core.py ctypes struct kept in lockstep),
plus an ftp.py packet codec (12-byte header + up to 239 data bytes) and link.send_ftp. Verifies: the
message round-trips native==python with the full 251-byte payload intact; ftp.encode/decode is exact;
link.send_ftp emits a valid, CRC-correct frame whose payload decodes back to the request; ACK/NAK
opcodes and NAK error codes are read correctly."""
import os
import sys
import struct

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import mavlink
import core
import ftp
from link import Link

fail = []

# 1) FILE_TRANSFER_PROTOCOL round-trips native==python with the whole 251-byte payload --------------
ftp_pkt = ftp.encode(seq=1, session=0, opcode=ftp.OP_LIST_DIRECTORY, offset=0, data=b"/fs/microsd")
if len(ftp_pkt) != 251:
    fail.append(f"ftp.encode should be 251 bytes, got {len(ftp_pkt)}")
payload = bytes((0, 42, 1)) + ftp_pkt                         # target_network 0, sys 42, comp 1
frame = mavlink.frame_v2(mavlink.FILE_TRANSFER_PROTOCOL, payload, 7, 42, 1, crc_fn=core.crc_extra)
nat, pyp = core.Parser().feed(frame), mavlink.PyParser().feed(frame)
print("native backend live:", core.NATIVE)
if len(nat) != 1 or len(pyp) != 1:
    fail.append(f"FTP decode count native={len(nat)} py={len(pyp)}")
else:
    for k in ("target_network", "target_system", "target_component", "payload"):
        if nat[0].fields.get(k) != pyp[0].fields.get(k):
            fail.append(f"FTP.{k}: native={nat[0].fields.get(k)!r} py={pyp[0].fields.get(k)!r}")
    if nat[0].fields.get("payload") != ftp_pkt:
        fail.append("FTP payload did not survive native decode intact")
    if nat[0].fields.get("target_system") != 42:
        fail.append(f"FTP target_system {nat[0].fields.get('target_system')}, want 42")

# 2) ftp.encode/decode is exact (header fields + data) ----------------------------------------------
dec = ftp.decode(ftp_pkt)
if (dec["seq"] != 1 or dec["session"] != 0 or dec["opcode"] != ftp.OP_LIST_DIRECTORY
        or dec["offset"] != 0 or dec["size"] != len(b"/fs/microsd") or dec["data"] != b"/fs/microsd"):
    fail.append(f"ftp.decode mismatch: {dec}")

read_pkt = ftp.encode(seq=5, session=3, opcode=ftp.OP_READ_FILE, offset=8192, data=b"")
rdec = ftp.decode(read_pkt)
if rdec["seq"] != 5 or rdec["session"] != 3 or rdec["offset"] != 8192 or rdec["size"] != 0:
    fail.append(f"ftp read request decode mismatch: {rdec}")

# 3) link.send_ftp emits a valid frame whose payload decodes back to the request --------------------
class CaptureLink(Link):
    def __init__(self):
        super().__init__()
        self.sent = []
        self._open = True
        self.remote = True

    def _write(self, data):
        self.sent.append(data)


lk = CaptureLink()
lk.send_ftp(target_sys=42, seq=7, session=0, opcode=ftp.OP_OPEN_FILE_RO, offset=0, data=b"/log/1.ulg")
if len(lk.sent) != 1:
    fail.append(f"send_ftp should emit 1 frame, got {len(lk.sent)}")
else:
    fr = lk.sent[0]
    msgs = core.Parser().feed(fr)
    if len(msgs) != 1 or msgs[0].msgid != mavlink.FILE_TRANSFER_PROTOCOL:
        fail.append(f"send_ftp frame did not decode as FILE_TRANSFER_PROTOCOL: {msgs}")
    else:
        pkt = ftp.decode(msgs[0].fields["payload"])
        if (pkt["opcode"] != ftp.OP_OPEN_FILE_RO or pkt["seq"] != 7 or pkt["data"] != b"/log/1.ulg"
                or msgs[0].fields["target_system"] != 42):
            fail.append(f"send_ftp payload wrong: {pkt} target={msgs[0].fields.get('target_system')}")

# 4) ACK / NAK opcodes + NAK error code read correctly ----------------------------------------------
ack = ftp.decode(ftp.encode(seq=8, session=0, opcode=ftp.OP_ACK, data=b"\x00" * 4))
if not ftp.is_ack(ack) or ftp.nak_error(ack) is not None:
    fail.append(f"ACK misread: is_ack={ftp.is_ack(ack)} nak_error={ftp.nak_error(ack)}")
nak = ftp.decode(ftp.encode(seq=9, session=0, opcode=ftp.OP_NAK, data=bytes([ftp.ERR_FILE_NOT_FOUND])))
if ftp.is_ack(nak) or ftp.nak_error(nak) != ftp.ERR_FILE_NOT_FOUND:
    fail.append(f"NAK misread: is_ack={ftp.is_ack(nak)} nak_error={ftp.nak_error(nak)}")
eof = ftp.decode(ftp.encode(seq=10, session=0, opcode=ftp.OP_NAK, data=bytes([ftp.ERR_EOF])))
if ftp.nak_error(eof) != ftp.ERR_EOF:
    fail.append(f"EOF NAK misread: {ftp.nak_error(eof)}")

print("FTP FAILED: " + "; ".join(fail) if fail else
      "FTP PASSED (FILE_TRANSFER_PROTOCOL round-trips native==python with the full 251-byte payload; "
      "ftp.encode/decode exact; link.send_ftp emits a CRC-valid frame decoding back to the request; "
      "ACK/NAK + NAK error codes read correctly)")
sys.stdout.flush()
os._exit(1 if fail else 0)
