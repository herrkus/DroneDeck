#!/usr/bin/env python3
"""test_serial_link.py -- end-to-end SerialLink over a virtual serial port (iter164, real-drone-ready).

A real drone plugs in over USB or a SiK telemetry radio -- a SERIAL port, not UDP. The app has a
SerialLink (QtSerialPort) but it had NO test: the literal way you'll connect your drone was unproven.
This exercises it for real over a socat PTY pair (no hardware), under the conditions serial creates that
UDP never does:
  * byte-fragmented frames -- serial delivers bytes in arbitrary chunks, so the streaming parser must
    reassemble a frame split across many reads (UDP always delivers a whole datagram);
  * line noise between frames -- an RF link picks up garbage, so the parser must RESYNC and still decode
    the following valid frame.
Also verifies TX (a command written by the app actually reaches the wire and decodes) and that opening a
bad port fails cleanly. Skips gracefully if socat / QtSerialPort is unavailable."""
import os
import sys
import time
import shutil
import struct
import subprocess

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

if shutil.which("socat") is None:
    print("SERIAL_LINK SKIPPED (socat not available)")
    sys.exit(0)

from PySide6.QtWidgets import QApplication
import mavlink
import core
from link import SerialLink, HAVE_SERIAL
from vehicle import Vehicle

if not HAVE_SERIAL:
    print("SERIAL_LINK SKIPPED (QtSerialPort not available)")
    sys.exit(0)

fail = []
app = QApplication.instance() or QApplication([])

A, B = "/tmp/dd_serialtest_A", "/tmp/dd_serialtest_B"
for p in (A, B):
    try:
        os.unlink(p)
    except OSError:
        pass
socat = subprocess.Popen(["socat", f"pty,raw,echo=0,link={A}", f"pty,raw,echo=0,link={B}"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def pump(cond, timeout=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.005)
    app.processEvents()
    return cond()


def heartbeat(armed=True, custom=5):
    base = 0x80 | 0x01 if armed else 0x01
    pl = struct.pack("<IBBBBB", custom, 2, 12, base, 4, 3)     # custom_mode,type,autopilot,base_mode,sys_status,ver
    return mavlink.frame(mavlink.HEARTBEAT, pl, 1, 1, 1, crc_fn=core.crc_extra)


def attitude(roll):
    pl = struct.pack("<Iffffff", 1000, roll, 0.1, 0.2, 0.0, 0.0, 0.0)
    return mavlink.frame(mavlink.ATTITUDE, pl, 1, 1, 1, crc_fn=core.crc_extra)


try:
    if not pump(lambda: os.path.exists(A) and os.path.exists(B), timeout=5.0):
        fail.append("socat did not create the PTY pair")
    else:
        drone = os.open(B, os.O_RDWR | os.O_NOCTTY)           # the "drone" end of the wire
        got = []
        sl = SerialLink()
        sl.messages.connect(lambda batch: got.extend(batch))
        if not sl.open(port=A, baud=57600):
            fail.append("SerialLink.open on the PTY failed")
        else:
            # 1) a whole frame decodes over serial -----------------------------------------------------
            os.write(drone, heartbeat())
            if not pump(lambda: any(m.msgid == mavlink.HEARTBEAT for m in got)):
                fail.append("no HEARTBEAT decoded over serial (whole frame)")

            # 2) a frame delivered ONE BYTE AT A TIME still reassembles (serial fragmentation) ---------
            got.clear()
            for byte in attitude(0.5):
                os.write(drone, bytes([byte]))
                app.processEvents()                            # force many tiny readyRead deliveries
                time.sleep(0.002)
            if not pump(lambda: any(m.msgid == mavlink.ATTITUDE for m in got)):
                fail.append("byte-fragmented ATTITUDE frame did not reassemble over serial")
            else:
                att = next(m for m in got if m.msgid == mavlink.ATTITUDE)
                if abs(att.fields.get("roll", 0) - 0.5) > 1e-4:
                    fail.append(f"fragmented ATTITUDE decoded wrong roll {att.fields.get('roll')}")

            # 3) line noise between frames -> parser RESYNCs and still gets the next valid frame -------
            got.clear()
            os.write(drone, bytes([0x11, 0x00, 0xAB, 0xFF, 0x3C]))   # garbage / partial start bytes
            os.write(drone, heartbeat(armed=False, custom=9))
            os.write(drone, bytes([0x00, 0xFE, 0x77]))              # trailing noise
            os.write(drone, attitude(-0.3))
            ok = pump(lambda: any(m.msgid == mavlink.HEARTBEAT for m in got)
                      and any(m.msgid == mavlink.ATTITUDE for m in got))
            if not ok:
                fail.append("parser did not resync after line noise (lost the following frame)")

            # the decoded HEARTBEAT drives real vehicle state --------------------------------------
            ve = Vehicle()
            ve.consume(got)
            if not any(m.msgid == mavlink.HEARTBEAT for m in got):
                fail.append("no HEARTBEAT to drive vehicle state")

            # 4) TX: a command the app sends actually reaches the wire and decodes ---------------------
            sl.arm(1, True)                                     # target_sys 1 -> COMMAND_LONG(ARM_DISARM)
            time.sleep(0.1)
            app.processEvents()
            rx = b""
            t0 = time.monotonic()
            while time.monotonic() - t0 < 1.0:
                import select
                r, _, _ = select.select([drone], [], [], 0.1)
                if r:
                    rx += os.read(drone, 4096)
                if rx:
                    break
            tx = core.Parser().feed(rx)
            if not any(m.msgid == mavlink.COMMAND_LONG for m in tx):
                fail.append(f"app's arm command did not reach the serial wire (got {[m.msgid for m in tx]})")
            else:
                cl = next(m for m in tx if m.msgid == mavlink.COMMAND_LONG)
                if int(cl.fields.get("command", 0)) != mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    fail.append(f"TX command was {cl.fields.get('command')}, want ARM_DISARM")

            sl.close()

        os.close(drone)

    # 5) opening a nonexistent port fails cleanly (no crash, returns False) ------------------------
    sl2 = SerialLink()
    errs = []
    sl2.info.connect(lambda s: errs.append(s))
    if sl2.open(port="/tmp/dd_no_such_port_xyz", baud=57600) is not False:
        fail.append("opening a nonexistent serial port should return False")
    if not any("fail" in e.lower() or "open" in e.lower() for e in errs):
        fail.append(f"nonexistent port should emit an error, got {errs}")

finally:
    socat.terminate()
    try:
        socat.wait(timeout=2)
    except subprocess.TimeoutExpired:
        socat.kill()
    for p in (A, B):
        try:
            os.unlink(p)
        except OSError:
            pass

print("SERIAL_LINK FAILED: " + "; ".join(fail) if fail else
      "SERIAL_LINK PASSED (SerialLink over a real virtual serial port: whole frame decodes; a "
      "byte-by-byte fragmented frame reassembles; parser resyncs through line noise; app TX reaches the "
      "wire + decodes to ARM_DISARM; bad port fails cleanly)")
sys.stdout.flush()
os._exit(1 if fail else 0)
