#!/usr/bin/env python3
"""test_links.py -- exercise the TCP and serial transports end to end.

TCP: stand up a local MAVLink server, connect the real TcpLink to it, pump the
Qt loop and confirm frames decode through the native parser. Serial: open a
pseudo-terminal, point SerialLink at the slave end, feed frames from the master
end. The serial leg is best-effort (some platforms won't drive a pty through
QSerialPort); TCP is the hard gate.
"""
import os
import sys
import socket
import threading
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "app"))

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer

import mavlink
from link import TcpLink, SerialLink, HAVE_SERIAL


def make_stream(n):
    """n heartbeat+attitude frame pairs as one byte blob."""
    blob = b""
    for i in range(n):
        blob += mavlink.frame(mavlink.HEARTBEAT, mavlink.enc_heartbeat(2, 0x81, 5),
                              i & 0xFF, 1, 1, crc_fn=mavlink.crc16_mcrf4xx)
        blob += mavlink.frame(mavlink.ATTITUDE, mavlink.enc_attitude(0.1, -0.2, 1.5, i),
                              i & 0xFF, 1, 1, crc_fn=mavlink.crc16_mcrf4xx)
    return blob


app = QApplication([])
results = {}


# ---- TCP -------------------------------------------------------------------
srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", 0))
srv.listen(1)
tcp_port = srv.getsockname()[1]


def serve():
    try:
        conn, _ = srv.accept()
    except OSError:
        return
    for i in range(60):
        try:
            conn.sendall(make_stream(1))
        except OSError:
            break
        time.sleep(0.02)
    try:
        conn.close()
    except OSError:
        pass


threading.Thread(target=serve, daemon=True).start()

tcp = TcpLink()
tcp_got = {"n": 0, "names": set()}
tcp.messages.connect(lambda b: (tcp_got.__setitem__("n", tcp_got["n"] + len(b)),
                                [tcp_got["names"].add(m.name) for m in b]))
tcp.open(host="127.0.0.1", port=tcp_port)
QTimer.singleShot(1500, app.quit)
app.exec()
tcp.close()
srv.close()
results["tcp"] = (tcp_got["n"] > 30 and {"HEARTBEAT", "ATTITUDE"} <= tcp_got["names"], tcp_got)


# ---- Serial over a pty (best effort) ---------------------------------------
serial_note = "skipped"
serial_ok = None
if HAVE_SERIAL and hasattr(os, "openpty"):
    import pty
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    sl = SerialLink()
    if sl.open(port=slave_name, baud=57600):
        ser_got = {"n": 0, "names": set()}
        sl.messages.connect(lambda b: (ser_got.__setitem__("n", ser_got["n"] + len(b)),
                                       [ser_got["names"].add(m.name) for m in b]))

        def feed():
            try:
                os.write(master, make_stream(20))
            except OSError:
                pass

        QTimer.singleShot(150, feed)
        QTimer.singleShot(1200, app.quit)
        app.exec()
        sl.close()
        serial_ok = ser_got["n"] > 10 and {"HEARTBEAT", "ATTITUDE"} <= ser_got["names"]
        serial_note = f"{ser_got['n']} msgs, types {sorted(ser_got['names'])}"
    else:
        serial_note = "pty not drivable by QSerialPort here (needs real port)"
    try:
        os.close(master)
        os.close(slave)
    except OSError:
        pass

# ---- report ----------------------------------------------------------------
tcp_ok, tcp_got = results["tcp"]
print(f"TCP   : {tcp_got['n']} msgs, types {sorted(tcp_got['names'])} -> {'OK' if tcp_ok else 'FAIL'}")
print(f"Serial: {serial_note} -> {'OK' if serial_ok else ('FAIL' if serial_ok is False else 'n/a')}")

# TCP is the gate; serial only fails the suite if it opened but didn't deliver.
hard_fail = (not tcp_ok) or (serial_ok is False)
print("LINKS FAILED" if hard_fail else "LINKS PASSED")
sys.exit(1 if hard_fail else 0)
