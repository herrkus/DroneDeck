"""_ports.py -- test helper: hand out an OS-assigned free UDP port.

The GUI/integration tests used to hardcode udp:14550, which is exactly the port a live PX4 SITL or a
running DroneDeck GUI uses -- so running the suite while a drone/SITL was connected produced spurious
failures (the test's socket bind collided with the live one). Asking the OS for a free port (bind to
0, read it back, release) is guaranteed unused at call time -- strictly better than a fixed number or
$RANDOM (which can still collide) -- and lets the whole suite run green alongside a real drone.

The bind-then-release reuse window is negligible for the serial test runner: the port is handed to a
single test process that binds it immediately.
"""
import socket


def free_udp_port():
    """Return a currently-free UDP port on the loopback interface."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]
    finally:
        s.close()
