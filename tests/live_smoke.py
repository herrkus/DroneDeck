#!/usr/bin/env python3
"""live_smoke.py -- END-TO-END verification against a REAL running autopilot (PX4 SITL or a real
drone) on udp:14550. Unlike the headless unit/adversarial tests, this proves the whole live path:
telemetry actually flows and is finite+sane, the instruments/map render real data without crashing,
and a parameter download round-trips from the vehicle (the send path + protocol, not a mock).

NOT part of run_all.sh -- it needs a live vehicle. Run it directly after `git pull`/changes to
confirm nothing regressed on the wire. If no heartbeat arrives within the timeout it SKIPS (exit 0)
so it is safe to run with no SITL up. Read-only: no arming, no mode change, no mission upload.

Usage:  QT_QPA_PLATFORM=offscreen python3 tests/live_smoke.py [udp_port]
"""
import os
import sys
import time
import math

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QCoreApplication

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


# wait up to 15s for the first MAVLink message; SKIP cleanly if nothing is transmitting
t0 = time.monotonic()
while ve.msg_count == 0 and time.monotonic() - t0 < 15:
    pump(0.5)
if ve.msg_count == 0:
    print(f"LIVE SMOKE SKIPPED (no MAVLink traffic on udp:{PORT} within 15s -- is a SITL/vehicle up?)")
    sys.exit(0)

pump(9)          # let position/attitude/battery/GPS establish

fails = []


def chk(name, cond, val):
    print(f"  [{'ok' if cond else 'XX'}] {name}: {val}")
    if not cond:
        fails.append(name)


print(f"=== LIVE TELEMETRY on udp:{PORT} ===")
chk("link open", bool(win.link and win.link.is_open), win.link and win.link.is_open)
age = time.monotonic() - ve.last_heartbeat
chk("heartbeat recent (<3s)", age < 3.0, f"{age:.2f}s")
chk("have_position + finite", ve.have_position and math.isfinite(ve.lat) and math.isfinite(ve.lon),
    f"{ve.lat:.6f},{ve.lon:.6f}")
chk("attitude finite", all(math.isfinite(x) for x in (ve.roll, ve.pitch, ve.heading)),
    f"roll={ve.roll:.3f} pitch={ve.pitch:.3f} hdg={ve.heading:.1f}")
chk("GPS fix>=3 + sats>0", ve.fix_type >= 3 and ve.satellites > 0, f"fix={ve.fix_type} sats={ve.satellites}")
chk("battery volt>0", ve.voltage > 0, f"{ve.voltage:.2f}V {ve.battery_remaining}%")
chk("mode decoded", bool(ve.mode) and ve.mode != "?", ve.mode)
stats = win.link.parser.stats if (win.link and win.link.parser) else (0, 0)
chk("parser ok>0", stats[0] > 0, f"ok={stats[0]} drop={stats[1]}")

# render with REAL data -- the NaN-safety fixes must not break the normal (finite) path
try:
    ve.last_heartbeat = time.monotonic()
    win._refresh()
    win.adi.grab(); win.compass.grab(); win.map.grab()
    print("  [ok] instruments + map render real data")
except Exception as e:
    fails.append("render")
    print(f"  [XX] render crashed: {type(e).__name__}: {e}")

# parameter download round-trip (live send path + protocol)
print("=== PARAM DOWNLOAD ROUND-TRIP ===")
win.params.download()
pump(12)
n = len(win.params.values)
chk("params downloaded (>100)", n > 100, n)
for pn in ("MAV_SYS_ID", "BAT1_N_CELLS", "MC_PITCH_P"):
    print(f"    {pn} = {win.params.values.get(pn)}")

try:
    win.link.close()
except Exception:
    pass

if fails:
    print("LIVE SMOKE FAILED:", fails)
    sys.exit(1)
print("LIVE SMOKE PASSED (real telemetry finite+sane, render ok, param round-trip ok)")
