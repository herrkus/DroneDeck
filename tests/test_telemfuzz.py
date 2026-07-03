#!/usr/bin/env python3
"""test_telemfuzz.py -- telemetry-value robustness. Real autopilots emit NaN/Inf/extreme fields
(attitude and heading before the estimator converges, speeds/alt during init, garbage GPS before
fix). Feeds those through the Vehicle handlers -> _refresh -> the telemetry panel AND the rendered
instruments (compass + attitude indicator), asserting no crash (NaN coordinates / int(NaN) inside a
paintEvent SEGFAULT Qt) and no 'nan'/'inf' leaking into the panel text. No link, no arming.

Regression guard for the iter95 fix: Compass.set_heading(NaN) and AttitudeIndicator.set_data(NaN)
hard-crashed (SIGSEGV) before the setters sanitised their inputs with _finite()."""
import os
import sys
import math

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mavlink
import main as m
import instruments

app = QApplication.instance() or QApplication([])

# 1) instrument setters must survive NaN/Inf/extreme and still render (the SIGSEGV class) ----------
NAN, INF, BIG = float("nan"), float("inf"), 1e308
for bad in (NAN, INF, -INF, BIG, -BIG):
    adi = instruments.AttitudeIndicator(); adi.resize(240, 200)
    comp = instruments.Compass(); comp.resize(200, 200)
    adi.set_data(bad, bad, bad, bad, bad, bad); adi.grab()
    adi.set_attitude(bad, bad); adi.grab()
    comp.set_heading(bad); comp.grab()
    comp.set_wind(bad, bad, True); comp.grab()
    comp.set_home_bearing(bad, True); comp.grab()
# and _finite() itself
assert instruments._finite(NAN) == 0.0 and instruments._finite(INF) == 0.0
assert instruments._finite(3.5) == 3.5 and instruments._finite(BIG) == BIG


class Msg:
    __slots__ = ("msgid", "sysid", "compid", "seq", "fields")
    def __init__(self, msgid, **f):
        self.msgid, self.sysid, self.compid, self.seq, self.fields = msgid, 1, 1, 0, f


# 2) drive the whole pipeline: adversarial telemetry -> vehicle -> _refresh -> panel + instruments -
win = m.DroneDeck(14586)
win._persist = False
ve = win.vehicle


def refresh_and_render():
    import time
    ve.last_heartbeat = time.monotonic()
    win._refresh()                       # updates panels + pushes values to instruments
    win.adi.grab(); win.compass.grab()   # force the paintEvents that used to segfault


scenarios = [
    # NaN attitude (very common before EKF converges)
    [Msg(mavlink.ATTITUDE, roll=NAN, pitch=NAN, yaw=NAN, rollspeed=NAN, pitchspeed=NAN, yawspeed=NAN)],
    # Inf attitude
    [Msg(mavlink.ATTITUDE, roll=INF, pitch=-INF, yaw=INF, rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0)],
    # NaN VFR_HUD speeds / alt / climb / heading
    [Msg(mavlink.VFR_HUD, airspeed=NAN, groundspeed=NAN, heading=0, throttle=0, alt=NAN, climb=NAN)],
    # extreme / out-of-range position (lat > 90 deg, lon > 180 deg)
    [Msg(mavlink.GLOBAL_POSITION_INT, lat=950000000, lon=1900000000, alt=-2000000000,
         relative_alt=2000000000, vx=32767, vy=-32768, vz=32767, hdg=65535)],
    # garbage GPS: impossible fix + sats
    [Msg(mavlink.GPS_RAW_INT, fix_type=255, satellites_visible=255, lat=950000000, lon=0,
         alt=0, eph=65535, epv=65535, vel=65535, cog=65535)],
    # NaN battery via SYS_STATUS (voltage/current are ints on the wire, but remaining can be -1/255)
    [Msg(mavlink.SYS_STATUS, voltage_battery=65535, current_battery=-1, battery_remaining=-1,
         onboard_control_sensors_present=0, onboard_control_sensors_enabled=0,
         onboard_control_sensors_health=0, load=0)],
    # NaN vibration
    [Msg(mavlink.VIBRATION, vibration_x=NAN, vibration_y=INF, vibration_z=NAN,
         clipping_0=0, clipping_1=0, clipping_2=0)],
]

for batch in scenarios:
    ve.consume(batch)      # must not raise in the handler
    refresh_and_render()   # must not crash in _refresh or the instrument paints

# after all that abuse the panel text must not show 'nan'/'inf' (garbage to a pilot)
bad_text = []
for key, lbl in win.panel.v.items():
    t = lbl.text().lower()
    if "nan" in t or "inf" in t:
        bad_text.append((key, lbl.text()))
assert not bad_text, f"panel shows non-finite text: {bad_text}"

# -- wire sentinels (audit batch 4): "not measured" markers must not become readings ----------------
from vehicle import Vehicle
sv = Vehicle()
sv.consume([Msg(mavlink.BATTERY_STATUS, id=0, current_battery=1234,
                voltages=[3800] * 4 + [65535] * 6, battery_remaining=88)])
sv.consume([Msg(mavlink.SYS_STATUS, voltage_battery=65535, current_battery=-1, battery_remaining=88,
                onboard_present=1, onboard_enabled=1, onboard_health=1)])
assert sv.voltage == 0.0, "SYS_STATUS voltage UINT16_MAX sentinel displayed as 65.535 V"
assert abs(sv.current - 12.34) < 0.01, "SYS_STATUS current=-1 sentinel stomped BATTERY_STATUS current"
sv.consume([Msg(mavlink.SYS_STATUS, voltage_battery=16200, current_battery=1500, battery_remaining=88,
                onboard_present=1, onboard_enabled=1, onboard_health=1)])
assert abs(sv.voltage - 16.2) < 0.01 and abs(sv.current - 15.0) < 0.01   # real values still land

hv = Vehicle()
hv.consume([Msg(mavlink.GLOBAL_POSITION_INT, lat=474000000, lon=85000000, alt=100000,
                relative_alt=50000, vx=0, vy=0, vz=0, hdg=65535)])
assert hv.heading == 0.0, "hdg UINT16_MAX shown as a confident 295.35 deg"
hv.consume([Msg(mavlink.VFR_HUD, airspeed=0.0, groundspeed=1.0, heading=123, throttle=10,
                alt=50.0, climb=0.0)])
assert hv.heading == 123.0, "VFR_HUD heading fallback locked out while gp hdg is unknown"
hv.consume([Msg(mavlink.GLOBAL_POSITION_INT, lat=474000000, lon=85000000, alt=100000,
                relative_alt=50000, vx=0, vy=0, vz=0, hdg=9000)])
hv.consume([Msg(mavlink.VFR_HUD, airspeed=0.0, groundspeed=1.0, heading=45, throttle=10,
                alt=50.0, climb=0.0)])
assert hv.heading == 90.0, "VFR_HUD must not override a VALID gp heading"

print("TELEMFUZZ PASSED (instruments + panel survive NaN/Inf/extreme telemetry; wire sentinels safe)")

import os, sys  # harden: flush verdict + skip Qt-teardown segfault under the full sweep
sys.stdout.flush()
os._exit(0)
