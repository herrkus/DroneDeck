#!/usr/bin/env python3
"""test_voice.py -- spoken audio alerts (iter167, QGC parity / real-flying aid).

QGC speaks critical events aloud so a pilot watching the aircraft (not the screen) still hears them.
VoiceAlerts does the same for arm/disarm + mode changes (and, wired from main, battery/failsafe).
Verifies the edge detection and the enable gate with an injected capture function -- no live speech
engine needed: OFF by default and while disabled nothing is spoken; the FIRST update for a vehicle only
records state (never announces the initial condition); arm/disarm and real mode changes are announced
once each; '--'/None modes are ignored; vehicles are tracked independently per sysid; reset() forgets
state so a reconnect doesn't replay stale edges."""
import os
import sys
import types

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
from voice import VoiceAlerts

fail = []


def V(sysid=1, armed=False, mode="STABILIZE"):
    return types.SimpleNamespace(sysid=sysid, armed=armed, mode=mode)


# 1) disabled by default -> nothing spoken even across real transitions ------------------------------
spoken = []
va = VoiceAlerts(speak=spoken.append)              # enabled defaults to False
va.update(V(armed=False))
va.update(V(armed=True))
if va.enabled or spoken:
    fail.append(f"disabled VoiceAlerts must stay silent, spoke {spoken}")
if not va.available:
    fail.append("available should be True when a speak function is injected")

# 2) enabled: first update records only; then arm/disarm + mode changes announce once each ----------
spoken = []
va = VoiceAlerts(speak=spoken.append, enabled=True)
va.update(V(armed=False, mode="STABILIZE"))        # first sight -> record, no announcement
if spoken:
    fail.append(f"first update should not announce, spoke {spoken}")
va.update(V(armed=True, mode="STABILIZE"))         # armed edge
va.update(V(armed=True, mode="RTL"))               # mode edge (armed unchanged)
va.update(V(armed=True, mode="RTL"))               # no change -> silent
va.update(V(armed=False, mode="RTL"))              # disarm edge (mode unchanged)
if spoken != ["Armed", "Mode RTL", "Disarmed"]:
    fail.append(f"edge announcements wrong: {spoken}")

# 3) placeholder / empty modes are never announced --------------------------------------------------
spoken = []
va = VoiceAlerts(speak=spoken.append, enabled=True)
va.update(V(mode="--"))
va.update(V(mode=None))
va.update(V(mode=""))
if spoken:
    fail.append(f"placeholder modes should not be announced, spoke {spoken}")

# 4) vehicles are independent per sysid -------------------------------------------------------------
spoken = []
va = VoiceAlerts(speak=spoken.append, enabled=True)
va.update(V(sysid=1, armed=False))
va.update(V(sysid=2, armed=False))                 # a different vehicle's first sight -> record only
va.update(V(sysid=1, armed=True))                  # only vehicle 1 armed
if spoken != ["Armed"]:
    fail.append(f"per-sysid edge tracking wrong: {spoken}")
# sysid 0 (unknown vehicle) is ignored entirely
before = list(spoken)
va.update(V(sysid=0, armed=True))
if spoken != before:
    fail.append("sysid 0 should be ignored")

# 5) set_enabled toggles the gate; reset() forgets state (reconnect doesn't replay) -----------------
spoken = []
va = VoiceAlerts(speak=spoken.append, enabled=True)
va.update(V(armed=True))                            # record armed=True (first sight)
va.set_enabled(False)
va.update(V(armed=False))                           # disarm edge but disabled -> silent
if spoken:
    fail.append(f"set_enabled(False) should mute, spoke {spoken}")
va.set_enabled(True)
va.reset()
va.update(V(armed=True))                            # after reset this is a first sight again -> silent
if spoken:
    fail.append(f"reset() should drop state so this is a first-sight (silent), spoke {spoken}")

print("VOICE FAILED: " + "; ".join(fail) if fail else
      "VOICE PASSED (off by default + silent while disabled; first update records only; arm/disarm + "
      "mode changes announced once each; '--'/None modes ignored; per-sysid independent; sysid 0 ignored; "
      "set_enabled mutes; reset() drops stale state)")
sys.stdout.flush()
os._exit(1 if fail else 0)
