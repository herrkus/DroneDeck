#!/usr/bin/env python3
"""test_settingsload.py -- startup settings/config load robustness. QSettings values live in a
user-editable file (~/.config) that can be corrupted or hand-edited; load_settings() runs at startup,
so an uncaught error there means the app won't LAUNCH. Verifies that every persisted key survives
garbage input (invalid JSON, wrong type, non-finite, junk strings) without crashing, and that a
persisted 'nan'/'inf' never seeds a non-finite takeoff altitude or map centre (same class as the
iter99/100 file-load fixes).

Uses a throwaway .ini via QSettings(path, IniFormat) -- the user's real ~/.config/DroneDeck is
NEVER touched. No link, no arming."""
import os
import sys
import math
import tempfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QSettings
import main as m

app = QApplication.instance() or QApplication([])

win = m.DroneDeck(14671)
win._persist = False
_tmpfiles = []


def load_with(values):
    """Point win at a fresh throwaway .ini holding `values`, then run the real load_settings()."""
    path = tempfile.mkstemp(suffix=".ini")[1]
    _tmpfiles.append(path)
    s = QSettings(path, QSettings.IniFormat)
    for k, v in values.items():
        s.setValue(k, v)
    s.sync()
    win.settings = s
    win.load_settings()          # must never raise
    return win


def assert_sane():
    assert isinstance(win._takeoff_alt, float) and math.isfinite(win._takeoff_alt), win._takeoff_alt
    assert all(math.isfinite(c) for c in win.map.center), win.map.center


# 1) every key corrupted at once -- startup must survive ------------------------------------------
load_with({
    "win/geometry": b"\x00\x01\x02not-real-geometry",
    "win/state": b"\xff\xfe garbage",
    "link/transport": "NOPE",
    "link/target": 12345,
    "map/lat": "nan", "map/lon": "inf", "map/zoom": "abc", "map/provider": "Nonsense",
    "map/follow": "garbage",
    "flight/takeoff_alt": "nan",
    "links/configs": "{not valid json",
    "telem/hidden": "[unclosed",
})
assert_sane()

# 2) non-finite takeoff altitude -> default 25.0 (never NaN/Inf into the takeoff dialog) -----------
for bad in ("nan", "inf", "-inf", "notanumber"):
    load_with({"flight/takeoff_alt": bad})
    assert win._takeoff_alt == 25.0, f"takeoff_alt {bad!r} -> {win._takeoff_alt}"
load_with({"flight/takeoff_alt": "42.5"})
assert win._takeoff_alt == 42.5                      # a valid value is still honoured

# 3) non-finite persisted map centre must not be applied (stays finite) ---------------------------
before = win.map.center
load_with({"map/lat": "nan", "map/lon": "nan"})
assert all(math.isfinite(c) for c in win.map.center)

# 4) corrupt links/configs JSON -> falls back to [] (not a crash, not a partial object) -----------
load_with({"links/configs": "{broken"})
assert win.link_configs == []
# a COMPLETE config (name+transport+target, exactly what LinkEditDialog emits) still parses
load_with({"links/configs": '[{"name":"real","transport":"UDP","target":"127.0.0.1:14550"}]'})
assert isinstance(win.link_configs, list) and win.link_configs
# batch 9: a well-formed-JSON but SHAPE-broken value must be dropped, not crash the Links dialog
# (which does dict(c) + c['name']/['transport']/['target']). Old code passed these straight through.
load_with({"links/configs": '[1, 2, "x", {"name":"partial","transport":"UDP"}]'})   # no 'target'
assert win.link_configs == [], f"malformed link configs not filtered: {win.link_configs}"

# 5) corrupt telem/hidden JSON and a wrong-type (list where dict/str expected) -> no crash --------
load_with({"telem/hidden": "[1,2,3,unquoted]"})
assert_sane()
load_with({"telem/hidden": '["LINK","GPS"]'})        # valid list of group names
assert_sane()

# 6) completely empty settings -> all defaults, no crash ------------------------------------------
load_with({})
assert win._takeoff_alt == 25.0
assert_sane()

for p in _tmpfiles:
    try:
        os.unlink(p)
    except OSError:
        pass

print("SETTINGSLOAD PASSED (corrupt QSettings survive startup; no non-finite takeoff_alt / centre)")
