#!/usr/bin/env python3
"""test_settings.py -- QSettings persistence round-trip.

Uses an isolated XDG_CONFIG_HOME so the user's real config is never touched.
Saves link + map preferences from one window, then checks a fresh window
restores them.
"""
import os
import sys
import tempfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
_CFG = tempfile.mkdtemp(prefix="dronedeck-cfg-")
os.environ["XDG_CONFIG_HOME"] = _CFG          # QSettings .conf lands here

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # for _ports
from _ports import free_udp_port
PORT = free_udp_port()

from PySide6.QtWidgets import QApplication
import main as appmain

app = QApplication([])

win1 = appmain.DroneDeck(PORT)
win1._persist = True
win1.map.set_zoom(11)
win1.map.center = (50.0, 10.0)
win1.transport_combo.setCurrentText("TCP")
win1.link_edit.setText("1.2.3.4:5760")
win1.chk_follow.setChecked(False)
win1.save_settings()
win1.close()                                  # frees the UDP socket

win2 = appmain.DroneDeck(PORT)
win2.load_settings()

got = dict(zoom=win2.map.zoom, lat=round(win2.map.center[0], 4), lon=round(win2.map.center[1], 4),
           transport=win2.transport_combo.currentText(), target=win2.link_edit.text(),
           follow=win2.map.follow)
print("restored:", got)
win2.close()

fail = []
if got["zoom"] != 11:
    fail.append(f"zoom {got['zoom']} != 11")
if got["lat"] != 50.0 or got["lon"] != 10.0:
    fail.append(f"map center {got['lat']},{got['lon']} != 50,10")
if got["transport"] != "TCP":
    fail.append(f"transport {got['transport']} != TCP")
if got["target"] != "1.2.3.4:5760":
    fail.append(f"target {got['target']!r}")
if got["follow"] is not False:
    fail.append(f"follow {got['follow']} != False")

print("SETTINGS FAILED: " + "; ".join(fail) if fail else "SETTINGS PASSED")
sys.exit(1 if fail else 0)
