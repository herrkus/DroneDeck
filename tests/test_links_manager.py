#!/usr/bin/env python3
"""test_links_manager.py -- saved comm-link configs: model, connect signal, persistence."""
import os
import sys
import tempfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = tempfile.mkdtemp(prefix="dronedeck-links-")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
from links_manager import LinkEditDialog, LinksDialog
import main as appmain

app = QApplication([])
fail = []

# LinkEditDialog round-trips its fields
cfg = {"name": "SITL", "transport": "TCP", "target": "127.0.0.1:5760"}
ed = LinkEditDialog(cfg)
if ed.config() != cfg:
    fail.append(f"LinkEditDialog.config() mismatch: {ed.config()}")

# LinksDialog lists configs and emits the selected one on connect
configs = [cfg, {"name": "Radio", "transport": "Serial", "target": "/dev/ttyUSB0:57600"}]
dlg = LinksDialog(configs)
if dlg.list.count() != 2:
    fail.append(f"list shows {dlg.list.count()} rows, expected 2")
emitted = {}
dlg.connectRequested.connect(lambda c: emitted.update(c))
dlg.list.setCurrentRow(1)
dlg._connect()
if emitted.get("name") != "Radio":
    fail.append(f"connect emitted {emitted!r}, expected the Radio config")

# persistence through the main window's settings
win = appmain.DroneDeck(14550)
win._persist = True
win.link_configs = configs
win.save_settings()
win.close()
win2 = appmain.DroneDeck(14550)
win2.load_settings()
if win2.link_configs != configs:
    fail.append(f"persisted configs mismatch: {win2.link_configs}")
win2.close()

print("restored configs:", win2.link_configs)
print("LINKS MANAGER FAILED: " + "; ".join(fail) if fail else "LINKS MANAGER PASSED")
sys.exit(1 if fail else 0)
