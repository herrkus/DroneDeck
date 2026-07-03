#!/usr/bin/env python3
"""test_parammeta.py -- parameter metadata: units/range/desc/enum (iter160, QGC parity).

QGC makes parameters usable by showing each one's units, valid range, description and enum labels from
the autopilot's own definition files. DroneDeck loads the SAME authoritative files (ArduPilot
apm.pdef.xml / PX4 parameters.json) -- no hardcoded guesses. Verifies: both file formats parse into the
same normalised shape (incl. name-prefix stripping and negative enum codes); load() auto-detects XML vs
JSON; tooltip()/out_of_range() behave; and the param editor shows a name tooltip when metadata is loaded
and flags an out-of-documented-range edit (orange + warning) without blocking the write."""
import os
import sys
import tempfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import parammeta

fail = []

APM_XML = """<?xml version="1.0"?>
<paramfile><vehicles><parameters name="ArduCopter">
 <param humanName="Arming Check" name="ARMING_CHECK" documentation="Bitmask of pre-arm checks">
  <field name="Units">bit</field><field name="Range">0 1023</field>
  <field name="Values">0:Disabled,1:All</field><field name="RebootRequired">False</field></param>
 <param humanName="RTL Altitude" name="ArduCopter:RTL_ALT" documentation="Return altitude">
  <field name="Units">cm</field><field name="Range">0 800000</field></param>
</parameters></vehicles></paramfile>"""

PX4_JSON = """{"parameters":[
 {"name":"MC_ROLLRATE_P","type":"Float","units":"","min":0.01,"max":0.5,
  "shortDesc":"Roll rate P","longDesc":"Roll rate proportional gain"},
 {"name":"COM_DISARM_LAND","type":"Float","units":"s","min":0.0,"max":20.0,
  "shortDesc":"Disarm on land","rebootRequired":true,
  "values":[{"value":0,"description":"Disabled"},{"value":-1,"description":"Never"}]}
]}"""

tmp = tempfile.mkdtemp(prefix="parammeta_")
xml_path = os.path.join(tmp, "apm.pdef.xml")
json_path = os.path.join(tmp, "parameters.json")
open(xml_path, "w").write(APM_XML)
open(json_path, "w").write(PX4_JSON)

# 1) ArduPilot XML parses (units/range/values/reboot; name prefix stripped) -------------------------
pm = parammeta.ParamMeta()
added = pm.load(xml_path)                            # load() must auto-detect XML by leading '<'
if added != 2 or len(pm) != 2:
    fail.append(f"apm load count wrong: added={added} len={len(pm)}")
ac = pm.get("ARMING_CHECK")
if not ac or ac["units"] != "bit" or ac["min"] != 0 or ac["max"] != 1023:
    fail.append(f"ARMING_CHECK meta wrong: {ac}")
if not ac or ac["values"] != {0: "Disabled", 1: "All"} or "pre-arm" not in ac["desc"]:
    fail.append(f"ARMING_CHECK values/desc wrong: {ac}")
if pm.get("RTL_ALT") is None:                        # "ArduCopter:RTL_ALT" -> "RTL_ALT"
    fail.append("name prefix 'ArduCopter:' not stripped for RTL_ALT")

# 2) PX4 JSON parses (min/max/desc/reboot; negative enum code) --------------------------------------
pj = parammeta.ParamMeta()
if pj.load(json_path) != 2:                          # auto-detect JSON (not leading '<')
    fail.append("px4 load count wrong")
rr = pj.get("MC_ROLLRATE_P")
if not rr or abs(rr["min"] - 0.01) > 1e-9 or abs(rr["max"] - 0.5) > 1e-9 or rr["desc"] != "Roll rate proportional gain":
    fail.append(f"MC_ROLLRATE_P meta wrong: {rr}")
dl = pj.get("COM_DISARM_LAND")
if not dl or dl["reboot"] is not True or dl["values"] != {0: "Disabled", -1: "Never"}:
    fail.append(f"COM_DISARM_LAND meta wrong: {dl}")

# 3) tooltip() + out_of_range() ---------------------------------------------------------------------
tip = pm.tooltip("ARMING_CHECK")
if "range: 0.0 .. 1023.0" not in tip or "units: bit" not in tip or "0=Disabled" not in tip:
    fail.append(f"tooltip wrong: {tip!r}")
if pm.tooltip("NOT_A_PARAM") != "":
    fail.append("tooltip for unknown param should be ''")
if not pm.out_of_range("ARMING_CHECK", 5000) or pm.out_of_range("ARMING_CHECK", 100):
    fail.append("out_of_range wrong for ARMING_CHECK")
if pm.out_of_range("NOT_A_PARAM", 1e9):              # unknown -> never flagged
    fail.append("out_of_range should be False for unknown param")

# 4) editor: tooltip shown + out-of-range edit flagged orange (still writable) ----------------------
from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QColor
from params import ParamManager, ParamDialog
app = QApplication.instance() or QApplication([])
mgr = ParamManager(lambda: None, lambda: (1, 1))
dlg = ParamDialog(mgr)
dlg.meta = pm                                        # inject loaded metadata
dlg._on_param("ARMING_CHECK", 5.0, 0, 1)            # add a row with metadata present
nitem = dlg.table.item(dlg.rows["ARMING_CHECK"], 0)
if "pre-arm" not in nitem.toolTip():
    fail.append(f"name tooltip not applied: {nitem.toolTip()!r}")

vitem = dlg.table.item(dlg.rows["ARMING_CHECK"], 1)
vitem.setText("5000")                                # out of [0,1023] -> itemChanged -> _item_changed
if dlg.edited.get("ARMING_CHECK") != 5000:
    fail.append("out-of-range edit must still be recorded as writable (soft warn only)")
if vitem.foreground().color().name().lower() != QColor("#ff9f43").name().lower():
    fail.append(f"out-of-range edit not flagged orange: {vitem.foreground().color().name()}")
if "outside documented range" not in vitem.toolTip():
    fail.append(f"out-of-range tooltip missing: {vitem.toolTip()!r}")

vitem.setText("100")                                 # back in range -> normal pending colour, tip cleared
if vitem.foreground().color().name().lower() != QColor("#ffd24a").name().lower() or vitem.toolTip():
    fail.append("in-range edit should be normal pending colour with no warning tooltip")

# a param with no metadata loaded gets no tooltip and edits normally --------------------------------
dlg._on_param("SOME_UNKNOWN_PARAM", 1.0, 0, 1)
if dlg.table.item(dlg.rows["SOME_UNKNOWN_PARAM"], 0).toolTip():
    fail.append("unknown param should have no tooltip")

print("PARAMMETA FAILED: " + "; ".join(fail) if fail else
      "PARAMMETA PASSED (ArduPilot apm.pdef.xml + PX4 parameters.json parse to one normalised shape, "
      "name-prefix stripped, negative enum codes ok; load() auto-detects; tooltip/out_of_range correct; "
      "editor shows metadata tooltip + flags out-of-range edits orange without blocking the write)")
sys.stdout.flush()
os._exit(1 if fail else 0)
