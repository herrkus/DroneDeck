#!/usr/bin/env python3
"""test_paramfile.py -- parameter file save/load. ParamDialog.save_params_to() writes the
downloaded parameters in QGC's tab-separated .params format; load_params_from() parses that back
(and a plain name,value CSV), skipping comments/blanks/garbage. Verifies a round-trip and the
parser robustness. Pure logic (no link needed to save/parse)."""
import os
import sys
import tempfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
from params import ParamManager, ParamDialog
import mavlink

app = QApplication([])

INT_TYPES = (1, 2, 3, 4, 5, 6, 7, 8)

mgr = ParamManager(lambda: None, lambda: 1)
mgr.values = {"MC_PITCH_P": 6.5, "BAT_N_CELLS": 4.0, "MPC_XY_VEL_MAX": 12.0, "COM_RC_IN_MODE": 1.0,
              "SYS_BITMASK": 1234567890.0,        # large int32 -- must NOT lose digits (was a bug)
              "SENS_FLT_MAX": 3.4028235e38}       # FLT_MAX sentinel -- float32 round-trip
mgr.type_of = {"MC_PITCH_P": mavlink.MAV_PARAM_TYPE_REAL32, "BAT_N_CELLS": 6,
               "MPC_XY_VEL_MAX": mavlink.MAV_PARAM_TYPE_REAL32, "COM_RC_IN_MODE": 6,
               "SYS_BITMASK": 6, "SENS_FLT_MAX": mavlink.MAV_PARAM_TYPE_REAL32}
dlg = ParamDialog(mgr)
assert dlg.btn_save is not None and dlg.btn_load is not None

# save -> QGC tab format
path = os.path.join(tempfile.mkdtemp(), "vehicle.params")
n = dlg.save_params_to(path)
assert n == 6
content = open(path).read()
assert content.startswith("#") and "\t" in content and "MC_PITCH_P" in content
assert "1234567890" in content, "large int must be written in full, not scientific notation"

# round-trip: load back gives the same names + values (type-aware tolerance, like _values_match)
loaded = ParamDialog.load_params_from(path)
assert set(loaded) == set(mgr.values), (set(loaded), set(mgr.values))
for k, v in mgr.values.items():
    if mgr.type_of.get(k) in INT_TYPES:
        assert loaded[k] == round(v), (k, loaded[k], v)                     # integers exact
    else:
        assert abs(loaded[k] - v) <= max(1e-4, abs(v) * 1e-3), (k, loaded[k], v)  # float32 rel

# parser robustness: comments, blanks, CSV rows, and un-parseable rows are handled
p2 = os.path.join(tempfile.mkdtemp(), "mixed.params")
open(p2, "w").write(
    "# a comment\n"
    "\n"
    "MC_ROLL_P,7.0\n"                 # plain name,value CSV
    "FOO,not_a_number\n"             # bad value -> skipped
    "1\t1\tBAT_LOW_THR\t0.15\t9\n"    # QGC tab row
    "garbage line with no delimiter\n"
)
loaded2 = ParamDialog.load_params_from(p2)
assert loaded2 == {"MC_ROLL_P": 7.0, "BAT_LOW_THR": 0.15}, loaded2

print("PARAMFILE PASSED")
