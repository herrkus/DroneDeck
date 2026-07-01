#!/usr/bin/env python3
"""test_paramdiff.py -- parameter file compare. The param dialog can compare a saved .params file
against the vehicle's live values and show the differences WITHOUT writing (QGC-style config check).
Verifies diff_params(): type-aware change detection (integer bitmasks exact, floats tolerant),
missing-on-vehicle detection, identical->empty, and the 'Compare...' button wiring. No link."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import mavlink
from params import ParamDialog

app = QApplication([])

REAL = mavlink.MAV_PARAM_TYPE_REAL32
INT = 6                                  # MAV_PARAM_TYPE_INT32 (types 1-8 are integer types)

current = {"MPC_XY_VEL_MAX": 12.0, "COM_RC_IN_MODE": 1.0, "BAT_N_CELLS": 4.0,
           "EKF2_AID_MASK": 24.0}
types = {"MPC_XY_VEL_MAX": REAL, "COM_RC_IN_MODE": INT, "BAT_N_CELLS": INT, "EKF2_AID_MASK": INT}

# file: one real differs, one int differs, one identical, one int identical-ish, one not on vehicle
loaded = {"MPC_XY_VEL_MAX": 8.0,       # differs (real)
          "COM_RC_IN_MODE": 1.0,       # identical
          "BAT_N_CELLS": 6.0,          # differs (int)
          "EKF2_AID_MASK": 24.0,       # identical (int)
          "MC_PITCHRATE_P": 0.15}      # not on vehicle -> missing

changed, missing = ParamDialog.diff_params(current, types, loaded)
cnames = {c[0]: (c[1], c[2]) for c in changed}
assert set(cnames) == {"MPC_XY_VEL_MAX", "BAT_N_CELLS"}, cnames
assert cnames["MPC_XY_VEL_MAX"] == (12.0, 8.0)
assert cnames["BAT_N_CELLS"] == (4.0, 6.0)
assert missing == ["MC_PITCHRATE_P"], missing
# changed is sorted by name
assert [c[0] for c in changed] == sorted(c[0] for c in changed)

# identical file -> no differences
same_changed, same_missing = ParamDialog.diff_params(current, types, dict(current))
assert same_changed == [] and same_missing == [], (same_changed, same_missing)

# a tiny float wobble within float32 tolerance is NOT flagged as a change (type-aware match)
near = dict(current); near["MPC_XY_VEL_MAX"] = 12.0 + 1e-7
nc, nm = ParamDialog.diff_params(current, types, near)
assert nc == [] and nm == [], (nc, nm)

# the dialog exposes the compare entrypoints (the handler opens a modal at runtime, so it is not
# invoked here -- only the pure logic above is exercised)
assert callable(getattr(ParamDialog, "_compare_file", None))
assert callable(getattr(ParamDialog, "diff_params", None))

print("PARAMDIFF PASSED")
