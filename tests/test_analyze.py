"""Offline test for the Analyze log-plot (analyze.py): synthesise a .tlog, extract
series, and exercise single + multi-signal plotting and CSV export. Needs no vehicle
or link, so it is always safe to run (unlike the GUI tests that bind udp:14550)."""
import os
import sys
import struct

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "app"))

from PySide6.QtWidgets import QApplication, QFileDialog
from PySide6.QtCore import Qt

import mavlink
from tlog import TlogWriter, read_tlog
from analyze import AnalyzeDialog, LogPlot, extract_series, series_to_csv

app = QApplication.instance() or QApplication([])
tmp = os.path.join(os.path.dirname(__file__), "_analyze_tmp.tlog")
outcsv = os.path.join(os.path.dirname(__file__), "_analyze_tmp.csv")

w = TlogWriter(tmp)
t0 = 1_700_000_000_000_000                      # fixed epoch (us) -> deterministic times
for i in range(15):
    gp = struct.pack("<IiiiihhhH", i * 100, 0, 0, 500000 + i * 1000, i * 1000, 0, 0, 0, 0)
    w.write(mavlink.frame(mavlink.GLOBAL_POSITION_INT, gp, i, 1, 1), t0 + i * 100000)
    vh = struct.pack("<ffffhH", 0.0, 5.0 + i * 0.1, float(i), 0.5, 90, 60)
    w.write(mavlink.frame(mavlink.VFR_HUD, vh, i, 1, 1), t0 + i * 100000)
w.close()

try:
    # extraction + CSV
    series, units = extract_series(read_tlog(tmp))
    alt = series["Altitude (rel)"]
    assert len(alt) == 15 and abs(alt[0][1]) < 1e-6 and abs(alt[-1][1] - 14.0) < 1e-6, alt[-1]
    csv = series_to_csv(alt, "Altitude (rel)", "m").splitlines()
    assert csv[0] == "time_s,Altitude (rel) (m)" and len(csv) == 16, csv[0]

    # LogPlot: single then multi, both render; crosshair in multi mode renders
    lp = LogPlot()
    lp.resize(500, 300)
    lp.set_series(alt, "Altitude (rel)", "m")
    assert len(lp.series) == 1 and not lp.grab().isNull()
    lp.set_multi([("Altitude (rel)", "m", alt), ("Ground speed", "m/s", series["Ground speed"])])
    assert len(lp.series) == 2 and not lp.grab().isNull()
    lp._cursor_t = 0.7
    assert not lp.grab().isNull()

    # dialog: checklist populated + first ticked; ticking a second overlays it
    dlg = AnalyzeDialog(None, "/tmp")
    dlg.open_tlog(tmp)
    assert dlg._checked() == ["Altitude (rel)"] and len(dlg.plot.series) == 1
    for i in range(dlg.fieldlist.count()):
        if dlg.fieldlist.item(i).text() == "Ground speed":
            dlg.fieldlist.item(i).setCheckState(Qt.Checked)
    assert len(dlg._checked()) == 2 and len(dlg.plot.series) == 2

    # export the first ticked signal
    QFileDialog.getSaveFileName = lambda *a, **k: (outcsv, "CSV (*.csv)")
    dlg._export_csv()
    assert len(open(outcsv).read().strip().splitlines()) == 16

    print("ANALYZE PASSED")
finally:
    for f in (tmp, outcsv):
        if os.path.exists(f):
            os.remove(f)
