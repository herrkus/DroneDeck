"""geotagdialog.py -- GeoTag Images dialog (thin UI over geotag.py).

Pick a flight log + the photo folder + an output folder, then stamp GPS EXIF into the photos. Kept
separate from geotag.py so that module stays Qt-free and unit-testable.
"""
import os

from PySide6.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QGridLayout, QLineEdit, QPushButton,
                               QLabel, QFileDialog)

import geotag


class GeotagDialog(QDialog):
    def __init__(self, default_dir, parent=None):
        super().__init__(parent)
        self.setWindowTitle("GeoTag Images")
        self.resize(600, 220)
        self._default = default_dir or os.path.expanduser("~")
        self.tlog_edit = QLineEdit()
        self.photo_edit = QLineEdit()
        self.out_edit = QLineEdit()
        grid = QGridLayout()
        rows = [("Flight log (.tlog)", self.tlog_edit, self._pick_tlog),
                ("Photo folder", self.photo_edit, self._pick_photos),
                ("Output folder", self.out_edit, self._pick_out)]
        for r, (label, edit, browse) in enumerate(rows):
            grid.addWidget(QLabel(label), r, 0)
            grid.addWidget(edit, r, 1)
            b = QPushButton("Browse...")
            b.clicked.connect(browse)
            grid.addWidget(b, r, 2)

        self.status = QLabel("Select a flight log, the photo folder, and an output folder, then GeoTag.")
        self.status.setWordWrap(True)
        run = QPushButton("GeoTag")
        run.clicked.connect(self._go)
        close = QPushButton("Close")
        close.clicked.connect(self.close)
        row = QHBoxLayout()
        row.addWidget(run)
        row.addStretch(1)
        row.addWidget(close)
        lay = QVBoxLayout(self)
        lay.addLayout(grid)
        lay.addWidget(self.status)
        lay.addStretch(1)
        lay.addLayout(row)

    def _pick_tlog(self):
        p, _ = QFileDialog.getOpenFileName(self, "Flight log", self._default,
                                           "Telemetry logs (*.tlog);;All files (*)")
        if p:
            self.tlog_edit.setText(p)

    def _pick_photos(self):
        p = QFileDialog.getExistingDirectory(self, "Photo folder", self._default)
        if p:
            self.photo_edit.setText(p)
            if not self.out_edit.text().strip():
                self.out_edit.setText(os.path.join(p, "geotagged"))

    def _pick_out(self):
        p = QFileDialog.getExistingDirectory(self, "Output folder", self._default)
        if p:
            self.out_edit.setText(p)

    def _go(self):
        self._run(self.tlog_edit.text().strip(), self.photo_edit.text().strip(),
                  self.out_edit.text().strip())

    def _run(self, tlog, photos, out):
        """Split out so tests can drive it without the file pickers."""
        if not (tlog and photos and out):
            self.status.setText("Please set the flight log, photo folder, and output folder.")
            return
        if not os.path.isfile(tlog):
            self.status.setText(f"Flight log not found: {tlog}")
            return
        if not os.path.isdir(photos):
            self.status.setText(f"Photo folder not found: {photos}")
            return
        try:
            res = geotag.geotag(tlog, photos, out)
        except Exception as ex:
            self.status.setText(f"GeoTag failed: {ex}")
            return
        msg = (f"Tagged {res['tagged']} of {res['photos']} photo(s) -> {out}  "
               f"({res['events']} camera event(s) in the log)")
        if res["unmatched"]:
            msg += f".  {res['unmatched']} unmatched -- event and photo counts differ"
        if res["errors"]:
            msg += f".  {len(res['errors'])} file error(s)"
        self.status.setText(msg)
