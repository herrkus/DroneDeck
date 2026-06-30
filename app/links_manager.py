"""links_manager.py -- saved comm-link configurations (QGC-style link manager).

Lets the user create, edit, delete and quick-connect named link configs
(UDP / TCP / Serial / Replay). The list is owned by the main window and
persisted via QSettings; this module only provides the dialogs.
"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QFormLayout,
                               QListWidget, QListWidgetItem, QPushButton, QLineEdit,
                               QComboBox, QLabel, QDialogButtonBox)

TRANSPORTS = ["UDP", "TCP", "Serial", "Replay"]
HINTS = {"UDP": "port (e.g. 14550)", "TCP": "host:port (e.g. 127.0.0.1:5760)",
         "Serial": "device:baud (e.g. /dev/ttyACM0:57600)", "Replay": "path[@speed]"}


class LinkEditDialog(QDialog):
    def __init__(self, cfg=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit link" if cfg else "New link")
        cfg = cfg or {}
        form = QFormLayout(self)
        self.name = QLineEdit(cfg.get("name", ""))
        self.transport = QComboBox()
        self.transport.addItems(TRANSPORTS)
        self.transport.setCurrentText(cfg.get("transport", "UDP"))
        self.target = QLineEdit(cfg.get("target", ""))
        self.transport.currentTextChanged.connect(
            lambda t: self.target.setPlaceholderText(HINTS.get(t, "")))
        self.target.setPlaceholderText(HINTS.get(self.transport.currentText(), ""))
        form.addRow("Name", self.name)
        form.addRow("Transport", self.transport)
        form.addRow("Target", self.target)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        form.addRow(bb)

    def config(self):
        name = self.name.text().strip() or f"{self.transport.currentText()} link"
        return {"name": name, "transport": self.transport.currentText(),
                "target": self.target.text().strip()}


class LinksDialog(QDialog):
    connectRequested = Signal(dict)

    def __init__(self, configs, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Comm Links")
        self.resize(420, 320)
        self.configs = [dict(c) for c in configs]
        v = QVBoxLayout(self)
        v.addWidget(QLabel("Saved links (double-click to connect):"))
        self.list = QListWidget()
        self.list.itemDoubleClicked.connect(lambda _: self._connect())
        v.addWidget(self.list, 1)
        row = QHBoxLayout()
        for label, slot in (("Add", self._add), ("Edit", self._edit),
                            ("Remove", self._remove), ("Connect", self._connect)):
            b = QPushButton(label)
            b.clicked.connect(slot)
            row.addWidget(b)
        row.addStretch(1)
        close = QPushButton("Close")
        close.clicked.connect(self.accept)
        row.addWidget(close)
        v.addLayout(row)
        self._refresh()

    def _refresh(self):
        self.list.clear()
        for c in self.configs:
            QListWidgetItem(f"{c['name']}  —  {c['transport']} {c['target']}", self.list)

    def _add(self):
        dlg = LinkEditDialog(parent=self)
        if dlg.exec() == QDialog.Accepted:
            self.configs.append(dlg.config())
            self._refresh()

    def _edit(self):
        r = self.list.currentRow()
        if r < 0:
            return
        dlg = LinkEditDialog(self.configs[r], parent=self)
        if dlg.exec() == QDialog.Accepted:
            self.configs[r] = dlg.config()
            self._refresh()

    def _remove(self):
        r = self.list.currentRow()
        if r >= 0:
            del self.configs[r]
            self._refresh()

    def _connect(self):
        r = self.list.currentRow()
        if r >= 0:
            self.connectRequested.emit(self.configs[r])
            self.accept()
