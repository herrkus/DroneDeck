"""video.py -- FPV / camera video pane.

Plays an RTSP / UDP / HTTP / file video source via QtMultimedia when it is
available, and degrades to a clear placeholder when it is not (QtMultimedia is
an optional dependency -- the rest of DroneDeck never depends on it).
"""
from __future__ import annotations
import os

from PySide6.QtCore import Qt, QUrl
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLineEdit,
                               QPushButton, QLabel)

try:
    from PySide6.QtMultimedia import QMediaPlayer, QAudioOutput
    from PySide6.QtMultimediaWidgets import QVideoWidget
    HAVE_MULTIMEDIA = True
except Exception:                       # pragma: no cover - depends on the host
    HAVE_MULTIMEDIA = False


class VideoPane(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        v = QVBoxLayout(self)
        v.setContentsMargins(4, 4, 4, 4)
        v.setSpacing(4)

        row = QHBoxLayout()
        self.url = QLineEdit()
        self.url.setPlaceholderText("rtsp://… / udp://@:5600 / http://… / file path")
        self.btn_play = QPushButton("Play")
        self.btn_stop = QPushButton("Stop")
        self.btn_play.clicked.connect(self.play)
        self.btn_stop.clicked.connect(self.stop)
        row.addWidget(self.url, 1)
        row.addWidget(self.btn_play)
        row.addWidget(self.btn_stop)
        v.addLayout(row)

        if HAVE_MULTIMEDIA:
            self.player = QMediaPlayer(self)
            self.audio = QAudioOutput(self)
            self.player.setAudioOutput(self.audio)
            self.video = QVideoWidget(self)
            self.video.setStyleSheet("background:#000;")
            self.player.setVideoOutput(self.video)
            self.player.errorOccurred.connect(self._on_error)
            self.player.playbackStateChanged.connect(self._on_state)
            v.addWidget(self.video, 1)
        else:
            self.video = None
            ph = QLabel("Video unavailable\n\nInstall the PySide6 multimedia plugin\n"
                        "and a GStreamer backend to view RTSP/UDP/FPV streams.")
            ph.setAlignment(Qt.AlignCenter)
            ph.setStyleSheet("color:#8a90a0; background:#000;")
            v.addWidget(ph, 1)
            self.btn_play.setEnabled(False)
            self.btn_stop.setEnabled(False)

        self.status = QLabel("multimedia ready" if HAVE_MULTIMEDIA else "multimedia backend not installed")
        self.status.setStyleSheet("color:#8fa3bf;")
        v.addWidget(self.status)

    def set_source(self, url):
        self.url.setText(url)

    def play(self):
        if not HAVE_MULTIMEDIA:
            self.status.setText("video backend unavailable")
            return
        url = self.url.text().strip()
        if not url:
            self.status.setText("enter a stream URL or file path")
            return
        qurl = QUrl(url) if "://" in url else QUrl.fromLocalFile(os.path.abspath(url))
        self.player.setSource(qurl)
        self.player.play()
        self.status.setText(f"opening {url}")

    def stop(self):
        if HAVE_MULTIMEDIA:
            self.player.stop()
            self.status.setText("stopped")

    def _on_error(self, _err, msg):        # pragma: no cover - host dependent
        self.status.setText(f"error: {msg}" if msg else "stream error")

    def _on_state(self, state):            # pragma: no cover - host dependent
        names = {QMediaPlayer.PlayingState: "playing",
                 QMediaPlayer.PausedState: "paused",
                 QMediaPlayer.StoppedState: "stopped"}
        self.status.setText(names.get(state, ""))
