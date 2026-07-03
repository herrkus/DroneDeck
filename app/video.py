"""video.py -- FPV / camera video pane + stream recording.

Plays an RTSP / UDP / HTTP / file video source via QtMultimedia when it is
available, and degrades to a clear placeholder when it is not (QtMultimedia is
an optional dependency -- the rest of DroneDeck never depends on it).

Recording is independent of the display path: VideoRecorder runs a GStreamer
pipeline (gst-launch-1.0) that pulls the live H.264 RTP stream and muxes it to
a Matroska (.mkv) file -- Matroska is used deliberately so a recording cut short
by a crash / power loss is still playable (a truncated .mp4 is not). This is
what QGroundControl does for its "record video" feature.
"""
from __future__ import annotations
import os
import time
import signal
import shutil
import subprocess

from PySide6.QtCore import Qt, QUrl
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLineEdit,
                               QPushButton, QLabel)

try:
    from PySide6.QtMultimedia import QMediaPlayer, QAudioOutput
    from PySide6.QtMultimediaWidgets import QVideoWidget
    HAVE_MULTIMEDIA = True
except Exception:                       # pragma: no cover - depends on the host
    HAVE_MULTIMEDIA = False

HAVE_GST = shutil.which("gst-launch-1.0") is not None


class VideoRecorder:
    """Record a live H.264 video stream (udp:// RTP or rtsp://) to a Matroska file via GStreamer.

    Transport-agnostic of the display: it spawns its own gst-launch-1.0, so recording works even when
    the QtMultimedia preview backend is missing. gst-launch runs with -e so a SIGINT flushes an
    end-of-stream through matroskamux and the .mkv finalises cleanly."""

    def __init__(self, recdir=None):
        self.recdir = recdir or os.path.expanduser("~/Videos/DroneDeck")
        self.proc = None
        self.out_path = None

    @property
    def is_recording(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    @staticmethod
    def _pipeline_args(url, out_path):
        """gst-launch argv to record `url` to `out_path`, or None if the URL isn't a live H.264 stream.

        Only live udp:// / rtsp:// are recordable without re-encoding (a http:// or file source is
        already a file). Kept as an argv LIST -- never a shell string -- so a URL can't inject a command."""
        url = (url or "").strip()
        sink = ["!", "matroskamux", "!", "filesink", f"location={out_path}"]
        if url.startswith("udp://"):
            port = url[len("udp://"):].lstrip("@").rsplit(":", 1)[-1]
            if not port.isdigit():
                return None
            src = ["udpsrc", f"port={port}",
                   "caps=application/x-rtp,media=video,encoding-name=H264,payload=96",
                   "!", "rtpjitterbuffer", "latency=200", "!", "rtph264depay", "!", "h264parse"]
        elif url.startswith("rtsp://"):
            src = ["rtspsrc", f"location={url}", "latency=200", "protocols=tcp+udp",
                   "!", "rtph264depay", "!", "h264parse"]
        else:
            return None
        return ["gst-launch-1.0", "-e", "-q"] + src + sink

    def start(self, url, out_path=None) -> bool:
        if self.is_recording or not HAVE_GST:
            return False
        if out_path is None:
            os.makedirs(self.recdir, exist_ok=True)
            out_path = os.path.join(self.recdir, time.strftime("drone_%Y%m%d_%H%M%S.mkv"))
        args = self._pipeline_args(url, out_path)
        if args is None:
            return False
        try:
            self.proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            self.proc = None
            return False
        self.out_path = out_path
        return True

    def stop(self):
        """Stop recording and return the finalised file path (SIGINT -> EOS so the .mkv closes cleanly)."""
        if self.proc is None:
            return None
        out = self.out_path
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
        self.proc = None
        return out


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
        self.btn_rec = QPushButton("Record")
        self.btn_rec.setToolTip("Record the live udp:// / rtsp:// H.264 stream to a Matroska (.mkv) file")
        self.btn_play.clicked.connect(self.play)
        self.btn_stop.clicked.connect(self.stop)
        self.btn_rec.clicked.connect(self._toggle_record)
        row.addWidget(self.url, 1)
        row.addWidget(self.btn_play)
        row.addWidget(self.btn_stop)
        row.addWidget(self.btn_rec)
        v.addLayout(row)

        self.recorder = VideoRecorder()
        self.btn_rec.setEnabled(HAVE_GST)      # recording works even without the QtMultimedia preview
        if not HAVE_GST:
            self.btn_rec.setToolTip("Install gst-launch (GStreamer) to record the video stream")

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

    def _toggle_record(self):
        if self.recorder.is_recording:
            path = self.recorder.stop()
            self.btn_rec.setText("Record")
            self.status.setText(f"recording saved: {path}" if path else "recording stopped")
            return
        url = self.url.text().strip()
        if self.recorder.start(url):
            self.btn_rec.setText("Stop Rec")
            self.status.setText(f"recording {url} -> {self.recorder.out_path}")
        elif not HAVE_GST:
            self.status.setText("recording needs GStreamer (gst-launch)")
        else:
            self.status.setText("recording needs a live udp:// or rtsp:// stream URL")

    def _on_error(self, _err, msg):        # pragma: no cover - host dependent
        self.status.setText(f"error: {msg}" if msg else "stream error")

    def _on_state(self, state):            # pragma: no cover - host dependent
        names = {QMediaPlayer.PlayingState: "playing",
                 QMediaPlayer.PausedState: "paused",
                 QMediaPlayer.StoppedState: "stopped"}
        self.status.setText(names.get(state, ""))
