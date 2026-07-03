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

from PySide6.QtCore import Qt, QUrl, QTimer
from PySide6.QtGui import QImage, QPainter, QColor
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLineEdit,
                               QPushButton, QLabel)

try:
    from PySide6.QtMultimedia import QMediaPlayer, QAudioOutput
    from PySide6.QtMultimediaWidgets import QVideoWidget
    HAVE_MULTIMEDIA = True
except Exception:                       # pragma: no cover - depends on the host
    HAVE_MULTIMEDIA = False

HAVE_GST = shutil.which("gst-launch-1.0") is not None

try:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
    Gst.init(None)
    HAVE_GST_PY = True
except Exception:                       # pragma: no cover - depends on the host
    HAVE_GST_PY = False


class GstVideoWidget(QWidget):
    """Display a live H.264 stream (udp:// RTP or rtsp://) via GStreamer.

    QtMultimedia's QMediaPlayer cannot open a raw UDP RTP stream (it treats udp://@:PORT as a file and
    errors) -- which is exactly why QGroundControl renders video through GStreamer. This runs a decode
    pipeline into an RGB appsink and paints the frames, so a real drone's FPV downlink actually shows.
    Frames are pulled non-blocking on a QTimer, so GStreamer never blocks the Qt event loop."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(160, 120)
        self._img = None
        self._pipe = None
        self._sink = None
        self._status = "no signal"
        self._timer = QTimer(self)
        self._timer.setInterval(30)
        self._timer.timeout.connect(self._pull)

    @staticmethod
    def available():
        return HAVE_GST_PY

    @staticmethod
    def _pipeline_desc(url, sink="ddsink"):
        """gst pipeline string decoding `url` to an RGB appsink, or None if it isn't a live H.264 URL."""
        url = (url or "").strip()
        tail = (f" ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=RGB "
                f"! appsink name={sink} max-buffers=3 drop=true sync=false")
        if url.startswith("udp://"):
            port = url[len("udp://"):].lstrip("@").rsplit(":", 1)[-1]
            if not port.isdigit():
                return None
            return (f"udpsrc port={port} caps=application/x-rtp,media=video,encoding-name=H264,payload=96"
                    f" ! rtpjitterbuffer latency=200" + tail)
        if url.startswith("rtsp://"):
            return f"rtspsrc location={url} latency=200 protocols=tcp+udp" + tail
        return None

    def start(self, url) -> bool:
        self.stop()
        if not HAVE_GST_PY:
            return False
        desc = self._pipeline_desc(url)
        if desc is None:
            return False
        try:
            self._pipe = Gst.parse_launch(desc)
            self._sink = self._pipe.get_by_name("ddsink")
            self._pipe.set_state(Gst.State.PLAYING)
        except Exception:
            self._pipe = self._sink = None
            return False
        self._status = "connecting..."
        self._timer.start()
        self.update()
        return True

    def stop(self):
        self._timer.stop()
        if self._pipe is not None:
            self._pipe.set_state(Gst.State.NULL)
        self._pipe = self._sink = None
        self._img = None
        self._status = "no signal"
        self.update()

    def _pull(self):
        if self._sink is None:
            return
        sample = self._sink.emit("try-pull-sample", 0)      # 0 ns timeout -> non-blocking
        if not sample:
            return
        st = sample.get_caps().get_structure(0)
        w, h = st.get_value("width"), st.get_value("height")
        buf = sample.get_buffer()
        ok, mi = buf.map(Gst.MapFlags.READ)
        if not ok:
            return
        try:
            stride = mi.size // h                           # gst rounds rows up to 4 bytes; honour it
            # copy() -- the mapped GStreamer memory is unmapped/reused right after this call
            self._img = QImage(bytes(mi.data), w, h, stride, QImage.Format_RGB888).copy()
        finally:
            buf.unmap(mi)
        self._status = ""
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor("#000000"))
        if self._img is not None and not self._img.isNull():
            scaled = self._img.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            p.drawImage((self.width() - scaled.width()) // 2,
                        (self.height() - scaled.height()) // 2, scaled)
        else:
            p.setPen(QColor("#8a90a0"))
            p.drawText(self.rect(), Qt.AlignCenter, self._status)


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

        # Two display backends share the video area, picked per-source by play(): QtMultimedia for
        # files / http, and GStreamer for live udp:// / rtsp:// (which QtMultimedia cannot open).
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

        self.gst_video = GstVideoWidget(self) if HAVE_GST_PY else None
        if self.gst_video is not None:
            v.addWidget(self.gst_video, 1)
            self.gst_video.hide()

        if self.video is None and self.gst_video is None:
            ph = QLabel("Video unavailable\n\nInstall PySide6 QtMultimedia or GStreamer (python-gobject)\n"
                        "to view RTSP / UDP / FPV streams.")
            ph.setAlignment(Qt.AlignCenter)
            ph.setStyleSheet("color:#8a90a0; background:#000;")
            v.addWidget(ph, 1)
            self.btn_play.setEnabled(False)
            self.btn_stop.setEnabled(False)

        self.status = QLabel("video ready" if (HAVE_MULTIMEDIA or HAVE_GST_PY) else "no video backend installed")
        self.status.setStyleSheet("color:#8fa3bf;")
        v.addWidget(self.status)

    def set_source(self, url):
        self.url.setText(url)

    def play(self):
        url = self.url.text().strip()
        if not url:
            self.status.setText("enter a stream URL or file path")
            return
        live = url.startswith(("udp://", "rtsp://"))
        # live RTP/RTSP -> GStreamer (QtMultimedia can't open raw UDP RTP); files / http -> QtMultimedia
        if live and self.gst_video is not None:
            if self.video is not None:
                self.player.stop()
                self.video.hide()
            self.gst_video.show()
            if self.gst_video.start(url):
                self.status.setText(f"playing {url} (GStreamer)")
            else:
                self.status.setText("could not start the GStreamer pipeline")
            return
        if HAVE_MULTIMEDIA:
            if self.gst_video is not None:
                self.gst_video.stop()
                self.gst_video.hide()
            self.video.show()
            qurl = QUrl(url) if "://" in url else QUrl.fromLocalFile(os.path.abspath(url))
            self.player.setSource(qurl)
            self.player.play()
            self.status.setText(f"opening {url}")
            return
        self.status.setText("live udp:// / rtsp:// needs GStreamer; QtMultimedia handles files / http")

    def stop(self):
        if self.gst_video is not None:
            self.gst_video.stop()
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
