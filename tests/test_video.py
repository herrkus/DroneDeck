#!/usr/bin/env python3
"""test_video.py -- the video pane constructs and degrades gracefully.

We cannot verify real stream playback headlessly, so this checks the plumbing:
the pane builds whether or not QtMultimedia is present, set_source/play/stop
never raise, and play() with no source reports a status instead of crashing.
"""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import video

app = QApplication([])
pane = video.VideoPane()

fail = []
print("QtMultimedia available:", video.HAVE_MULTIMEDIA)

# empty source -> a status message, no crash
try:
    pane.play()
    if "URL" not in pane.status.text() and "unavailable" not in pane.status.text():
        fail.append(f"empty play() gave unexpected status: {pane.status.text()!r}")
except Exception as e:
    fail.append(f"play() with no source raised {e!r}")

# set a (bogus) source and drive play/stop -- must not raise
try:
    pane.set_source("rtsp://127.0.0.1:5554/none")
    pane.play()
    pane.stop()
except Exception as e:
    fail.append(f"play/stop raised {e!r}")

# a local-file style source path -- must not raise either
try:
    pane.set_source("/nonexistent/video.mp4")
    pane.play()
    pane.stop()
except Exception as e:
    fail.append(f"file-source play/stop raised {e!r}")

print("status after drive:", pane.status.text())
print("VIDEO FAILED: " + "; ".join(fail) if fail else "VIDEO PASSED")
sys.stdout.flush()
os._exit(1 if fail else 0)
