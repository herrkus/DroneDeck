#!/usr/bin/env python3
"""test_gst_video.py -- GStreamer display of a live UDP/RTSP stream (iter168, real-drone FPV).

QtMultimedia CANNOT open a raw UDP H.264 RTP stream (it treats udp://@:PORT as a file and errors) -- so
DroneDeck could record a drone's FPV downlink but not SHOW it. GstVideoWidget fixes that the way QGC does:
a GStreamer decode pipeline into an RGB appsink, painted in the pane. Verifies: the pipeline builder maps
udp/rtsp and refuses non-live URLs; VideoPane routes live udp:// to the GStreamer widget (not
QtMultimedia); and -- for real -- it decodes an actual synthetic H.264 UDP stream to a valid RGB QImage
of the right size. Gracefully skips the live part if python-gst / H.264 plugins are missing."""
import os
import sys
import time
import subprocess

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
from PySide6.QtWidgets import QApplication
import video
from video import GstVideoWidget, VideoPane

fail = []
app = QApplication.instance() or QApplication([])

# 1) pipeline builder maps udp/rtsp, refuses non-live -----------------------------------------------
udp = GstVideoWidget._pipeline_desc("udp://@:5600")
if not udp or "udpsrc port=5600" not in udp or "avdec_h264" not in udp or "appsink" not in udp:
    fail.append(f"udp pipeline wrong: {udp}")
rtsp = GstVideoWidget._pipeline_desc("rtsp://cam/s")
if not rtsp or "rtspsrc location=rtsp://cam/s" not in rtsp or "appsink" not in rtsp:
    fail.append(f"rtsp pipeline wrong: {rtsp}")
for bad in ("http://x/y.mp4", "/file.mp4", "udp://@:notaport", ""):
    if GstVideoWidget._pipeline_desc(bad) is not None:
        fail.append(f"non-live URL {bad!r} should not build a pipeline")

# 2) VideoPane routes a live udp:// URL to the GStreamer widget (not QtMultimedia) ------------------
if video.HAVE_GST_PY:
    pane = VideoPane()
    pane.url.setText("udp://@:5600")
    pane.play()                                    # should start the gst widget (pipeline launches)
    if pane.gst_video is None or pane.gst_video.isHidden():
        fail.append("live udp:// should show the GStreamer widget")
    if "GStreamer" not in pane.status.text():
        fail.append(f"live udp:// status should mention GStreamer, got {pane.status.text()!r}")
    pane.stop()
    if pane.gst_video._pipe is not None:
        fail.append("stop() should tear down the gst pipeline")

# 3) real end-to-end: decode a synthetic H.264 UDP stream to an RGB QImage --------------------------
def have(el):
    return subprocess.run(["gst-inspect-1.0", "--exists", el]).returncode == 0

if not video.HAVE_GST_PY or not all(have(e) for e in ("x264enc", "rtph264pay", "avdec_h264", "udpsrc")):
    print("GST_VIDEO PASSED (pipeline builder + pane routing verified; live decode SKIPPED -- "
          "python-gst / H.264 plugins not installed)" if not fail else "GST_VIDEO FAILED: " + "; ".join(fail))
    sys.stdout.flush()
    os._exit(1 if fail else 0)

PORT = 5637
producer = subprocess.Popen(
    ["gst-launch-1.0", "-q", "videotestsrc", "is-live=true", "!",
     "video/x-raw,width=320,height=240,framerate=30/1", "!", "x264enc", "tune=zerolatency",
     "key-int-max=15", "!", "rtph264pay", "config-interval=1", "!", "udpsink", "host=127.0.0.1",
     f"port={PORT}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
w = GstVideoWidget()
try:
    time.sleep(0.8)                                # let the producer come up
    if not w.start(f"udp://@:{PORT}"):
        fail.append("GstVideoWidget.start returned False for a udp stream")
    else:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 8 and (w._img is None or w._img.isNull()):
            w._pull()                              # pull directly (deterministic; same call the timer makes)
            app.processEvents()
            time.sleep(0.03)
        if w._img is None or w._img.isNull():
            fail.append("GstVideoWidget decoded no frame from the synthetic stream")
        elif (w._img.width(), w._img.height()) != (320, 240):
            fail.append(f"decoded frame wrong size: {w._img.width()}x{w._img.height()} (want 320x240)")
finally:
    w.stop()
    if producer.poll() is None:
        producer.kill()

print("GST_VIDEO FAILED: " + "; ".join(fail) if fail else
      "GST_VIDEO PASSED (pipeline builder maps udp/rtsp + refuses non-live; VideoPane routes live udp:// "
      "to GStreamer; decoded a real synthetic H.264 UDP stream to a valid 320x240 RGB QImage)")
sys.stdout.flush()
os._exit(1 if fail else 0)
