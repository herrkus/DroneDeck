#!/usr/bin/env python3
"""test_video_record.py -- record a live video stream to disk (iter166, QGC parity).

DroneDeck could display an FPV/camera stream but not RECORD it -- QGC records the downlink to a file so
you keep the footage. VideoRecorder runs a GStreamer pipeline that muxes the live H.264 RTP stream to a
Matroska (.mkv), chosen so a recording cut short by a crash is still playable. Verifies: the pipeline
builder maps udp:// / rtsp:// correctly (and refuses non-live URLs), builds an argv LIST (no shell
injection from a URL), targets .mkv; and -- for real -- records a synthetic H.264 UDP stream to a valid,
non-empty Matroska file, then finalises cleanly on stop. Gracefully skips the live part if the needed
GStreamer plugins aren't installed."""
import os
import sys
import time
import shutil
import subprocess

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))
import video
from video import VideoRecorder

fail = []

# 1) pipeline builder: udp/rtsp map correctly, non-live refused, argv list, mkv target ---------------
udp = VideoRecorder._pipeline_args("udp://@:5600", "/tmp/out.mkv")
if not udp or "udpsrc" not in udp or "port=5600" not in udp or "matroskamux" not in udp:
    fail.append(f"udp pipeline wrong: {udp}")
if not udp or udp[0] != "gst-launch-1.0" or "-e" not in udp or "location=/tmp/out.mkv" not in udp:
    fail.append(f"udp pipeline missing gst-launch/-e/filesink: {udp}")

rtsp = VideoRecorder._pipeline_args("rtsp://cam/stream", "/tmp/o.mkv")
if not rtsp or "rtspsrc" not in rtsp or "location=rtsp://cam/stream" not in rtsp:
    fail.append(f"rtsp pipeline wrong: {rtsp}")

for bad in ("http://x/y.mp4", "/some/file.mp4", "udp://@:notaport", ""):
    if VideoRecorder._pipeline_args(bad, "/tmp/o.mkv") is not None:
        fail.append(f"non-live/invalid URL {bad!r} should not be recordable")

# argv must be a list (never a shell string) so a hostile URL can't inject a command
inj = VideoRecorder._pipeline_args("rtsp://h/$(rm -rf ~)", "/tmp/o.mkv")
if inj is not None and not isinstance(inj, list):
    fail.append("pipeline must be an argv list, not a shell string")

# recorder with no active process reports not-recording and stop() is a safe no-op
rec = VideoRecorder(recdir="/tmp/dd_vrec")
if rec.is_recording or rec.stop() is not None:
    fail.append("fresh recorder should not be recording; stop() should be a no-op")

# 2) real end-to-end: record a synthetic H.264 UDP stream to a valid Matroska --------------------------
def have(el):
    return subprocess.run(["gst-inspect-1.0", "--exists", el]).returncode == 0

if not video.HAVE_GST or not all(have(e) for e in ("x264enc", "rtph264pay", "udpsrc",
                                                   "rtph264depay", "matroskamux")):
    print("VIDEO_RECORD PASSED (pipeline builder verified; live record SKIPPED -- GStreamer H.264 "
          "plugins not installed)" if not fail else "VIDEO_RECORD FAILED: " + "; ".join(fail))
    sys.stdout.flush()
    os._exit(1 if fail else 0)

PORT = 5623
out = "/tmp/dd_vrec/e2e.mkv"
os.makedirs("/tmp/dd_vrec", exist_ok=True)
try:
    os.unlink(out)
except OSError:
    pass

producer = None
try:
    if not rec.start(f"udp://@:{PORT}", out_path=out):     # recorder listens first
        fail.append("VideoRecorder.start returned False for a udp stream")
    else:
        if not rec.is_recording:
            fail.append("recorder should report is_recording after start")
        time.sleep(0.6)                                    # let udpsrc bind before we send
        producer = subprocess.Popen(
            ["gst-launch-1.0", "-e", "videotestsrc", "num-buffers=60", "!",
             "video/x-raw,framerate=30/1", "!", "x264enc", "tune=zerolatency", "!",
             "rtph264pay", "!", "udpsink", "host=127.0.0.1", f"port={PORT}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        producer.wait(timeout=15)                          # ~2 s of frames then EOS
        time.sleep(0.6)
        saved = rec.stop()
        if rec.is_recording:
            fail.append("recorder still recording after stop()")
        if saved != out or not os.path.exists(out):
            fail.append(f"recording file missing: {saved}")
        elif os.path.getsize(out) < 1000:
            fail.append(f"recording too small ({os.path.getsize(out)} B) -- captured nothing")
        else:
            with open(out, "rb") as f:
                magic = f.read(4)
            if magic != b"\x1a\x45\xdf\xa3":               # EBML/Matroska magic -> a real, finalised mkv
                fail.append(f"output is not a valid Matroska file (magic {magic!r})")
finally:
    if producer and producer.poll() is None:
        producer.kill()
    if rec.is_recording:
        rec.stop()

print("VIDEO_RECORD FAILED: " + "; ".join(fail) if fail else
      "VIDEO_RECORD PASSED (pipeline builder maps udp/rtsp, refuses non-live + injection, targets .mkv; "
      "recorded a real synthetic H.264 UDP stream to a valid non-empty Matroska file, finalised on stop)")
sys.stdout.flush()
os._exit(1 if fail else 0)
