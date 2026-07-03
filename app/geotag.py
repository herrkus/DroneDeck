"""geotag.py -- write GPS EXIF into survey photos from a flight log (photogrammetry).

Completes the mapping pipeline DroneDeck already supports (survey-grid planning + camera capture): after
a survey flight, correlate the logged camera-shutter events (CAMERA_FEEDBACK) from a .tlog with the SD
card's JPEGs, in capture order, and stamp each photo's GPS EXIF so Pix4D / WebODM / ODM can build the
map. This is QGC's "Analyze > GeoTag Images" equivalent. Pure functions here; the dialog wraps them.
"""
import os

import mavlink
import core


def extract_feedback(tlog_path):
    """Return the camera geotag events in a .tlog as [(lat_deg, lon_deg, alt_m), ...] in capture order.

    Uses CAMERA_FEEDBACK (ArduPilot emits one per shutter with the position at trigger time)."""
    from tlog import read_tlog
    parser = core.Parser()
    events = []
    for _ts, frame in read_tlog(tlog_path):
        for m in parser.feed(frame):
            if m.msgid == mavlink.CAMERA_FEEDBACK:
                events.append((m.fields["lat"] / 1e7, m.fields["lng"] / 1e7,
                               float(m.fields.get("alt_msl", 0.0))))
    return events


def list_photos(photo_dir):
    """JPEGs in a folder, sorted by name (capture order for sequentially-named survey shots)."""
    return sorted(f for f in os.listdir(photo_dir)
                  if f.lower().endswith((".jpg", ".jpeg")))


def _dms(value):
    """Decimal degrees -> (deg, min, sec) floats (the form PIL serialises as 3 RATIONALs)."""
    value = abs(value)
    d = int(value)
    m = int((value - d) * 60)
    s = (value - d - m / 60) * 3600
    return (float(d), float(m), float(round(s, 5)))


def write_gps_exif(src, dst, lat, lon, alt):
    """Copy `src` to `dst` with GPS EXIF (lat/lon/alt) written. Requires Pillow (PIL)."""
    try:
        from PIL import Image
    except ImportError as e:                       # keep the rest of the app usable without Pillow
        raise RuntimeError("GeoTag needs the Pillow (PIL) library") from e
    img = Image.open(src)
    exif = img.getexif()
    gps = exif.get_ifd(0x8825)                     # GPSInfo IFD (mutated in place, re-serialised on save)
    gps.update({
        1: "N" if lat >= 0 else "S", 2: _dms(lat),
        3: "E" if lon >= 0 else "W", 4: _dms(lon),
        5: 0 if alt >= 0 else 1, 6: float(abs(alt)),
    })
    exif[0x8825] = gps
    img.save(dst, "JPEG", exif=exif)


def geotag(tlog_path, photo_dir, out_dir):
    """Tag the photos in `photo_dir` from `tlog_path`'s camera events, writing copies to `out_dir`.

    Matches the Nth event to the Nth photo (capture order). Returns a summary dict; when the event and
    photo counts differ, only min(n) are tagged and BOTH counts are reported (no silent truncation)."""
    events = extract_feedback(tlog_path)
    photos = list_photos(photo_dir)
    os.makedirs(out_dir, exist_ok=True)
    n = min(len(events), len(photos))
    tagged, errors = 0, []
    for i in range(n):
        lat, lon, alt = events[i]
        try:
            write_gps_exif(os.path.join(photo_dir, photos[i]),
                           os.path.join(out_dir, photos[i]), lat, lon, alt)
            tagged += 1
        except Exception as ex:                    # one bad JPEG shouldn't abort the whole batch
            errors.append(f"{photos[i]}: {ex}")
    return {"events": len(events), "photos": len(photos), "tagged": tagged,
            "unmatched": abs(len(events) - len(photos)), "errors": errors}
