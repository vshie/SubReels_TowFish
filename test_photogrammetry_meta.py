#!/usr/bin/env python3
"""Checks for the shared photogrammetry metadata path.

Covers the parts that are easy to get subtly wrong: compass
interpolation across the 0/360 wrap, the EXIF/XMP tag set actually
landing in the JPEG, and the telemetry CSV timing rescale.

Run with: python3 test_photogrammetry_meta.py
"""
from __future__ import annotations

import csv
import io
import struct
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "app"))

import piexif

# Pillow is only needed to synthesise an input JPEG, and it is not a
# runtime dependency of the extension. Without it the embedding checks
# are skipped and the rest still run.
try:
    from PIL import Image
except ImportError:
    Image = None

import photogrammetry_meta as pm
from extract_geotagged_frames import TelemetryTrack, _interp_angular

FAILURES: list[str] = []


def check(label, condition, detail=""):
    if condition:
        print(f"  ok   {label}")
    else:
        print(f"  FAIL {label} {detail}")
        FAILURES.append(label)


def approx(a, b, tol=1e-6):
    return a is not None and abs(a - b) <= tol


def tiny_jpeg() -> bytes:
    """A real (if tiny) encoded JPEG -- piexif walks actual segments."""
    buf = io.BytesIO()
    Image.new("RGB", (8, 8), (40, 90, 140)).save(buf, format="JPEG")
    return buf.getvalue()


def find_xmp(jpeg: bytes) -> str | None:
    """Pull the XMP packet back out of its APP1 segment."""
    marker = b"http://ns.adobe.com/xap/1.0/\x00"
    i = jpeg.find(marker)
    if i < 0:
        return None
    hdr = jpeg.rfind(b"\xff\xe1", 0, i)
    seg_len = struct.unpack(">H", jpeg[hdr + 2:hdr + 4])[0]
    payload = jpeg[hdr + 4:hdr + 2 + seg_len]
    return payload[len(marker):].decode("utf-8", "replace")


print("\ncircular heading interpolation")
# Straight across the wrap: 350 -> 10 should pass through 0, not 180.
wrap = [(0.0, 350.0), (1.0, 10.0)]
check("midpoint of 350/10 is 0", approx(_interp_angular(wrap, 0.5) % 360, 0.0, 1e-9),
      f"got {_interp_angular(wrap, 0.5)}")
check("quarter point of 350/10 is 355",
      approx(_interp_angular(wrap, 0.25), 355.0, 1e-9),
      f"got {_interp_angular(wrap, 0.25)}")
# A naive linear interpolation would give 180 here; that is the bug.
check("wrap result is not the linear-average 180",
      abs(_interp_angular(wrap, 0.5) - 180.0) > 1.0)
# Ordinary case with no wrap involved.
plain = [(0.0, 90.0), (1.0, 100.0)]
check("non-wrapping midpoint of 90/100 is 95",
      approx(_interp_angular(plain, 0.5), 95.0, 1e-9),
      f"got {_interp_angular(plain, 0.5)}")
check("clamps below range", approx(_interp_angular(wrap, -5.0), 350.0, 1e-9))
check("clamps above range", approx(_interp_angular(wrap, 99.0), 10.0, 1e-9))


ts_local = datetime(2026, 7, 28, 15, 45, 30, 250000)
ts_utc = datetime(2026, 7, 29, 1, 45, 30, 250000, tzinfo=timezone.utc)


def check_embedding():
    out = pm.embed_metadata(
        tiny_jpeg(),
        lat=19.312648, lon=-155.888358, alt_m=-3.5, heading_deg=127.4,
        ts_local=ts_local, ts_utc=ts_utc,
        tilt_deg=-88.0, depth_m=3.5, temp_c=24.6, roll_deg=1.5, pitch_deg=-2.0,
    )
    exif = piexif.load(out)
    gps = exif["GPS"]
    check("GPSLatitudeRef N", gps[piexif.GPSIFD.GPSLatitudeRef] == b"N")
    check("GPSLongitudeRef W", gps[piexif.GPSIFD.GPSLongitudeRef] == b"W")
    check("GPSAltitudeRef=1 below sea", gps[piexif.GPSIFD.GPSAltitudeRef] == 1)
    check("GPSAltitude 3.5 m", gps[piexif.GPSIFD.GPSAltitude] == (3500, 1000))
    check("GPSImgDirectionRef true", gps[piexif.GPSIFD.GPSImgDirectionRef] == b"T")
    check("GPSImgDirection 127.4", gps[piexif.GPSIFD.GPSImgDirection] == (12740, 100))
    check("GPSVersionID present", piexif.GPSIFD.GPSVersionID in gps)
    check("GPSDateStamp is UTC date", gps[piexif.GPSIFD.GPSDateStamp] == b"2026:07:29")
    check("DateTimeOriginal is local",
          exif["Exif"][piexif.ExifIFD.DateTimeOriginal] == b"2026:07:28 15:45:30")
    check("SubSecTimeOriginal 250",
          exif["Exif"][piexif.ExifIFD.SubSecTimeOriginal] == b"250")
    comment = exif["Exif"][piexif.ExifIFD.UserComment]
    check("UserComment carries depth", b"depth=3.50m" in comment)
    check("UserComment carries roll", b"roll=+1.5deg" in comment)
    check("UserComment carries tilt", b"tilt=-88.0deg" in comment)

    xmp = find_xmp(out)
    check("XMP packet present", xmp is not None)
    if xmp:
        check("XMP uses Pix4D namespace", "http://pix4d.com/camera/1.0/" in xmp)
        check("Camera:Yaw is heading", "<Camera:Yaw>127.40</Camera:Yaw>" in xmp, xmp)
        # Nadir camera: -90 plus the body pitch of -2.0.
        check("Camera:Pitch is nadir+body",
              "<Camera:Pitch>-92.00</Camera:Pitch>" in xmp, xmp)
        check("Camera:Roll is body roll", "<Camera:Roll>1.50</Camera:Roll>" in xmp, xmp)


def check_no_orientation():
    bare = pm.embed_metadata(
        tiny_jpeg(), lat=19.3, lon=-155.8, alt_m=-2.0, heading_deg=None,
        ts_local=ts_local, ts_utc=ts_utc, roll_deg=None, pitch_deg=None,
        depth_m=2.0,
    )
    check("GPS still written", piexif.GPSIFD.GPSLatitude in piexif.load(bare)["GPS"])
    check("no XMP when nothing was measured", find_xmp(bare) is None)


print("\nEXIF + XMP embedding")
if Image is None:
    print("  SKIPPED -- needs Pillow (pip3 install Pillow)")
else:
    check_embedding()

print("\nno orientation -> no XMP claim")
if Image is None:
    print("  SKIPPED -- needs Pillow (pip3 install Pillow)")
else:
    check_no_orientation()


print("\ntelemetry CSV header agreement between writer and reader")
# main.py can't be imported here (it pulls in gi/flask/websockets), so
# read the header literal straight out of the source instead. This is
# the check that matters: the reader keys on column names, so a rename
# on the writer side would silently produce metadata-less frames.
import ast

main_src = (Path(__file__).resolve().parent / "app" / "main.py").read_text()
video_header = None
for node in ast.walk(ast.parse(main_src)):
    if (isinstance(node, ast.Assign)
            and any(getattr(t, "id", None) == "_VIDEO_CSV_HEADER"
                    for t in node.targets)):
        video_header = [el.value for el in node.value.elts]
check("_VIDEO_CSV_HEADER found in main.py", video_header is not None)

reader_columns = {
    "video_time_s", "timestamp", "lat", "lon", "towfish_altitude_m",
    "towfish_heading_deg", "towfish_roll_deg", "towfish_pitch_deg",
    "depth_m", "temperature_c", "camera_tilt_deg",
}
if video_header:
    missing = reader_columns - set(video_header)
    check("every column the reader wants is written", not missing,
          f"missing {sorted(missing)}")


print("\ntelemetry CSV -> TelemetryTrack")
with tempfile.TemporaryDirectory() as td:
    csv_path = Path(td) / "clip_telemetry.csv"
    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(video_header)

        def row(t, stamp, lat, lon, heading, blank_gps=False):
            values = {
                "timestamp": stamp,
                "video_time_s": f"{t:.3f}",
                "video_file": "clip",
                "wp": "wp03",
                "lat": "" if blank_gps else f"{lat:.6f}",
                "lon": "" if blank_gps else f"{lon:.6f}",
                "altitude_m": "0.30",
                "towfish_altitude_m": "-3.50",
                "towfish_heading_deg": f"{heading:.1f}",
                "towfish_roll_deg": "1.5",
                "towfish_pitch_deg": "-2.0",
                "depth_m": "3.50",
                "temperature_c": "24.60",
                "camera_tilt_deg": "-88.0",
                "telem_ms": "42.0",
            }
            w.writerow([values.get(c, "") for c in video_header])

        row(0.0, "2026-07-28 15:45:30.000", 19.312648, -155.888358, 350.0)
        # Middle row has a GPS dropout but still reports orientation.
        row(1.0, "2026-07-28 15:45:31.000", 0, 0, 0.0, blank_gps=True)
        row(2.0, "2026-07-28 15:45:32.000", 19.312700, -155.888300, 10.0)

    track = TelemetryTrack.from_csv(csv_path)
    check("track loaded", bool(track))
    check("has orientation", track.has_orientation)
    check("start_local anchored to first row",
          track.start_local == datetime(2026, 7, 28, 15, 45, 30))

    mid = track.sample(1.0)
    # GPS was blank at t=1.0, so lat/lon interpolate across the gap
    # rather than the whole sample being discarded.
    check("lat interpolated across the GPS gap",
          approx(mid["lat"], (19.312648 + 19.312700) / 2, 1e-9),
          f"got {mid['lat']}")
    check("orientation preserved during GPS gap", approx(mid["heading"], 0.0, 1e-9)
          or approx(mid["heading"], 360.0, 1e-9), f"got {mid['heading']}")
    check("altitude from towfish column", approx(mid["alt"], -3.5, 1e-9))
    check("depth read", approx(mid["depth"], 3.5, 1e-9))
    check("tilt read", approx(mid["tilt"], -88.0, 1e-9))

    at2 = track.sample(2.0)
    check("heading at last sample is 10", approx(at2["heading"], 10.0, 1e-9))


print("\nlegacy SRT fallback")
with tempfile.TemporaryDirectory() as td:
    srt_path = Path(td) / "clip.srt"
    srt_path.write_text(
        "1\n00:00:00,000 --> 00:00:00,200\n"
        "latitude: 19.312648 longitude: -155.888358 altitude: -3.5\n\n"
        "2\n00:00:00,200 --> 00:00:00,400\n"
        "latitude:  longitude:  altitude: \n\n"
        "3\n00:00:00,400 --> 00:00:00,600\n"
        "latitude: 19.312700 longitude: -155.888300 altitude: -3.7\n\n"
    )
    legacy = TelemetryTrack.from_srt(srt_path)
    check("legacy SRT still parses", bool(legacy))
    check("blank SRT entry skipped", legacy.sample_count == 2)
    check("legacy has no orientation", not legacy.has_orientation)
    s = legacy.sample(0.1)
    check("legacy lat read", approx(s["lat"], 19.312648, 1e-9))
    check("legacy heading is None", s["heading"] is None)


print()
if FAILURES:
    print(f"{len(FAILURES)} FAILED: {FAILURES}")
    raise SystemExit(1)
print("all checks passed")
