#!/usr/bin/env python3
"""Extract 2 Hz JPEG frames from a finished BlueOS recording and embed
GPS / timestamp EXIF tags from the matching .srt sidecar.

Usage:
    python3 extract_geotagged_frames.py VIDEO.mp4 [--srt VIDEO.srt]
                                         [--out OUT_DIR] [--fps 2]
                                         [--quality 2]

The SRT format is the one written by ``app/main.py::update_srt_file``::

    1
    00:00:00,000 --> 00:00:00,200
    latitude: 19.312648 longitude: -155.888358 altitude: -3.5

Each output JPEG gets:
    * GPSLatitude / GPSLongitude / GPSAltitude (with AltitudeRef=1 below sea)
    * GPSTimeStamp / GPSDateStamp in UTC
    * DateTimeOriginal / DateTimeDigitized in local time
    * Software tag identifying this script

Recording start time is parsed from the filename pattern
``..._YYYYMMDD_HHMMSS.*`` (the same convention the recorder writes).
"""
from __future__ import annotations

import argparse
import bisect
import os
import re
import subprocess
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import piexif


# ---------------------------------------------------------------------------
# SRT parsing
# ---------------------------------------------------------------------------

_TS_RE = re.compile(r"(\d\d):(\d\d):(\d\d),(\d{1,3})\s*-->\s*"
                    r"(\d\d):(\d\d):(\d\d),(\d{1,3})")
_POS_RE = re.compile(r"latitude:\s*(-?\d+(?:\.\d+)?)\s+"
                     r"longitude:\s*(-?\d+(?:\.\d+)?)\s+"
                     r"altitude:\s*(-?\d+(?:\.\d+)?)")


def _ts_to_seconds(h: str, m: str, s: str, ms: str) -> float:
    return int(h) * 3600 + int(m) * 60 + int(s) + int(ms) / 1000.0


def parse_srt(srt_path: Path) -> list[tuple[float, float, float, float]]:
    """Return list of (mid_seconds, lat, lon, alt) sorted by time."""
    text = srt_path.read_text(encoding="utf-8", errors="replace")
    entries: list[tuple[float, float, float, float]] = []
    # Iterate over (timestamp, position) pairs; the SRT sequence number
    # and trailing blank line are ignored.
    for block in re.split(r"\r?\n\r?\n", text.strip()):
        ts_m = _TS_RE.search(block)
        pos_m = _POS_RE.search(block)
        if not (ts_m and pos_m):
            continue
        t0 = _ts_to_seconds(*ts_m.group(1, 2, 3, 4))
        t1 = _ts_to_seconds(*ts_m.group(5, 6, 7, 8))
        mid = (t0 + t1) / 2.0
        lat = float(pos_m.group(1))
        lon = float(pos_m.group(2))
        alt = float(pos_m.group(3))
        entries.append((mid, lat, lon, alt))
    entries.sort(key=lambda e: e[0])
    return entries


def interpolate_position(entries, t: float):
    """Linear-interpolate (lat, lon, alt) at time ``t``; clamp at ends."""
    if not entries:
        return None
    if t <= entries[0][0]:
        return entries[0][1:]
    if t >= entries[-1][0]:
        return entries[-1][1:]
    times = [e[0] for e in entries]
    i = bisect.bisect_left(times, t)
    a = entries[i - 1]
    b = entries[i]
    span = b[0] - a[0]
    frac = (t - a[0]) / span if span > 0 else 0.0
    lat = a[1] + (b[1] - a[1]) * frac
    lon = a[2] + (b[2] - a[2]) * frac
    alt = a[3] + (b[3] - a[3]) * frac
    return lat, lon, alt


# ---------------------------------------------------------------------------
# EXIF
# ---------------------------------------------------------------------------

def _decimal_deg_to_dms_rationals(deg: float):
    deg = abs(float(deg))
    d = int(deg)
    m_full = (deg - d) * 60.0
    m = int(m_full)
    s = (m_full - m) * 60.0
    return ((d, 1), (m, 1), (int(round(s * 10000)), 10000))


def build_gps_exif_bytes(lat: float, lon: float, alt: float,
                         ts_local: datetime, ts_utc: datetime) -> bytes:
    gps_ifd = {
        piexif.GPSIFD.GPSVersionID: (2, 3, 0, 0),
        piexif.GPSIFD.GPSLatitudeRef: b"N" if lat >= 0 else b"S",
        piexif.GPSIFD.GPSLatitude: _decimal_deg_to_dms_rationals(lat),
        piexif.GPSIFD.GPSLongitudeRef: b"E" if lon >= 0 else b"W",
        piexif.GPSIFD.GPSLongitude: _decimal_deg_to_dms_rationals(lon),
        # AltitudeRef = 1 when below sea level (towfish underwater).
        piexif.GPSIFD.GPSAltitudeRef: 1 if alt < 0 else 0,
        piexif.GPSIFD.GPSAltitude: (int(round(abs(alt) * 1000)), 1000),
        piexif.GPSIFD.GPSTimeStamp: (
            (ts_utc.hour, 1),
            (ts_utc.minute, 1),
            (int(round((ts_utc.second + ts_utc.microsecond / 1e6) * 100)), 100),
        ),
        piexif.GPSIFD.GPSDateStamp: ts_utc.strftime("%Y:%m:%d").encode("ascii"),
    }
    dt_str = ts_local.strftime("%Y:%m:%d %H:%M:%S").encode("ascii")
    subsec = f"{int(ts_local.microsecond / 1000):03d}".encode("ascii")
    exif_ifd = {
        piexif.ExifIFD.DateTimeOriginal: dt_str,
        piexif.ExifIFD.DateTimeDigitized: dt_str,
        piexif.ExifIFD.SubSecTimeOriginal: subsec,
        piexif.ExifIFD.SubSecTimeDigitized: subsec,
    }
    image_ifd = {
        piexif.ImageIFD.DateTime: dt_str,
        piexif.ImageIFD.Software: b"BlueOS-VideoRecorder extract_geotagged_frames",
    }
    return piexif.dump({"0th": image_ifd, "Exif": exif_ifd, "GPS": gps_ifd})


# ---------------------------------------------------------------------------
# Video helpers
# ---------------------------------------------------------------------------

_FILENAME_TS_RE = re.compile(r"(\d{8})_(\d{6})")


def parse_recording_start(video_path: Path) -> datetime | None:
    """Parse '...YYYYMMDD_HHMMSS...' from the filename as naive local time."""
    m = _FILENAME_TS_RE.search(video_path.name)
    if not m:
        return None
    return datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S")


def probe_duration_seconds(video_path: Path) -> float:
    out = subprocess.check_output([
        "ffprobe", "-v", "error",
        "-show_entries", "format=duration",
        "-of", "default=noprint_wrappers=1:nokey=1",
        str(video_path),
    ], text=True).strip()
    return float(out)


def extract_frames(video_path: Path, out_dir: Path, fps: float,
                   quality: int) -> list[Path]:
    """Run a single ffmpeg pass, returning the produced JPEG paths in order.

    With ``-vf fps=N`` ffmpeg samples one frame every ``1/N`` seconds
    starting at t=0, so ``frame_000001.jpg`` is t=0, ``frame_000002.jpg``
    is t=1/N, and so on. We rely on this mapping below.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    pattern = out_dir / "frame_%06d.jpg"
    cmd = [
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
        "-i", str(video_path),
        "-vf", f"fps={fps}",
        "-q:v", str(quality),
        str(pattern),
    ]
    subprocess.run(cmd, check=True)
    return sorted(out_dir.glob("frame_*.jpg"))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("video", type=Path, help="Path to .mp4 (or .ts) recording")
    ap.add_argument("--srt", type=Path, default=None,
                    help="Path to .srt sidecar (default: same stem as video)")
    ap.add_argument("--out", type=Path, default=None,
                    help="Output directory (default: <video stem>_frames/)")
    ap.add_argument("--fps", type=float, default=2.0,
                    help="Sampling frequency in Hz (default: 2.0)")
    ap.add_argument("--quality", type=int, default=2,
                    help="ffmpeg JPEG quality, 1=best 31=worst (default: 2)")
    ap.add_argument("--rename-by-time", action="store_true",
                    help="Rename outputs to frame_<sec>.jpg "
                         "instead of frame_<seq>.jpg")
    args = ap.parse_args()

    video: Path = args.video.resolve()
    if not video.is_file():
        print(f"error: video not found: {video}", file=sys.stderr)
        return 2

    srt: Path = (args.srt or video.with_suffix(".srt")).resolve()
    if not srt.is_file():
        print(f"error: srt not found: {srt}", file=sys.stderr)
        return 2

    out_dir: Path = (args.out or video.with_name(video.stem + "_frames")).resolve()

    start_local = parse_recording_start(video)
    if start_local is None:
        print(f"warning: could not parse start time from {video.name}; "
              "DateTimeOriginal will be relative to epoch.", file=sys.stderr)
    duration = probe_duration_seconds(video)
    entries = parse_srt(srt)
    if not entries:
        print(f"error: no GPS entries parsed from {srt}", file=sys.stderr)
        return 2

    print(f"[info] video    : {video}")
    print(f"[info] srt      : {srt}  ({len(entries)} entries)")
    print(f"[info] duration : {duration:.2f}s")
    print(f"[info] start    : {start_local} (local)")
    print(f"[info] sampling : {args.fps} Hz "
          f"-> ~{int(duration * args.fps) + 1} frames")
    print(f"[info] out dir  : {out_dir}")

    print("[info] extracting frames with ffmpeg ...")
    frames = extract_frames(video, out_dir, args.fps, args.quality)
    print(f"[info] extracted {len(frames)} JPEGs; embedding EXIF ...")

    period = 1.0 / args.fps
    # Cache the local TZ offset once so each per-frame UTC conversion is cheap
    # (and so a recording that crosses DST keeps the same offset throughout,
    # which is what the operator expects from a single dive).
    local_tz = datetime.now().astimezone().tzinfo

    csv_lines = ["seq,t_seconds,filename,lat,lon,altitude_m,iso_local"]
    written = 0
    skipped_no_gps = 0
    for idx, jpg in enumerate(frames):
        t = idx * period
        if t > duration + period:
            break
        pos = interpolate_position(entries, t)
        if pos is None:
            skipped_no_gps += 1
            continue
        lat, lon, alt = pos

        if start_local is not None:
            ts_local_naive = start_local + timedelta(seconds=t)
            ts_local_aware = ts_local_naive.replace(tzinfo=local_tz)
            ts_utc = ts_local_aware.astimezone(timezone.utc)
        else:
            ts_local_naive = datetime(1970, 1, 1) + timedelta(seconds=t)
            ts_utc = ts_local_naive.replace(tzinfo=timezone.utc)

        exif_bytes = build_gps_exif_bytes(lat, lon, alt, ts_local_naive, ts_utc)
        try:
            piexif.insert(exif_bytes, str(jpg))
        except Exception as e:
            print(f"warn: piexif.insert failed for {jpg.name}: {e}",
                  file=sys.stderr)
            continue

        if args.rename_by_time:
            new_name = jpg.with_name(f"frame_{t:08.3f}s.jpg")
            jpg.rename(new_name)
            jpg = new_name

        csv_lines.append(
            f"{idx + 1},{t:.3f},{jpg.name},"
            f"{lat:.7f},{lon:.7f},{alt:.2f},"
            f"{ts_local_naive.isoformat(timespec='milliseconds')}"
        )
        written += 1

    csv_path = out_dir / "frames.csv"
    csv_path.write_text("\n".join(csv_lines) + "\n", encoding="utf-8")

    print(f"[done] wrote {written} geotagged JPEGs"
          + (f" ({skipped_no_gps} skipped, no GPS)" if skipped_no_gps else ""))
    print(f"[done] manifest: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
