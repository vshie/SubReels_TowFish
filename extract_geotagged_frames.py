#!/usr/bin/env python3
"""Extract 2 Hz JPEG frames from a finished BlueOS recording and embed
GPS, orientation and timestamp metadata from the matching telemetry
sidecar.

Usage:
    python3 extract_geotagged_frames.py VIDEO.mp4 [--telemetry VIDEO_telemetry.csv]
                                         [--srt VIDEO.srt]
                                         [--out OUT_DIR] [--fps 2]
                                         [--quality 2]

Two sidecar formats are understood, preferred in this order:

1. ``VIDEO_telemetry.csv`` -- written by ``app/main.py`` at 5 Hz during
   recording. Carries the full telemetry set (position, towfish
   heading/roll/pitch, altitude, depth, temperature, camera tilt), so
   extracted frames come out with the *same* metadata a live timelapse
   JPEG would have had.

2. ``VIDEO.srt`` -- the position-only subtitle sidecar::

       1
       00:00:00,000 --> 00:00:00,200
       latitude: 19.312648 longitude: -155.888358 altitude: -3.5

   Used as a fallback for recordings made before the CSV existed. Those
   frames get GPS and timestamps but no orientation, because the SRT
   never carried it.

Each output JPEG gets whatever the sidecar supports:
    * GPSLatitude / GPSLongitude / GPSAltitude (height above the seabed)
    * GPSImgDirection (true heading) -- CSV only
    * XMP Camera:Yaw/Pitch/Roll in the Pix4D namespace, read by Agisoft
      Metashape, Pix4D and OpenDroneMap/WebODM -- CSV only
    * XMP Camera:GPSXYAccuracy / GPSZAccuracy reference priors -- CSV only
    * XMP Towfish:MountPitchBody / DepthBelowSurface / SonarBottomDepth
    * UserComment / ImageDescription with roll, pitch, tilt, depth, temp
    * GPSTimeStamp / GPSDateStamp in UTC
    * DateTimeOriginal / DateTimeDigitized in local time
    * Software tag identifying this script

The EXIF/XMP builders are shared with the live capture path (see
``app/photogrammetry_meta.py``) so both routes emit an identical tag set.

Recording start time comes from the telemetry CSV's wall-clock column
when available, otherwise from the filename pattern ``..._YYYYMMDD_HHMMSS.*``
(the same convention the recorder writes).
"""
from __future__ import annotations

import argparse
import bisect
import csv
import re
import subprocess
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "app"))
import photogrammetry_meta as pm  # noqa: E402  (after sys.path setup)


# ---------------------------------------------------------------------------
# Telemetry track
# ---------------------------------------------------------------------------

_TS_RE = re.compile(r"(\d\d):(\d\d):(\d\d),(\d{1,3})\s*-->\s*"
                    r"(\d\d):(\d\d):(\d\d),(\d{1,3})")
_POS_RE = re.compile(r"latitude:\s*(-?\d+(?:\.\d+)?)\s+"
                     r"longitude:\s*(-?\d+(?:\.\d+)?)\s+"
                     r"altitude:\s*(-?\d+(?:\.\d+)?)")

# Fields carried per sample. Everything is interpolated linearly except
# the ones in ANGULAR_FIELDS, which are directions in degrees and must
# be interpolated the short way around the circle.
FIELDS = ("lat", "lon", "alt", "heading", "roll", "pitch",
          "depth", "temp", "tilt", "mount_pitch", "sonar_depth")
ANGULAR_FIELDS = frozenset({"heading"})


def _ts_to_seconds(h: str, m: str, s: str, ms: str) -> float:
    return int(h) * 3600 + int(m) * 60 + int(s) + int(ms) / 1000.0


def _interp_linear(series: list[tuple[float, float]], t: float) -> float:
    """Linear interpolation over a sorted (time, value) series; clamped."""
    if t <= series[0][0]:
        return series[0][1]
    if t >= series[-1][0]:
        return series[-1][1]
    i = bisect.bisect_left([p[0] for p in series], t)
    (t0, v0), (t1, v1) = series[i - 1], series[i]
    span = t1 - t0
    if span <= 0:
        return v0
    return v0 + (v1 - v0) * ((t - t0) / span)


def _interp_angular(series: list[tuple[float, float]], t: float) -> float:
    """Interpolate a compass bearing the short way around the circle.

    Interpolating degrees linearly is wrong across the 0/360 wrap: the
    midpoint of 359 and 1 comes out as 180, pointing the camera exactly
    backwards. Unwrapping the step into [-180, 180) before interpolating
    always takes the shorter arc, at a constant rate. An exact 180 deg
    step has no shorter arc; it resolves westward, which is arbitrary but
    deterministic (and can't arise between real 5 Hz samples anyway).
    Result is normalised to 0..360.
    """
    if t <= series[0][0]:
        return series[0][1] % 360.0
    if t >= series[-1][0]:
        return series[-1][1] % 360.0
    i = bisect.bisect_left([p[0] for p in series], t)
    (t0, a0), (t1, a1) = series[i - 1], series[i]
    span = t1 - t0
    if span <= 0:
        return a0 % 360.0
    step = ((a1 - a0 + 180.0) % 360.0) - 180.0
    return (a0 + step * ((t - t0) / span)) % 360.0


class TelemetryTrack:
    """Sparse per-field time series sampled at arbitrary video times.

    Each field keeps its own series so a dropout in one signal (say a
    stale GPS fix) doesn't discard the orientation recorded at the same
    instant -- which is exactly what happens during the telemetry gaps
    the recorder deliberately leaves blank.
    """

    def __init__(self, series: dict[str, list[tuple[float, float]]],
                 start_local: datetime | None = None):
        self.series = {k: sorted(v) for k, v in series.items() if v}
        self.start_local = start_local

    def __bool__(self) -> bool:
        return bool(self.series)

    @property
    def sample_count(self) -> int:
        return max((len(v) for v in self.series.values()), default=0)

    @property
    def has_orientation(self) -> bool:
        return any(f in self.series for f in ("heading", "roll", "pitch"))

    def sample(self, t: float) -> dict[str, float | None]:
        out: dict[str, float | None] = {}
        for field in FIELDS:
            s = self.series.get(field)
            if not s:
                out[field] = None
            elif field in ANGULAR_FIELDS:
                out[field] = _interp_angular(s, t)
            else:
                out[field] = _interp_linear(s, t)
        return out

    # -- loaders ---------------------------------------------------------

    @classmethod
    def from_csv(cls, path: Path) -> "TelemetryTrack":
        """Load the recorder's ``*_telemetry.csv`` sidecar.

        Columns are matched by name, so the file can gain columns
        without breaking this reader.
        """
        column_map = {
            "lat": "lat",
            "lon": "lon",
            "alt": "towfish_altitude_m",
            "heading": "towfish_heading_deg",
            "roll": "towfish_roll_deg",
            "pitch": "towfish_pitch_deg",
            "depth": "depth_m",
            "temp": "temperature_c",
            "tilt": "camera_tilt_deg",
            "mount_pitch": "camera_mount_pitch_body_deg",
            "sonar_depth": "sonar_bottom_depth_m",
        }
        series: dict[str, list[tuple[float, float]]] = {f: [] for f in FIELDS}
        start_local = None

        with path.open(newline="", encoding="utf-8", errors="replace") as f:
            for row in csv.DictReader(f):
                try:
                    t = float(row["video_time_s"])
                except (KeyError, TypeError, ValueError):
                    continue
                for field, column in column_map.items():
                    raw = (row.get(column) or "").strip()
                    if not raw:
                        continue
                    try:
                        series[field].append((t, float(raw)))
                    except ValueError:
                        continue
                if start_local is None:
                    stamp = (row.get("timestamp") or "").strip()
                    if stamp:
                        try:
                            # Anchor wall-clock to the video timeline using
                            # the first row, so DateTimeOriginal reflects
                            # when the frame was actually recorded rather
                            # than whatever the filename claims.
                            start_local = (
                                datetime.strptime(stamp, "%Y-%m-%d %H:%M:%S.%f")
                                - timedelta(seconds=t)
                            )
                        except ValueError:
                            pass
        return cls(series, start_local)

    @classmethod
    def from_srt(cls, path: Path) -> "TelemetryTrack":
        """Load the position-only SRT sidecar (legacy recordings)."""
        series: dict[str, list[tuple[float, float]]] = {f: [] for f in FIELDS}
        text = path.read_text(encoding="utf-8", errors="replace")
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
            series["lat"].append((mid, float(pos_m.group(1))))
            series["lon"].append((mid, float(pos_m.group(2))))
            series["alt"].append((mid, float(pos_m.group(3))))
        return cls(series)


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

def resolve_track(video: Path, csv_arg: Path | None,
                  srt_arg: Path | None) -> tuple[TelemetryTrack, Path]:
    """Pick the richest available sidecar and load it.

    An explicit ``--telemetry``/``--srt`` wins; otherwise the CSV is
    preferred over the SRT because it carries orientation.
    """
    if csv_arg is not None:
        return TelemetryTrack.from_csv(csv_arg.resolve()), csv_arg.resolve()
    if srt_arg is not None:
        return TelemetryTrack.from_srt(srt_arg.resolve()), srt_arg.resolve()

    csv_path = video.with_name(video.stem + "_telemetry.csv")
    if csv_path.is_file():
        return TelemetryTrack.from_csv(csv_path), csv_path
    srt_path = video.with_suffix(".srt")
    if srt_path.is_file():
        return TelemetryTrack.from_srt(srt_path), srt_path
    raise FileNotFoundError(
        f"no telemetry sidecar next to {video.name} "
        f"(looked for {csv_path.name} and {srt_path.name})"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("video", type=Path, help="Path to .mp4 (or .ts) recording")
    ap.add_argument("--telemetry", type=Path, default=None,
                    help="Path to *_telemetry.csv sidecar "
                         "(default: same stem as video)")
    ap.add_argument("--srt", type=Path, default=None,
                    help="Path to .srt sidecar; position-only fallback "
                         "for recordings made before the CSV existed")
    ap.add_argument("--out", type=Path, default=None,
                    help="Output directory (default: <video stem>_frames/)")
    ap.add_argument("--fps", type=float, default=2.0,
                    help="Sampling frequency in Hz (default: 2.0)")
    ap.add_argument("--quality", type=int, default=2,
                    help="ffmpeg JPEG quality, 1=best 31=worst (default: 2)")
    ap.add_argument("--rename-by-time", action="store_true",
                    help="Rename outputs to frame_<sec>.jpg "
                         "instead of frame_<seq>.jpg")
    ap.add_argument("--tow-offset", type=float, default=7.0,
                    help="Layback used during the recording, metres. Only "
                         "affects the GPSXYAccuracy prior written for "
                         "photogrammetry (default: 7.0)")
    args = ap.parse_args()

    video: Path = args.video.resolve()
    if not video.is_file():
        print(f"error: video not found: {video}", file=sys.stderr)
        return 2

    try:
        track, sidecar = resolve_track(video, args.telemetry, args.srt)
    except FileNotFoundError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    if not track:
        print(f"error: no telemetry parsed from {sidecar}", file=sys.stderr)
        return 2

    out_dir: Path = (args.out or video.with_name(video.stem + "_frames")).resolve()

    # The CSV's own wall clock beats the filename: it records when each
    # sample was actually taken, and survives a renamed video file.
    start_local = track.start_local or parse_recording_start(video)
    if start_local is None:
        print(f"warning: could not determine start time for {video.name}; "
              "DateTimeOriginal will be relative to epoch.", file=sys.stderr)
    duration = probe_duration_seconds(video)

    print(f"[info] video    : {video}")
    print(f"[info] sidecar  : {sidecar.name}  ({track.sample_count} samples)")
    print(f"[info] duration : {duration:.2f}s")
    print(f"[info] start    : {start_local} (local)")
    print(f"[info] sampling : {args.fps} Hz "
          f"-> ~{int(duration * args.fps) + 1} frames")
    print(f"[info] out dir  : {out_dir}")
    if not track.has_orientation:
        print("[warn] sidecar has no orientation data; frames will carry "
              "GPS only (no Camera:Yaw/Pitch/Roll for photogrammetry)",
              file=sys.stderr)

    print("[info] extracting frames with ffmpeg ...")
    frames = extract_frames(video, out_dir, args.fps, args.quality)
    print(f"[info] extracted {len(frames)} JPEGs; embedding metadata ...")

    period = 1.0 / args.fps
    # Cache the local TZ offset once so each per-frame UTC conversion is cheap
    # (and so a recording that crosses DST keeps the same offset throughout,
    # which is what the operator expects from a single dive).
    local_tz = datetime.now().astimezone().tzinfo

    csv_lines = ["seq,t_seconds,filename,lat,lon,altitude_m,"
                 "heading_deg,roll_deg,pitch_deg,depth_m,temp_c,tilt_deg,"
                 "iso_local"]
    written = 0
    skipped_no_gps = 0
    for idx, jpg in enumerate(frames):
        t = idx * period
        if t > duration + period:
            break
        s = track.sample(t)
        if s["lat"] is None or s["lon"] is None:
            skipped_no_gps += 1
            continue

        if start_local is not None:
            ts_local_naive = start_local + timedelta(seconds=t)
            ts_local_aware = ts_local_naive.replace(tzinfo=local_tz)
            ts_utc = ts_local_aware.astimezone(timezone.utc)
        else:
            ts_local_naive = datetime(1970, 1, 1) + timedelta(seconds=t)
            ts_utc = ts_local_naive.replace(tzinfo=timezone.utc)

        xy_acc, z_acc = pm.reference_accuracy_m(args.tow_offset, s["alt"])
        try:
            data = pm.embed_metadata(
                jpg.read_bytes(),
                s["lat"], s["lon"], s["alt"], s["heading"],
                ts_local_naive, ts_utc,
                tilt_deg=s["tilt"], depth_m=s["depth"], temp_c=s["temp"],
                roll_deg=s["roll"], pitch_deg=s["pitch"],
                mount_pitch_deg=s["mount_pitch"],
                sonar_depth_m=s["sonar_depth"],
                xy_accuracy_m=xy_acc, z_accuracy_m=z_acc,
                software=pm.SOFTWARE_EXTRACT,
            )
            jpg.write_bytes(data)
        except Exception as e:
            print(f"warn: metadata embed failed for {jpg.name}: {e}",
                  file=sys.stderr)
            continue

        if args.rename_by_time:
            new_name = jpg.with_name(f"frame_{t:08.3f}s.jpg")
            jpg.rename(new_name)
            jpg = new_name

        def col(key: str, fmt: str) -> str:
            v = s[key]
            return format(v, fmt) if v is not None else ""

        csv_lines.append(
            f"{idx + 1},{t:.3f},{jpg.name},"
            f"{col('lat', '.7f')},{col('lon', '.7f')},{col('alt', '.2f')},"
            f"{col('heading', '.1f')},{col('roll', '.1f')},"
            f"{col('pitch', '.1f')},{col('depth', '.2f')},"
            f"{col('temp', '.2f')},{col('tilt', '.1f')},"
            f"{ts_local_naive.isoformat(timespec='milliseconds')}"
        )
        written += 1

    manifest = out_dir / "frames.csv"
    manifest.write_text("\n".join(csv_lines) + "\n", encoding="utf-8")

    print(f"[done] wrote {written} geotagged JPEGs"
          + (f" ({skipped_no_gps} skipped, no GPS)" if skipped_no_gps else ""))
    print(f"[done] manifest: {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
