"""Photogrammetry metadata: EXIF GPS/orientation + Pix4D XMP for JPEGs.

Single source of truth shared by the two producers of geotagged stills:

* ``app/main.py`` -- live 2 Hz timelapse capture (manual and transect).
* ``extract_geotagged_frames.py`` -- offline frame extraction from a
  recorded video plus its telemetry sidecar.

Both paths must emit byte-identical tag sets, otherwise a survey's
imagery imports differently depending on which capture mode produced
it. Keeping the builders here is what makes that guarantee hold.

Orientation conventions, which are easy to get wrong and silent when
they are:

* ``Camera:Yaw`` is the towfish true heading (0..360).
* ``Camera:Pitch`` is measured **from nadir**, because that is what both
  Pix4D and Metashape mean by pitch -- a camera looking straight down is
  0, not -90. The towfish mount is a movable earth-frame-stabilised
  servo (MNT1_PITCH_MIN/MAX +/-70 deg), so this is derived from the
  measured tilt rather than assumed.
* ``Towfish:MountPitchBody`` carries the raw servo deflection relative to
  the fish itself. That describes the physical install rather than the
  attitude of the moment, and no standard namespace has a slot for it.

Metashape reads ``Camera:*`` when "Load camera orientation angles from
XMP meta data" and "Load camera location accuracy from XMP meta data"
are enabled in Preferences -> Advanced. The ``Towfish:*`` tags are
reachable from a Metashape Python script via ``camera.photo.meta``.
"""
from __future__ import annotations

import io
import logging
import math

import piexif

logger = logging.getLogger(__name__)

SOFTWARE_LIVE = b"BlueOS-VideoRecorder Towfish"
SOFTWARE_EXTRACT = b"BlueOS-VideoRecorder extract_geotagged_frames"

# Earth-frame tilt of a camera aimed straight down. Tilt is reported
# negative-down by the towfish, so nadir is -90 and the Pix4D/Metashape
# pitch is the signed departure from it.
NADIR_TILT_DEG = -90.0

TOWFISH_XMP_NS = "http://subreels.io/towfish/1.0/"

# Reference-accuracy priors, in metres, published so photogrammetry
# software can weight position and height apart instead of splitting the
# difference between them. The two axes are not remotely comparable:
# position is the boat's fix dead-reckoned back along an assumed heading,
# while height comes from pressure and sonar.
GPS_XY_ACCURACY_BASE_M = 3.0         # boat GNSS + heading error at zero layback
GPS_XY_ACCURACY_LAYBACK_FRAC = 0.25  # layback contributes 25% of its length
GPS_Z_ACCURACY_DEPTH_M = 0.3         # pressure depth alone
GPS_Z_ACCURACY_ALTITUDE_M = 0.5      # altitude = two sensors differenced


def _finite(value):
    """Return ``value`` as a float, or ``None`` if it is not finite.

    Guards every numeric that reaches piexif. A NaN reaching a rational
    conversion raises inside the tag builder, and because that used to
    happen before any tag was written a single bad telemetry sample
    stripped the frame of *all* metadata, not just the offending field.
    """
    if value is None:
        return None
    try:
        num = float(value)
    except (TypeError, ValueError):
        return None
    return num if math.isfinite(num) else None


def reference_accuracy_m(tow_offset_m, altitude_m=None):
    """``(xy_accuracy, z_accuracy)`` priors in metres.

    XY error grows with the layback because the fix is pushed astern
    along an *assumed* bearing: a fish 30 m back that is yawed 10 degrees
    off that bearing is 5 m from where we claim it is. Z is either a
    pressure reading or the difference of two soundings, an order of
    magnitude better either way.
    """
    offset = _finite(tow_offset_m) or 0.0
    xy = GPS_XY_ACCURACY_BASE_M + GPS_XY_ACCURACY_LAYBACK_FRAC * max(0.0, offset)
    z = (GPS_Z_ACCURACY_ALTITUDE_M if _finite(altitude_m) is not None
         else GPS_Z_ACCURACY_DEPTH_M)
    return (round(xy, 2), z)


def camera_pitch_from_tilt(tilt_deg, body_pitch_deg=None):
    """Camera pitch in the Pix4D/Metashape sense: degrees off nadir.

    ``tilt_deg`` is the earth-frame camera tilt (negative down), so a
    nadir camera gives 0 and the -70 deg the mount actually parks at
    gives +20 (tilted that far toward the horizon).

    With no tilt reading the mount angle is unknown, so this falls back
    to assuming the camera is nadir-fixed to the body and returns the
    body pitch. That is the same assumption the old code made, but it is
    now the fallback rather than the rule.
    """
    tilt = _finite(tilt_deg)
    if tilt is not None:
        return tilt - NADIR_TILT_DEG
    body = _finite(body_pitch_deg)
    return body if body is not None else None


def decimal_deg_to_dms_rationals(deg):
    """Convert decimal degrees -> EXIF-style ((d,1),(m,1),(s*10000,10000))."""
    deg = abs(float(deg))
    d = int(deg)
    m_full = (deg - d) * 60.0
    m = int(m_full)
    s = (m_full - m) * 60.0
    return ((d, 1), (m, 1), (int(round(s * 10000)), 10000))


def build_user_comment(lat, lon, alt_m, heading_deg, ts_utc,
                       tilt_deg=None, depth_m=None, temp_c=None,
                       roll_deg=None, pitch_deg=None,
                       mount_pitch_deg=None, sonar_depth_m=None):
    """Human-readable telemetry bundle for UserComment/ImageDescription.

    Position and altitude have standard tags, but the rest do not, so
    everything is also written as plain text that any photo viewer will
    surface. ``alt`` is height above the seabed; ``depth`` is the
    pressure depth below the surface and ``sonar`` the boat sounding the
    altitude was derived from, both kept so an altitude can be audited
    back to its inputs.
    """
    parts = []
    lat_f, lon_f = _finite(lat), _finite(lon)
    if lat_f is not None and lon_f is not None:
        parts.append(f"pos={lat_f:.6f},{lon_f:.6f}")
    for label, value, fmt in (
        ("alt", alt_m, "{:.2f}m"),
        ("hdg", heading_deg, "{:.1f}deg"),
        ("roll", roll_deg, "{:+.1f}deg"),
        ("pitch", pitch_deg, "{:+.1f}deg"),
        ("tilt", tilt_deg, "{:+.1f}deg"),
        ("mount", mount_pitch_deg, "{:+.1f}deg"),
        ("depth", depth_m, "{:.2f}m"),
        ("sonar", sonar_depth_m, "{:.2f}m"),
        ("temp", temp_c, "{:.1f}C"),
    ):
        num = _finite(value)
        if num is not None:
            parts.append(f"{label}=" + fmt.format(num))
    parts.append(f"utc={ts_utc.strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3]}Z")
    return " ".join(parts)


def build_gps_exif_bytes(lat, lon, alt_m, heading_deg, ts_local, ts_utc,
                         tilt_deg=None, depth_m=None, temp_c=None,
                         roll_deg=None, pitch_deg=None,
                         mount_pitch_deg=None, sonar_depth_m=None,
                         software=SOFTWARE_LIVE):
    """Build piexif-encoded EXIF bytes embedding the towfish position.

    * ``lat`` / ``lon`` are the tow point's fix already laid back along
      the towfish heading by the caller.
    * ``alt_m`` is the towfish height **above the seabed**, computed from
      the boat's sounder and the towfish pressure depth. It is written as
      ``GPSAltitude`` because that is the only altitude tag Metashape
      reads. Note this makes the camera Z prior follow the bathymetry
      rather than a fixed datum; ``GPSZAccuracy`` in the XMP is set
      accordingly so the solver does not over-trust it.
    * ``heading_deg`` becomes ``GPSImgDirection`` for map clients.
      Metashape ignores it -- it takes yaw from ``Camera:Yaw`` in the XMP.
    * ``ts_local`` / ``ts_utc`` populate ``DateTimeOriginal`` (local
      wall-clock for the operator) and ``GPSDateStamp`` /
      ``GPSTimeStamp`` (always UTC, per EXIF spec).

    Returns ``None`` when there's nothing useful to embed.
    """
    lat = _finite(lat)
    lon = _finite(lon)
    alt_m = _finite(alt_m)
    heading_deg = _finite(heading_deg)

    have_gps = lat is not None and lon is not None
    have_heading = heading_deg is not None
    have_extra = any(_finite(v) is not None for v in
                     (tilt_deg, depth_m, temp_c, roll_deg, pitch_deg,
                      mount_pitch_deg, sonar_depth_m))
    if not (have_gps or have_heading or have_extra):
        return None

    gps_ifd = {piexif.GPSIFD.GPSVersionID: (2, 3, 0, 0)}

    if have_gps:
        gps_ifd[piexif.GPSIFD.GPSLatitudeRef] = b'N' if lat >= 0 else b'S'
        gps_ifd[piexif.GPSIFD.GPSLatitude] = decimal_deg_to_dms_rationals(lat)
        gps_ifd[piexif.GPSIFD.GPSLongitudeRef] = b'E' if lon >= 0 else b'W'
        gps_ifd[piexif.GPSIFD.GPSLongitude] = decimal_deg_to_dms_rationals(lon)
        if alt_m is not None:
            # Height above the seabed is a height above a reference
            # surface, so ref 0. Ref 1 would claim it is below sea level.
            gps_ifd[piexif.GPSIFD.GPSAltitudeRef] = 1 if alt_m < 0 else 0
            gps_ifd[piexif.GPSIFD.GPSAltitude] = (int(round(abs(alt_m) * 1000)), 1000)

    if have_heading:
        # 'T' = true heading. The towfish yaw is from the IMU so this
        # is the absolute heading the camera is pointing.
        gps_ifd[piexif.GPSIFD.GPSImgDirectionRef] = b'T'
        gps_ifd[piexif.GPSIFD.GPSImgDirection] = (int(round(heading_deg * 100)), 100)

    # GPS time/date in UTC per spec, regardless of system locale.
    gps_ifd[piexif.GPSIFD.GPSTimeStamp] = (
        (ts_utc.hour, 1),
        (ts_utc.minute, 1),
        (int(round((ts_utc.second + ts_utc.microsecond / 1e6) * 100)), 100),
    )
    gps_ifd[piexif.GPSIFD.GPSDateStamp] = ts_utc.strftime('%Y:%m:%d').encode('ascii')

    dt_str = ts_local.strftime('%Y:%m:%d %H:%M:%S').encode('ascii')
    subsec = f"{int(ts_local.microsecond / 1000):03d}".encode('ascii')
    exif_ifd = {
        piexif.ExifIFD.DateTimeOriginal: dt_str,
        piexif.ExifIFD.DateTimeDigitized: dt_str,
        piexif.ExifIFD.SubSecTimeOriginal: subsec,
        piexif.ExifIFD.SubSecTimeDigitized: subsec,
    }
    image_ifd = {
        piexif.ImageIFD.DateTime: dt_str,
        piexif.ImageIFD.Software: software,
    }

    comment = build_user_comment(lat, lon, alt_m, heading_deg, ts_utc,
                                 tilt_deg=tilt_deg, depth_m=depth_m,
                                 temp_c=temp_c, roll_deg=roll_deg,
                                 pitch_deg=pitch_deg,
                                 mount_pitch_deg=mount_pitch_deg,
                                 sonar_depth_m=sonar_depth_m)
    if comment:
        comment_bytes = comment.encode('ascii', 'replace')
        # EXIF UserComment needs an 8-byte character-code prefix.
        exif_ifd[piexif.ExifIFD.UserComment] = b"ASCII\x00\x00\x00" + comment_bytes
        image_ifd[piexif.ImageIFD.ImageDescription] = comment_bytes

    try:
        return piexif.dump({"0th": image_ifd, "Exif": exif_ifd, "GPS": gps_ifd})
    except Exception:
        logger.exception("EXIF dump failed")
        return None


def build_camera_xmp(yaw_deg=None, pitch_deg=None, roll_deg=None,
                     xy_accuracy_m=None, z_accuracy_m=None,
                     mount_pitch_deg=None, depth_m=None,
                     sonar_depth_m=None):
    """Build an XMP packet carrying camera orientation for photogrammetry.

    Orientation goes in the Pix4D ``Camera`` namespace
    (``Camera:Yaw/Pitch/Roll``), the de-facto standard read by Pix4D,
    Agisoft Metashape and OpenDroneMap/WebODM. ``pitch_deg`` must already
    be nadir-referenced (see :func:`camera_pitch_from_tilt`) -- 0 for a
    camera looking straight down.

    ``Camera:GPSXYAccuracy`` / ``Camera:GPSZAccuracy`` publish the very
    asymmetric error budget of a towed body: position is a dead-reckoned
    layback worth metres, depth is a pressure reading worth centimetres.
    Metashape loads these as per-camera reference accuracies, which stops
    it averaging a good Z against a poor XY.

    The ``Towfish`` namespace carries what has no standard home -- the
    body-frame mount angle and the two depths the altitude came from.
    Metashape exposes them to scripts via ``camera.photo.meta``.

    Returns the XMP string, or ``None`` when there is nothing to write.
    """
    camera_tags = []
    for name, value in (("Yaw", yaw_deg), ("Pitch", pitch_deg),
                        ("Roll", roll_deg),
                        ("GPSXYAccuracy", xy_accuracy_m),
                        ("GPSZAccuracy", z_accuracy_m)):
        num = _finite(value)
        if num is not None:
            camera_tags.append(f"    <Camera:{name}>{num:.2f}</Camera:{name}>")

    towfish_tags = []
    for name, value in (("MountPitchBody", mount_pitch_deg),
                        ("DepthBelowSurface", depth_m),
                        ("SonarBottomDepth", sonar_depth_m)):
        num = _finite(value)
        if num is not None:
            towfish_tags.append(
                f"    <Towfish:{name}>{num:.2f}</Towfish:{name}>")

    if not camera_tags and not towfish_tags:
        return None
    body = "\n".join(camera_tags + towfish_tags)
    return (
        '<?xpacket begin="\ufeff" id="W5M0MpCehiHzreSzNTczkc9d"?>'
        '<x:xmpmeta xmlns:x="adobe:ns:meta/">'
        '<rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#">'
        '<rdf:Description rdf:about="" '
        'xmlns:Camera="http://pix4d.com/camera/1.0/" '
        f'xmlns:Towfish="{TOWFISH_XMP_NS}">\n'
        f'{body}\n'
        '  </rdf:Description>'
        '</rdf:RDF>'
        '</x:xmpmeta>'
        '<?xpacket end="w"?>'
    )


def insert_xmp_app1(jpeg_bytes, xmp_packet):
    """Splice an APP1 XMP segment in just after the JPEG SOI marker.

    Raw byte insertion so the image is never re-encoded (no quality loss).
    A second APP1 alongside piexif's Exif APP1 is valid -- readers key off
    each segment's namespace header. Returns the input unchanged on any
    structural surprise or if the packet won't fit a single APP1.
    """
    try:
        if jpeg_bytes[0:2] != b"\xff\xd8":
            return jpeg_bytes
        payload = b"http://ns.adobe.com/xap/1.0/\x00" + xmp_packet.encode("utf-8")
        seg_len = len(payload) + 2
        if seg_len > 0xFFFF:
            return jpeg_bytes
        app1 = b"\xff\xe1" + seg_len.to_bytes(2, "big") + payload
        return jpeg_bytes[:2] + app1 + jpeg_bytes[2:]
    except Exception:
        logger.exception("XMP insert failed")
        return jpeg_bytes


def embed_metadata(jpeg_bytes, lat, lon, alt_m, heading_deg, ts_local, ts_utc,
                   tilt_deg=None, depth_m=None, temp_c=None,
                   roll_deg=None, pitch_deg=None,
                   mount_pitch_deg=None, sonar_depth_m=None,
                   xy_accuracy_m=None, z_accuracy_m=None,
                   software=SOFTWARE_LIVE):
    """Stamp EXIF + Pix4D XMP into ``jpeg_bytes``, returning the new bytes.

    ``alt_m`` is height above the seabed. ``tilt_deg`` is the earth-frame
    camera tilt and ``pitch_deg`` the towfish body pitch; the conversion
    to a nadir-referenced ``Camera:Pitch`` happens here so every caller
    gets the same convention. Failure is non-fatal -- the caller gets
    back whatever was successfully stamped (possibly the untouched
    input), because a frame with no metadata still beats losing the
    frame.
    """
    try:
        exif_bytes = build_gps_exif_bytes(
            lat, lon, alt_m, heading_deg, ts_local, ts_utc,
            tilt_deg=tilt_deg, depth_m=depth_m, temp_c=temp_c,
            roll_deg=roll_deg, pitch_deg=pitch_deg,
            mount_pitch_deg=mount_pitch_deg, sonar_depth_m=sonar_depth_m,
            software=software,
        )
        if exif_bytes is not None:
            # piexif.insert with raw bytes requires either a path or a
            # BytesIO output buffer; BytesIO keeps the rewrite in memory
            # so the caller still writes the file exactly once.
            out_buf = io.BytesIO()
            piexif.insert(exif_bytes, jpeg_bytes, out_buf)
            jpeg_bytes = out_buf.getvalue()

        # Only claim an orientation when something was actually measured:
        # camera_pitch_from_tilt() falls back to assuming a nadir-fixed
        # mount, which is a fine refinement alongside a real heading but
        # must not become an XMP packet of its own during a dropout.
        if any(_finite(v) is not None for v in
               (heading_deg, roll_deg, pitch_deg, tilt_deg, mount_pitch_deg)):
            xmp = build_camera_xmp(
                yaw_deg=heading_deg,
                pitch_deg=camera_pitch_from_tilt(tilt_deg, pitch_deg),
                roll_deg=roll_deg,
                xy_accuracy_m=xy_accuracy_m,
                z_accuracy_m=z_accuracy_m,
                mount_pitch_deg=mount_pitch_deg,
                depth_m=depth_m,
                sonar_depth_m=sonar_depth_m,
            )
            if xmp is not None:
                jpeg_bytes = insert_xmp_app1(jpeg_bytes, xmp)
    except Exception:
        logger.exception("Photogrammetry metadata embed failed")
    return jpeg_bytes
