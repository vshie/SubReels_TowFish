"""Photogrammetry metadata: EXIF GPS/orientation + Pix4D XMP for JPEGs.

Single source of truth shared by the two producers of geotagged stills:

* ``app/main.py`` -- live 2 Hz timelapse capture (manual and transect).
* ``extract_geotagged_frames.py`` -- offline frame extraction from a
  recorded video plus its telemetry sidecar.

Both paths must emit byte-identical tag sets, otherwise a survey's
imagery imports differently depending on which capture mode produced
it. Keeping the builders here is what makes that guarantee hold.

The orientation convention is the same everywhere: ``Camera:Yaw`` is the
towfish true heading (0..360), and because the camera is fixed looking
straight down the earth-frame ``Camera:Pitch`` is nadir (-90 deg) when
the towfish is level, offset by the body pitch. Raw body pitch/roll stay
in the telemetry CSV and the EXIF ``UserComment``.
"""
from __future__ import annotations

import io
import logging

import piexif

logger = logging.getLogger(__name__)

SOFTWARE_LIVE = b"BlueOS-VideoRecorder Towfish"
SOFTWARE_EXTRACT = b"BlueOS-VideoRecorder extract_geotagged_frames"

# Earth-frame pitch of the fixed straight-down camera when the towfish
# is level. Body pitch is added to it.
NADIR_PITCH_DEG = -90.0


def camera_pitch_from_body(body_pitch_deg):
    """Earth-frame camera pitch for the fixed nadir camera.

    A missing body pitch is treated as level rather than unknown: the
    camera is bolted looking down, so nadir is a better estimate than
    omitting orientation entirely.
    """
    return NADIR_PITCH_DEG + (body_pitch_deg if body_pitch_deg is not None else 0.0)


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
                       roll_deg=None, pitch_deg=None):
    """Human-readable telemetry bundle for UserComment/ImageDescription.

    Altitude, heading and position have (partial) standard tags, but
    tilt/depth/temp/roll/pitch do not, so everything is also written as
    plain text that any photo viewer will surface.
    """
    parts = []
    if lat is not None and lon is not None:
        parts.append(f"pos={lat:.6f},{lon:.6f}")
    if alt_m is not None:
        parts.append(f"alt={alt_m:.2f}m")
    if heading_deg is not None:
        parts.append(f"hdg={heading_deg:.1f}deg")
    if roll_deg is not None:
        parts.append(f"roll={roll_deg:+.1f}deg")
    if pitch_deg is not None:
        parts.append(f"pitch={pitch_deg:+.1f}deg")
    if tilt_deg is not None:
        parts.append(f"tilt={tilt_deg:+.1f}deg")
    if depth_m is not None:
        parts.append(f"depth={depth_m:.2f}m")
    if temp_c is not None:
        parts.append(f"temp={temp_c:.1f}C")
    parts.append(f"utc={ts_utc.strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3]}Z")
    return " ".join(parts)


def build_gps_exif_bytes(lat, lon, alt_m, heading_deg, ts_local, ts_utc,
                         tilt_deg=None, depth_m=None, temp_c=None,
                         roll_deg=None, pitch_deg=None,
                         software=SOFTWARE_LIVE):
    """Build piexif-encoded EXIF bytes embedding the towfish position.

    * ``lat`` / ``lon`` are the tow point's fix already laid back along
      the towfish heading by the caller.
    * ``alt_m`` is the towfish AHRS2 altitude (negative when underwater)
      -- written as ``GPSAltitude`` with ``GPSAltitudeRef = 1`` (below
      sea level) when negative.
    * ``heading_deg`` is the towfish yaw (true heading, 0..360) and
      becomes ``GPSImgDirection`` so map clients render the camera
      bearing at each frame.
    * ``ts_local`` / ``ts_utc`` populate ``DateTimeOriginal`` (local
      wall-clock for the operator) and ``GPSDateStamp`` /
      ``GPSTimeStamp`` (always UTC, per EXIF spec).

    Returns ``None`` when there's nothing useful to embed.
    """
    have_gps = lat is not None and lon is not None
    have_heading = heading_deg is not None
    have_extra = (tilt_deg is not None or depth_m is not None
                  or temp_c is not None or roll_deg is not None
                  or pitch_deg is not None)
    if not (have_gps or have_heading or have_extra):
        return None

    gps_ifd = {piexif.GPSIFD.GPSVersionID: (2, 3, 0, 0)}

    if have_gps:
        gps_ifd[piexif.GPSIFD.GPSLatitudeRef] = b'N' if lat >= 0 else b'S'
        gps_ifd[piexif.GPSIFD.GPSLatitude] = decimal_deg_to_dms_rationals(lat)
        gps_ifd[piexif.GPSIFD.GPSLongitudeRef] = b'E' if lon >= 0 else b'W'
        gps_ifd[piexif.GPSIFD.GPSLongitude] = decimal_deg_to_dms_rationals(lon)
        if alt_m is not None:
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
                                 pitch_deg=pitch_deg)
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


def build_camera_xmp(yaw_deg=None, pitch_deg=None, roll_deg=None):
    """Build an XMP packet carrying camera orientation for photogrammetry.

    Uses the Pix4D ``Camera`` namespace (``Camera:Yaw/Pitch/Roll``), which
    is the de-facto standard read by Pix4D, Agisoft Metashape and
    OpenDroneMap/WebODM. ``pitch_deg`` is expected to already be the
    earth-frame camera pitch (see :func:`camera_pitch_from_body`), not
    the raw towfish body pitch.

    Returns the XMP string, or ``None`` when no orientation is available.
    """
    tags = []
    if yaw_deg is not None:
        tags.append(f"    <Camera:Yaw>{yaw_deg:.2f}</Camera:Yaw>")
    if pitch_deg is not None:
        tags.append(f"    <Camera:Pitch>{pitch_deg:.2f}</Camera:Pitch>")
    if roll_deg is not None:
        tags.append(f"    <Camera:Roll>{roll_deg:.2f}</Camera:Roll>")
    if not tags:
        return None
    body = "\n".join(tags)
    return (
        '<?xpacket begin="\ufeff" id="W5M0MpCehiHzreSzNTczkc9d"?>'
        '<x:xmpmeta xmlns:x="adobe:ns:meta/">'
        '<rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#">'
        '<rdf:Description rdf:about="" '
        'xmlns:Camera="http://pix4d.com/camera/1.0/">\n'
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
                   software=SOFTWARE_LIVE):
    """Stamp EXIF + Pix4D XMP into ``jpeg_bytes``, returning the new bytes.

    ``pitch_deg`` is the raw towfish *body* pitch; the nadir conversion
    for ``Camera:Pitch`` happens here so every caller gets the same
    convention. Failure is non-fatal -- the caller gets back whatever
    was successfully stamped (possibly the untouched input), because a
    frame with no metadata still beats losing the frame.
    """
    try:
        exif_bytes = build_gps_exif_bytes(
            lat, lon, alt_m, heading_deg, ts_local, ts_utc,
            tilt_deg=tilt_deg, depth_m=depth_m, temp_c=temp_c,
            roll_deg=roll_deg, pitch_deg=pitch_deg, software=software,
        )
        if exif_bytes is not None:
            # piexif.insert with raw bytes requires either a path or a
            # BytesIO output buffer; BytesIO keeps the rewrite in memory
            # so the caller still writes the file exactly once.
            out_buf = io.BytesIO()
            piexif.insert(exif_bytes, jpeg_bytes, out_buf)
            jpeg_bytes = out_buf.getvalue()

        # Only claim an orientation when something was actually measured.
        # camera_pitch_from_body() assumes nadir for a missing body pitch,
        # which is a fine refinement alongside a real heading but must not
        # become an XMP packet of its own during a telemetry dropout.
        if heading_deg is not None or roll_deg is not None or pitch_deg is not None:
            xmp = build_camera_xmp(yaw_deg=heading_deg,
                                   pitch_deg=camera_pitch_from_body(pitch_deg),
                                   roll_deg=roll_deg)
            if xmp is not None:
                jpeg_bytes = insert_xmp_app1(jpeg_bytes, xmp)
    except Exception:
        logger.exception("Photogrammetry metadata embed failed")
    return jpeg_bytes
