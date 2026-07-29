from __future__ import annotations

from flask import Flask, jsonify, request, send_file
import asyncio
import errno
import json
import os
import glob
import subprocess
from datetime import datetime
import logging
import time
import requests
import threading
import math
import websockets
import csv
import re
from datetime import timezone
from websockets.exceptions import ConnectionClosed

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402  (after require_version)

import usb_storage
import photogrammetry_meta
from mavlink_params import (
    LOCAL_MAVLINK2REST_URL,
    ParamClient,
    ParamReadError,
)
from mavlink_writer import (
    MavlinkWriter,
    get_default_writer,
    MODE_STABILIZE,
    MODE_ALT_HOLD,
    MODE_MANUAL,
    Z_CHANNEL,
    Z_PWM_ASCEND,
    Z_PWM_DESCEND,
    Z_PWM_NEUTRAL,
    Z_PWM_MIN,
    Z_PWM_MAX,
    focus_pwm_to_pct,
    focus_pct_to_pwm,
    FOCUS_PWM_MIN as MAV_FOCUS_PWM_MIN,
    FOCUS_PWM_MAX as MAV_FOCUS_PWM_MAX,
)

app = Flask(__name__)
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Mode state -- the recorder runs in exactly one mode at a time:
#   "idle"       : nothing happening
#   "video"      : RTSP H.264 RecordingSession is active
#   "timelapse"  : 2 Hz HTTP-snap TimelapseSession is active
# Mode transitions go through _mode_lock so the Flask routes can never
# leave the recorder in a half-started state across both modes.
# ---------------------------------------------------------------------------
MODE_IDLE = "idle"
MODE_VIDEO = "video"
MODE_TIMELAPSE = "timelapse"
# Automatic recording mode: the recorder watches the tow vehicle's
# mission state via mavlink2rest and drives a RecordingSession on its
# own (one .ts file per waypoint leg, rolled at each MISSION_CURRENT.seq
# change). Mutually exclusive with both manual MODE_VIDEO and
# MODE_TIMELAPSE.
MODE_TRANSECT = "transect"

_mode_lock = threading.Lock()
mode = MODE_IDLE

def _set_mode(new_mode):
    """Module-level setter so the recording sessions can flip global ``mode``.

    Always called while holding ``_mode_lock`` so the Flask routes never
    observe a transient half-state during start/stop transitions.
    """
    global mode
    mode = new_mode

# Active RTSP session (RecordingSession instance) -- non-None iff mode == "video".
_session = None
# Active timelapse session (TimelapseSession instance) -- non-None iff mode == "timelapse".
_timelapse = None

# ``recording`` and ``start_time`` are kept as module-level mirrors of
# the active session's state so the data-lake WebSocket and existing
# sidecar threads (SRT/ASS/ISP) keep working unchanged.
recording = False
start_time = None

# Sidecar lifecycle (one set per RecordingSession; spans every part the
# watchdog produces). All threads stop when stop_*_thread is set.
srt_thread = None
stop_srt_thread = False
current_srt_file_rtsp = None
current_video_file_rtsp = None  # path of the FIRST part; used for filesize/list

# Per-video telemetry CSV, written by the same thread (and from the same
# telemetry sample) as the SRT. ``current_video_csv_wp`` is the leg's
# waypoint label, parsed once from the .ts filename at creation.
current_video_csv_file = None
current_video_csv_wp = None

isp_log_thread = None
stop_isp_log_thread = False
current_isp_log_file = None

current_events_file = None

ass_thread = None
stop_ass_thread = False
current_ass_file = None
ass_subtitle_counter = 0

# Sidecar timing reference. SRT/ASS/CSV entries are timestamped relative
# to this epoch and (re)scaled to the encoded video duration when a file
# is finalised. For manual recording it equals ``start_time`` for the
# whole session; in transect mode it is reset at every leg rollover so
# each per-leg sidecar starts at 00:00.
sidecar_epoch = None
# Guards swaps of the current_srt_file_rtsp / current_ass_file / counters
# / sidecar_epoch globals against the SRT/ASS writer threads while a leg
# rotates its sidecars. Re-entrant so the writer can hold it across its
# cheap bookkeeping section.
_sidecar_lock = threading.RLock()

# RTSP endpoint for the RadCam video stream. The camera was reconfigured
# so ``stream_0`` now serves H.264 (was H.265) -- the H.264 path is the
# only one ``_build_pipeline_description`` knows about. Variable name kept
# generic (``RTSP_ENDPOINT``) so swapping the camera/codec in the future
# only needs the parser/depayloader updated, not call sites.
RTSP_ENDPOINT = "rtsp://admin:blue@192.168.2.10:554/stream_0"

# WebSocket server for Cockpit data lake variables
DATA_LAKE_WS_HOST = "0.0.0.0"
DATA_LAKE_WS_PORT = 8765
DATA_LAKE_RECORDING_VAR = "video-recorder-recording"

# Mavlink URLs (local vehicle)
ahrs2_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/AHRS2'
vfr_hud_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/VFR_HUD'
baro_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/SCALED_PRESSURE2'
rc_channels_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/RC_CHANNELS'

# Persisted configuration — survives restarts
CONFIG_FILE = "/app/videorecordings/subreels_towfish_config.json"
DEFAULT_TOW_VEHICLE_IP = "192.168.2.12"
DEFAULT_CONTAINER_FORMAT = "mp4"
VALID_CONTAINER_FORMATS = ("mp4", "mpegts")
DEFAULT_STREAM_PROTOCOL = "udp"
VALID_STREAM_PROTOCOLS = ("udp", "tcp")
DEFAULT_SNAPSHOT_URL = "http://192.168.2.10/cgi-bin/onesnap.cgi"
# What the automatic transect monitor captures per leg:
#   "video"     -> one RTSP .ts file per waypoint leg (+ per-leg SRT/ASS)
#   "timelapse" -> 2 Hz geotagged JPEGs into one subfolder per leg
# Default is 2 Hz still images -- the towfish survey workflow this
# extension exists for produces stills, not video, and defaulting to
# timelapse means a fresh install is immediately usable without the
# operator having to open the config drawer to flip the capture type.
DEFAULT_TRANSECT_CAPTURE_TYPE = "timelapse"
VALID_TRANSECT_CAPTURE_TYPES = ("video", "timelapse")
# Where to put new recordings:
#   "usb"   -> attached USB drive (mounted at /mnt/usb) when usable, else
#              fall back to the local extension volume.
#   "local" -> always use the local extension volume (/app/videorecordings).
# "Usable" requires the drive to be mounted *and* have at least
# usb_storage.USB_MIN_FREE_GB free; below that we fall back automatically
# to local storage so a near-full stick can't wedge the recorder.
DEFAULT_STORAGE_PREFERENCE = "usb"
VALID_STORAGE_PREFERENCES = ("usb", "local")

# ── Towed-body layback ───────────────────────────────────────────────────
# The towfish has no GPS of its own, so every geotag is the tow point's
# fix pushed backwards along a heading. Both the distance and which
# heading to use are operator-configurable, because real layback depends
# on tether scope, tow speed and depth.
#
# The heading source matters when the fish and the boat are not aligned
# (turns, crosswind, cross-current):
#   "towfish" -> the fish's own yaw. Best when the fish tracks straight
#                behind and the boat is being pushed off its course.
#   "boat"    -> the tow vehicle's yaw. Best when the fish is yawing on
#                the tether but the tow direction is steady.
#   "average" -> circular mean of the two, as a compromise.
DEFAULT_TOW_OFFSET_M = 7.0
DEFAULT_TOW_HEADING_SOURCE = "towfish"
VALID_TOW_HEADING_SOURCES = ("towfish", "boat", "average")
# 0 disables the offset (geotag straight from the tow point). The upper
# bound is a sanity rail, not a physical limit -- it only exists to stop
# a fat-fingered entry from throwing fixes a kilometre off.
TOW_OFFSET_MIN_M = 0.0
TOW_OFFSET_MAX_M = 300.0

# One-push white-balance loop that runs while the transect monitor is
# enabled. The operator can flip this off from the widget for scenes
# where re-triggering AWB every 2 minutes would produce a visible
# colour jump (e.g. crossing shadow boundaries). Default on.
DEFAULT_AWB_LOOP_ENABLED = True
AWB_LOOP_INTERVAL_S = 120.0
# Direct-camera one-push AWB endpoint. This is the same POST that
# radcam-manager proxies internally (setImageAdjustmentEx with
# ``onceAWB=1``). The older HAUV.lua ``cgi_action`` GET path now returns
# "error user/pwd" on current RadCam firmware -- verified against the
# camera on 2026-07-11 -- so we use the POST path instead.
RADCAM_AWB_URL = 'http://192.168.2.10/action/setImageAdjustmentEx'
RADCAM_AWB_BODY = {"onceAWB": 1}

# ── Survey parameter checker ─────────────────────────────────────────────
# Autopilot parameters that have to be right before a tow survey, split
# across the two vehicles involved:
#
#   "boat"    -> the ArduRover tow boat, reached at ``tow_vehicle_ip``
#   "towfish" -> the local ArduSub towfish (host.docker.internal)
#
# ``default`` is our starting recommendation, not a hard requirement:
# every target is operator-editable and persisted, because the right
# values shift with hull, tow point and sea state. The checker compares
# what the vehicle reports against the *saved target*, never against the
# constant below.
PARAM_VEHICLES = ("boat", "towfish")

PARAM_SPECS = [
    {
        "name": "TURN_RADIUS",
        "vehicle": "boat",
        "default": 2.50,
        "unit": "m",
        "decimals": 2,
        "min": 0.1,
        "max": 100.0,
        "desc": "Radius the boat uses to round a waypoint. Tight enough "
                "that the towfish is not dragged across its own track.",
    },
    {
        "name": "WP_PIVOT_ANGLE",
        "vehicle": "boat",
        "default": 0.0,
        "unit": "deg",
        "decimals": 0,
        "min": 0.0,
        "max": 180.0,
        "desc": "0 disables pivot turns, so the boat keeps way on through "
                "every corner instead of stopping and spinning.",
    },
    {
        "name": "WP_SPEED",
        "vehicle": "boat",
        "default": 1.0,
        "unit": "m/s",
        "decimals": 2,
        "min": 0.8,
        "max": 1.1,
        "presets": [0.8, 0.9, 1.0, 1.1],
        "desc": "Target speed while running an AUTO mission leg.",
    },
    {
        "name": "CRUISE_SPEED",
        "vehicle": "boat",
        "default": 1.0,
        "unit": "m/s",
        "decimals": 2,
        "min": 0.8,
        "max": 1.1,
        "presets": [0.8, 0.9, 1.0, 1.1],
        "desc": "Speed the throttle controller trims around. Keep it equal "
                "to WP_SPEED so the boat does not fight its own mission.",
    },
    {
        "name": "ATC_ANG_RLL_P",
        "vehicle": "towfish",
        "default": 0.00,
        "unit": "",
        "decimals": 3,
        "min": 0.0,
        "max": 12.0,
        "desc": "Roll angle P gain. Zero leaves roll passive so the tow "
                "cable, not the autopilot, sets the fish attitude.",
    },
    {
        "name": "ATC_RAT_RLL_D",
        "vehicle": "towfish",
        "default": 0.0072,
        "unit": "",
        "decimals": 4,
        "min": 0.0,
        "max": 0.5,
        "desc": "Roll rate D gain -- damps the roll oscillation the tow "
                "cable induces.",
    },
    {
        "name": "ATC_RAT_RLL_FLTE",
        "vehicle": "towfish",
        "default": 3.0,
        "unit": "Hz",
        "decimals": 2,
        "min": 0.0,
        "max": 100.0,
        "desc": "Roll rate error filter cutoff.",
    },
    {
        "name": "ATC_RAT_RLL_FLTD",
        "vehicle": "towfish",
        "default": 4.0,
        "unit": "Hz",
        "decimals": 2,
        "min": 0.0,
        "max": 100.0,
        "desc": "Roll rate derivative filter cutoff.",
    },
]

PARAM_SPECS_BY_NAME = {spec["name"]: spec for spec in PARAM_SPECS}

DEFAULT_PARAM_TARGETS = {spec["name"]: float(spec["default"])
                         for spec in PARAM_SPECS}


def _sanitize_param_targets(raw):
    """Coerce a saved/posted target map into ``{name: float}``.

    Unknown names are dropped and out-of-range values are clamped to the
    spec envelope, so a hand-edited config file or a stale browser tab
    can never push a nonsense value at an autopilot.
    """
    targets = dict(DEFAULT_PARAM_TARGETS)
    if not isinstance(raw, dict):
        return targets
    for name, value in raw.items():
        spec = PARAM_SPECS_BY_NAME.get(name)
        if spec is None:
            continue
        try:
            num = float(value)
        except (TypeError, ValueError):
            continue
        if num != num or num in (float("inf"), float("-inf")):
            continue
        targets[name] = max(float(spec["min"]), min(float(spec["max"]), num))
    return targets


def _sanitize_tow_offset_m(raw, fallback=DEFAULT_TOW_OFFSET_M):
    """Coerce a saved/posted layback distance to a clamped float."""
    try:
        num = float(raw)
    except (TypeError, ValueError):
        return fallback
    if num != num or num in (float("inf"), float("-inf")):
        return fallback
    return max(TOW_OFFSET_MIN_M, min(TOW_OFFSET_MAX_M, num))


def _sanitize_tow_heading_source(raw, fallback=DEFAULT_TOW_HEADING_SOURCE):
    """Coerce a saved/posted heading-source name to a known mode."""
    if not isinstance(raw, str):
        return fallback
    value = raw.strip().lower()
    return value if value in VALID_TOW_HEADING_SOURCES else fallback


def load_config():
    """Load persisted configuration from disk, returning defaults on failure."""
    defaults = {
        "tow_vehicle_ip": DEFAULT_TOW_VEHICLE_IP,
        "container_format": DEFAULT_CONTAINER_FORMAT,
        "stream_protocol": DEFAULT_STREAM_PROTOCOL,
        "snapshot_url": DEFAULT_SNAPSHOT_URL,
        "transect_capture_type": DEFAULT_TRANSECT_CAPTURE_TYPE,
        "storage_preference": DEFAULT_STORAGE_PREFERENCE,
        "awb_loop_enabled": DEFAULT_AWB_LOOP_ENABLED,
        "param_targets": dict(DEFAULT_PARAM_TARGETS),
        "tow_offset_m": DEFAULT_TOW_OFFSET_M,
        "tow_heading_source": DEFAULT_TOW_HEADING_SOURCE,
    }
    try:
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, 'r') as f:
                saved = json.load(f)
            defaults.update(saved)
    except Exception as e:
        logger.warning(f"Could not load config file: {e}")
    if defaults["container_format"] not in VALID_CONTAINER_FORMATS:
        defaults["container_format"] = DEFAULT_CONTAINER_FORMAT
    if defaults["stream_protocol"] not in VALID_STREAM_PROTOCOLS:
        defaults["stream_protocol"] = DEFAULT_STREAM_PROTOCOL
    if not isinstance(defaults["snapshot_url"], str) or not defaults["snapshot_url"].strip():
        defaults["snapshot_url"] = DEFAULT_SNAPSHOT_URL
    if defaults["transect_capture_type"] not in VALID_TRANSECT_CAPTURE_TYPES:
        defaults["transect_capture_type"] = DEFAULT_TRANSECT_CAPTURE_TYPE
    if defaults["storage_preference"] not in VALID_STORAGE_PREFERENCES:
        defaults["storage_preference"] = DEFAULT_STORAGE_PREFERENCE
    defaults["awb_loop_enabled"] = bool(defaults.get("awb_loop_enabled",
                                                     DEFAULT_AWB_LOOP_ENABLED))
    defaults["param_targets"] = _sanitize_param_targets(
        defaults.get("param_targets"))
    defaults["tow_offset_m"] = _sanitize_tow_offset_m(
        defaults.get("tow_offset_m"))
    defaults["tow_heading_source"] = _sanitize_tow_heading_source(
        defaults.get("tow_heading_source"))
    return defaults

def save_config(cfg):
    """Persist configuration dict to disk."""
    try:
        os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
        with open(CONFIG_FILE, 'w') as f:
            json.dump(cfg, f, indent=2)
    except Exception as e:
        logger.error(f"Could not save config file: {e}")

def _blueboat_gps_url():
    return f'http://{tow_vehicle_ip}/mavlink2rest/mavlink/vehicles/1/components/1/messages/GLOBAL_POSITION_INT'

def _blueboat_attitude_url():
    return f'http://{tow_vehicle_ip}/mavlink2rest/mavlink/vehicles/1/components/1/messages/ATTITUDE'

def _blueboat_vfr_hud_url():
    return f'http://{tow_vehicle_ip}/mavlink2rest/mavlink/vehicles/1/components/1/messages/VFR_HUD'

def _tow_mavlink_url(msg_name):
    """Compose a mavlink2rest URL on the tow vehicle for one message name.

    Used by ``get_mission_state()`` to read mission/navigation messages
    from the ArduRover-based tow vehicle (HEARTBEAT, MISSION_CURRENT,
    NAV_CONTROLLER_OUTPUT, ...). Same host as the BlueBoat GPS / VFR_HUD
    helpers above.
    """
    return f'http://{tow_vehicle_ip}/mavlink2rest/mavlink/vehicles/1/components/1/messages/{msg_name}'

# ── ArduRover constants ──────────────────────────────────────────────────
# Custom mode numbers for ArduRover (Plane / Sub / Copter use different
# tables). AUTO is the only mode we care about for transect triggering.
# See https://ardupilot.org/rover/docs/parameters.html#mode-rc-or-channel
ROVER_MODE_AUTO = 10
# base_mode field on HEARTBEAT: bit 0x80 == MAV_MODE_FLAG_SAFETY_ARMED
MAV_MODE_FLAG_SAFETY_ARMED = 0x80

# How long ``get_mission_state()`` waits for one mavlink2rest GET. Has to
# stay short: the TransectMonitor calls this at ~3 Hz, and the tow vehicle
# can briefly be slow when MAVLink is congested mid-mission.
_MISSION_HTTP_TIMEOUT_S = 0.6

def _mav_get_message(url):
    """Single mavlink2rest GET, returning the ``message`` dict or None.

    ``NAV_CONTROLLER_OUTPUT`` returns an empty body (zero bytes) when the
    autopilot is idle, which json.loads chokes on; we treat any unparseable
    or non-200 response as "not currently available" rather than an error.
    """
    try:
        resp = requests.get(url, timeout=_MISSION_HTTP_TIMEOUT_S)
        if resp.status_code != 200 or not resp.content:
            return None
        try:
            data = resp.json()
        except ValueError:
            return None
        msg = data.get('message')
        if not isinstance(msg, dict):
            return None
        # Stash the freshness timestamp on the dict so callers (the
        # STATUSTEXT dedup, in particular) can tell whether the same
        # payload was reposted vs a genuinely new sample.
        msg['__last_update'] = (data.get('status', {})
                                 .get('time', {})
                                 .get('last_update'))
        return msg
    except Exception as e:
        logger.debug("mavlink2rest GET %s failed: %s", url, e)
        return None

def get_ping_sonar_snapshot():
    """Read the BlueBoat's Ping sonar (published as DISTANCE_SENSOR) from
    the tow vehicle's mavlink2rest.

    Returns metres of water below the surface craft plus the sensor's
    range envelope and confidence, or ``None`` when the boat is
    unreachable / not publishing a distance sensor. Because the tow
    vehicle's mavlink2rest runs *on the boat*, an offline boat makes the
    GET time out and yields None (no risk of a stale cached reading).

    ``current_distance``/min/max are centimetres per the MAVLink spec.
    Ping1D reports downward (orientation PITCH_270), so this is the depth
    of water under the boat -- independent of the towfish's own baro depth.
    """
    msg = _mav_get_message(_tow_mavlink_url('DISTANCE_SENSOR'))
    if not msg:
        return None
    cur = msg.get('current_distance')
    if not isinstance(cur, (int, float)):
        return None

    def _cm_to_m(v):
        return round(v / 100.0, 2) if isinstance(v, (int, float)) else None

    mn = msg.get('min_distance')
    mx = msg.get('max_distance')
    # Ping1D reports current_distance well beyond max_distance when it has
    # no bottom lock (e.g. in air, or water deeper than its range). Treat a
    # reading outside [min, max] as "no lock" so the widget shows -- rather
    # than a bogus range like 90 m.
    in_range = (isinstance(mn, (int, float)) and isinstance(mx, (int, float))
                and mn <= cur <= mx)
    quality = msg.get('signal_quality')
    orientation = msg.get('orientation')
    if isinstance(orientation, dict):
        orientation = orientation.get('type')
    return {
        "distance_m": _cm_to_m(cur),
        "min_m": _cm_to_m(mn),
        "max_m": _cm_to_m(mx),
        "in_range": in_range,
        # DISTANCE_SENSOR.signal_quality: 0..100, 0 == "unknown". Absent on
        # older Ping firmware, so pass through only when it's a real number.
        "signal_quality": (quality if isinstance(quality, (int, float)) and quality > 0
                           else None),
        "orientation": orientation,
    }


def _decode_statustext(msg):
    """Assemble the array-of-chars STATUSTEXT payload into a Python string.

    mavlink2rest serialises STATUSTEXT.text as a 50-element char array
    (one JSON string per byte) and pads with NULs. Returns the trimmed
    string or '' if the message is missing/empty.
    """
    if not msg:
        return ''
    chars = msg.get('text') or []
    if not isinstance(chars, list):
        return ''
    return ''.join(c for c in chars if isinstance(c, str)).rstrip('\x00').rstrip()

def get_mission_state():
    """Snapshot the tow vehicle's mission/navigation state.

    Returns a dict with every key always present (Nones where unavailable)
    so the TransectMonitor state machine can read it without defensive
    .get() chains. Designed to be safe to call ~3 Hz from a background
    thread; each underlying GET is bounded by ``_MISSION_HTTP_TIMEOUT_S``
    and a failure on any one message degrades that field to None rather
    than raising.

    NAV_CONTROLLER_OUTPUT is only published while the vehicle is actively
    navigating (AUTO/GUIDED with a target), so ``wp_dist``/``target_bearing``
    will be None during idle/HOLD even when the rest of the snapshot is
    populated -- this is normal and the monitor's "navigating" predicate
    uses ``wp_dist`` *or* ``groundspeed`` to handle both cases.
    """
    hb = _mav_get_message(_tow_mavlink_url('HEARTBEAT'))
    mc = _mav_get_message(_tow_mavlink_url('MISSION_CURRENT'))
    nav = _mav_get_message(_tow_mavlink_url('NAV_CONTROLLER_OUTPUT'))
    vfr = _mav_get_message(_tow_mavlink_url('VFR_HUD'))
    st = _mav_get_message(_tow_mavlink_url('STATUSTEXT'))

    mode_num = hb.get('custom_mode') if hb else None
    base_mode = ((hb or {}).get('base_mode') or {}).get('bits', 0) or 0
    armed = bool(base_mode & MAV_MODE_FLAG_SAFETY_ARMED)

    return {
        'mode_num': mode_num,
        'armed': armed,
        'mission_seq': mc.get('seq') if mc else None,
        'wp_dist': nav.get('wp_dist') if nav else None,
        'target_bearing': nav.get('target_bearing') if nav else None,
        'xtrack_error': nav.get('xtrack_error') if nav else None,
        'groundspeed': vfr.get('groundspeed') if vfr else None,
        'heading': vfr.get('heading') if vfr else None,
        'statustext': _decode_statustext(st),
        # Used by the monitor to dedup STATUSTEXT (mavlink2rest holds
        # only the *latest* string, so the same "Reached waypoint #N"
        # payload can be served for many seconds; the timestamp lets us
        # tell whether it's actually new).
        'statustext_time': (st or {}).get('__last_update'),
        'statustext_severity': (((st or {}).get('severity') or {})
                                 .get('type')),
    }

_cfg = load_config()
tow_vehicle_ip = _cfg["tow_vehicle_ip"]
container_format = _cfg["container_format"]
stream_protocol = _cfg["stream_protocol"]
snapshot_url = _cfg["snapshot_url"]
transect_capture_type = _cfg["transect_capture_type"]
storage_preference = _cfg["storage_preference"]
awb_loop_enabled = _cfg["awb_loop_enabled"]
param_targets = _cfg["param_targets"]
tow_offset_m = _cfg["tow_offset_m"]
tow_heading_source = _cfg["tow_heading_source"]

def _persist_config():
    """Snapshot the currently-live config globals to disk."""
    save_config({
        "tow_vehicle_ip": tow_vehicle_ip,
        "container_format": container_format,
        "stream_protocol": stream_protocol,
        "snapshot_url": snapshot_url,
        "transect_capture_type": transect_capture_type,
        "storage_preference": storage_preference,
        "awb_loop_enabled": awb_loop_enabled,
        "param_targets": param_targets,
        "tow_offset_m": tow_offset_m,
        "tow_heading_source": tow_heading_source,
    })

# Recording-storage state
#
# ``usb_recording`` is True when the *currently active* video or
# timelapse session is writing to the USB mount. A background watcher
# triggers a one-shot failover to local storage if the drive either
# disappears or fills up mid-recording. ``usb_failover_count`` is reset
# only at startup;
# it surfaces in /status so the UI can warn the operator that a
# session was forced onto the SD card.
usb_recording = False
usb_failover_count = 0
#: Why the last failover fired -- "lost" (drive gone) or "full" (out of
#: space). Distinct causes with the same symptom, and the operator's fix
#: differs, so /status reports which one it was.
usb_failover_reason = None

# Local fallback root for recordings that don't (or no longer) live
# on USB. Using a named constant lets every capture path call the
# same resolver instead of hardcoding "/app/videorecordings".
LOCAL_RECORDING_DIR = "/app/videorecordings"

#: Free space on the local fallback below which we warn loudly. Failing
#: over onto an SD card that is itself nearly full only buys a few minutes,
#: and that is worth saying out loud rather than discovering in the logs.
_LOCAL_LOW_FREE_MB = 2048

# Towfish (ArduSub) telemetry is read through the local BlueOS host the
# extension runs on (host.docker.internal), same as depth/altitude/temp,
# rather than a hardcoded 192.168.2.2.
towfish_attitude_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/ATTITUDE'
servo_output_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/SERVO_OUTPUT_RAW'

# Camera tilt is driven by SERVO16 (SERVO16_FUNCTION = 7, mount pitch)
# with the mount running earth-frame stabilised (MNT1_TYPE = 1). We read
# the servo PWM from SERVO_OUTPUT_RAW and map it to the *body-frame* mount
# angle (camera pitch relative to the towfish), then add the vehicle pitch
# to publish the *world-relative* (earth-frame) angle the mount is holding.
#
# Calibration is the vehicle's own SERVO16/MNT1 configuration, confirmed
# against the towfish 00000061 dataflash log (angle = -0.10723*pwm +
# 162.575, 0.45 deg rms over 28,640 samples). Because SERVO16_REVERSED = 1
# the travel is inverted: SERVO16_MIN maps to MNT1_PITCH_MAX and
# SERVO16_MAX maps to MNT1_PITCH_MIN. The old hardcoded 1100/1900 -> +/-90
# mapping was wrong for this frame (recorded +144 deg where the mount was
# actually at -66.9 deg). Adjust these if the vehicle params change.
TILT_SERVO_CHANNEL = 16
TILT_PWM_MIN = 865            # SERVO16_MIN
TILT_PWM_MAX = 2170           # SERVO16_MAX
TILT_SERVO_REVERSED = True    # SERVO16_REVERSED
TILT_MOUNT_PITCH_MIN_DEG = -70.0  # MNT1_PITCH_MIN (camera fully down)
TILT_MOUNT_PITCH_MAX_DEG = 70.0   # MNT1_PITCH_MAX (camera fully up)

# Camera ISP info endpoint
camera_isp_url = 'http://192.168.2.10/action/getISPInfo'

def get_depth_data():
    """Get depth data from AHRS2 message (altitude is negative underwater). Returns 0.0 on failure."""
    try:
        response = requests.get(ahrs2_url, timeout=1)
        if response.status_code == 200:
            # In ArduSub, altitude is negative for depth underwater
            altitude = response.json()['message'].get('altitude', 0.0)
            # Convert altitude to depth (positive value for underwater)
            depth = -altitude if altitude < 0 else 0.0
            return depth
    except Exception as e:
        logger.debug(f"Error fetching depth data: {str(e)}")
    return 0.0

def get_vfr_hud_data():
    """Get climb rate from VFR_HUD message. Returns 0.0 on failure."""
    try:
        response = requests.get(vfr_hud_url, timeout=1)
        if response.status_code == 200:
            climb = response.json()['message'].get('climb', 0.0)
            return climb
    except Exception as e:
        logger.debug(f"Error fetching VFR_HUD data: {str(e)}")
    return 0.0

def get_baro_data():
    """Get temperature from SCALED_PRESSURE2 message. Returns 0.0 on failure."""
    try:
        response = requests.get(baro_url, timeout=1)
        if response.status_code == 200:
            temperature = response.json()['message'].get('temperature', 0.0) / 100.0  # Convert to degrees C
            return temperature
    except Exception as e:
        logger.debug(f"Error fetching baro data: {str(e)}")
    return 0.0

def get_light_output():
    """Get light output percentage from RC channels. Returns 0 on failure."""
    try:
        response = requests.get(rc_channels_url, timeout=1)
        data = response.json()
        
        if 'message' in data and 'chan9_raw' in data['message']:
            raw_value = data['message']['chan9_raw']
            
            # Convert from 1100-1900 range to 0-100%
            if raw_value <= 1100:
                percentage = 0
            elif raw_value >= 1900:
                percentage = 100
            else:
                percentage = round((raw_value - 1100) / 8.0)  # 800 range / 8 = percentage
                
            return percentage
        return 0  # Default to 0% if not available
    except Exception as e:
        logger.debug(f"Error getting light output: {str(e)}")
        return 0

def get_blueboat_gps_position():
    """Get GPS position from tow vehicle. Returns (lat, lon, alt) or (None, None, None) on failure."""
    try:
        response = requests.get(_blueboat_gps_url(), timeout=1)
        if response.status_code == 200:
            message = response.json().get('message', {})
            lat = message.get('lat', None)
            lon = message.get('lon', None)
            alt = message.get('alt', None)  # Altitude in mm
            
            # GLOBAL_POSITION_INT uses degrees * 1e7, convert to decimal degrees
            # Altitude is in mm, convert to meters
            if lat is not None and lon is not None:
                lat_decimal = lat / 1e7
                lon_decimal = lon / 1e7
                alt_meters = alt / 1000.0 if alt is not None else 0.0
                return (lat_decimal, lon_decimal, alt_meters)
        return (None, None, None)
    except Exception as e:
        logger.debug(f"Error fetching BlueBoat GPS position: {str(e)}")
        return (None, None, None)

def get_towfish_heading():
    """Get heading from towfish ATTITUDE message. Returns heading in degrees or None on failure."""
    try:
        response = requests.get(towfish_attitude_url, timeout=1)
        if response.status_code == 200:
            message = response.json().get('message', {})
            yaw = message.get('yaw', None)  # Yaw in radians
            
            if yaw is not None:
                # Convert radians to degrees
                heading_deg = math.degrees(yaw)
                # Normalize to 0-360
                heading_deg = heading_deg % 360
                if heading_deg < 0:
                    heading_deg += 360
                return heading_deg
        return None
    except Exception as e:
        logger.debug(f"Error fetching towfish heading: {str(e)}")
        return None

def get_towfish_attitude():
    """Get yaw, roll and pitch from towfish ATTITUDE message.

    Returns a dict with degrees. ``yaw`` is normalised to a 0..360 true
    heading; ``roll``/``pitch`` are signed (+roll = right-down, +pitch =
    nose-up per the ArduPilot/MAVLink body frame).
    """
    try:
        response = requests.get(towfish_attitude_url, timeout=1)
        if response.status_code == 200:
            message = response.json().get('message', {})
            result = {}
            yaw = message.get('yaw')
            if yaw is not None:
                yaw_deg = math.degrees(yaw) % 360
                if yaw_deg < 0:
                    yaw_deg += 360
                result['yaw'] = yaw_deg
            roll = message.get('roll')
            if roll is not None:
                result['roll'] = math.degrees(roll)
            pitch = message.get('pitch')
            if pitch is not None:
                result['pitch'] = math.degrees(pitch)
            return result
    except Exception as e:
        logger.debug(f"Error fetching towfish attitude: {str(e)}")
    return {}

def get_blueboat_attitude():
    """Get yaw and pitch from tow vehicle ATTITUDE message. Returns dict with degrees."""
    try:
        response = requests.get(_blueboat_attitude_url(), timeout=1)
        if response.status_code == 200:
            message = response.json().get('message', {})
            result = {}
            yaw = message.get('yaw')
            if yaw is not None:
                yaw_deg = math.degrees(yaw) % 360
                if yaw_deg < 0:
                    yaw_deg += 360
                result['yaw'] = yaw_deg
            pitch = message.get('pitch')
            if pitch is not None:
                result['pitch'] = math.degrees(pitch)
            return result
    except Exception as e:
        logger.debug(f"Error fetching BlueBoat attitude: {str(e)}")
    return {}

def get_blueboat_speed():
    """Get groundspeed from tow vehicle VFR_HUD message. Returns speed in m/s or None."""
    try:
        response = requests.get(_blueboat_vfr_hud_url(), timeout=1)
        if response.status_code == 200:
            return response.json()['message'].get('groundspeed')
    except Exception as e:
        logger.debug(f"Error fetching BlueBoat speed: {str(e)}")
    return None

def calculate_offset_position(lat, lon, heading_deg, offset_meters):
    """
    Calculate a new position offset behind the given heading.
    
    Args:
        lat: Latitude in decimal degrees
        lon: Longitude in decimal degrees
        heading_deg: Heading in degrees (0=North, 90=East)
        offset_meters: Distance to offset (positive = behind)
    
    Returns:
        (new_lat, new_lon) tuple
    """
    # Earth's radius in meters
    R = 6378137.0
    
    # Calculate the bearing opposite to heading (180 degrees behind)
    opposite_bearing_rad = math.radians((heading_deg + 180) % 360)
    
    # Convert lat/lon to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    
    # Angular distance
    d = offset_meters / R
    
    # Calculate new position
    new_lat_rad = math.asin(
        math.sin(lat_rad) * math.cos(d) +
        math.cos(lat_rad) * math.sin(d) * math.cos(opposite_bearing_rad)
    )
    
    new_lon_rad = lon_rad + math.atan2(
        math.sin(opposite_bearing_rad) * math.sin(d) * math.cos(lat_rad),
        math.cos(d) - math.sin(lat_rad) * math.sin(new_lat_rad)
    )
    
    # Convert back to degrees
    new_lat = math.degrees(new_lat_rad)
    new_lon = math.degrees(new_lon_rad)
    
    return (new_lat, new_lon)

def circular_mean_deg(a_deg, b_deg):
    """Mean of two compass headings, taken the short way around.

    Plain arithmetic averaging breaks across north: (350 + 10) / 2 gives
    180, pointing the layback exactly backwards. Averaging the unit
    vectors instead gives 0.
    """
    a_rad = math.radians(a_deg)
    b_rad = math.radians(b_deg)
    mean = math.atan2(math.sin(a_rad) + math.sin(b_rad),
                      math.cos(a_rad) + math.cos(b_rad))
    return math.degrees(mean) % 360


def resolve_tow_heading(towfish_heading=None):
    """Heading to lay the towfish back along, per ``tow_heading_source``.

    ``towfish_heading`` lets a caller that already read ATTITUDE this
    cycle pass it in rather than paying for a second mavlink2rest
    round-trip. Whichever source is configured, an unavailable reading
    falls back to the other vehicle before giving up, so a dropout on
    one link degrades the estimate instead of dropping the offset.
    """
    def fish():
        return (towfish_heading if towfish_heading is not None
                else get_towfish_heading())

    def boat():
        return get_blueboat_attitude().get('yaw')

    if tow_heading_source == "average":
        f, b = fish(), boat()
        if f is not None and b is not None:
            return circular_mean_deg(f, b)
        return f if f is not None else b

    if tow_heading_source == "boat":
        b = boat()
        return b if b is not None else fish()

    f = fish()
    return f if f is not None else boat()


def offset_towed_position(lat, lon, towfish_heading=None):
    """Push a tow-point fix back to where the towfish is estimated to be.

    Returns the input position unchanged when the offset is disabled or
    no heading is available from either vehicle.
    """
    if lat is None or lon is None:
        return (None, None)
    if tow_offset_m <= 0:
        return (lat, lon)
    heading = resolve_tow_heading(towfish_heading)
    if heading is None:
        return (lat, lon)
    return calculate_offset_position(lat, lon, heading, tow_offset_m)


def get_towing_gps_position():
    """Estimated towfish position: the tow vehicle's fix, laid back.

    Returns (lat, lon) or (None, None) if the tow vehicle has no fix.
    """
    lat, lon, _alt = get_blueboat_gps_position()
    return offset_towed_position(lat, lon)

def get_isp_info():
    """Get camera ISP info from the camera endpoint.
    Returns dict with ISO, AGain, DGain, ISPDGain, ExpTime, Exposure, device_mac or None on failure."""
    try:
        response = requests.get(camera_isp_url, timeout=2)
        if response.status_code == 200:
            data = response.json()
            device_mac = data.get('device_mac', '')
            isp_info_str = data.get('isp_info', '')
            
            # Parse the isp_info string: "ISP Info: ISO:100 AGain:1024 DGain:1024 ISPDGain:1028 ExpTime:2711 Exposure:2721 HistError:0\t"
            result = {
                'device_mac': device_mac,
                'ISO': None,
                'AGain': None,
                'DGain': None,
                'ISPDGain': None,
                'ExpTime': None,
                'Exposure': None
            }
            
            # Extract values using regex
            patterns = {
                'ISO': r'ISO:(\d+)',
                'AGain': r'AGain:(\d+)',
                'DGain': r'DGain:(\d+)',
                'ISPDGain': r'ISPDGain:(\d+)',
                'ExpTime': r'ExpTime:(\d+)',
                'Exposure': r'Exposure:(\d+)'
            }
            
            for key, pattern in patterns.items():
                match = re.search(pattern, isp_info_str)
                if match:
                    result[key] = int(match.group(1))
            
            return result
    except Exception as e:
        logger.debug(f"Error fetching ISP info: {str(e)}")
    return None

def create_isp_log_file(video_path):
    """Create a new CSV file for ISP logging with headers."""
    base, _ = os.path.splitext(video_path)
    csv_path = base + '_isp.csv'
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'device_mac', 'ISO', 'AGain', 'DGain', 'ISPDGain', 'ExpTime', 'Exposure'])
    return csv_path

def update_isp_log():
    """Update ISP log file with camera exposure data, once per second."""
    global stop_isp_log_thread, current_isp_log_file
    
    while not stop_isp_log_thread and recording and current_isp_log_file:
        try:
            isp_data = get_isp_info()
            if isp_data:
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # Millisecond precision
                
                with open(current_isp_log_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        timestamp,
                        isp_data['device_mac'],
                        isp_data['ISO'],
                        isp_data['AGain'],
                        isp_data['DGain'],
                        isp_data['ISPDGain'],
                        isp_data['ExpTime'],
                        isp_data['Exposure']
                    ])
            
            time.sleep(1)  # Log once per second
        except Exception as e:
            logger.error(f"Error updating ISP log: {str(e)}")
            time.sleep(1)

def create_srt_file(video_path):
    """Create a new .srt file for WebODM position data"""
    base, _ = os.path.splitext(video_path)
    srt_path = base + '.srt'
    # Create empty file
    with open(srt_path, 'w') as f:
        pass
    return srt_path

# Per-video telemetry sidecar. Carries the same telemetry columns (same
# names, units and formatting) as the timelapse ``telemetry.csv`` so a
# survey's video legs and its still-image legs are post-processed with
# one schema. The image-specific columns (seq/filename/frame/size_bytes/
# snap timing) are replaced by ``video_time_s``, which is the key
# ``extract_geotagged_frames.py`` interpolates on.
#
# ``altitude_m`` is the tow vehicle's GPS altitude above MSL (matching
# the timelapse column of that name); ``towfish_altitude_m`` is the
# towfish AHRS2 altitude, which is the value that lands in EXIF
# ``GPSAltitude``. Both are recorded because they are not the same
# quantity and photogrammetry needs the latter.
_VIDEO_CSV_HEADER = [
    'timestamp', 'video_time_s', 'video_file', 'wp',
    'lat', 'lon', 'altitude_m', 'towfish_altitude_m', 'towfish_heading_deg',
    'towfish_roll_deg', 'towfish_pitch_deg',
    'depth_m', 'temperature_c', 'camera_tilt_deg', 'telem_ms',
]

_WP_LABEL_RE = re.compile(r'_(wp\d+)(?:_|$)')


def create_video_telemetry_csv(video_path):
    """Create the per-video telemetry CSV sidecar and write its header.

    Returns ``(csv_path, wp_label)``; ``wp_label`` is parsed out of the
    leg filename (``..._wp03.ts``) so every row can carry it the way the
    timelapse CSV does, and is ``None`` for manual recordings.
    """
    base, _ = os.path.splitext(video_path)
    csv_path = base + '_telemetry.csv'
    with open(csv_path, 'w', newline='') as f:
        csv.writer(f).writerow(_VIDEO_CSV_HEADER)
    m = _WP_LABEL_RE.search(os.path.basename(base))
    return csv_path, (m.group(1) if m else None)


def adjust_video_csv_timing(csv_path, video_duration):
    """Scale the CSV's ``video_time_s`` column to the encoded duration.

    The rows are written on wall-clock timing, so they need the same
    rescale the SRT and ASS get -- otherwise frame extraction would
    sample telemetry at drifting offsets. Mirrors
    :func:`adjust_srt_timing`, including its 1% no-op threshold.
    """
    try:
        with open(csv_path, 'r', newline='') as f:
            rows = list(csv.reader(f))
        if len(rows) < 2:
            logger.warning("Telemetry CSV has no data rows, nothing to adjust")
            return False

        header, data = rows[0], rows[1:]
        try:
            t_idx = header.index('video_time_s')
        except ValueError:
            logger.warning("Telemetry CSV missing video_time_s column")
            return False

        times = []
        for row in data:
            try:
                times.append(float(row[t_idx]))
            except (ValueError, IndexError):
                times.append(None)
        max_t = max((t for t in times if t is not None), default=0.0)
        if max_t <= 0:
            logger.warning("No valid telemetry CSV timestamps found")
            return False

        scale = video_duration / max_t
        if abs(scale - 1.0) < 0.01:
            logger.info("Telemetry CSV timing already within 1%% of video duration")
            return True

        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            for row, t in zip(data, times):
                if t is not None:
                    row[t_idx] = f"{t * scale:.3f}"
                w.writerow(row)

        logger.info("Telemetry CSV timing adjusted (scaled by %.4f)", scale)
        return True
    except Exception:
        logger.exception("Error adjusting telemetry CSV timing")
        return False

def format_srt_timestamp(seconds):
    """Format seconds into SRT timestamp format (HH:MM:SS,mmm)"""
    hours = int(seconds // 3600)
    minutes = int((seconds % 3600) // 60)
    secs = int(seconds % 60)
    milliseconds = int((seconds - int(seconds)) * 1000)
    return f"{hours:02d}:{minutes:02d}:{secs:02d},{milliseconds:03d}"

# Global counter for SRT subtitle entries
srt_subtitle_counter = 0

#: Set once so a missing ffprobe warns a single time instead of on every part.
_ffprobe_missing_logged = False


def get_video_duration(video_path):
    """Encoded duration of ``video_path`` in seconds via ffprobe, else None.

    Returning None is survivable -- callers leave sidecars on their
    wall-clock timeline instead of rescaling to the encoded duration --
    but it means subtitle/telemetry timing drifts from the video, so a
    missing ffprobe is logged once at WARNING rather than swallowed.
    """
    global _ffprobe_missing_logged
    try:
        cmd = [
            'ffprobe', '-v', 'error',
            '-show_entries', 'format=duration',
            '-of', 'default=noprint_wrappers=1:nokey=1',
            video_path
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        if result.returncode == 0:
            return float(result.stdout.strip())
        logger.error("ffprobe failed on %s (rc=%s): %s",
                     video_path, result.returncode, result.stderr.strip())
    except FileNotFoundError:
        if not _ffprobe_missing_logged:
            _ffprobe_missing_logged = True
            logger.warning(
                "ffprobe not installed -- SRT/CSV sidecars keep their "
                "wall-clock timing and are not rescaled to the encoded "
                "duration. Install ffmpeg in the extension image.")
    except Exception as e:
        logger.error(f"Error getting video duration: {str(e)}")
    return None

def list_session_parts(first_part_path):
    """Return every on-disk part file produced by the session that started
    with ``first_part_path``.

    With ``splitmuxsink`` the recorder writes one or more sibling files
    named ``<basename>_part<NN>_<NNNNN>.<ext>`` whenever the watchdog
    rebuilds the pipeline (RTSP drop, file stall, etc). This helper
    returns them in on-disk order so callers can sum durations or list
    sidecar artifacts for the whole logical recording.
    """
    if not first_part_path:
        return []
    base, ext = os.path.splitext(first_part_path)
    # Strip the trailing _partNN_NNNNN suffix that splitmuxsink appends.
    m = re.match(r"^(.*)_part\d{2,}_\d+$", base)
    prefix = m.group(1) if m else base
    pattern = f"{prefix}_part*_*{ext}"
    parts = sorted(glob.glob(pattern))
    if parts:
        return parts
    if os.path.exists(first_part_path):
        return [first_part_path]
    return []

def sum_session_video_duration(first_part_path):
    """Sum the ffprobe duration of every part written by this session.

    Used to scale SRT/ASS timestamps when the watchdog rebuilt the
    pipeline mid-recording: the sidecar files are wall-clock timed
    across the whole session, but each .ts part has its own PTS
    timeline (splitmuxsink resets per-segment), so we have to add the
    individual durations.
    """
    total = 0.0
    saw_any = False
    for p in list_session_parts(first_part_path):
        d = get_video_duration(p)
        if d is not None and d > 0:
            total += d
            saw_any = True
    return total if saw_any else None

def parse_srt_timestamp(timestamp_str):
    """Parse SRT timestamp (HH:MM:SS,mmm) to seconds"""
    parts = timestamp_str.replace(',', '.').split(':')
    hours = int(parts[0])
    minutes = int(parts[1])
    seconds = float(parts[2])
    return hours * 3600 + minutes * 60 + seconds

def adjust_srt_timing(srt_path, video_duration):
    """
    Adjust SRT timestamps to match actual video duration.
    Reads the SRT, calculates scaling factor, rewrites with corrected times.
    """
    try:
        # Read existing SRT file
        with open(srt_path, 'r') as f:
            content = f.read()
        
        if not content.strip():
            logger.warning("SRT file is empty, nothing to adjust")
            return False
        
        # Parse SRT entries
        entries = []
        blocks = content.strip().split('\n\n')
        
        max_srt_time = 0
        for block in blocks:
            lines = block.strip().split('\n')
            if len(lines) >= 3:
                entry_num = lines[0]
                time_line = lines[1]
                text = '\n'.join(lines[2:])
                
                # Parse timestamps
                start_str, end_str = time_line.split(' --> ')
                start_sec = parse_srt_timestamp(start_str)
                end_sec = parse_srt_timestamp(end_str)
                
                max_srt_time = max(max_srt_time, end_sec)
                entries.append({
                    'num': entry_num,
                    'start': start_sec,
                    'end': end_sec,
                    'text': text
                })
        
        if not entries or max_srt_time == 0:
            logger.warning("No valid SRT entries found")
            return False
        
        # Calculate scaling factor
        srt_duration = max_srt_time
        scale_factor = video_duration / srt_duration
        
        logger.info(f"SRT duration: {srt_duration:.2f}s, Video duration: {video_duration:.2f}s, Scale factor: {scale_factor:.4f}")
        
        # Only adjust if difference is significant (more than 1%)
        if abs(scale_factor - 1.0) < 0.01:
            logger.info("SRT timing is already within 1% of video duration, no adjustment needed")
            return True
        
        # Rewrite SRT with adjusted timestamps
        with open(srt_path, 'w') as f:
            for entry in entries:
                adjusted_start = entry['start'] * scale_factor
                adjusted_end = entry['end'] * scale_factor
                
                f.write(f"{entry['num']}\n")
                f.write(f"{format_srt_timestamp(adjusted_start)} --> {format_srt_timestamp(adjusted_end)}\n")
                f.write(f"{entry['text']}\n")
                f.write("\n")
        
        logger.info(f"SRT timing adjusted successfully (scaled by {scale_factor:.4f})")
        return True
        
    except Exception as e:
        logger.error(f"Error adjusting SRT timing: {str(e)}")
        return False

def get_towfish_altitude():
    """Get altitude from towfish AHRS2 message. Returns altitude in meters (negative underwater)."""
    try:
        response = requests.get(ahrs2_url, timeout=1)
        if response.status_code == 200:
            altitude = response.json()['message'].get('altitude', 0.0)
            return altitude
    except Exception as e:
        logger.debug(f"Error fetching towfish altitude: {str(e)}")
    return 0.0

def tilt_pwm_to_body_deg(pwm):
    """Map a raw SERVO16 PWM to the body-frame mount pitch (degrees).

    This is the camera pitch *relative to the towfish body* -- the raw
    servo deflection. ``SERVO16_REVERSED`` inverts the travel so the
    minimum PWM points the camera up (MNT1_PITCH_MAX) and the maximum PWM
    points it down (MNT1_PITCH_MIN). Returns ``None`` for a missing/zero
    PWM (servo not driven, e.g. disarmed).
    """
    if not pwm:  # None or 0 -> servo not driven
        return None
    span = TILT_PWM_MAX - TILT_PWM_MIN
    if span == 0:
        return None
    frac = (pwm - TILT_PWM_MIN) / span
    if TILT_SERVO_REVERSED:
        frac = 1.0 - frac
    angle = (TILT_MOUNT_PITCH_MIN_DEG
             + frac * (TILT_MOUNT_PITCH_MAX_DEG - TILT_MOUNT_PITCH_MIN_DEG))
    # Clamp to the configured mount travel so an out-of-range PWM can't
    # yield an absurd angle.
    lo = min(TILT_MOUNT_PITCH_MIN_DEG, TILT_MOUNT_PITCH_MAX_DEG)
    hi = max(TILT_MOUNT_PITCH_MIN_DEG, TILT_MOUNT_PITCH_MAX_DEG)
    return max(lo, min(hi, angle))


def get_towfish_camera_tilt(vehicle_pitch_deg=None):
    """Camera tilt (degrees), world-relative (earth-frame). Negative = down.

    The mount runs earth-frame stabilised, so the servo continuously
    trims the *body-frame* angle to hold a fixed earth-frame pointing.
    The number worth publishing (for EXIF / photogrammetry) is that
    world-relative angle, which is::

        world_pitch = body_pitch(SERVO16) + vehicle_pitch

    ``vehicle_pitch_deg`` should be the towfish ATTITUDE pitch in degrees;
    if omitted it is fetched here (one extra mavlink2rest GET). Returns
    ``None`` when the servo PWM is unavailable (e.g. disarmed, channel 0).
    """
    try:
        response = requests.get(servo_output_url, timeout=1)
        if response.status_code == 200:
            message = response.json().get('message', {})
            pwm = message.get(f'servo{TILT_SERVO_CHANNEL}_raw', None)
            body_deg = tilt_pwm_to_body_deg(pwm)
            if body_deg is None:
                return None
            if vehicle_pitch_deg is None:
                vehicle_pitch_deg = get_towfish_attitude().get('pitch')
            world_deg = body_deg + (vehicle_pitch_deg or 0.0)
            return world_deg
    except Exception as e:
        logger.debug(f"Error fetching camera tilt: {str(e)}")
    return None

def create_ass_file(video_path):
    """Create a new .ass (Advanced SubStation Alpha) subtitle file for full telemetry overlay."""
    base, _ = os.path.splitext(video_path)
    ass_path = base + '.ass'
    with open(ass_path, 'w') as f:
        f.write("[Script Info]\n")
        f.write("Title: Telemetry Data\n")
        f.write("ScriptType: v4.00+\n")
        f.write("WrapStyle: 0\n")
        f.write("ScaledBorderAndShadow: yes\n")
        f.write("PlayResX: 1920\n")
        f.write("PlayResY: 1080\n")
        f.write("\n")
        f.write("[V4+ Styles]\n")
        f.write("Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, "
                "OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, "
                "ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, "
                "Alignment, MarginL, MarginR, MarginV, Encoding\n")
        f.write("Style: Telem,Consolas,16,&H00FFFFFF,&H000000FF,&H00000000,"
                "&H80000000,-1,0,0,0,100,100,0,0,3,0,0,7,10,10,10,1\n")
        f.write("\n")
        f.write("[Events]\n")
        f.write("Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text\n")
    return ass_path

def format_ass_timestamp(seconds):
    """Format seconds into ASS timestamp format (H:MM:SS.cc)"""
    hours = int(seconds // 3600)
    minutes = int((seconds % 3600) // 60)
    secs = int(seconds % 60)
    centiseconds = int((seconds - int(seconds)) * 100)
    return f"{hours}:{minutes:02d}:{secs:02d}.{centiseconds:02d}"

def parse_ass_timestamp(ts):
    """Parse ASS timestamp (H:MM:SS.cc) to seconds."""
    parts = ts.strip().split(':')
    hours = int(parts[0])
    minutes = int(parts[1])
    sec_parts = parts[2].split('.')
    seconds = int(sec_parts[0])
    centiseconds = int(sec_parts[1]) if len(sec_parts) > 1 else 0
    return hours * 3600 + minutes * 60 + seconds + centiseconds / 100.0

def update_ass_file():
    """Update ASS overlay with labeled telemetry at 5 Hz.

    Like the SRT writer, timestamps are relative to ``sidecar_epoch``
    (reset per leg in transect mode). Any field whose source is not fresh
    is rendered blank ("--") rather than re-using the last value, so the
    overlay never shows stale numbers during a telemetry dropout.
    """
    global stop_ass_thread, ass_subtitle_counter

    ass_update_rate = 5

    while not stop_ass_thread and recording:
        try:
            with _sidecar_lock:
                ass_path = current_ass_file
                epoch = sidecar_epoch
                if ass_path and epoch:
                    ass_subtitle_counter += 1
                    elapsed = (datetime.now() - epoch).total_seconds()
                    if elapsed < 0:
                        elapsed = 0.0
                    have_entry = True
                else:
                    have_entry = False

            if have_entry:
                start_ts = format_ass_timestamp(elapsed)
                end_ts = format_ass_timestamp(elapsed + 1 / ass_update_rate)

                tf_att = get_towfish_attitude()
                tf_depth = get_depth_data()
                tf_climb = get_vfr_hud_data()
                tf_temp = get_baro_data()

                bb_lat, bb_lon, _ = get_blueboat_gps_position()
                bb_att = get_blueboat_attitude()
                bb_spd = get_blueboat_speed()

                tf_yaw = f"Yaw:{tf_att['yaw']:.1f}" if 'yaw' in tf_att else "Yaw:--"
                tf_roll = f"Roll:{tf_att['roll']:.1f}" if 'roll' in tf_att else "Roll:--"
                tf_dep = f"Dep:{tf_depth:.1f}" if tf_depth is not None else "Dep:--"
                tf_clm = f"Clm:{tf_climb:.2f}" if tf_climb is not None else "Clm:--"
                tf_tmp = f"Tmp:{tf_temp:.1f}" if tf_temp is not None else "Tmp:--"
                tf_parts = f"TF {tf_yaw} {tf_roll} {tf_dep} {tf_clm} {tf_tmp}"

                # BlueBoat fields come from the tow vehicle -- blank them
                # when not fresh rather than holding the last fix.
                bb_gps = f"{bb_lat:.6f},{bb_lon:.6f}" if bb_lat is not None else "--,--"
                bb_yaw = f"Yaw:{bb_att['yaw']:.1f}" if 'yaw' in bb_att else "Yaw:--"
                bb_pitch = f"Pitch:{bb_att['pitch']:.1f}" if 'pitch' in bb_att else "Pitch:--"
                bb_speed = f"Spd:{bb_spd:.1f}" if bb_spd is not None else "Spd:--"
                bb_parts = f"BB {bb_gps} {bb_yaw} {bb_pitch} {bb_speed}"

                text = f"{tf_parts} | {bb_parts}"
                line = f"Dialogue: 0,{start_ts},{end_ts},Telem,,0,0,0,,{text}\n"

                try:
                    with open(ass_path, 'a') as f:
                        f.write(line)
                except Exception as e:
                    logger.debug("ASS append failed (%s): %s", ass_path, e)

            time.sleep(1 / ass_update_rate)
        except Exception as e:
            logger.error(f"Error updating ASS file: {str(e)}")
            time.sleep(1)

def adjust_ass_timing(ass_path, video_duration):
    """Adjust ASS dialogue timestamps to match actual video duration."""
    try:
        with open(ass_path, 'r') as f:
            lines = f.readlines()

        header_lines = []
        dialogue_lines = []
        max_time = 0

        for line in lines:
            if line.startswith('Dialogue:'):
                dialogue_lines.append(line)
                parts = line.split(',', 9)
                if len(parts) >= 3:
                    end_sec = parse_ass_timestamp(parts[2])
                    max_time = max(max_time, end_sec)
            else:
                header_lines.append(line)

        if not dialogue_lines or max_time == 0:
            logger.warning("No valid ASS dialogue entries found")
            return False

        scale = video_duration / max_time
        if abs(scale - 1.0) < 0.01:
            logger.info("ASS timing already within 1% of video duration")
            return True

        with open(ass_path, 'w') as f:
            for line in header_lines:
                f.write(line)
            for line in dialogue_lines:
                parts = line.split(',', 9)
                if len(parts) >= 3:
                    start_sec = parse_ass_timestamp(parts[1]) * scale
                    end_sec = parse_ass_timestamp(parts[2]) * scale
                    parts[1] = format_ass_timestamp(start_sec)
                    parts[2] = format_ass_timestamp(end_sec)
                f.write(','.join(parts))

        logger.info(f"ASS timing adjusted (scaled by {scale:.4f})")
        return True
    except Exception as e:
        logger.error(f"Error adjusting ASS timing: {str(e)}")
        return False

def fetch_telemetry_block():
    """Read the full towfish + tow-vehicle telemetry block in one shot.

    Shared by the timelapse capture loop (where it runs in a worker
    thread parallel to the JPEG fetch so the samples share the camera
    shutter's wall-clock window) and by the video sidecar writer. One
    definition means a video leg and a still-image leg record the same
    fields, computed the same way.

    Returns a dict (always populated, with None values where data was
    unavailable) plus a ``fetch_ms`` field recording how long the reads
    took, so consumers can judge staleness.
    """
    t0 = time.monotonic()
    try:
        bb_lat, bb_lon, bb_alt = get_blueboat_gps_position()
        # One ATTITUDE read gives heading (yaw), roll and pitch, so we
        # geotag and record the full towfish orientation per sample
        # without extra mavlink2rest round-trips.
        att = get_towfish_attitude()
        heading = att.get('yaw')
        roll = att.get('roll')
        pitch = att.get('pitch')
        gps_lat, gps_lon = offset_towed_position(bb_lat, bb_lon, heading)
        tow_alt = get_towfish_altitude()
        depth = get_depth_data()
        temp = get_baro_data()
        # World-relative camera pitch: reuse the ATTITUDE pitch we
        # already fetched instead of a second round-trip.
        tilt = get_towfish_camera_tilt(vehicle_pitch_deg=pitch)
    except Exception:
        logger.exception("Telemetry fetch raised")
        bb_lat = bb_lon = bb_alt = None
        gps_lat = gps_lon = None
        heading = None
        roll = None
        pitch = None
        tow_alt = None
        depth = None
        temp = None
        tilt = None
    return {
        'bb_lat': bb_lat, 'bb_lon': bb_lon, 'bb_alt': bb_alt,
        'gps_lat': gps_lat, 'gps_lon': gps_lon,
        'heading': heading, 'roll': roll, 'pitch': pitch,
        'tow_alt': tow_alt,
        'depth': depth, 'temp': temp, 'tilt': tilt,
        'fetch_ms': round((time.monotonic() - t0) * 1000, 1),
    }


def update_geo_sidecars():
    """Write the SRT position cue and the telemetry CSV row, 5 Hz.

    Both files are produced from a *single* telemetry sample per tick,
    so they can never disagree about a given instant -- the SRT stays a
    minimal, player-readable position track while the CSV carries the
    full orientation set that photogrammetry needs.

    Timestamps are relative to ``sidecar_epoch`` (reset per leg in
    transect mode), then rescaled to the encoded duration when the files
    are finalised. When tow-vehicle telemetry is *not fresh* (GPS fetch
    failed/timed out) the SRT entry is written with EMPTY position values
    rather than carrying the last known fix forward -- gaps stay explicit
    and never get back-filled with stale coordinates.
    """
    global stop_srt_thread, srt_subtitle_counter

    srt_update_rate = 5  # Updates per second (5 Hz for position data)

    while not stop_srt_thread and recording:
        try:
            # Cheap bookkeeping under the lock: capture the files + epoch
            # and claim a sequence number atomically so a concurrent leg
            # rotation can't interleave a half-written entry.
            with _sidecar_lock:
                srt_path = current_srt_file_rtsp
                csv_path = current_video_csv_file
                wp_label = current_video_csv_wp
                epoch = sidecar_epoch
                if srt_path and epoch:
                    srt_subtitle_counter += 1
                    entry_num = srt_subtitle_counter
                    elapsed = (datetime.now() - epoch).total_seconds()
                    if elapsed < 0:
                        elapsed = 0.0
                else:
                    entry_num = None

            if entry_num is not None:
                ts_local = datetime.now()
                start_timestamp = format_srt_timestamp(elapsed)
                end_timestamp = format_srt_timestamp(elapsed + 1 / srt_update_rate)

                # Fresh fetch every iteration; a None lat/lon means the
                # tow vehicle is unreachable / data is stale.
                telem = fetch_telemetry_block()
                offset_lat = telem['gps_lat']
                offset_lon = telem['gps_lon']
                towfish_alt = telem['tow_alt']

                if offset_lat is not None and offset_lon is not None:
                    alt_str = f"{towfish_alt:.1f}" if towfish_alt is not None else ""
                    pos_line = (f"latitude: {offset_lat:.6f} "
                                f"longitude: {offset_lon:.6f} "
                                f"altitude: {alt_str}")
                else:
                    # Not fresh -> empty values (no stale carry-forward).
                    pos_line = "latitude:  longitude:  altitude: "

                srt_entry = (
                    f"{entry_num}\n"
                    f"{start_timestamp} --> {end_timestamp}\n"
                    f"{pos_line}\n"
                    f"\n"
                )
                try:
                    with open(srt_path, 'a') as f:
                        f.write(srt_entry)
                except Exception as e:
                    logger.debug("SRT append failed (%s): %s", srt_path, e)

                if csv_path:
                    try:
                        with open(csv_path, 'a', newline='') as f:
                            csv.writer(f).writerow(
                                _video_csv_row(ts_local, elapsed, csv_path,
                                               wp_label, telem)
                            )
                    except Exception as e:
                        logger.debug("Telemetry CSV append failed (%s): %s",
                                     csv_path, e)

            time.sleep(1 / srt_update_rate)
        except Exception as e:
            logger.error(f"Error updating geo sidecars: {str(e)}")
            time.sleep(1)


def _video_csv_row(ts_local, elapsed, csv_path, wp_label, telem):
    """Format one ``_VIDEO_CSV_HEADER`` row from a telemetry sample.

    Number formatting matches the timelapse CSV column-for-column so
    both sidecars parse identically downstream.
    """
    def num(value, fmt):
        return format(value, fmt) if value is not None else ""

    video_file = os.path.basename(csv_path).replace('_telemetry.csv', '')
    return [
        ts_local.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
        f"{elapsed:.3f}",
        video_file,
        wp_label or "",
        num(telem['gps_lat'], '.6f'),
        num(telem['gps_lon'], '.6f'),
        num(telem['bb_alt'], '.2f'),
        num(telem['tow_alt'], '.2f'),
        num(telem['heading'], '.1f'),
        num(telem['roll'], '.1f'),
        num(telem['pitch'], '.1f'),
        num(telem['depth'], '.2f'),
        num(telem['temp'], '.2f'),
        num(telem['tilt'], '.1f'),
        f"{telem['fetch_ms']:.1f}",
    ]

async def data_lake_handler(websocket):
    """Stream recording state to Cockpit data lake clients."""
    logger.info(f"Data lake client connected: {websocket.remote_address}")
    try:
        while True:
            value = "true" if recording else "false"
            await websocket.send(f"{DATA_LAKE_RECORDING_VAR}={value}")
            await asyncio.sleep(0.5)
    except ConnectionClosed:
        logger.info(f"Data lake client disconnected: {websocket.remote_address}")

async def data_lake_main():
    """Start the data lake WebSocket server and run forever."""
    async with websockets.serve(data_lake_handler, DATA_LAKE_WS_HOST, DATA_LAKE_WS_PORT):
        logger.info(f"Data lake WebSocket server started on ws://{DATA_LAKE_WS_HOST}:{DATA_LAKE_WS_PORT}")
        await asyncio.Future()

def start_data_lake_server():
    """Run the data lake WebSocket server in a background thread."""
    def run():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(data_lake_main())

    thread = threading.Thread(target=run, daemon=True)
    thread.start()

# ---------------------------------------------------------------------------
# Recording diagnostics: event log, GStreamer stderr, Pi4 stats, watchdog
# ---------------------------------------------------------------------------

def create_events_file(video_path):
    """Create a new NDJSON file for recording events."""
    base, _ = os.path.splitext(video_path)
    events_path = base + '_events.ndjson'
    with open(events_path, 'w') as f:
        pass
    return events_path

def log_event(event_type, detail=""):
    """Write a timestamped event to the recording events log."""
    if not current_events_file:
        return
    try:
        event = {
            "ts": time.time(),
            "time": datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            "event": event_type,
            "detail": str(detail)
        }
        with open(current_events_file, 'a') as f:
            f.write(json.dumps(event) + '\n')
    except Exception as e:
        logger.debug(f"Error writing event log: {e}")

def _read_disk_free_mb(path=None):
    """Read free disk space on a recording path in MB.

    Defaults to the local extension volume to preserve the historical
    behaviour of the /status endpoint; callers that record to USB can
    pass the resolved path to get the *active* target's free space.
    """
    target = path or LOCAL_RECORDING_DIR
    try:
        stat = os.statvfs(target)
        return round((stat.f_bavail * stat.f_frsize) / (1024 * 1024), 1)
    except Exception:
        return None


def _resolve_recording_dir(subfolder=None, force_local=False):
    """Decide where the next recording should be written.

    Returns a tuple ``(dir_path, on_usb)``. When the user prefers USB
    *and* the drive is currently usable (mounted with at least
    ``USB_MIN_FREE_GB`` free) the path is on the USB mount; otherwise
    we fall back to ``LOCAL_RECORDING_DIR``. The local fallback is
    automatic so a near-full or unplugged stick can never wedge the
    recorder.

    ``subfolder`` is appended under the USB ``Towfish/`` root when
    given (used for per-session timelapse folders); video recordings
    that share one root pass ``None`` and get the Towfish root itself.

    ``force_local`` is set by the failover path so a session that
    just lost USB doesn't immediately re-pick USB on restart.
    """
    use_usb = (
        not force_local
        and storage_preference == "usb"
        and usb_storage.is_usable()
    )
    if use_usb:
        try:
            if subfolder:
                rec_dir = usb_storage.get_recording_dir(subfolder)
            else:
                rec_dir = usb_storage.get_recording_dir("")
                # get_recording_dir always returns ``<root>/Towfish/<subfolder>``;
                # an empty subfolder just gives us ``<root>/Towfish/`` which is
                # exactly what we want for files that live at the session root.
            return rec_dir, True
        except Exception as e:
            logger.warning(
                f"USB resolve failed ({e}); falling back to local storage"
            )

    if storage_preference == "usb" and not force_local:
        logger.info(
            "USB preferred but not usable (mounted=%s, fstype=%s, free_mb=%s); "
            "falling back to local storage at %s",
            usb_storage.is_mounted(),
            usb_storage.get_fstype() or None,
            usb_storage.get_free_mb(),
            LOCAL_RECORDING_DIR,
        )

    os.makedirs(LOCAL_RECORDING_DIR, exist_ok=True)
    if subfolder:
        local = os.path.join(LOCAL_RECORDING_DIR, subfolder)
        os.makedirs(local, exist_ok=True)
        return local, False
    return LOCAL_RECORDING_DIR, False


# ── Mid-recording USB failover ───────────────────────────────────────────
# A background daemon watches the USB drive while a session is writing to
# it and moves the session to local storage on either failure mode:
#
#   "lost"  -- drive yanked, FS error: usb_storage.is_healthy() goes False.
#   "full"  -- drive ran out of room mid-mission. A full filesystem still
#              stats fine, so health never catches this; we watch free
#              space against usb_storage.USB_MIN_FREE_MB_RECORDING instead
#              and move while there is still room to finalise the file.
#
# Either way we kill the current session and immediately restart on local
# storage with force_local=True so we don't bounce straight back.

_USB_HEALTH_INTERVAL_S = 5.0
_usb_health_thread = None
_usb_health_stop = threading.Event()

#: Set by a capture path that has actually hit ENOSPC, so the watcher can
#: fail over on the next tick instead of waiting for free space to cross the
#: low-water mark. Belt-and-braces for a drive that fills faster than the
#: poll interval, or one whose free-space reporting we can't trust.
_usb_space_alarm = threading.Event()


def _raise_usb_space_alarm(where):
    """Flag that a write hit ENOSPC on the USB drive.

    Called from capture threads, which must not run the failover
    themselves: it stops and restarts the very session they belong to,
    so doing it inline would have a thread join itself. Handing the work
    to the watcher keeps all session teardown on one thread.
    """
    if not usb_recording:
        return
    if not _usb_space_alarm.is_set():
        logger.error("USB write hit ENOSPC in %s; requesting failover", where)
    _usb_space_alarm.set()


def _handle_usb_failover(reason="lost"):
    """Stop the active USB-backed session and restart it on local storage.

    Runs from the USB health watcher thread; takes ``_mode_lock`` so it
    can never race the Flask /start, /stop, /timelapse routes or the
    TransectMonitor's own start/stop calls.

    ``reason`` is ``"lost"`` (drive gone) or ``"full"`` (out of space), and
    is surfaced on /status so the operator can tell a yanked stick from one
    that simply filled up.
    """
    global usb_recording, usb_failover_count, usb_failover_reason
    with _mode_lock:
        if not usb_recording:
            return
        active_mode = mode
        detail = ("ran out of space" if reason == "full" else "was lost")
        logger.warning("USB failover: storage %s mid-recording "
                       "(mode=%s, free_mb=%s)",
                       detail, active_mode, usb_storage.get_free_mb())
        log_event("usb_failover",
                  f"USB storage {detail} during recording "
                  f"(mode={active_mode}, reason={reason})")
        usb_failover_count += 1
        usb_failover_reason = reason

        # Moving to a local disk that is itself nearly full just relocates
        # the problem, and the operator can only act on it if we say so.
        local_free = _read_disk_free_mb(LOCAL_RECORDING_DIR)
        if local_free is not None and local_free < _LOCAL_LOW_FREE_MB:
            logger.error("USB failover: local storage is also low "
                         "(%.0f MB free at %s) -- recording may stop shortly",
                         local_free, LOCAL_RECORDING_DIR)
            log_event("local_storage_low",
                      f"{local_free:.0f} MB free at {LOCAL_RECORDING_DIR}")

        try:
            if active_mode == MODE_VIDEO:
                _stop_video_session()
                _start_video_session(base_prefix="video_rtsp",
                                     target_mode=MODE_VIDEO,
                                     force_local=True)
            elif active_mode == MODE_TIMELAPSE:
                # Stop the manual timelapse session, then restart it on
                # local storage. Mirrors the dropcam failover path.
                global _timelapse
                session = _timelapse
                _timelapse = None
                if session is not None:
                    session.stop()
                _set_mode(MODE_IDLE)
                # Re-create on local storage, continuing the day's folder
                # and global sequence (start() re-scans it) under a fresh
                # source_tag so the failover frames stay attributable.
                subfolder = _survey_day_subfolder()
                source_tag = _make_source_tag("tl")
                out_dir, _ = _resolve_recording_dir(
                    subfolder=subfolder, force_local=True,
                )
                new_session = TimelapseSession(snap_url=snapshot_url,
                                               out_dir=out_dir,
                                               source_tag=source_tag)
                new_session.on_usb = False
                new_session.start()
                _timelapse = new_session
                _set_mode(MODE_TIMELAPSE)
            elif active_mode == MODE_TRANSECT:
                # Transect is the mode an actual survey runs in, so tearing
                # it down on a storage problem means abandoning the mission
                # the boat is still flying. Ask the monitor to swap the
                # in-flight capture onto local storage and stay in
                # "recording" instead; it re-opens the session at the
                # current waypoint so the leg continues.
                swapped = False
                if _transect_monitor is not None:
                    try:
                        swapped = _transect_monitor.swap_to_local_storage()
                    except Exception:
                        logger.exception("transect storage swap failed")
                if not swapped:
                    # Swap failed, so fall back to the old behaviour: stop
                    # everything rather than let a session keep writing at
                    # a drive that is gone or full. The operator can
                    # re-enable, which will start on local storage.
                    logger.warning(
                        "transect storage swap unavailable; disabling monitor")
                    if _transect_monitor is not None:
                        try:
                            _transect_monitor.disable()
                        except Exception:
                            logger.exception("transect monitor disable failed")
                    if _session is not None:
                        try:
                            _stop_video_session()
                        except Exception:
                            logger.exception("post-disable video stop failed")
                    if _timelapse is not None:
                        try:
                            _stop_transect_timelapse()
                        except Exception:
                            logger.exception("post-disable timelapse stop failed")
                    _set_mode(MODE_IDLE)
            else:
                logger.warning("USB failover: unexpected mode=%s, no-op",
                               active_mode)
                usb_recording = False
                return
        except Exception:
            logger.exception("USB failover: restart on local storage failed")
            usb_recording = False
            return

        # The mode-specific branches above either restarted on local
        # storage (MODE_VIDEO / MODE_TIMELAPSE) and reset usb_recording
        # via _start_*/_stop_*, or tore everything down (MODE_TRANSECT).
        # Clear the flag explicitly to be safe.
        if mode == MODE_IDLE:
            usb_recording = False

        logger.info("USB failover complete: now recording to local storage "
                    "(mode=%s, failovers=%d)", mode, usb_failover_count)
        log_event("usb_failover_complete",
                  f"resumed mode={mode} on local storage")


def _gst_error_is_no_space(err):
    """Is this GStreamer GError a "disk is full" from a sink?

    Prefer the typed domain/code so we aren't matching on message text,
    which is translated; fall back to a substring only if the typed check
    isn't available on this GStreamer build.
    """
    try:
        if err.matches(Gst.ResourceError.quark(), Gst.ResourceError.NO_SPACE_LEFT):
            return True
    except Exception:
        pass
    return "no space left" in (getattr(err, "message", "") or "").lower()


def _usb_health_loop():
    """Trigger failover when the USB drive is lost or fills up mid-recording."""
    while not _usb_health_stop.is_set():
        try:
            if usb_recording:
                reason = None
                if not usb_storage.is_healthy():
                    reason = "lost"
                elif _usb_space_alarm.is_set():
                    reason = "full"
                elif not usb_storage.has_recording_headroom():
                    reason = "full"
                if reason is not None:
                    _handle_usb_failover(reason)
                    # Clear only after the swap, so a capture thread that
                    # trips ENOSPC again on the new target can re-arm it.
                    _usb_space_alarm.clear()
        except Exception:
            logger.exception("USB health watcher raised")
        _usb_health_stop.wait(_USB_HEALTH_INTERVAL_S)


def _start_usb_health_watcher():
    """Start the USB health watcher thread (idempotent)."""
    global _usb_health_thread
    if _usb_health_thread and _usb_health_thread.is_alive():
        return
    _usb_health_stop.clear()
    _usb_health_thread = threading.Thread(
        target=_usb_health_loop, name="usb-health", daemon=True,
    )
    _usb_health_thread.start()
    logger.info("USB health watcher started (interval=%.1fs)",
                _USB_HEALTH_INTERVAL_S)

# ---------------------------------------------------------------------------
# In-process GStreamer recording with auto-restart watchdog.
#
# Replaces the legacy ``subprocess.Popen(["gst-launch-1.0", ...])`` path.
# The pipeline shape itself mirrors the hauv-v2 branch's RTSP recorder
# (UDP transport, latency=5000, h264parse config-interval=-1, big leaky
# queue between parser and muxer); the only structural addition here is
# ``splitmuxsink`` so the watchdog can roll a new on-disk part on each
# pipeline rebuild without losing what was already recorded. Key
# motivations:
#
# * RTSP drops at 4K bitrates silently kill the recorder. The watchdog
#   here detects ERROR/EOS on the GStreamer bus *and* file-stall and
#   restarts the pipeline automatically; per-restart files are written
#   via ``splitmuxsink`` so we never lose the previously-recorded data.
#   Only the /stop Flask route ever sets the stop event -- everything
#   else respawns.
# * Codec is H.264 (was H.265): at 4K the H.265 RTSP path on this camera
#   kept dropping at random large file sizes. H.264 over the same RTSP
#   transport is markedly more stable end-to-end, at the cost of larger
#   files. The user reconfigured the camera so ``stream_0`` now serves
#   H.264.
# * The 1-hour MPEG-TS PTS offset that breaks VLC playback comes from
#   ``mpegtsmux`` reusing rtspsrc's first PTS. ``splitmuxsink`` opens a
#   fresh muxer per fragment (PTS resets to zero) so every part plays
#   straight away.
# * ``wait-for-keyframe=true`` + ``alignment=au`` + ``config-interval=-1``
#   guarantee every part starts at a decodable SPS+PPS+IDR.
# * ``async-finalize=false`` per the doris h265 docstring: their tests
#   showed ``async-finalize=true`` could silently freeze splitmuxsink
#   mid-rotation while the bus continued to report the pipeline healthy.
# ---------------------------------------------------------------------------

_RESTART_BACKOFF_S = 1.0
_MIN_GOOD_RUNTIME_S = 3.0
_MAX_BACKOFF_S = 5.0
_STOP_EOS_TIMEOUT_S = 5.0

# How many consecutive 5-second polls with no file growth count as a
# stall (and thus a hung pipeline that needs respawning).
_STALL_POLLS_BEFORE_RESTART = 3

_gst_init_lock = threading.Lock()
_gst_initialized = False

def _ensure_gst_init():
    global _gst_initialized
    with _gst_init_lock:
        if not _gst_initialized:
            Gst.init(None)
            _gst_initialized = True
            logger.info("GStreamer initialized (version %s)", Gst.version_string())

#: ``splitmuxsink max-size-bytes`` value used when recording to a vfat USB.
#: FAT32 caps single files at 4 GiB; rolling at 3.5 GiB leaves headroom for
#: muxer overhead so the active fragment can finalise before the cap hits.
FAT32_MAX_PART_BYTES = int(3.5 * 1024 * 1024 * 1024)

_element_prop_cache = {}


def _element_has_property(factory_name, prop_name):
    """Whether the installed ``factory_name`` element exposes ``prop_name``.

    Element properties come and go between GStreamer releases and the
    BlueOS base image is not pinned to one, so anything version-dependent
    has to be probed rather than assumed. This matters because
    ``Gst.parse_launch`` treats an unknown property as a *fatal* parse
    error: a single flag the runtime doesn't recognise takes out the whole
    recorder rather than degrading. Cached because it's asked once per
    pipeline build and the answer can't change while we're running.
    """
    key = (factory_name, prop_name)
    if key not in _element_prop_cache:
        present = False
        try:
            _ensure_gst_init()
            element = Gst.ElementFactory.make(factory_name, None)
            if element is None:
                logger.warning("property probe: element %s unavailable", factory_name)
            else:
                present = element.find_property(prop_name) is not None
        except Exception:
            logger.exception("property probe failed for %s %s",
                             factory_name, prop_name)
        _element_prop_cache[key] = present
        logger.info("property probe: %s has %s = %s",
                    factory_name, prop_name, present)
    return _element_prop_cache[key]


def _build_pipeline_description(rtsp_url, container_fmt, proto,
                                mux_name="muxsink", max_size_bytes=0):
    """Build the gst-parse_launch description for one RTSP H.264 session.

    Mirrors the hauv-v2 branch's RTSP recording pipeline (UDP transport,
    latency=5000, h264parse config-interval=-1, big leaky queue between
    parser and muxer) but swaps the single ``filesink`` for a
    ``splitmuxsink`` so the watchdog can roll a new on-disk part on each
    pipeline rebuild without losing what was already recorded.

    Why we switched off H.265 here: at 4K bitrates the H.265 RTSP path
    on this camera kept dropping at random large file sizes, which
    forced a watchdog respawn and produced lots of short, unplayable
    parts. H.264 over the same RTSP transport is markedly more stable
    end-to-end at the cost of larger files. The user reconfigured the
    camera so ``stream_0`` now serves H.264.

    Per-fragment PTS reset by splitmuxsink also keeps VLC happy (the
    old single-mpegtsmux path produced a 1-hour PTS offset that broke
    playback until you scrubbed past it).

    ``max_size_bytes`` (default 0 = unlimited) asks splitmuxsink to roll
    a new part once the current file reaches the byte threshold. Used
    when the resolved target lives on a vfat USB stick to stay under
    the FAT32 4 GiB per-file cap.

    Deliberately *no* ``stream-format`` capsfilter ahead of the muxer:
    the two containers disagree (``mp4mux`` only accepts ``avc``, while
    ``mpegtsmux`` only accepts ``byte-stream``), so pinning either one
    makes the other container fail to link at parse time. Leaving it out
    lets ``h264parse`` negotiate whatever the configured muxer asks for.
    """
    if container_fmt == "mpegts":
        muxer_factory = "mpegtsmux"
    else:
        muxer_factory = "mp4mux"
    proto = proto if proto in VALID_STREAM_PROTOCOLS else DEFAULT_STREAM_PROTOCOL
    # Starting a part mid-GOP yields a leading burst of undecodable
    # frames, which "wait-for-keyframe" avoids -- but it only exists from
    # GStreamer 1.20, and asking for it on 1.16 is a fatal parse error.
    # splitmuxsink still gates each fragment on a keyframe regardless, so
    # omitting it on older runtimes costs nothing at a file boundary.
    depay = "rtph264depay"
    if _element_has_property("rtph264depay", "wait-for-keyframe"):
        depay += " wait-for-keyframe=true"
    return (
        f"rtspsrc location={rtsp_url} latency=5000 "
        f"protocols={proto} retry=5 timeout=5000000 "
        f"! {depay} "
        f"! h264parse config-interval=-1 "
        # hauv-v2 leaky queue: absorbs RTSP jitter so a brief mux stall
        # never back-pressures the depayloader (which would otherwise
        # drop the RTP session). 30 s of headroom, no byte/buffer cap,
        # leak the oldest frames downstream if anything ever wedges.
        f"! queue max-size-time=30000000000 max-size-bytes=0 max-size-buffers=0 "
        f"leaky=downstream silent=true "
        f"! splitmuxsink name={mux_name} max-size-time=0 "
        f"max-size-bytes={int(max_size_bytes)} "
        f"muxer-factory={muxer_factory} send-keyframe-requests=true "
        f"async-finalize=false"
    )

def _finalize_leg_sidecars(srt_path, ass_path, csv_path, ts_path):
    """Rescale a closed leg's SRT/ASS/CSV to that leg's encoded duration.

    Runs in a short-lived daemon thread. ``splitmuxsink`` finalises the
    ``.ts`` around the time the next fragment opens, so we briefly poll
    for a stable, non-empty file before asking ffprobe for its duration.
    A missing duration leaves the (unscaled, wall-clock-timed) sidecars
    in place rather than corrupting them.
    """
    try:
        duration = None
        deadline = time.monotonic() + 8.0
        last_size = -1
        while time.monotonic() < deadline:
            if ts_path and os.path.exists(ts_path):
                sz = os.path.getsize(ts_path)
                if sz > 0 and sz == last_size:
                    duration = get_video_duration(ts_path)
                    if duration:
                        break
                last_size = sz
            time.sleep(0.5)
        if not duration and ts_path and os.path.exists(ts_path):
            duration = get_video_duration(ts_path)
        if duration:
            if srt_path and os.path.exists(srt_path):
                adjust_srt_timing(srt_path, duration)
            if ass_path and os.path.exists(ass_path):
                adjust_ass_timing(ass_path, duration)
            if csv_path and os.path.exists(csv_path):
                adjust_video_csv_timing(csv_path, duration)
            logger.info("Per-leg sidecars finalised for %s (%.2fs)",
                        os.path.basename(ts_path) if ts_path else "?", duration)
        else:
            logger.warning("Per-leg finalise: no duration for %s, sidecars left unscaled",
                           ts_path)
    except Exception:
        logger.exception("Per-leg sidecar finalise failed")


def _rotate_leg_sidecars(prev_ts_path, new_ts_path):
    """Swap SRT/ASS/CSV to a new leg file, finalising the previous leg's set.

    Called from the ``splitmuxsink`` format-location callback (GStreamer
    streaming thread) when a new leg ``.ts`` opens. Kept cheap: creates
    the new empty sidecars, repoints the globals under ``_sidecar_lock``
    (resetting counters + ``sidecar_epoch`` so the new leg starts at
    00:00), and hands the just-closed set to a background finaliser so
    ffprobe never blocks the streaming thread.
    """
    global current_srt_file_rtsp, current_ass_file
    global current_video_csv_file, current_video_csv_wp
    global srt_subtitle_counter, ass_subtitle_counter, sidecar_epoch

    with _sidecar_lock:
        old_srt = current_srt_file_rtsp
        old_ass = current_ass_file
        old_csv = current_video_csv_file

    new_srt = create_srt_file(new_ts_path)
    new_ass = create_ass_file(new_ts_path)
    new_csv, new_wp = create_video_telemetry_csv(new_ts_path)

    with _sidecar_lock:
        current_srt_file_rtsp = new_srt
        current_ass_file = new_ass
        current_video_csv_file = new_csv
        current_video_csv_wp = new_wp
        srt_subtitle_counter = 0
        ass_subtitle_counter = 0
        sidecar_epoch = datetime.now()

    threading.Thread(
        target=_finalize_leg_sidecars,
        args=(old_srt, old_ass, old_csv, prev_ts_path),
        name="leg-sidecar-finalise", daemon=True,
    ).start()
    log_event("leg_sidecars_rotated", os.path.basename(new_ts_path))


class RecordingSession:
    """One logical recording (one /start ... /stop call).

    Owns a background thread that keeps a GStreamer pipeline alive,
    rebuilding it on RTSP drops or file stalls. Each rebuild produces
    a new on-disk part via ``splitmuxsink`` so we never lose what was
    already written.
    """

    def __init__(self, rtsp_url, out_dir, base_filename, ext,
                 container_fmt, proto, per_leg_sidecars=False,
                 fat_size_cap=False):
        self._rtsp_url = rtsp_url
        self._out_dir = out_dir
        self._base_filename = base_filename  # "video_rtsp_<TS>" or "transect_<TS>"
        self._ext = ext  # ".ts" or ".mp4"
        self._container_fmt = container_fmt
        self._proto = proto
        # When True (transect mode), every fragment after the first gets
        # its own fresh SRT/ASS sidecar named to match, and the previous
        # fragment's sidecars are finalised to that file's duration. When
        # False (manual recording), one sidecar set spans the whole
        # session and is rescaled over the sum of part durations at /stop.
        self.per_leg_sidecars = per_leg_sidecars
        # When True, splitmuxsink will roll a new part at FAT32_MAX_PART_BYTES
        # so a long recording on a vfat USB never trips the FAT 4 GiB cap.
        self._fat_size_cap = fat_size_cap
        # Set by the caller after start() to remember which storage tier
        # this session is currently writing to. Read by the watchdog loop
        # so a USB drive that disappears mid-recording can trigger failover.
        self.on_usb = False
        # Counts fragments splitmuxsink has opened this session; the first
        # already has sidecars created at /start, so rotation only kicks
        # in from the second fragment onward.
        self._fragment_count = 0

        self._stop_event = threading.Event()
        self._thread = None
        self._lock = threading.Lock()

        self._pipeline = None
        self._muxsink = None
        self._current_part = 0
        # Optional label appended to the next fragment filename. The
        # TransectMonitor flips this via :meth:`split_now` to tag each
        # leg's .ts file with its WP index, e.g. ``_wp03``. None means
        # "no extra label", which preserves the historical naming for
        # the manual /start path.
        self._next_label = None
        self.restart_count = 0
        self.last_pattern = None
        self.last_exit = None
        # First on-disk part (used by /filesize, sidecar timing, etc.)
        self.first_part_path = None
        # Diagnostic counters surfaced via /status
        self.gst_errors = 0
        self.gst_warnings = 0
        self.file_stalls = 0
        # Counts splitmuxsink ``split-now`` invocations so /status can
        # show how many legs a transect session has rolled.
        self.split_count = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start(self):
        if self._thread is not None:
            raise RuntimeError("RecordingSession already started")
        _ensure_gst_init()
        self._thread = threading.Thread(
            target=self._watchdog, name="rec-watchdog", daemon=True,
        )
        self._thread.start()

    def is_alive(self):
        return self._thread is not None and self._thread.is_alive()

    def stop(self, timeout_s=12.0):
        """Request stop; wait for the pipeline thread to finalise."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)
            if self._thread.is_alive():
                logger.warning("RecordingSession watchdog did not exit in %.1fs", timeout_s)

    def split_now(self, label=None):
        """Roll a fresh fragment immediately, stamping the next file with ``label``.

        Used by :class:`TransectMonitor` at each waypoint boundary so the
        upcoming ``.ts`` file carries the new WP index in its filename
        (e.g. ``transect_<TS>_part03_00000_wp03.ts``). The existing
        ``splitmuxsink`` keeps the RTSP pipeline alive across the split,
        so the only "missing" footage is the few hundred ms the muxer
        spends rotating to the next file at an IDR boundary.

        Safe to call from any thread: ``_next_label`` is set under the
        same lock the format-location callback observes, and the
        ``split-now`` action signal is async (the muxer dispatches it
        to its streaming thread, which then calls back into
        ``_on_format_location``).

        No-op if the pipeline isn't running yet (the very first part
        comes up naturally on PLAYING).
        """
        with self._lock:
            self._next_label = label
            mux = self._muxsink
        if mux is None:
            return False
        try:
            mux.emit("split-now")
            self.split_count += 1
            log_event("split_now", f"label={label}")
            return True
        except Exception:
            logger.exception("splitmuxsink split-now emit failed")
            return False

    # ------------------------------------------------------------------
    # splitmuxsink format-location callback
    # ------------------------------------------------------------------
    def _on_format_location(self, _splitmux, fragment_id):
        """Compose the on-disk path for the next .ts/.mp4 fragment.

        Honours ``self._next_label`` -- set by :meth:`split_now` from the
        transect monitor -- so each leg's file is named with its WP
        index (``..._wp03.ts``). Reads-and-clears the label atomically
        under the same lock ``split_now`` uses, so a second split that
        fires while we're still composing this path can't lose its label.
        """
        with self._lock:
            label = self._next_label
            self._next_label = None
        label_part = f"_{label}" if label else ""
        path = os.path.join(
            self._out_dir,
            f"{self._base_filename}_part{self._current_part:02d}_"
            f"{int(fragment_id):05d}{label_part}{self._ext}",
        )
        prev_path = self.last_pattern
        self.last_pattern = path
        if self.first_part_path is None:
            self.first_part_path = path
        self._fragment_count += 1
        log_event("part_opened", path)
        # Per-leg sidecar rotation (transect only): when a new leg file
        # opens, hand the just-closed file's SRT/ASS to a background
        # finaliser and start fresh sidecars matching the new file. The
        # first fragment is skipped -- its sidecars were created at /start.
        if self.per_leg_sidecars and self._fragment_count > 1:
            try:
                _rotate_leg_sidecars(prev_path, path)
            except Exception:
                logger.exception("per-leg sidecar rotation failed")
        return path

    # ------------------------------------------------------------------
    # Pipeline build / run
    # ------------------------------------------------------------------
    def _build_pipeline(self):
        desc = _build_pipeline_description(
            self._rtsp_url, self._container_fmt, self._proto,
            max_size_bytes=(FAT32_MAX_PART_BYTES if self._fat_size_cap else 0),
        )
        logger.info("RECORD building pipeline part=%d: %s",
                    self._current_part, desc)
        pipeline = Gst.parse_launch(desc)
        if not isinstance(pipeline, Gst.Pipeline):
            raise RuntimeError("Gst.parse_launch returned non-pipeline")
        muxsink = pipeline.get_by_name("muxsink")
        if muxsink is None:
            raise RuntimeError("splitmuxsink 'muxsink' not found in pipeline")
        muxsink.connect("format-location", self._on_format_location)
        with self._lock:
            self._pipeline = pipeline
            self._muxsink = muxsink
        return pipeline

    def _run_one(self):
        """Run one pipeline instance until stop / ERROR / EOS / stall.

        Returns ``(exit_reason, runtime_s)``.
        """
        t0 = time.monotonic()
        try:
            pipeline = self._build_pipeline()
        except Exception as e:
            logger.exception("RECORD pipeline build failed")
            log_event("pipeline_build_failed", str(e))
            return "build_failed", time.monotonic() - t0

        bus = pipeline.get_bus()
        ret = pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            logger.warning("RECORD set_state(PLAYING) returned FAILURE")
            log_event("set_state_failure", f"part={self._current_part}")
            self._teardown_pipeline()
            return "set_state_failure", time.monotonic() - t0

        exit_reason = "unknown"
        last_size_check_t = time.monotonic()
        last_size = 0
        stall_polls = 0
        try:
            while not self._stop_event.is_set():
                msg = bus.timed_pop_filtered(
                    0, Gst.MessageType.ERROR | Gst.MessageType.EOS
                    | Gst.MessageType.WARNING,
                )
                if msg is not None:
                    if msg.type == Gst.MessageType.ERROR:
                        err, dbg = msg.parse_error()
                        self.gst_errors += 1
                        logger.warning("gst[part=%d] error: %s (%s)",
                                       self._current_part, err.message, dbg)
                        log_event("gst_error", f"{err.message} | {dbg}")
                        exit_reason = f"error:{err.message}"
                        # A full disk otherwise looks like any other sink
                        # error, and the watchdog would rebuild the pipeline
                        # against the same full drive forever. Flag it so the
                        # health watcher moves us to local storage instead.
                        if _gst_error_is_no_space(err):
                            _raise_usb_space_alarm("gstreamer sink")
                        break
                    if msg.type == Gst.MessageType.EOS:
                        logger.info("gst[part=%d] unexpected EOS", self._current_part)
                        log_event("unexpected_eos", f"part={self._current_part}")
                        exit_reason = "unexpected_eos"
                        break
                    if msg.type == Gst.MessageType.WARNING:
                        warn, dbg = msg.parse_warning()
                        self.gst_warnings += 1
                        logger.warning("gst[part=%d] warning: %s (%s)",
                                       self._current_part, warn.message, dbg)
                        log_event("gst_warning", f"{warn.message} | {dbg}")

                # File-stall poll (every 5 s).
                now = time.monotonic()
                if now - last_size_check_t >= 5.0:
                    last_size_check_t = now
                    cur_path = self.last_pattern
                    if cur_path and os.path.exists(cur_path):
                        cur_size = os.path.getsize(cur_path)
                        if last_size > 0 and cur_size == last_size:
                            stall_polls += 1
                            self.file_stalls += 1
                            log_event("file_stall",
                                      f"part={self._current_part} no growth for "
                                      f"{stall_polls} polls ({cur_size} bytes)")
                            if stall_polls >= _STALL_POLLS_BEFORE_RESTART:
                                logger.warning(
                                    "RECORD part=%d stall threshold hit; restarting",
                                    self._current_part,
                                )
                                exit_reason = "file_stall"
                                break
                        else:
                            stall_polls = 0
                        last_size = cur_size

                time.sleep(0.1)

            # Clean stop requested by /stop.
            if self._stop_event.is_set() and exit_reason == "unknown":
                logger.info("RECORD part=%d stopping (user); sending EOS",
                            self._current_part)
                exit_reason = self._send_eos_with_timeout(pipeline, bus)
        finally:
            self._teardown_pipeline()

        return exit_reason, time.monotonic() - t0

    def _send_eos_with_timeout(self, pipeline, bus):
        """Send EOS in a worker thread and wait, falling back to NULL.

        On a live ``rtspsrc protocols=udp`` pipeline the
        ``send_event(EOS)`` call has been observed to block for many
        minutes, starving the rest of the process. Run it in a thread
        and bound it with a wall-clock timeout so we always reach the
        ``set_state(NULL)`` in :meth:`_teardown_pipeline`.
        """
        done = threading.Event()

        def _send():
            try:
                pipeline.send_event(Gst.Event.new_eos())
            except Exception:
                logger.exception("RECORD send_event(EOS) raised")
            finally:
                done.set()

        threading.Thread(target=_send, daemon=True).start()
        if not done.wait(timeout=_STOP_EOS_TIMEOUT_S):
            logger.warning(
                "RECORD part=%d send_event(EOS) blocked > %.1fs; forcing NULL",
                self._current_part, _STOP_EOS_TIMEOUT_S,
            )
            return "stopped_forced_send"

        deadline = time.monotonic() + _STOP_EOS_TIMEOUT_S
        while time.monotonic() < deadline:
            msg = bus.timed_pop_filtered(
                0, Gst.MessageType.ERROR | Gst.MessageType.EOS,
            )
            if msg is not None and msg.type in (
                Gst.MessageType.EOS, Gst.MessageType.ERROR,
            ):
                return "stopped_clean"
            time.sleep(0.05)
        logger.warning("RECORD part=%d EOS not observed within %.1fs",
                       self._current_part, _STOP_EOS_TIMEOUT_S)
        return "stopped_forced"

    def _teardown_pipeline(self):
        """NULL-state the pipeline in a worker thread with a timeout.

        ``set_state(NULL)`` can block on a misbehaving rtspsrc; bound
        it so the watchdog can always make forward progress.
        """
        with self._lock:
            pipeline = self._pipeline
            self._pipeline = None
            self._muxsink = None
        if pipeline is None:
            return
        done = threading.Event()

        def _null():
            try:
                pipeline.set_state(Gst.State.NULL)
            except Exception:
                logger.exception("RECORD set_state(NULL) raised")
            finally:
                done.set()

        threading.Thread(target=_null, daemon=True).start()
        if not done.wait(timeout=10.0):
            logger.error(
                "RECORD set_state(NULL) blocked > 10s; abandoning pipeline reference",
            )

    # ------------------------------------------------------------------
    # Watchdog loop
    # ------------------------------------------------------------------
    def _watchdog(self):
        backoff = _RESTART_BACKOFF_S
        while not self._stop_event.is_set():
            try:
                exit_reason, runtime_s = self._run_one()
            except Exception as e:
                logger.exception("RECORD pipeline run raised")
                exit_reason, runtime_s = f"exception:{e}", 0.0
            self.last_exit = {
                "part": self._current_part,
                "reason": exit_reason,
                "runtime_s": round(runtime_s, 2),
                "pattern": self.last_pattern,
            }
            if self._stop_event.is_set():
                logger.info(
                    "RECORD pipeline part=%d exited reason=%s runtime=%.1fs (user stop)",
                    self._current_part, exit_reason, runtime_s,
                )
                log_event("recording_stopped",
                          f"part={self._current_part} reason={exit_reason}")
                break
            logger.warning(
                "RECORD pipeline part=%d exited reason=%s runtime=%.1fs - restarting",
                self._current_part, exit_reason, runtime_s,
            )
            log_event("watchdog_restart",
                      f"part={self._current_part} reason={exit_reason} "
                      f"runtime_s={runtime_s:.1f}")
            self.restart_count += 1
            if runtime_s >= _MIN_GOOD_RUNTIME_S:
                backoff = _RESTART_BACKOFF_S
            else:
                backoff = min(_MAX_BACKOFF_S, backoff * 1.5)
            if self._stop_event.wait(timeout=backoff):
                break
            self._current_part += 1
        logger.info("RECORD watchdog exiting; total parts=%d, restarts=%d",
                    self._current_part + 1, self.restart_count)

# ---------------------------------------------------------------------------
# Timelapse mode: 2 Hz HTTP snapshot loop.
#
# Uses the camera's built-in snapshot CGI (default
# http://192.168.2.10/cgi-bin/onesnap.cgi -- same path the doris
# tony-video branch hits) so we don't have to share the camera's
# single RTSP session with the recorder. Mode is mutually exclusive
# with video; the operator picks one mode at /start time.
# ---------------------------------------------------------------------------

_TIMELAPSE_PERIOD_S = 0.5  # 2 Hz
_TIMELAPSE_HTTP_TIMEOUT_S = 1.5

# ``towfish_altitude_m`` is appended at the end rather than slotted in
# next to ``altitude_m`` so readers that index this CSV positionally keep
# working against surveys recorded before it existed. It holds the
# towfish AHRS2 altitude -- the value written to EXIF GPSAltitude --
# whereas ``altitude_m`` is the tow vehicle's GPS altitude above MSL.
_TIMELAPSE_CSV_HEADER = [
    'timestamp', 'seq', 'filename', 'source_tag', 'wp', 'frame', 'size_bytes',
    'lat', 'lon', 'altitude_m', 'towfish_heading_deg',
    'towfish_roll_deg', 'towfish_pitch_deg',
    'depth_m', 'temperature_c', 'camera_tilt_deg',
    'snap_ms', 'telem_ms', 'sync_skew_ms', 'towfish_altitude_m',
]


def _survey_day_subfolder():
    """One folder per survey day: ``survey_YYYYMMDD`` (local date).

    All timelapse/transect image captures for the day land here, so the
    photogrammetry workflow gets a single folder of uniquely-named,
    chronologically-sortable JPEGs instead of nested per-leg subfolders.
    """
    return f"survey_{datetime.now():%Y%m%d}"


def _make_source_tag(prefix):
    """Short per-session tag: ``tr``/``tl`` + ``HHMMSS`` of session start.

    Embedded in every filename so frames from different capture runs on
    the same day never collide and stay attributable to their session.
    """
    return f"{prefix}{datetime.now():%H%M%S}"


class TimelapseSession:
    """Background thread that GETs JPEGs from the camera's snap CGI.

    Single-folder survey layout (see ``SURVEY_IMAGE_NAMING.md``): every
    frame lands directly in ``out_dir`` (the ``survey_YYYYMMDD`` day
    folder) with a globally-unique, chronologically-sortable name:

      * Transect (``per_leg=True``):
        ``{seq:06d}_{source_tag}_{wp}_{frame:05d}.jpg``
      * Manual timelapse (``per_leg=False``):
        ``{seq:06d}_{source_tag}_{frame:05d}.jpg``

    ``seq`` is a global counter across the whole survey day (continued
    from any images already in the folder, so multiple sessions and
    extension restarts keep climbing). ``frame`` is per-waypoint (or
    per-session) and resets on each :meth:`set_leg`. One shared
    ``telemetry.csv`` in the day folder gets a row per frame, keyed by
    the final filename. For ``per_leg`` the loop captures nothing until
    the monitor calls :meth:`set_leg`.
    """

    @staticmethod
    def _scan_max_seq(folder):
        """Highest existing ``NNNNNN_`` filename prefix in ``folder`` (0 if none).

        Lets a new session continue the day's global sequence rather than
        clobbering earlier captures, and survives extension restarts.
        """
        mx = 0
        try:
            for name in os.listdir(folder):
                if (name.endswith('.jpg') and len(name) >= 6
                        and name[:6].isdigit()):
                    mx = max(mx, int(name[:6]))
        except FileNotFoundError:
            pass
        except Exception:
            logger.exception("TIMELAPSE seq scan failed: %s", folder)
        return mx

    def __init__(self, snap_url, out_dir, per_leg=False, source_tag=None):
        self._snap_url = snap_url
        self._survey_dir = out_dir
        self._csv_path = os.path.join(out_dir, "telemetry.csv")
        self._per_leg = per_leg
        # Fallback tag if a caller forgets one, so filenames stay valid.
        self.source_tag = source_tag or _make_source_tag("tl")
        self._stop_event = threading.Event()
        self._thread = None
        self.start_time = None
        self.snap_count = 0          # frames this session
        self.miss_count = 0
        self.bytes_written = 0       # cumulative JPEG bytes this session
        self.last_snap_size_bytes = 0
        self.last_snap_path = None
        self.last_snap_at = None
        # Overwritten by whoever resolved the recording directory. Defaulted
        # here so the capture loop can always test it: it is read on the
        # ENOSPC path, which is exactly when we can least afford an
        # AttributeError from a caller that forgot to set it.
        self.on_usb = False
        # Wall-clock gap (ms) between the most recent snap finishing
        # and its parallel telemetry fetch finishing. Surfaces the
        # tightness of snap/GPS sync on /status so a field operator
        # can spot mavlink2rest going slow without scraping the CSV.
        self.last_sync_skew_ms = -1.0

        self._leg_lock = threading.Lock()
        # Global day sequence (seeded from the folder in start()), the
        # current waypoint label, and the per-waypoint frame counter.
        self._global_seq = 0
        # Manual timelapse has no waypoint and is ready immediately;
        # transect waits for the first set_leg() before capturing.
        self._wp_label = None
        self._frame = 0
        self.leg_count = 0

    def _write_csv_header(self, csv_path):
        try:
            with open(csv_path, 'w', newline='') as f:
                csv.writer(f).writerow(_TIMELAPSE_CSV_HEADER)
        except Exception:
            logger.exception("TIMELAPSE failed to write CSV header: %s", csv_path)

    def start(self):
        if self._thread is not None:
            raise RuntimeError("TimelapseSession already started")
        os.makedirs(self._survey_dir, exist_ok=True)
        # Continue the day's global sequence from whatever's already on
        # disk in the survey folder.
        self._global_seq = self._scan_max_seq(self._survey_dir)
        # One shared telemetry.csv for the whole day: write the header
        # only when creating the file, then append across sessions.
        if not os.path.exists(self._csv_path):
            self._write_csv_header(self._csv_path)
        self.start_time = datetime.now()
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._loop, name="timelapse-loop", daemon=True,
        )
        self._thread.start()

    def set_leg(self, label):
        """Begin a new waypoint leg (per_leg only) within the same folder.

        Only flips the current waypoint label and resets the per-waypoint
        ``frame`` counter -- no new subfolder, no new CSV. The global day
        sequence keeps climbing. Safe to call from the monitor thread
        while the capture loop runs.
        """
        if not self._per_leg:
            return None
        with self._leg_lock:
            self._wp_label = label
            self._frame = 0
            self.leg_count += 1
        return self._survey_dir

    def is_alive(self):
        return self._thread is not None and self._thread.is_alive()

    def stop(self, timeout_s=5.0):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)

    def _fetch_jpeg(self):
        try:
            resp = requests.get(self._snap_url, timeout=_TIMELAPSE_HTTP_TIMEOUT_S)
        except Exception as e:
            return None, f"http_error: {e}"
        if resp.status_code != 200:
            return None, f"http_status_{resp.status_code}"
        ctype = resp.headers.get("content-type", "")
        if not ctype.startswith("image"):
            return None, f"unexpected_content_type: {ctype!r}"
        body = resp.content
        if not body or not body.startswith(b"\xff\xd8"):
            return None, "not_jpeg_magic"
        return body, "ok"

    def _fetch_telemetry_block(self):
        """Read the full telemetry block in one shot.

        Called from a worker thread that runs in parallel with
        ``_fetch_jpeg`` so the GPS / heading / altitude samples share the
        same wall-clock window as the camera shutter (the snap CGI takes
        ~200 ms, each mavlink2rest GET ~10-300 ms; in parallel they
        overlap). The sampling itself lives at module scope so the video
        sidecar writer records an identical field set.
        """
        return fetch_telemetry_block()

    def _loop(self):
        next_fire = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()
            if now < next_fire:
                if self._stop_event.wait(timeout=next_fire - now):
                    break
            next_fire = max(next_fire + _TIMELAPSE_PERIOD_S, time.monotonic())

            # Per-leg mode: until the monitor sets the first waypoint
            # there's nothing to tag frames with, so idle without burning
            # the snap budget.
            with self._leg_lock:
                leg_ready = self._wp_label is not None
            if self._per_leg and not leg_ready:
                continue

            # ---- Synchronised capture ------------------------------
            # Stamp wall-clock ONCE, then fire JPEG + telemetry in
            # parallel so the GPS sample is taken in the same wall-
            # clock window as the camera shutter. Without this the
            # mavlink2rest GETs add 200-500 ms of skew on top of the
            # camera's own ~200 ms snap latency, which at typical tow
            # speeds (1-2 m/s) is already 0.2-1 m of position error.
            ts_local = datetime.now()
            ts_utc = datetime.now(tz=timezone.utc)
            t_start = time.monotonic()

            telem_holder = {'data': None, 'done_at': None}

            def _telem_worker():
                telem_holder['data'] = self._fetch_telemetry_block()
                telem_holder['done_at'] = time.monotonic()

            telem_thread = threading.Thread(
                target=_telem_worker, name="tl-telem", daemon=True,
            )
            telem_thread.start()

            jpeg, reason = self._fetch_jpeg()
            snap_done_at = time.monotonic()

            # Allow the telemetry fetch up to one full snap period
            # past the JPEG response before we give up. In the typical
            # case both finish within ~300 ms of each other so this
            # join returns immediately.
            telem_thread.join(timeout=_TIMELAPSE_PERIOD_S)
            telemetry = telem_holder['data']
            telem_done_at = telem_holder['done_at']

            snap_ms = round((snap_done_at - t_start) * 1000, 1)
            telem_ms = (telemetry or {}).get('fetch_ms', -1.0)
            sync_skew_ms = (
                round(abs(snap_done_at - telem_done_at) * 1000, 1)
                if telem_done_at is not None else -1.0
            )

            if jpeg is None:
                self.miss_count += 1
                logger.warning(
                    "TIMELAPSE snap miss (%s) snap_ms=%.1f telem_ms=%.1f",
                    reason, snap_ms, telem_ms,
                )
                continue

            if telemetry is None:
                logger.warning(
                    "TIMELAPSE telemetry timed out (snap_ms=%.1f, "
                    "limit=%.0f ms); EXIF will lack GPS this frame",
                    snap_ms, _TIMELAPSE_PERIOD_S * 1000,
                )
                bb_lat = bb_lon = bb_alt = None
                gps_lat = gps_lon = None
                heading = None
                roll = None
                pitch = None
                tow_alt = None
                depth = None
                temp = None
                tilt = None
            else:
                bb_lat = telemetry['bb_lat']
                bb_lon = telemetry['bb_lon']
                bb_alt = telemetry['bb_alt']
                gps_lat = telemetry['gps_lat']
                gps_lon = telemetry['gps_lon']
                heading = telemetry['heading']
                roll = telemetry.get('roll')
                pitch = telemetry.get('pitch')
                tow_alt = telemetry['tow_alt']
                depth = telemetry['depth']
                temp = telemetry['temp']
                tilt = telemetry['tilt']

            # Embed GPS+heading+tilt+timestamp into EXIF and the camera
            # orientation into Pix4D-namespace XMP before writing, so the
            # on-disk JPEG is self-describing (geotagged in any standard
            # map / photo viewer, and orientable by Metashape/Pix4D/WebODM).
            # Shared with extract_geotagged_frames.py so video-derived
            # frames carry an identical tag set. Failure is non-fatal:
            # the raw JPEG and the CSV row still get written.
            jpeg = photogrammetry_meta.embed_metadata(
                jpeg, gps_lat, gps_lon, tow_alt, heading, ts_local, ts_utc,
                tilt_deg=tilt, depth_m=depth, temp_c=temp,
                roll_deg=roll, pitch_deg=pitch,
            )

            # Claim the global day sequence + per-waypoint frame number
            # atomically so a concurrent set_leg() can't reuse a frame
            # index or straddle a leg boundary.
            with self._leg_lock:
                wp = self._wp_label
                if self._per_leg and wp is None:
                    self.miss_count += 1
                    continue
                self._global_seq += 1
                seq = self._global_seq
                self._frame += 1
                frame = self._frame
            self.snap_count += 1
            if wp:
                filename = f"{seq:06d}_{self.source_tag}_{wp}_{frame:05d}.jpg"
            else:
                filename = f"{seq:06d}_{self.source_tag}_{frame:05d}.jpg"
            path = os.path.join(self._survey_dir, filename)
            csv_path = self._csv_path
            try:
                with open(path, 'wb') as f:
                    f.write(jpeg)
            except OSError as e:
                # A frame that ran out of room part-way through leaves a
                # truncated JPEG behind. Drop it rather than leave a corrupt
                # image in the survey folder for the photogrammetry run to
                # trip over; the CSV row is skipped below with it.
                if e.errno == errno.ENOSPC:
                    logger.error("TIMELAPSE out of space writing %s", path)
                    try:
                        os.unlink(path)
                    except OSError:
                        pass
                    if self.on_usb:
                        _raise_usb_space_alarm("timelapse frame")
                else:
                    logger.exception("TIMELAPSE write failed: %s", path)
                self.miss_count += 1
                continue
            except Exception:
                logger.exception("TIMELAPSE write failed: %s", path)
                self.miss_count += 1
                continue
            self.bytes_written += len(jpeg)
            self.last_snap_path = path
            self.last_snap_size_bytes = len(jpeg)
            self.last_snap_at = ts_local
            self.last_sync_skew_ms = sync_skew_ms

            # CSV sidecar keeps its historical schema and adds three
            # timing columns at the end so you can audit per-frame
            # snap-vs-telemetry sync after the fact:
            #   altitude_m    = BlueBoat GPS altitude above MSL (bb_alt)
            #   depth_m       = towfish depth (positive)
            #   snap_ms       = JPEG HTTP round-trip
            #   telem_ms      = parallel mavlink2rest round-trip
            #   sync_skew_ms  = wall-clock gap between snap done /
            #                   telemetry done (small = tight sync)
            try:
                with open(csv_path, 'a', newline='') as f:
                    w = csv.writer(f)
                    w.writerow([
                        ts_local.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                        seq, filename, self.source_tag, wp or "", frame,
                        len(jpeg),
                        f"{gps_lat:.6f}" if gps_lat is not None else "",
                        f"{gps_lon:.6f}" if gps_lon is not None else "",
                        f"{bb_alt:.2f}" if bb_alt is not None else "",
                        f"{heading:.1f}" if heading is not None else "",
                        f"{roll:.1f}" if roll is not None else "",
                        f"{pitch:.1f}" if pitch is not None else "",
                        f"{depth:.2f}" if depth is not None else "",
                        f"{temp:.2f}" if temp is not None else "",
                        f"{tilt:.1f}" if tilt is not None else "",
                        f"{snap_ms:.1f}",
                        f"{telem_ms:.1f}" if telem_ms >= 0 else "",
                        f"{sync_skew_ms:.1f}" if sync_skew_ms >= 0 else "",
                        f"{tow_alt:.2f}" if tow_alt is not None else "",
                    ])
            except Exception:
                logger.exception("TIMELAPSE CSV write failed")
        logger.info("TIMELAPSE loop exiting (snaps=%d, misses=%d)",
                    self.snap_count, self.miss_count)

# ---------------------------------------------------------------------------
# Flask routes
# ---------------------------------------------------------------------------

@app.route('/')
def index():
    return app.send_static_file('index.html')

@app.route('/register_service')
def register_service():
    return '''
    {
        "name": "SubReels: TowFish",
        "description": "Towed-body video survey: RTSP recording, geotagged 2 Hz stills, and mission-triggered transect capture",
        "icon": "mdi-video",
        "company": "Blue Robotics",
        "version": "1.0.0",
        "webpage": "https://github.com/vshie/SubReels_TowFish",
        "api": "https://github.com/bluerobotics/BlueOS-docker"
    }
    '''

@app.route('/config', methods=['GET', 'POST'])
def config():
    global tow_vehicle_ip, container_format, stream_protocol, snapshot_url
    global transect_capture_type, storage_preference, awb_loop_enabled
    global tow_offset_m, tow_heading_source

    if request.method == 'POST':
        # The AWB-loop toggle is safe to change while a mode is active
        # (it only affects a 2-min background HTTP timer). Everything
        # else still needs an idle/disabled monitor.
        data = request.get_json(silent=True) or {}
        touches_only_awb = (
            set(data.keys()) <= {"awb_loop_enabled"} and "awb_loop_enabled" in data
        )
        if not touches_only_awb:
            if mode != MODE_IDLE:
                return jsonify({"success": False,
                                "message": f"Cannot change config while {mode} active"}), 400
            # The transect monitor owns the recorder even while sitting in
            # its "waiting" state, so block capture-type changes then too.
            if _transect_monitor is not None and _transect_monitor.state != "disabled":
                return jsonify({"success": False,
                                "message": "Cannot change config while transect monitor is enabled"}), 400

        changed = False

        new_ip = data.get('tow_vehicle_ip', '').strip()
        if new_ip:
            tow_vehicle_ip = new_ip
            changed = True

        new_capture = data.get('transect_capture_type')
        if new_capture is not None:
            new_capture = str(new_capture).strip().lower()
            if new_capture not in VALID_TRANSECT_CAPTURE_TYPES:
                return jsonify({"success": False,
                                "message": f"transect_capture_type must be one of {VALID_TRANSECT_CAPTURE_TYPES}"}), 400
            transect_capture_type = new_capture
            changed = True

        new_awb = data.get('awb_loop_enabled')
        if new_awb is not None:
            new_awb_bool = bool(new_awb) if isinstance(new_awb, bool) else str(new_awb).lower() in ("1", "true", "yes", "on")
            if new_awb_bool != awb_loop_enabled:
                awb_loop_enabled = new_awb_bool
                changed = True
                _apply_awb_loop_state_change()

        new_fmt = data.get('container_format', '').strip().lower()
        if new_fmt:
            if new_fmt not in VALID_CONTAINER_FORMATS:
                return jsonify({"success": False,
                                "message": f"container_format must be one of {VALID_CONTAINER_FORMATS}"}), 400
            container_format = new_fmt
            changed = True

        new_proto = data.get('stream_protocol', '').strip().lower()
        if new_proto:
            if new_proto not in VALID_STREAM_PROTOCOLS:
                return jsonify({"success": False,
                                "message": f"stream_protocol must be one of {VALID_STREAM_PROTOCOLS}"}), 400
            stream_protocol = new_proto
            changed = True

        new_snap = data.get('snapshot_url')
        if new_snap is not None:
            new_snap = str(new_snap).strip()
            if not new_snap.startswith(('http://', 'https://')):
                return jsonify({"success": False,
                                "message": "snapshot_url must start with http:// or https://"}), 400
            snapshot_url = new_snap
            changed = True

        new_storage = data.get('storage_preference')
        if new_storage is not None:
            new_storage = str(new_storage).strip().lower()
            if new_storage not in VALID_STORAGE_PREFERENCES:
                return jsonify({"success": False,
                                "message": f"storage_preference must be one of {VALID_STORAGE_PREFERENCES}"}), 400
            storage_preference = new_storage
            changed = True

        new_offset = data.get('tow_offset_m')
        if new_offset is not None:
            try:
                offset_val = float(new_offset)
            except (TypeError, ValueError):
                return jsonify({"success": False,
                                "message": "tow_offset_m must be a number"}), 400
            if not (TOW_OFFSET_MIN_M <= offset_val <= TOW_OFFSET_MAX_M):
                return jsonify({"success": False,
                                "message": f"tow_offset_m must be between {TOW_OFFSET_MIN_M} and {TOW_OFFSET_MAX_M} m"}), 400
            tow_offset_m = _sanitize_tow_offset_m(offset_val)
            changed = True

        new_heading_src = data.get('tow_heading_source')
        if new_heading_src is not None:
            new_heading_src = str(new_heading_src).strip().lower()
            if new_heading_src not in VALID_TOW_HEADING_SOURCES:
                return jsonify({"success": False,
                                "message": f"tow_heading_source must be one of {VALID_TOW_HEADING_SOURCES}"}), 400
            tow_heading_source = new_heading_src
            changed = True

        if not changed:
            return jsonify({"success": False, "message": "No valid fields provided"}), 400

        _persist_config()
        logger.info(
            "Config updated: tow_vehicle_ip=%s, container_format=%s, "
            "stream_protocol=%s, snapshot_url=%s, transect_capture_type=%s, "
            "storage_preference=%s, awb_loop_enabled=%s, tow_offset_m=%s, "
            "tow_heading_source=%s",
            tow_vehicle_ip, container_format, stream_protocol, snapshot_url,
            transect_capture_type, storage_preference, awb_loop_enabled,
            tow_offset_m, tow_heading_source,
        )
        return jsonify({"success": True,
                        "tow_vehicle_ip": tow_vehicle_ip,
                        "container_format": container_format,
                        "stream_protocol": stream_protocol,
                        "snapshot_url": snapshot_url,
                        "transect_capture_type": transect_capture_type,
                        "storage_preference": storage_preference,
                        "awb_loop_enabled": awb_loop_enabled,
                        "tow_offset_m": tow_offset_m,
                        "tow_heading_source": tow_heading_source})

    resp = jsonify({
        "rtsp_endpoint": RTSP_ENDPOINT,
        "tow_vehicle_ip": tow_vehicle_ip,
        "container_format": container_format,
        "stream_protocol": stream_protocol,
        "snapshot_url": snapshot_url,
        "transect_capture_type": transect_capture_type,
        "storage_preference": storage_preference,
        "awb_loop_enabled": awb_loop_enabled,
        "tow_offset_m": tow_offset_m,
        "tow_heading_source": tow_heading_source,
        "tow_heading_sources": list(VALID_TOW_HEADING_SOURCES),
        "tow_offset_min_m": TOW_OFFSET_MIN_M,
        "tow_offset_max_m": TOW_OFFSET_MAX_M,
    })
    resp.headers['Cache-Control'] = 'no-store'
    return resp

# ── Shared video-session lifecycle helpers ───────────────────────────────
# Used by both the manual ``/start`` & ``/stop`` Flask routes *and* the
# automatic ``TransectMonitor``. Both helpers assume the caller already
# holds ``_mode_lock`` so the transect monitor and HTTP routes can never
# race each other into a half-open state.
def _start_video_session(base_prefix, target_mode, initial_label=None,
                         per_leg_sidecars=False, force_local=False):
    """Stand up a new RecordingSession + sidecars; flip ``mode`` to ``target_mode``.

    Returns the anchor path of the (yet-to-exist) first part. Raises on
    failure, after cleaning up any partial state the caller would
    otherwise have to undo. ``base_prefix`` is the filename root (e.g.
    ``video_rtsp`` for manual recording or ``transect_<TS>`` for the
    monitor); ``target_mode`` is the mode the caller wants to be in once
    the session is live (``MODE_VIDEO`` or ``MODE_TRANSECT``).

    ``initial_label`` (optional) is the first leg label the transect
    monitor wants stamped onto the very first .ts file -- e.g. ``wp01``.
    Without it the historical "no label" naming is preserved for the
    manual recording path.

    ``per_leg_sidecars`` (transect only): when True each leg .ts gets its
    own matching .srt/.ass/_telemetry.csv, rotated automatically when
    splitmuxsink opens each new fragment.

    ``force_local`` is set by the failover path so a session that just
    lost the USB drive doesn't immediately re-pick it on restart.
    """
    global recording, start_time, sidecar_epoch
    global srt_thread, stop_srt_thread, current_srt_file_rtsp, current_video_file_rtsp, srt_subtitle_counter
    global current_video_csv_file, current_video_csv_wp
    global isp_log_thread, stop_isp_log_thread, current_isp_log_file
    global current_events_file
    global ass_thread, stop_ass_thread, current_ass_file, ass_subtitle_counter
    global _session, usb_recording

    out_dir, on_usb = _resolve_recording_dir(force_local=force_local)
    os.makedirs(out_dir, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    ext = ".ts" if container_format == "mpegts" else ".mp4"

    # FAT32 (vfat) caps single files at 4 GiB. When we land on a vfat
    # USB stick, ask splitmuxsink to roll a new on-disk part well below
    # that limit so a long manual recording can never trip the cap.
    fat_cap = on_usb and usb_storage.is_fat_like()

    base_filename = f"{base_prefix}_{timestamp}"
    label_suffix = f"_{initial_label}" if initial_label else ""
    # Sidecars are named off the *first* part path so the existing
    # post-processing helpers (adjust_srt_timing, etc.) keep their
    # historical pair-by-name relationship.
    anchor_path = os.path.join(out_dir,
                               f"{base_filename}_part00_00000{label_suffix}{ext}")
    current_video_file_rtsp = anchor_path

    current_srt_file_rtsp = create_srt_file(anchor_path)
    srt_subtitle_counter = 0
    current_video_csv_file, current_video_csv_wp = create_video_telemetry_csv(anchor_path)
    current_isp_log_file = create_isp_log_file(anchor_path)
    current_ass_file = create_ass_file(anchor_path)
    ass_subtitle_counter = 0
    current_events_file = create_events_file(anchor_path)

    log_event("recording_starting",
              f"container={container_format} proto={stream_protocol} "
              f"mode={target_mode} prefix={base_prefix} "
              f"storage={'usb' if on_usb else 'local'}"
              f"{' (fat_cap=3.5GiB)' if fat_cap else ''}")

    session = RecordingSession(
        rtsp_url=RTSP_ENDPOINT,
        out_dir=out_dir,
        base_filename=base_filename,
        ext=ext,
        container_fmt=container_format,
        proto=stream_protocol,
        per_leg_sidecars=per_leg_sidecars,
        fat_size_cap=fat_cap,
    )
    session.on_usb = on_usb
    # Pre-seed the first leg's label so the very first ``_on_format_location``
    # callback (which happens before the monitor sees its first state
    # transition) already names the file with the WP index.
    if initial_label:
        session._next_label = initial_label
    try:
        session.start()
    except Exception:
        logger.exception("Failed to start RecordingSession")
        _set_mode(MODE_IDLE)
        _session = None
        current_srt_file_rtsp = None
        current_video_file_rtsp = None
        current_video_csv_file = None
        current_video_csv_wp = None
        current_isp_log_file = None
        current_ass_file = None
        current_events_file = None
        raise

    _session = session
    usb_recording = on_usb
    _set_mode(target_mode)
    recording = True
    start_time = datetime.now()
    # Subtitle epoch starts with the session; transect leg rotations
    # reset it per leg via _rotate_leg_sidecars.
    sidecar_epoch = start_time

    stop_srt_thread = False
    srt_thread = threading.Thread(target=update_geo_sidecars, daemon=True)
    srt_thread.start()

    stop_isp_log_thread = False
    isp_log_thread = threading.Thread(target=update_isp_log, daemon=True)
    isp_log_thread.start()

    stop_ass_thread = False
    ass_thread = threading.Thread(target=update_ass_file, daemon=True)
    ass_thread.start()

    storage_label = "USB" if on_usb else "local"
    log_event("recording_started",
              f"anchor={anchor_path} storage={storage_label}")
    logger.info("Recording started: anchor=%s storage=%s",
                anchor_path, storage_label)
    return anchor_path

def _stop_video_session():
    """Tear down the active RecordingSession + sidecars; flip ``mode`` to IDLE.

    Drives the same post-processing the legacy ``/stop`` route did
    (sidecar joins, splitmuxsink finalise wait, SRT/ASS timing rescale
    over the *sum* of part durations so the timeline still matches the
    on-disk video even if the watchdog rebuilt the pipeline mid-session).
    Always leaves the recorder in MODE_IDLE on exit, even when the
    post-processing step itself raises -- so a hung helper can never
    wedge the mode state.
    """
    global recording, start_time
    global srt_thread, stop_srt_thread, current_srt_file_rtsp, current_video_file_rtsp
    global current_video_csv_file, current_video_csv_wp
    global isp_log_thread, stop_isp_log_thread, current_isp_log_file
    global current_events_file
    global ass_thread, stop_ass_thread, current_ass_file
    global _session, usb_recording

    log_event("recording_stopping", "Stop requested")

    video_anchor = current_video_file_rtsp
    srt_path = current_srt_file_rtsp
    ass_path = current_ass_file
    telemetry_csv_path = current_video_csv_file
    isp_log_path = current_isp_log_file
    events_path = current_events_file

    stop_srt_thread = True
    if srt_thread:
        srt_thread.join(timeout=2)
    stop_isp_log_thread = True
    if isp_log_thread:
        isp_log_thread.join(timeout=2)
    stop_ass_thread = True
    if ass_thread:
        ass_thread.join(timeout=2)

    session = _session
    _session = None
    per_leg = getattr(session, "per_leg_sidecars", False) if session else False
    last_leg_ts = None
    if session is not None:
        session.stop()
        # Prefer the actual first part path the splitmuxsink callback
        # observed; falls back to the anchor we composed at /start.
        if session.first_part_path:
            video_anchor = session.first_part_path
        last_leg_ts = session.last_pattern

    recording = False
    usb_recording = False
    start_time = None
    current_srt_file_rtsp = None
    current_video_file_rtsp = None
    current_video_csv_file = None
    current_video_csv_wp = None
    current_isp_log_file = None
    current_ass_file = None
    current_events_file = None
    _set_mode(MODE_IDLE)

    if isp_log_path and os.path.exists(isp_log_path):
        logger.info("ISP log file saved: %s", isp_log_path)
    if ass_path and os.path.exists(ass_path):
        logger.info("ASS telemetry file saved: %s", ass_path)
    if telemetry_csv_path and os.path.exists(telemetry_csv_path):
        logger.info("Telemetry CSV saved: %s", telemetry_csv_path)
    if events_path and os.path.exists(events_path):
        logger.info("Events log saved: %s", events_path)

    if per_leg:
        # Transect: earlier legs were already finalised on rotation; only
        # the *final* leg's sidecars remain, rescaled to that one leg's
        # encoded duration (NOT the session sum). Done in the background
        # so /transect/disable and the monitor's exit return promptly.
        if (srt_path or ass_path or telemetry_csv_path) and last_leg_ts:
            threading.Thread(
                target=_finalize_leg_sidecars,
                args=(srt_path, ass_path, telemetry_csv_path, last_leg_ts),
                name="leg-sidecar-finalise", daemon=True,
            ).start()
    elif video_anchor and srt_path and os.path.exists(srt_path):
        logger.info("Starting SRT timing adjustment post-processing...")
        time.sleep(3)  # let splitmuxsink finalise the trailing part
        video_duration = sum_session_video_duration(video_anchor)
        if video_duration:
            parts = list_session_parts(video_anchor)
            logger.info(
                "Session video duration: %.2fs across %d part(s)",
                video_duration, len(parts),
            )
            adjust_srt_timing(srt_path, video_duration)
            if ass_path and os.path.exists(ass_path):
                adjust_ass_timing(ass_path, video_duration)
            if telemetry_csv_path and os.path.exists(telemetry_csv_path):
                adjust_video_csv_timing(telemetry_csv_path, video_duration)
        else:
            logger.warning(
                "Could not determine session video duration, "
                "subtitle timing not adjusted",
            )

    logger.info("Recording stopped successfully")

def _start_transect_timelapse(initial_label, force_local=False):
    """Stand up a per-leg TimelapseSession; flip ``mode`` to MODE_TRANSECT.

    Mirror of :func:`_start_video_session` for the image capture type.
    Returns the survey-day folder (``survey_YYYYMMDD/``), which doubles
    as the manifest's parent. Caller must hold ``_mode_lock``.

    ``force_local`` is set by the failover path so a session that just
    lost the USB drive doesn't immediately re-pick it on restart.
    """
    global _timelapse, usb_recording
    subfolder = _survey_day_subfolder()
    source_tag = _make_source_tag("tr")
    out_dir, on_usb = _resolve_recording_dir(
        subfolder=subfolder, force_local=force_local,
    )

    session = TimelapseSession(snap_url=snapshot_url, out_dir=out_dir,
                               per_leg=True, source_tag=source_tag)
    session.on_usb = on_usb
    try:
        session.start()
        session.set_leg(initial_label)
    except Exception:
        logger.exception("Failed to start transect TimelapseSession")
        _set_mode(MODE_IDLE)
        _timelapse = None
        raise

    _timelapse = session
    usb_recording = on_usb
    _set_mode(MODE_TRANSECT)
    storage_label = "USB" if on_usb else "local"
    log_event("transect_timelapse_started",
              f"{out_dir} storage={storage_label}")
    logger.info("Transect timelapse started: %s (snap_url=%s, storage=%s)",
                out_dir, snapshot_url, storage_label)
    return out_dir

def _stop_transect_timelapse():
    """Tear down the transect TimelapseSession; flip ``mode`` to IDLE."""
    global _timelapse, usb_recording
    session = _timelapse
    _timelapse = None
    if session is not None:
        session.stop()
    usb_recording = False
    _set_mode(MODE_IDLE)
    logger.info("Transect timelapse stopped")

# ── Transect monitor ─────────────────────────────────────────────────────
# Background thread that polls the tow vehicle's mavlink2rest for mission
# state and drives the existing RecordingSession lifecycle automatically.
# State machine:
#
#   disabled  --enable--> waiting
#   waiting   --armed+AUTO+navigating--> recording
#   recording --MISSION_CURRENT.seq change (forward)--> recording (split_now)
#   recording --Mission Complete STATUSTEXT / seq jumps backward--> waiting
#   recording --left AUTO / disarmed / stopped, sustained 60s--> waiting
#   *         --disable--> disabled (stops any in-flight session)
#
# IMPORTANT -- record regardless of reachability: once recording, the
# session is NEVER torn down because the tow vehicle became unreachable.
# Only *positive*, freshly-observed evidence ends a session: a
# mission-complete STATUSTEXT, a backward seq jump (new mission loaded),
# or the vehicle being seen (fresh HEARTBEAT/NAV) to have left AUTO /
# disarmed / stopped navigating for a sustained grace window. Missing or
# stale telemetry is treated as "unknown", not "stop".
#
# Per-leg .ts files are produced by emitting splitmuxsink's "split-now"
# action signal with the new WP index stamped as the file label, so the
# RTSP pipeline stays alive across leg boundaries -- only the few
# hundred ms it takes the muxer to rotate at an IDR boundary is "lost".
# ─────────────────────────────────────────────────────────────────────────

# How often the monitor wakes to re-read mission state. 3 Hz gives ~330ms
# worst-case latency between MISSION_CURRENT.seq changing and the file
# rolling, which is much smaller than typical leg durations (>10s).
_TRANSECT_POLL_INTERVAL_S = 0.33

# Once recording, a positive stop condition (left AUTO, disarmed, or
# stopped navigating -- all observed from FRESH telemetry) must persist
# continuously for this long before we finalise the session. This rides
# out GCS-failsafe "CONTINUING AUTO MODE" events, brief mode-flicker, and
# telemetry blips. A mission-complete STATUSTEXT or a backward seq jump
# is definitive and bypasses this grace.
_TRANSECT_STOP_GRACE_S = 60.0

# Some mission items (DO_CHANGE_SPEED, condition commands, NAV_DELAY)
# advance MISSION_CURRENT.seq without changing the navigation target.
# If the prior "leg" was shorter than this, we don't emit a real
# split-now; instead we just relabel the in-flight file so the WP index
# still catches up. This avoids a long string of fractional-second
# .ts files at mission start.
_TRANSECT_MIN_LEG_DURATION_S = 1.0

# Below this groundspeed AND when wp_dist is missing, we treat the
# vehicle as "not navigating" -- avoids starting a session when the
# operator armed in AUTO but the mission hasn't moved yet.
_TRANSECT_NAV_SPEED_THRESHOLD = 0.2  # m/s


def _transect_manifest_path(anchor_path):
    """Sibling path of the session anchor: ``<prefix>_manifest.ndjson``.

    Lives next to the leg .ts files in the same recording directory so
    a single rsync brings the whole session.
    """
    base = os.path.basename(anchor_path)
    # Strip the ``_part00_NNNNN[_wpNN].ts`` suffix off the anchor name to
    # get a stable session prefix.
    stem = base.split("_part00_")[0] if "_part00_" in base else os.path.splitext(base)[0]
    return os.path.join(os.path.dirname(anchor_path), f"{stem}_manifest.ndjson")


def _manifest_write_header(path, anchor):
    """Truncate-and-write a one-row session header into the manifest file."""
    with open(path, "w") as f:
        f.write(json.dumps({
            "ts": time.time(),
            "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            "event": "session_start",
            "anchor": os.path.basename(anchor),
            "tow_vehicle_ip": tow_vehicle_ip,
        }) + "\n")


def _manifest_append(path, row):
    """Append one JSON row (newline-terminated) to the manifest, swallowing IO errors."""
    try:
        with open(path, "a") as f:
            f.write(json.dumps(row) + "\n")
    except Exception:
        logger.exception("Could not append row to transect manifest")


class TransectMonitor:
    """Background poller that drives one RecordingSession per AUTO mission.

    Designed to be cheap when idle: when ``state == "waiting"`` it just
    GETs five mavlink2rest messages at ~3 Hz. Only flips into
    ``"recording"`` once the tow vehicle is armed, in AUTO, and actually
    moving toward a waypoint.

    Thread safety:
      - Mode transitions (start/stop session) are wrapped in
        ``_mode_lock`` so they cannot interleave with the manual
        /start, /stop, /timelapse routes.
      - Reads of the shared ``_session`` global outside the lock are
        only used for diagnostics or to emit ``split-now`` (the muxer
        itself is reference-counted by GStreamer; emit-on-stale is safe).
    """

    def __init__(self):
        self._stop_event = threading.Event()
        self._thread = None

        # Public, surfaced on /status
        self.state = "disabled"  # disabled | waiting | recording
        self.last_event = ""
        self.last_event_at = None
        self.last_state = None  # last get_mission_state() snapshot
        self.current_seq = None
        self.leg_count = 0
        self.session_started_at = None
        # Capture type for the in-flight session, snapshotted from the
        # global config when the session starts ("video" or "timelapse")
        # so a mid-mission config change can't split one session across
        # both capture backends.
        self._capture_type = None

        # Per-leg bookkeeping
        self._leg_started_at = None
        self._leg_started_seq = None
        self._leg_started_position = (None, None, None)
        self._leg_started_heading = None
        self._leg_started_groundspeed = None
        self._leg_started_wp_dist = None
        self._manifest_path = None

        # STATUSTEXT dedup: mavlink2rest holds only the latest msg, so
        # the same payload can be replayed for many seconds. We use the
        # ``last_update`` timestamp on the wrapper as the dedup key.
        self._last_statustext_time = None

        # Monotonic timestamp at which a positive stop condition was first
        # observed continuously; None whenever the latest fresh telemetry
        # shows the vehicle still running the mission. Drives the 60s
        # stop grace.
        self._stop_evidence_since = None

        # Set when a "Mission Complete" STATUSTEXT arrives; consumed by
        # the next _tick to finalise the session. Cleared on enter/exit.
        self._mission_complete_latch = False

    # -- lifecycle ----------------------------------------------------
    def swap_to_local_storage(self):
        """Move the in-flight capture to local storage, staying in "recording".

        Called by the USB failover path (which already holds ``_mode_lock``)
        when the drive is lost or fills up mid-survey. The boat is still
        flying the mission, so the useful response is to reopen the capture
        on the SD card at the current waypoint rather than abandon the leg:
        the recording is interrupted for as long as the restart takes and
        picks up under a new file, but the survey keeps going and the
        manifest follows it.

        Returns True when the capture is running again on local storage.
        A False return means the caller should tear the monitor down.
        """
        if self.state != "recording":
            return False
        capture = self._capture_type
        seq = self.current_seq if self.current_seq is not None else 0
        wp_label = f"wp{int(seq):02d}"

        try:
            if capture == "timelapse":
                _stop_transect_timelapse()
            else:
                _stop_video_session()
        except Exception:
            logger.exception("transect swap: stopping the USB session failed")
            # Keep going regardless: whatever state the old session is in,
            # leaving the monitor wedged on a dead drive is worse.

        try:
            if capture == "timelapse":
                anchor = _start_transect_timelapse(wp_label, force_local=True)
            else:
                anchor = _start_video_session(
                    base_prefix="transect",
                    target_mode=MODE_TRANSECT,
                    initial_label=wp_label,
                    per_leg_sidecars=True,
                    force_local=True,
                )
        except Exception:
            logger.exception("transect swap: restart on local storage failed")
            return False

        # Point the manifest at the new session. The old one stays on the
        # USB drive describing the frames that made it there.
        try:
            if capture == "timelapse":
                tl = _timelapse
                tag = tl.source_tag if tl is not None else _make_source_tag("tr")
                self._manifest_path = os.path.join(anchor, f"{tag}_manifest.ndjson")
            else:
                self._manifest_path = _transect_manifest_path(anchor)
            _manifest_write_header(self._manifest_path, anchor)
        except Exception:
            logger.exception("transect swap: manifest re-open failed")

        self.leg_count += 1
        self.session_started_at = datetime.now()
        self._note_event(f"storage swapped to local at {wp_label}")
        logger.warning("Transect %s capture swapped to local storage at %s",
                       capture, wp_label)
        return True

    def enable(self):
        """Spin up the poll thread and arm the state machine."""
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("TransectMonitor already enabled")
        self._stop_event.clear()
        self.state = "waiting"
        self._note_event("monitor enabled")
        self._thread = threading.Thread(
            target=self._poll, name="transect-monitor", daemon=True,
        )
        self._thread.start()

    def disable(self):
        """Stop the poll thread and tear down any in-flight session."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=8.0)
        self._thread = None
        # If we were mid-recording, close it cleanly. Done under lock to
        # avoid racing the Flask routes.
        with _mode_lock:
            if mode == MODE_TRANSECT:
                try:
                    self._close_leg(self.last_state, "monitor_disabled")
                except Exception:
                    logger.exception("close_leg on disable failed")
                try:
                    if self._capture_type == "timelapse":
                        _stop_transect_timelapse()
                    else:
                        _stop_video_session()
                except Exception:
                    logger.exception("stop capture on disable failed")
        self.state = "disabled"
        self._note_event("monitor disabled")

    # -- poll loop ----------------------------------------------------
    def _poll(self):
        while not self._stop_event.is_set():
            try:
                self._tick()
            except Exception:
                logger.exception("TransectMonitor tick failed")
            # Use Event.wait so disable() can interrupt the sleep
            # immediately instead of waiting up to _POLL_INTERVAL_S.
            self._stop_event.wait(_TRANSECT_POLL_INTERVAL_S)

    def _tick(self):
        state = get_mission_state()
        self.last_state = state

        # STATUSTEXT: only the "new" payload matters (mavlink2rest holds
        # last value across many polls).
        if (state['statustext_time'] is not None
                and state['statustext_time'] != self._last_statustext_time):
            self._last_statustext_time = state['statustext_time']
            self._handle_statustext(state['statustext'])

        # Did we get a fresh HEARTBEAT this tick? (mode_num is None when
        # the vehicle is unreachable / mavlink2rest had no data.) We only
        # ever act on stop conditions when telemetry is fresh.
        hb_present = state['mode_num'] is not None
        is_auto = (state['mode_num'] == ROVER_MODE_AUTO)
        is_armed = bool(state['armed'])

        # "Navigating" = positive evidence of motion toward a waypoint.
        navigating = (
            (state['wp_dist'] is not None and state['wp_dist'] > 0)
            or (state['groundspeed'] is not None
                and state['groundspeed'] > _TRANSECT_NAV_SPEED_THRESHOLD)
        )
        # "Stopped" requires positive evidence too: at least one nav field
        # present AND showing no motion. If both nav fields are missing we
        # treat motion as UNKNOWN (not stopped), so flaky NAV/VFR telemetry
        # can't trigger a false stop.
        nav_known = (state['wp_dist'] is not None
                     or state['groundspeed'] is not None)
        nav_stopped = nav_known and not navigating

        if self.state == "waiting":
            if is_auto and is_armed and navigating:
                self._enter_recording(state)
            return

        if self.state == "recording":
            # 1) Definitive end: mission-complete STATUSTEXT latched.
            if self._mission_complete_latch:
                self._exit_recording(state, "mission_complete")
                return

            # 2) Definitive end: MISSION_CURRENT.seq jumped *backwards*,
            #    i.e. a new mission was loaded/restarted (your BIN log:
            #    WP#12 complete -> next mission starts at WP#1). Finalise
            #    this session cleanly; the waiting->recording path then
            #    starts a fresh transect_<TS>_... session for the new one.
            if (state['mission_seq'] is not None and self.current_seq is not None
                    and state['mission_seq'] < self.current_seq):
                self._exit_recording(
                    state,
                    f"mission_restart(seq {self.current_seq}->{state['mission_seq']})",
                )
                return

            # 3) Positive, fresh stop evidence -- and ONLY positive. No
            #    HEARTBEAT (unreachable) is "unknown", never a stop.
            stop_reason = None
            if hb_present:
                if not is_auto:
                    stop_reason = f"mode!=AUTO({state['mode_num']})"
                elif not is_armed:
                    stop_reason = "disarmed"
                elif nav_stopped:
                    stop_reason = "nav_stopped"
            if stop_reason is None:
                # Mission still running (or telemetry unknown) -> clear the
                # grace timer; we are emphatically still recording.
                self._stop_evidence_since = None
            else:
                now = time.monotonic()
                if self._stop_evidence_since is None:
                    self._stop_evidence_since = now
                    self._note_event(
                        f"stop evidence: {stop_reason} "
                        f"(grace {int(_TRANSECT_STOP_GRACE_S)}s)"
                    )
                elif (now - self._stop_evidence_since) > _TRANSECT_STOP_GRACE_S:
                    self._exit_recording(state, stop_reason)
                    return

            # 4) Leg boundary -- forward MISSION_CURRENT.seq change.
            if (state['mission_seq'] is not None
                    and state['mission_seq'] != self.current_seq):
                self._roll_leg(state)

    # -- state machine transitions -----------------------------------
    def _handle_statustext(self, txt):
        """React to a freshly-arrived STATUSTEXT line.

        Matching is case-insensitive -- ArduPilot emits mixed-case
        ``Mission Complete``, but log viewers (and some firmware builds)
        upper-case it. We only *latch* here; the next _tick consumes the
        latch and finalises, so the work always happens on the monitor
        thread under the normal flow.
        """
        if not txt:
            return
        self._note_event(f"statustext: {txt}")
        if "mission complete" in txt.lower():
            self._mission_complete_latch = True

    def _enter_recording(self, state):
        seq = state['mission_seq'] if state['mission_seq'] is not None else 0
        wp_label = f"wp{int(seq):02d}"
        # Snapshot the capture type once for the whole session.
        capture = transect_capture_type
        self._capture_type = capture
        try:
            with _mode_lock:
                if mode != MODE_IDLE:
                    self._note_event(f"cannot enter: mode={mode}")
                    return
                if capture == "timelapse":
                    anchor = _start_transect_timelapse(wp_label)
                else:
                    anchor = _start_video_session(
                        base_prefix="transect",
                        target_mode=MODE_TRANSECT,
                        initial_label=wp_label,
                        per_leg_sidecars=True,
                    )
        except Exception as e:
            logger.exception("TransectMonitor: start (%s) failed", capture)
            self._note_event(f"start failed: {e}")
            return

        # Manifest lives beside the session: a sibling .ndjson for video
        # parts, or a per-session ``<source_tag>_manifest.ndjson`` inside
        # the shared survey-day image folder (keyed by source_tag so two
        # runs on the same day don't overwrite each other's manifest).
        if capture == "timelapse":
            tl = _timelapse
            tag = tl.source_tag if tl is not None else _make_source_tag("tr")
            self._manifest_path = os.path.join(anchor, f"{tag}_manifest.ndjson")
        else:
            self._manifest_path = _transect_manifest_path(anchor)
        try:
            _manifest_write_header(self._manifest_path, anchor)
        except Exception:
            logger.exception("Manifest header write failed")

        self.state = "recording"
        self.current_seq = seq
        self.leg_count = 1
        self.session_started_at = datetime.now()
        # Fresh session -> clear any stale stop/complete bookkeeping.
        self._stop_evidence_since = None
        self._mission_complete_latch = False
        self._begin_leg(state, seq)
        self._note_event(f"started {capture} session at wp{seq:02d}")

    def _exit_recording(self, state, reason):
        try:
            self._close_leg(state, reason)
        except Exception:
            logger.exception("close_leg on exit failed")
        try:
            with _mode_lock:
                if mode == MODE_TRANSECT:
                    if self._capture_type == "timelapse":
                        _stop_transect_timelapse()
                    else:
                        _stop_video_session()
        except Exception:
            logger.exception("stop capture on exit failed")
        self.state = "waiting"
        self.current_seq = None
        self._leg_started_at = None
        self._leg_started_seq = None
        self.session_started_at = None
        self._manifest_path = None
        # Reset stop/complete bookkeeping so the next mission starts clean.
        self._stop_evidence_since = None
        self._mission_complete_latch = False
        self._note_event(f"stopped session: {reason}")

    def _roll_leg(self, state):
        """Close the previous leg and roll to a new file/folder for the new WP."""
        prior_leg_dur = 0.0
        if self._leg_started_at is not None:
            prior_leg_dur = (datetime.now() - self._leg_started_at).total_seconds()

        try:
            self._close_leg(state, "next_wp")
        except Exception:
            logger.exception("close_leg in roll_leg failed")

        new_seq = state['mission_seq']
        wp_label = f"wp{int(new_seq):02d}"

        short_leg = prior_leg_dur < _TRANSECT_MIN_LEG_DURATION_S
        if self._capture_type == "timelapse":
            tl = _timelapse  # snapshot for thread-safety
            if short_leg:
                # Bursty seq advance -> don't spin up a tiny leg folder;
                # let frames keep landing in the current one.
                self._note_event(
                    f"seq -> {new_seq} (no new folder, prior leg {prior_leg_dur:.1f}s)"
                )
            else:
                if tl is not None:
                    tl.set_leg(wp_label)
                self._note_event(f"seq -> {new_seq} (new leg folder)")
        else:
            session = _session  # snapshot for thread-safety
            if short_leg:
                # Don't emit a split-now -- just relabel the in-flight
                # file's *next* rollover so the WP catches up later.
                if session is not None:
                    with session._lock:
                        session._next_label = wp_label
                self._note_event(
                    f"seq -> {new_seq} (no split, prior leg {prior_leg_dur:.1f}s)"
                )
            else:
                if session is not None:
                    session.split_now(wp_label)
                self._note_event(f"seq -> {new_seq} (split)")

        self.current_seq = new_seq
        self.leg_count += 1
        self._begin_leg(state, new_seq)

    # -- per-leg manifest --------------------------------------------
    def _begin_leg(self, state, seq):
        self._leg_started_at = datetime.now()
        self._leg_started_seq = seq
        self._leg_started_position = get_blueboat_gps_position()
        self._leg_started_heading = state.get('heading')
        self._leg_started_groundspeed = state.get('groundspeed')
        self._leg_started_wp_dist = state.get('wp_dist')

    def _close_leg(self, state, reason):
        if self._leg_started_seq is None or not self._manifest_path:
            return
        end_lat, end_lon, _end_alt = get_blueboat_gps_position()
        end_time = datetime.now()
        end_state = state or {}
        row = {
            "ts": time.time(),
            "event": "leg_close",
            "seq": self._leg_started_seq,
            "start_time": (self._leg_started_at.strftime(
                "%Y-%m-%d %H:%M:%S.%f")[:-3] if self._leg_started_at else None),
            "end_time": end_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            "duration_s": (round((end_time - self._leg_started_at).total_seconds(), 2)
                           if self._leg_started_at else None),
            "start_lat": self._leg_started_position[0],
            "start_lon": self._leg_started_position[1],
            "end_lat": end_lat,
            "end_lon": end_lon,
            "heading_start": self._leg_started_heading,
            "groundspeed_start": self._leg_started_groundspeed,
            "wp_dist_start": self._leg_started_wp_dist,
            "wp_dist_end": end_state.get('wp_dist'),
            "close_reason": reason,
            "capture_type": self._capture_type,
        }
        if self._capture_type == "timelapse":
            tl = _timelapse
            # Single-folder layout: identify the leg by its waypoint label
            # and record how many frames it captured (per-waypoint count).
            row["leg_wp"] = tl._wp_label if tl is not None else None
            row["leg_images"] = tl._frame if tl is not None else None
        else:
            # ``last_pattern`` is the most recent path splitmuxsink
            # opened; for the closing row this is the file the leg
            # actually wrote to. None if the session has already torn
            # down (shouldn't happen here, but defended).
            row["leg_file"] = (os.path.basename(_session.last_pattern)
                               if _session is not None and _session.last_pattern else None)
        _manifest_append(self._manifest_path, row)

    # -- diagnostics --------------------------------------------------
    def _note_event(self, text):
        self.last_event = text
        self.last_event_at = datetime.now()
        try:
            log_event("transect", text)
        except Exception:
            pass
        logger.info("TRANSECT: %s", text)

    def snapshot(self):
        """Pull a JSON-serialisable status snapshot for /status."""
        last = self.last_state or {}
        # Current leg artifact + (for images) frame count, depending on
        # the active capture backend.
        current_leg_file = None
        leg_images = None
        if self._capture_type == "timelapse":
            tl = _timelapse
            if tl is not None and tl._wp_label:
                current_leg_file = tl._wp_label
                leg_images = tl._frame
        else:
            if _session and _session.last_pattern:
                current_leg_file = os.path.basename(_session.last_pattern)
        return {
            "enabled": self.state != "disabled",
            "state": self.state,
            # Active session's capture type, plus the configured default
            # so the UI can show the toggle even while idle/waiting.
            "capture_type": self._capture_type,
            "configured_capture_type": transect_capture_type,
            "mode_num": last.get('mode_num'),
            "armed": last.get('armed'),
            "mission_seq": last.get('mission_seq'),
            "wp_dist": last.get('wp_dist'),
            "groundspeed": last.get('groundspeed'),
            "heading": last.get('heading'),
            "leg_count": self.leg_count,
            "current_leg_file": current_leg_file,
            "leg_images": leg_images,
            "session_started_at": (self.session_started_at.isoformat()
                                    if self.session_started_at else None),
            "last_event": self.last_event,
            "last_event_at": (self.last_event_at.isoformat()
                              if self.last_event_at else None),
            "statustext": last.get('statustext'),
        }


# Singleton handle, populated by /transect/enable.
_transect_monitor = None

# ---------------------------------------------------------------------------
# Vehicle / optics helpers
# ---------------------------------------------------------------------------
#
# ArduSub custom_mode enum (see ``MODE_STABILIZE`` etc. imported at the
# top of the file). Only stabilize/althold are wired through the widget;
# manual is exposed as a special case for the "restore" button used
# during testing.
_ALLOWED_MODE_NAMES = {
    "stabilize": MODE_STABILIZE,
    "althold": MODE_ALT_HOLD,
    "alt_hold": MODE_ALT_HOLD,
    "manual": MODE_MANUAL,
}

# Optics survey defaults verified against the RadCam on this vehicle:
#   Zoom  -> RANGE 0   -> SERVO11 = 935  (fully out)
#   Zoom  -> RANGE 50  -> SERVO11 ~ 1392 (half)
#   Zoom  -> RANGE 100 -> SERVO11 = 1850 (full)
# Focus  -> RANGE 61.03 -> SERVO12 = 1639 (survey trim)
ZOOM_PRESETS_PCT = {"out": 0.0, "half": 50.0, "full": 100.0}
FOCUS_TRIM_PCT = 61.03  # matches SERVO12_TRIM=1639 the operator set


def _current_servo_pwm(channel: int) -> int | None:
    """Read ``SERVO_OUTPUT_RAW.servo{ch}_raw`` from local mavlink2rest.

    Same URL the tilt reader uses. Returns None on error or when the
    channel is not driven (mavlink2rest reports 0 while disarmed for
    motor outputs -- for the optics/mount channels this stays populated).
    """
    try:
        response = requests.get(servo_output_url, timeout=1)
        if response.status_code == 200:
            message = response.json().get('message', {})
            pwm = message.get(f'servo{int(channel)}_raw', None)
            if pwm is None:
                return None
            return int(pwm)
    except Exception as e:
        logger.debug("servo%s read failed: %s", channel, e)
    return None


def get_optics_snapshot() -> dict:
    """Return the raw PWM for the tilt / focus / zoom outputs.

    Cheap enough to call from /status without saturating mavlink2rest.
    Missing channels come back as None.
    """
    tilt_pwm = _current_servo_pwm(TILT_SERVO_CHANNEL)
    focus_pwm = _current_servo_pwm(12)
    zoom_pwm = _current_servo_pwm(11)
    focus_pct = focus_pwm_to_pct(focus_pwm) if focus_pwm else None
    # World-relative (earth-frame) camera pitch: reuse the tilt PWM we
    # just read, then add the vehicle pitch from ATTITUDE.
    tilt_body = tilt_pwm_to_body_deg(tilt_pwm)
    if tilt_body is None:
        tilt_deg = None
    else:
        veh_pitch = get_towfish_attitude().get('pitch') or 0.0
        tilt_deg = round(tilt_body + veh_pitch, 1)
    return {
        "tilt_pwm": tilt_pwm,
        "tilt_deg": tilt_deg,
        "focus_pwm": focus_pwm,
        "focus_pct": (round(focus_pct, 2) if focus_pct is not None else None),
        "focus_pwm_min": MAV_FOCUS_PWM_MIN,
        "focus_pwm_max": MAV_FOCUS_PWM_MAX,
        "focus_trim_pct": FOCUS_TRIM_PCT,
        "zoom_pwm": zoom_pwm,
    }


def _get_towfish_depth_m():
    """Depth (metres, positive-down) the ArduSub autopilot estimates from
    the barometer/pressure sensor -- i.e. the same value ALT_HOLD closes
    the loop on and QGroundControl shows as "Depth".

    Sourced from ``VFR_HUD.alt`` (negative underwater). NOTE: AHRS2.altitude
    is *not* populated on this towfish (reads a constant 0.0), so VFR_HUD is
    the reliable baro-depth feed even though the SRT overlay path still uses
    AHRS2.

    Returns ``None`` when mavlink2rest is unreachable so the widget can
    show ``--`` rather than a misleading 0.0 m.
    """
    try:
        r = requests.get(vfr_hud_url, timeout=1)
        if r.status_code == 200:
            alt = r.json().get('message', {}).get('alt', None)
            if alt is not None:
                return round(-alt, 2) if alt < 0 else 0.0
    except Exception as e:
        logger.debug("depth read failed: %s", e)
    return None


def get_vehicle_status_snapshot() -> dict:
    """Return current ArduSub custom_mode + armed bit + depth for the widget."""
    depth_m = _get_towfish_depth_m()
    try:
        r = requests.get(
            'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/HEARTBEAT',
            timeout=1,
        )
        if r.status_code == 200:
            msg = r.json().get('message') or {}
            base_bits = ((msg.get('base_mode') or {}).get('bits')) or 0
            custom_mode = msg.get('custom_mode')
            armed = bool(base_bits & MAV_MODE_FLAG_SAFETY_ARMED)
            mode_label = None
            if custom_mode == MODE_STABILIZE:
                mode_label = "STABILIZE"
            elif custom_mode == MODE_ALT_HOLD:
                mode_label = "ALT_HOLD"
            elif custom_mode == MODE_MANUAL:
                mode_label = "MANUAL"
            return {
                "armed": armed,
                "custom_mode": custom_mode,
                "mode_label": mode_label,
                "depth_m": depth_m,
            }
    except Exception as e:
        logger.debug("HEARTBEAT read failed: %s", e)
    return {"armed": None, "custom_mode": None, "mode_label": None,
            "depth_m": depth_m}


# --- one-push AWB (RadCam) -------------------------------------------
_awb_last_success_at: datetime | None = None
_awb_last_error: str | None = None


def trigger_awb_once() -> bool:
    """Fire one 'onceAWB=1' at the RadCam via setImageAdjustmentEx.

    Runs in whatever thread called us -- both the manual /optics/awb
    route and the AWB loop take advantage of that: they intentionally
    block for the full HTTP round-trip so a failure surfaces in the
    caller's log/http response. Never raises.

    The camera returns 200 with a JSON body containing ``code``; 0 is
    OK, non-zero (e.g. bad auth) is surfaced as an error string.
    """
    global _awb_last_success_at, _awb_last_error
    try:
        r = requests.post(RADCAM_AWB_URL, json=RADCAM_AWB_BODY, timeout=3)
        if r.status_code == 200:
            # Firmware always returns 200; look at the JSON body's code.
            body_text = (r.text or "").strip()
            code = None
            try:
                code = (r.json() or {}).get("code")
            except Exception:
                pass
            if code is None or code == 0:
                _awb_last_success_at = datetime.now()
                _awb_last_error = None
                logger.info("RadCam AWB (onceAWB=1) OK")
                return True
            _awb_last_error = f"code {code}: {body_text[:120]}"
        else:
            _awb_last_error = f"HTTP {r.status_code}"
    except Exception as e:
        _awb_last_error = str(e)
    logger.warning("RadCam AWB failed: %s", _awb_last_error)
    return False


# Background 2-minute AWB loop that only runs while the transect
# monitor is enabled AND awb_loop_enabled is true.
_awb_thread: threading.Thread | None = None
_awb_stop_event = threading.Event()


def _awb_loop_worker() -> None:
    """Fire AWB once now, then every AWB_LOOP_INTERVAL_S until stopped."""
    logger.info("AWB loop started (interval %.0fs)", AWB_LOOP_INTERVAL_S)
    trigger_awb_once()
    while not _awb_stop_event.wait(AWB_LOOP_INTERVAL_S):
        # Re-check the toggle + monitor state each tick so a config
        # change immediately halts the timer.
        if not awb_loop_enabled:
            break
        if _transect_monitor is None or _transect_monitor.state == "disabled":
            break
        trigger_awb_once()
    logger.info("AWB loop stopped")


def _start_awb_loop_if_wanted() -> None:
    """Spin up the AWB loop iff transect is enabled + toggle is on."""
    global _awb_thread
    if not awb_loop_enabled:
        return
    if _transect_monitor is None or _transect_monitor.state == "disabled":
        return
    if _awb_thread is not None and _awb_thread.is_alive():
        return
    _awb_stop_event.clear()
    _awb_thread = threading.Thread(
        target=_awb_loop_worker, name="radcam-awb-loop", daemon=True,
    )
    _awb_thread.start()


def _stop_awb_loop() -> None:
    """Signal the AWB loop to exit. Idempotent."""
    global _awb_thread
    if _awb_thread is None:
        return
    _awb_stop_event.set()
    _awb_thread.join(timeout=3.0)
    _awb_thread = None


def _apply_awb_loop_state_change() -> None:
    """React to a config change or transect state change.

    Called from three places: /config POST, /transect/enable, /transect/
    disable. Starts or stops the background timer to match the new
    (awb_loop_enabled, transect state) tuple.
    """
    if awb_loop_enabled and (
        _transect_monitor is not None and _transect_monitor.state != "disabled"
    ):
        _start_awb_loop_if_wanted()
    else:
        _stop_awb_loop()


# --- depth-jog thrust hold thread ------------------------------------
#
# ArduSub's RC3 override needs to be refreshed at ~5 Hz or the autopilot
# considers the override stale. We front the widget's button holds with a
# backend thread so a dropped browser event still times out (i.e. the
# thrust stops even if the client goes away).
_thrust_thread: threading.Thread | None = None
_thrust_stop_event = threading.Event()
_thrust_direction: str | None = None
_thrust_deadline: float | None = None
_thrust_pwm: int = Z_PWM_NEUTRAL
_thrust_lock = threading.Lock()
# Rolling release watchdog. Every /vehicle/thrust POST pushes the deadline
# this many seconds into the future; the frontend re-hits the route every
# ~200 ms while a button is held, so a live hold keeps extending itself
# indefinitely. If the posts stop -- released button, tab close, or a
# flaky link that swallowed the stop -- the override falls back to neutral
# within this window and AltHold retakes depth. 3 s tolerates a brief comms
# gap without pinning the vehicle in descend the way the 651 s hold in the
# towfish 00000061 log did.
_THRUST_KEEPALIVE_S = 3.0
_THRUST_REFRESH_HZ = 5.0


def _thrust_worker() -> None:
    """Push RC3 override at ~5 Hz until the deadline lapses or stop is set."""
    writer = get_default_writer()
    period = 1.0 / _THRUST_REFRESH_HZ
    logger.info("Thrust jog thread started")
    while not _thrust_stop_event.is_set():
        with _thrust_lock:
            deadline = _thrust_deadline
            pwm = _thrust_pwm
        if deadline is None or time.monotonic() >= deadline:
            break
        writer.rc_channels_override({Z_CHANNEL: pwm})
        _thrust_stop_event.wait(period)
    # Release the override on exit so AltHold's inner loop takes over.
    try:
        writer.rc_channels_override({})
    except Exception:
        logger.debug("rc release on thrust exit failed", exc_info=True)
    logger.info("Thrust jog thread exited")


def _start_thrust_thread_if_needed() -> None:
    global _thrust_thread
    if _thrust_thread is not None and _thrust_thread.is_alive():
        return
    _thrust_stop_event.clear()
    _thrust_thread = threading.Thread(
        target=_thrust_worker, name="vehicle-thrust-jog", daemon=True,
    )
    _thrust_thread.start()


def _stop_thrust_thread() -> None:
    global _thrust_thread, _thrust_direction, _thrust_deadline
    _thrust_stop_event.set()
    with _thrust_lock:
        _thrust_direction = None
        _thrust_deadline = None
    if _thrust_thread is not None:
        _thrust_thread.join(timeout=1.0)
    _thrust_thread = None


def get_thrust_status_snapshot() -> dict:
    """Report the depth-jog RC3 override the backend is currently pushing.

    The widget uses this for *live* feedback on the simulated pilot input
    -- it reflects what the autopilot is actually being told, so it stays
    honest even if the button press came from another client or the
    keep-alive is one poll away from timing out. ``direction``/``pwm`` are
    null whenever no override is live (autopilot has the stick back).
    """
    with _thrust_lock:
        direction = _thrust_direction
        deadline = _thrust_deadline
        pwm = _thrust_pwm
    active = bool(direction and deadline is not None
                  and time.monotonic() < deadline)
    return {
        "active": active,
        "direction": direction if active else None,
        "pwm": pwm if active else Z_PWM_NEUTRAL,
        "channel": Z_CHANNEL,
    }


# --- Startup optics preset -------------------------------------------
_STARTUP_OPTICS_MAX_ATTEMPTS = 20
_STARTUP_OPTICS_RETRY_S = 3.0


def _startup_optics_worker() -> None:
    """After Flask boots, drive tilt down + zoom Out once.

    mavlink2rest may not be ready the instant the container starts, so
    retry a bounded number of times with a short sleep between tries.
    Focus is intentionally left at the vehicle-configured trim.
    """
    writer = get_default_writer()
    logger.info("Startup optics init: tilt=DOWN, zoom=OUT")
    for attempt in range(1, _STARTUP_OPTICS_MAX_ATTEMPTS + 1):
        ok_tilt = writer.tilt_down()
        ok_zoom = writer.set_camera_zoom_range(ZOOM_PRESETS_PCT["out"])
        if ok_tilt and ok_zoom:
            logger.info("Startup optics init succeeded on attempt %d", attempt)
            return
        logger.debug("Startup optics attempt %d: tilt=%s zoom=%s",
                     attempt, ok_tilt, ok_zoom)
        time.sleep(_STARTUP_OPTICS_RETRY_S)
    logger.warning("Startup optics init gave up after %d attempts",
                   _STARTUP_OPTICS_MAX_ATTEMPTS)


def _kick_off_startup_optics() -> None:
    threading.Thread(
        target=_startup_optics_worker, name="startup-optics", daemon=True,
    ).start()


# ---------------------------------------------------------------------------
# Vehicle + optics Flask routes
# ---------------------------------------------------------------------------
@app.route('/vehicle/arm', methods=['POST'])
def vehicle_arm():
    """Arm or disarm the towfish (ArduSub).

    Body: ``{"armed": true|false}``. No safeguards beyond what ArduSub
    itself enforces -- widget is the primary control surface for the
    field operator and needs to be able to disarm mid-run.
    """
    data = request.get_json(silent=True) or {}
    if 'armed' not in data:
        return jsonify({"success": False, "message": "armed required"}), 400
    want_armed = bool(data['armed'])
    ok = get_default_writer().arm(want_armed)
    return jsonify({"success": ok, "armed": want_armed}), (200 if ok else 502)


@app.route('/vehicle/mode', methods=['POST'])
def vehicle_mode():
    """Set ArduSub flight mode via COMMAND_LONG DO_SET_MODE."""
    data = request.get_json(silent=True) or {}
    mode_name = str(data.get('mode', '')).strip().lower()
    if mode_name not in _ALLOWED_MODE_NAMES:
        return jsonify({
            "success": False,
            "message": f"mode must be one of {sorted(_ALLOWED_MODE_NAMES)}",
        }), 400
    ok = get_default_writer().set_mode(_ALLOWED_MODE_NAMES[mode_name])
    return jsonify({"success": ok, "mode": mode_name}), (200 if ok else 502)


@app.route('/vehicle/tilt-down', methods=['POST'])
def vehicle_tilt_down():
    ok = get_default_writer().tilt_down()
    return jsonify({"success": ok, "pitch_deg": -70.0}), (200 if ok else 502)


@app.route('/vehicle/thrust', methods=['POST'])
def vehicle_thrust():
    """Start / keep-alive a Z-axis RC override (depth jog).

    Body: ``{"direction": "up"|"down"|"stop", "pwm": <optional int>}``.
    Each POST extends the override lifetime by ``_THRUST_KEEPALIVE_S`` so
    if the frontend stops sending (tab close, network glitch, released
    button), the backend releases the override on its own within a short
    window and AltHold takes back over.

    ``pwm`` lets the operator tune the jog strength from the widget (how
    hard to drive the vertical thrusters). It is clamped to
    ``Z_PWM_MIN``..``Z_PWM_MAX``; when omitted the built-in
    ascend/descend defaults are used.
    """
    global _thrust_direction, _thrust_deadline, _thrust_pwm
    data = request.get_json(silent=True) or {}
    direction = str(data.get('direction', '')).strip().lower()
    if direction in ('', 'stop'):
        _stop_thrust_thread()
        return jsonify({"success": True, "direction": "stop"})
    if direction not in ('up', 'down'):
        return jsonify({"success": False,
                        "message": "direction must be up/down/stop"}), 400

    default_pwm = Z_PWM_ASCEND if direction == 'up' else Z_PWM_DESCEND
    pwm = default_pwm
    if data.get('pwm') is not None:
        try:
            pwm = int(round(float(data['pwm'])))
        except (TypeError, ValueError):
            return jsonify({"success": False,
                            "message": "pwm must be numeric"}), 400
    pwm = max(Z_PWM_MIN, min(Z_PWM_MAX, pwm))

    with _thrust_lock:
        _thrust_direction = direction
        _thrust_pwm = pwm
        _thrust_deadline = time.monotonic() + _THRUST_KEEPALIVE_S
    _start_thrust_thread_if_needed()
    return jsonify({"success": True, "direction": direction, "pwm": pwm})


@app.route('/vehicle/thrust/stop', methods=['POST'])
def vehicle_thrust_stop():
    _stop_thrust_thread()
    return jsonify({"success": True, "direction": "stop"})


@app.route('/optics/focus', methods=['POST'])
def optics_focus():
    """Absolute or relative focus RANGE.

    Body accepts either ``{"pct": <0..100>}`` for an absolute focus
    RANGE, or ``{"delta_pct": <±float>}`` for a relative nudge from the
    live SERVO12 PWM. ``{"trim": true}`` snaps back to the survey trim
    (61.03% ≈ SERVO12=1639).
    """
    data = request.get_json(silent=True) or {}
    if data.get('trim'):
        pct = FOCUS_TRIM_PCT
    elif 'pct' in data:
        try:
            pct = float(data['pct'])
        except (TypeError, ValueError):
            return jsonify({"success": False, "message": "pct must be a number"}), 400
    elif 'delta_pct' in data:
        try:
            delta = float(data['delta_pct'])
        except (TypeError, ValueError):
            return jsonify({"success": False,
                            "message": "delta_pct must be a number"}), 400
        cur_pwm = _current_servo_pwm(12)
        cur_pct = focus_pwm_to_pct(cur_pwm) if cur_pwm else FOCUS_TRIM_PCT
        pct = cur_pct + delta
    else:
        return jsonify({"success": False,
                        "message": "pct, delta_pct, or trim required"}), 400
    pct = max(0.0, min(100.0, pct))
    ok = get_default_writer().set_camera_focus_range(pct)
    return jsonify({
        "success": ok, "pct": round(pct, 2),
        "expected_pwm": focus_pct_to_pwm(pct),
    }), (200 if ok else 502)


@app.route('/optics/zoom', methods=['POST'])
def optics_zoom():
    """One of the three verified zoom presets (out / half / full)."""
    data = request.get_json(silent=True) or {}
    preset = str(data.get('preset', '')).strip().lower()
    if preset not in ZOOM_PRESETS_PCT:
        return jsonify({
            "success": False,
            "message": f"preset must be one of {sorted(ZOOM_PRESETS_PCT)}",
        }), 400
    pct = ZOOM_PRESETS_PCT[preset]
    ok = get_default_writer().set_camera_zoom_range(pct)
    return jsonify({"success": ok, "preset": preset, "pct": pct}), (200 if ok else 502)


@app.route('/optics/awb', methods=['POST'])
def optics_awb():
    """Manual one-shot RadCam auto white-balance."""
    ok = trigger_awb_once()
    return jsonify({
        "success": ok,
        "last_error": _awb_last_error if not ok else None,
    }), (200 if ok else 502)


@app.route('/start', methods=['GET'])
def start():
    """Start an H.264 recording session.

    Mutually exclusive with timelapse and transect modes. The actual
    GStreamer pipeline lives in a background ``RecordingSession``
    watchdog thread that auto-restarts on RTSP drops or file stalls;
    this handler just sets up sidecars and kicks the watchdog off via
    :func:`_start_video_session`. Only the /stop route ever asks the
    session to actually stop -- every other exit (ERROR/EOS/stall) is
    treated as something the watchdog should recover from.
    """
    with _mode_lock:
        if mode == MODE_VIDEO:
            return jsonify({"success": False, "message": "Already recording"}), 400
        if mode == MODE_TIMELAPSE:
            return jsonify({"success": False,
                            "message": "Timelapse is active; stop it first"}), 409
        if mode == MODE_TRANSECT:
            return jsonify({"success": False,
                            "message": "Transect monitor is active; disable it first"}), 409
        try:
            _start_video_session(base_prefix="video_rtsp",
                                 target_mode=MODE_VIDEO)
            return jsonify({"success": True})
        except Exception as e:
            logger.exception("Error in /start")
            return jsonify({"success": False, "message": str(e)}), 500

@app.route('/stop', methods=['GET'])
def stop():
    """Stop the active manual RecordingSession and finalise sidecars.

    Only stops sessions started via /start (mode == MODE_VIDEO). A
    transect-mode session is owned by the monitor and must be stopped
    via /transect/disable so the monitor can update its own state
    cleanly.
    """
    with _mode_lock:
        if mode != MODE_VIDEO:
            if mode == MODE_TRANSECT:
                return jsonify({"success": False,
                                "message": "Transect monitor is active; use /transect/disable"}), 409
            return jsonify({"success": True, "message": "Not recording"})
        try:
            _stop_video_session()
            return jsonify({"success": True})
        except Exception as e:
            logger.exception("Error in /stop")
            # _stop_video_session resets mode to IDLE in its own finally
            # path -- here we just surface the error to the caller.
            return jsonify({"success": False, "message": str(e)}), 500

@app.route('/status', methods=['GET'])
def get_status():
    """Status of whichever mode is currently active.

    ``mode`` plus the video-specific or timelapse-specific blocks let
    the widget switch UI without juggling two separate endpoints.
    """
    global recording, start_time, usb_recording
    try:
        # Auto-clean if the watchdog thread died unexpectedly while video mode
        # was supposed to be active (e.g. uncaught exception in _run_one).
        if mode == MODE_VIDEO and (_session is None or not _session.is_alive()):
            logger.warning("Recording session is no longer alive; clearing mode")
            log_event("session_died", "RecordingSession watchdog exited unexpectedly")
            recording = False
            usb_recording = False
            start_time = None
            _set_mode(MODE_IDLE)

        # Video stats
        sess = _session
        rtsp_alive = bool(sess and sess.is_alive())
        gst_errors = sess.gst_errors if sess else 0
        gst_warnings = sess.gst_warnings if sess else 0
        file_stalls = sess.file_stalls if sess else 0
        restarts = sess.restart_count if sess else 0
        last_exit = sess.last_exit if sess else None
        current_part_path = sess.last_pattern if sess else None

        if mode == MODE_IDLE:
            health = "idle"
        elif mode == MODE_VIDEO:
            if not rtsp_alive or gst_errors > 0:
                health = "failed"
            elif gst_warnings > 0 or file_stalls > 0 or restarts > 0:
                health = "degraded"
            else:
                health = "healthy"
        else:  # timelapse
            tl = _timelapse
            if tl is None or not tl.is_alive():
                health = "failed"
            elif tl.miss_count > 0 and tl.snap_count == 0:
                health = "failed"
            elif tl.miss_count > 0:
                health = "degraded"
            else:
                health = "healthy"

        # File size of the most recent in-progress part.
        file_size_mb = 0.0
        if current_part_path and os.path.exists(current_part_path):
            file_size_mb = round(os.path.getsize(current_part_path) / (1024 * 1024), 1)

        # Timelapse stats
        tl = _timelapse
        timelapse_block = {
            "active": mode == MODE_TIMELAPSE,
            "snap_count": tl.snap_count if tl else 0,
            "miss_count": tl.miss_count if tl else 0,
            "last_snap_size_bytes": tl.last_snap_size_bytes if tl else 0,
            "last_snap_path": (os.path.basename(tl.last_snap_path)
                               if tl and tl.last_snap_path else None),
            "last_snap_at": (tl.last_snap_at.isoformat()
                             if tl and tl.last_snap_at else None),
            # Snap-vs-telemetry sync skew (ms) for the most recent
            # frame; small numbers mean the EXIF GPS/heading was
            # sampled in the same wall-clock window as the JPEG.
            "last_sync_skew_ms": (tl.last_sync_skew_ms
                                  if tl and tl.last_sync_skew_ms is not None
                                  else -1.0),
            "snapshot_url": snapshot_url,
        }

        # Disk-free for the *active* recording target so the UI shows
        # the right number when writing to USB. Falls back to the local
        # extension volume otherwise.
        active_dir = None
        if mode == MODE_VIDEO and sess is not None:
            active_dir = sess._out_dir
        elif mode in (MODE_TIMELAPSE, MODE_TRANSECT) and tl is not None:
            active_dir = tl._survey_dir
        disk_free = _read_disk_free_mb(active_dir)
        local_disk_free = _read_disk_free_mb(LOCAL_RECORDING_DIR)
        active_start = start_time if mode == MODE_VIDEO else (tl.start_time if (tl and mode == MODE_TIMELAPSE) else None)

        usb_status = usb_storage.get_status()

        resp = jsonify({
            "mode": mode,
            "recording": recording,
            "start_time": active_start.isoformat() if active_start else None,
            "duration_seconds": round((datetime.now() - active_start).total_seconds(), 1) if active_start else 0,
            "rtsp_process_alive": rtsp_alive,
            "rtsp_endpoint": RTSP_ENDPOINT,
            "file_size_mb": file_size_mb,
            "current_part": (os.path.basename(current_part_path)
                              if current_part_path else None),
            "restarts": restarts,
            "last_exit": last_exit,
            # disk_free_mb tracks the active recording target so the UI
            # warns based on whichever drive the operator is actually
            # filling. local_disk_free_mb is the local extension volume,
            # always reported so the UI can compare both tiers.
            "disk_free_mb": disk_free,
            "local_disk_free_mb": local_disk_free,
            "gst_errors": gst_errors,
            "gst_warnings": gst_warnings,
            "file_stalls": file_stalls,
            "health": health,
            "container_format": container_format,
            "stream_protocol": stream_protocol,
            "timelapse": timelapse_block,
            # Storage selection + USB drive state for the storage card.
            "storage_preference": storage_preference,
            "usb_storage": usb_status,
            "usb_recording": usb_recording,
            "usb_failover_count": usb_failover_count,
            "usb_failover_reason": usb_failover_reason,
            "active_storage": "usb" if usb_recording else (
                "local" if mode != MODE_IDLE else None),
            # Per-poll snapshot of the automatic transect monitor. Null
            # when the monitor was never enabled; otherwise contains
            # the state machine state, last mission/nav telemetry, leg
            # count and per-event diagnostic strings (see
            # TransectMonitor.snapshot for the schema).
            "transect": (_transect_monitor.snapshot()
                         if _transect_monitor is not None else None),
            # Vehicle state -- armed bit + custom_mode string + baro depth
            # for the cockpit widget's persistent status strip.
            "vehicle": get_vehicle_status_snapshot(),
            # Live depth-jog RC3 override so the widget can show the
            # simulated pilot input the autopilot is actually receiving.
            "thrust": get_thrust_status_snapshot(),
            # BlueBoat Ping sonar (DISTANCE_SENSOR) -- water depth under
            # the surface craft. Null when the boat is unreachable.
            "ping_sonar": get_ping_sonar_snapshot(),
            # Live tilt / focus / zoom PWM readouts so the OPTICS tab
            # can render a phosphor readout without a separate call.
            "optics": get_optics_snapshot(),
            # AWB timer state so the toggle UI can show whether the
            # 2-min loop is actively running.
            "awb_loop_enabled": awb_loop_enabled,
            "awb_loop_active": (_awb_thread is not None and _awb_thread.is_alive()),
            "awb_last_success_at": (_awb_last_success_at.isoformat()
                                     if _awb_last_success_at else None),
            "awb_last_error": _awb_last_error,
        })
        resp.headers['Cache-Control'] = 'no-store'
        return resp
    except Exception as e:
        logger.error(f"Error in status endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

def _recording_roots():
    """Yield ``(label, root_path)`` for each storage tier that may hold
    recordings. ``label`` is ``"local"`` or ``"usb"`` and matches the
    optional ``?storage=`` selector on /download. The USB tier is only
    yielded when the drive is currently mounted; an unmounted USB
    contributes nothing to the listing.
    """
    yield "local", LOCAL_RECORDING_DIR
    usb_root = usb_storage.get_base_dir()
    if usb_root:
        yield "usb", usb_root


@app.route('/list', methods=['GET'])
def list_videos():
    """Return on-disk recordings across local + USB storage tiers.

    Each entry carries a ``storage`` field (``"local"`` or ``"usb"``)
    so the UI knows which drive it lives on; the download route reads
    the same field from the ``?storage=`` query string.
    """
    try:
        os.makedirs(LOCAL_RECORDING_DIR, exist_ok=True)

        videos = []
        timelapses = []
        for storage, root in _recording_roots():
            if not os.path.isdir(root):
                continue
            try:
                names = os.listdir(root)
            except Exception:
                continue
            for name in names:
                full = os.path.join(root, name)
                if os.path.isdir(full) and (name.startswith("survey_")
                                            or name.startswith("timelapse_")
                                            or name.startswith("transect_")):
                    # Count JPEGs recursively. survey_* folders are flat
                    # (single-folder layout); legacy transect_* nested one
                    # wpNN/ subfolder per leg -- os.walk covers both.
                    jpgs = 0
                    try:
                        for _root, _dirs, _files in os.walk(full):
                            jpgs += sum(1 for f in _files if f.endswith('.jpg'))
                    except Exception:
                        jpgs = 0
                    if jpgs > 0 or name.startswith(("survey_", "timelapse_")):
                        timelapses.append({
                            "name": name,
                            "snap_count": jpgs,
                            "storage": storage,
                        })
                elif name.endswith(('.mp4', '.ts')):
                    videos.append({"name": name, "storage": storage})
        videos.sort(key=lambda v: v["name"], reverse=True)
        timelapses.sort(key=lambda t: t["name"], reverse=True)
        return jsonify({"videos": videos, "timelapses": timelapses})
    except Exception as e:
        logger.error(f"Error in list endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/download/<path:filename>')
def download(filename):
    """Download any file under the local extension volume *or* the USB
    Towfish/ root (incl. ``timelapse_*/<jpg>``).

    The optional ``?storage=`` query string picks which root to serve
    from: ``"local"`` (default, preserves the historical behaviour) or
    ``"usb"`` (the mounted USB drive's Towfish/ folder). Path-traversal
    is guarded by realpath comparison against the chosen root.
    """
    try:
        storage = request.args.get('storage', 'local').strip().lower()
        if storage == 'usb':
            usb_root = usb_storage.get_base_dir()
            if not usb_root or not os.path.isdir(usb_root):
                return jsonify({"success": False,
                                "message": "USB storage not mounted"}), 404
            root = os.path.realpath(usb_root)
        else:
            root = os.path.realpath(LOCAL_RECORDING_DIR)

        full = os.path.realpath(os.path.join(root, filename))
        # Path-traversal guard: never serve outside the chosen root.
        if not full.startswith(root + os.sep):
            return jsonify({"success": False, "message": "Invalid path"}), 400
        if not os.path.exists(full):
            return jsonify({"success": False, "message": "Not found"}), 404
        return send_file(full, as_attachment=True)
    except Exception as e:
        logger.error(f"Error in download endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/filesize', methods=['GET'])
def get_filesize():
    """Return the active recording's size, or the most-recent file if idle.

    For timelapse the "size" is the cumulative bytes written to the
    active session folder, and the "filename" is the folder name.
    """
    try:
        video_dir = LOCAL_RECORDING_DIR

        if mode == MODE_VIDEO:
            sess = _session
            if sess and sess.last_pattern and os.path.exists(sess.last_pattern):
                path = sess.last_pattern
                # Sum every part written so far so the displayed size matches
                # the *whole* recording, not just the current splitmuxsink fragment.
                total = sum(
                    os.path.getsize(p) for p in list_session_parts(sess.first_part_path)
                    if os.path.exists(p)
                )
                return jsonify({
                    "success": True,
                    "filename": os.path.basename(path),
                    "size_bytes": total,
                    "recording": True,
                    "mode": mode,
                })

        if mode == MODE_TRANSECT:
            # Video transect: current leg .ts size.
            sess = _session
            if sess and sess.last_pattern and os.path.exists(sess.last_pattern):
                path = sess.last_pattern
                # For transect we report just the *current leg* size --
                # one file per WP, so summing across legs would be
                # misleading. The total-session size is implicit in the
                # transect_*_manifest.ndjson sidecar.
                return jsonify({
                    "success": True,
                    "filename": os.path.basename(path),
                    "size_bytes": os.path.getsize(path),
                    "recording": True,
                    "mode": mode,
                })
            # Image transect: survey-day folder name + cumulative bytes
            # written this session (tracked incrementally so we don't
            # re-stat a whole day of files every poll).
            tl = _timelapse
            if tl is not None and tl.last_snap_path:
                return jsonify({
                    "success": True,
                    "filename": os.path.basename(tl._survey_dir),
                    "size_bytes": tl.bytes_written,
                    "recording": True,
                    "mode": mode,
                    "snap_count": tl.snap_count,
                })

        if mode == MODE_TIMELAPSE:
            tl = _timelapse
            if tl is not None:
                return jsonify({
                    "success": True,
                    "filename": os.path.basename(tl._survey_dir),
                    "size_bytes": tl.bytes_written,
                    "recording": True,
                    "mode": mode,
                    "snap_count": tl.snap_count,
                })

        # Idle: report the most recent on-disk video.
        path = None
        filename = None
        if os.path.exists(video_dir):
            files = [f for f in os.listdir(video_dir) if f.endswith(('.mp4', '.ts'))]
            files.sort(reverse=True)
            if files:
                filename = files[0]
                path = os.path.join(video_dir, filename)

        if path and os.path.exists(path):
            return jsonify({
                "success": True,
                "filename": filename,
                "size_bytes": os.path.getsize(path),
                "recording": False,
                "mode": mode,
            })
        return jsonify({
            "success": True,
            "filename": None,
            "size_bytes": 0,
            "recording": False,
            "mode": mode,
        })
    except Exception as e:
        logger.error(f"Error in filesize endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

# ---------------------------------------------------------------------------
# Timelapse (2 Hz HTTP snap) endpoints
# ---------------------------------------------------------------------------

@app.route('/timelapse/start', methods=['GET'])
def timelapse_start():
    """Start the 2 Hz HTTP-snap timelapse loop.

    Mutually exclusive with video recording (HTTP 409 if a recording
    is already running).
    """
    global _timelapse, usb_recording
    with _mode_lock:
        if mode == MODE_TIMELAPSE:
            return jsonify({"success": False, "message": "Timelapse already running"}), 400
        if mode == MODE_VIDEO:
            return jsonify({"success": False,
                            "message": "Video recording is active; stop it first"}), 409
        try:
            subfolder = _survey_day_subfolder()
            source_tag = _make_source_tag("tl")
            out_dir, on_usb = _resolve_recording_dir(subfolder=subfolder)
            session = TimelapseSession(snap_url=snapshot_url, out_dir=out_dir,
                                       source_tag=source_tag)
            session.on_usb = on_usb
            try:
                session.start()
            except Exception as e:
                logger.exception("Failed to start TimelapseSession")
                return jsonify({"success": False, "message": str(e)}), 500
            _timelapse = session
            usb_recording = on_usb
            _set_mode(MODE_TIMELAPSE)
            storage_label = "USB" if on_usb else "local"
            logger.info("Timelapse started: %s (snap_url=%s, storage=%s)",
                        out_dir, snapshot_url, storage_label)
            return jsonify({
                "success": True,
                "folder": os.path.basename(out_dir),
                "snapshot_url": snapshot_url,
                "storage": "usb" if on_usb else "local",
            })
        except Exception as e:
            logger.exception("Error in /timelapse/start")
            return jsonify({"success": False, "message": str(e)}), 500

@app.route('/timelapse/stop', methods=['GET'])
def timelapse_stop():
    """Stop the active timelapse loop and return final stats."""
    global _timelapse, usb_recording
    with _mode_lock:
        if mode != MODE_TIMELAPSE:
            return jsonify({"success": True, "message": "Not running"})
        try:
            session = _timelapse
            _timelapse = None
            stats = {"snap_count": 0, "miss_count": 0, "folder": None}
            if session is not None:
                session.stop()
                stats["snap_count"] = session.snap_count
                stats["miss_count"] = session.miss_count
                if session.last_snap_path:
                    stats["folder"] = os.path.basename(os.path.dirname(session.last_snap_path))
            usb_recording = False
            _set_mode(MODE_IDLE)
            logger.info("Timelapse stopped: snaps=%d misses=%d",
                        stats["snap_count"], stats["miss_count"])
            return jsonify({"success": True, **stats})
        except Exception as e:
            logger.exception("Error in /timelapse/stop")
            _set_mode(MODE_IDLE)
            return jsonify({"success": False, "message": str(e)}), 500

@app.route('/transect/enable', methods=['GET', 'POST'])
def transect_enable():
    """Enable the automatic transect monitor.

    The monitor begins polling the tow vehicle's mission state in the
    background. It will only stand up a real RecordingSession once the
    vehicle is armed, in AUTO, and actually navigating. Mutually
    exclusive with manual MODE_VIDEO and MODE_TIMELAPSE.
    """
    global _transect_monitor
    with _mode_lock:
        if mode == MODE_VIDEO:
            return jsonify({"success": False,
                            "message": "Manual recording is active; stop it first"}), 409
        if mode == MODE_TIMELAPSE:
            return jsonify({"success": False,
                            "message": "Timelapse is active; stop it first"}), 409
        if _transect_monitor is not None and _transect_monitor.state != "disabled":
            return jsonify({"success": False, "message": "Already enabled"}), 400
        try:
            if _transect_monitor is None:
                _transect_monitor = TransectMonitor()
            _transect_monitor.enable()
            logger.info("Transect monitor enabled (tow_vehicle=%s)", tow_vehicle_ip)
            # Kick the AWB loop only if the operator's toggle is on;
            # the timer immediately fires one AWB, then rearms itself
            # every AWB_LOOP_INTERVAL_S until transect is disabled.
            _apply_awb_loop_state_change()
            return jsonify({"success": True,
                            "state": _transect_monitor.state,
                            "tow_vehicle_ip": tow_vehicle_ip,
                            "awb_loop_enabled": awb_loop_enabled})
        except Exception as e:
            logger.exception("Error in /transect/enable")
            return jsonify({"success": False, "message": str(e)}), 500

@app.route('/transect/disable', methods=['GET', 'POST'])
def transect_disable():
    """Disable the automatic transect monitor.

    If a session is in flight (mode == MODE_TRANSECT) it is stopped
    cleanly via :func:`_stop_video_session` and the trailing leg's
    manifest row is finalised first.
    """
    global _transect_monitor
    if _transect_monitor is None or _transect_monitor.state == "disabled":
        return jsonify({"success": True, "message": "Not enabled"})
    try:
        _transect_monitor.disable()
        # Always stop the AWB timer when transect is disabled -- the loop
        # is intentionally scoped to survey time.
        _stop_awb_loop()
        snap = _transect_monitor.snapshot()
        logger.info("Transect monitor disabled (legs=%d)", snap.get("leg_count", 0))
        return jsonify({"success": True, **snap})
    except Exception as e:
        logger.exception("Error in /transect/disable")
        return jsonify({"success": False, "message": str(e)}), 500

# ── Survey parameter checker ─────────────────────────────────────────────
# Reads and writes are slow enough (a MAVLink round-trip per parameter,
# retried on loss) that doing them inside a request would leave the page
# hanging for tens of seconds against an unreachable boat. Instead one
# background worker at a time owns the whole batch and the UI polls
# /params for progress.
_param_lock = threading.Lock()
_param_readings = {}
_param_links = {v: {"reachable": None, "checked_at": None}
                for v in PARAM_VEHICLES}
_param_job = {
    "running": False,
    "kind": None,
    "total": 0,
    "done": 0,
    "current": None,
    "started_at": None,
    "finished_at": None,
    "message": None,
}
_param_thread = None


def _param_client(vehicle):
    """Build a ParamClient aimed at one of the two vehicles.

    The boat's URL is rebuilt per call because the operator can change
    ``tow_vehicle_ip`` from this very page between batches.
    """
    if vehicle == "boat":
        return ParamClient(f'http://{tow_vehicle_ip}/mavlink2rest')
    return ParamClient(LOCAL_MAVLINK2REST_URL)


def _param_matches(spec, current, target):
    """Is the vehicle's value close enough to the target to call it set?

    Parameters cross the wire as float32 and ArduPilot rounds integer
    parameters, so an exact compare would flag correctly-set values as
    mismatched. The tolerance is relative for large values and absolute
    for ones near zero.
    """
    if current is None or target is None:
        return None
    return abs(float(current) - float(target)) <= max(1e-4,
                                                      abs(float(target)) * 1e-3)


def _param_snapshot():
    """Assemble the /params payload under the lock."""
    with _param_lock:
        readings = dict(_param_readings)
        job = dict(_param_job)
        links = {k: dict(v) for k, v in _param_links.items()}
    targets = dict(param_targets)

    params = []
    for spec in PARAM_SPECS:
        name = spec["name"]
        reading = readings.get(name, {})
        current = reading.get("value")
        target = targets.get(name)
        params.append({
            **{k: spec[k] for k in ("name", "vehicle", "unit", "decimals",
                                    "min", "max", "desc")},
            "presets": spec.get("presets"),
            "default": spec["default"],
            "target": target,
            "current": current,
            "matches": _param_matches(spec, current, target),
            "read_at": reading.get("read_at"),
            "error": reading.get("error"),
        })
    return {
        "params": params,
        "job": job,
        "links": links,
        "boat_url": f'http://{tow_vehicle_ip}/mavlink2rest',
        "towfish_url": LOCAL_MAVLINK2REST_URL,
    }


def _param_record(name, value=None, error=None, param_type=None):
    with _param_lock:
        _param_readings[name] = {
            "value": value,
            "error": error,
            "type": param_type,
            "read_at": datetime.now().isoformat(timespec="seconds"),
        }


def _param_job_update(**fields):
    with _param_lock:
        _param_job.update(fields)


def _param_worker(kind, names):
    """Run one check or apply batch across both vehicles.

    Vehicles are grouped so each ParamClient is built once and its
    reachability probed once; an unreachable vehicle short-circuits its
    whole group instead of burning a multi-second timeout per parameter.
    """
    try:
        by_vehicle = {}
        for name in names:
            spec = PARAM_SPECS_BY_NAME[name]
            by_vehicle.setdefault(spec["vehicle"], []).append(name)

        done = 0
        failures = 0
        for vehicle, vehicle_names in by_vehicle.items():
            client = _param_client(vehicle)
            reachable = client.is_reachable()
            with _param_lock:
                _param_links[vehicle] = {
                    "reachable": reachable,
                    "checked_at": datetime.now().isoformat(timespec="seconds"),
                }
            if not reachable:
                label = "tow boat" if vehicle == "boat" else "towfish"
                for name in vehicle_names:
                    _param_record(name, error=f"{label} not responding")
                    done += 1
                    failures += 1
                _param_job_update(done=done)
                continue

            for name in vehicle_names:
                _param_job_update(current=name, done=done)
                try:
                    if kind == "apply":
                        result = client.write(name, param_targets[name])
                    else:
                        result = client.read(name)
                    _param_record(name, value=result.get("value"),
                                  param_type=result.get("type"))
                except ParamReadError as e:
                    _param_record(name, error=str(e))
                    failures += 1
                except Exception as e:
                    logger.warning("Parameter %s on %s failed: %s",
                                   name, vehicle, e)
                    _param_record(name, error=str(e))
                    failures += 1
                done += 1
                _param_job_update(done=done)

        verb = "Applied" if kind == "apply" else "Checked"
        message = f"{verb} {done - failures}/{done} parameters"
        if failures:
            message += f" -- {failures} failed"
        _param_job_update(message=message)
    except Exception as e:
        logger.error("Parameter %s batch crashed: %s", kind, e)
        _param_job_update(message=f"Batch failed: {e}")
    finally:
        _param_job_update(running=False, current=None,
                          finished_at=datetime.now().isoformat(timespec="seconds"))


def _param_start_job(kind, names):
    """Spawn the worker unless one is already running.

    Returns ``(started, message)``.
    """
    global _param_thread
    with _param_lock:
        if _param_job["running"]:
            return False, "A parameter operation is already running"
        _param_job.update({
            "running": True,
            "kind": kind,
            "total": len(names),
            "done": 0,
            "current": None,
            "started_at": datetime.now().isoformat(timespec="seconds"),
            "finished_at": None,
            "message": None,
        })
    _param_thread = threading.Thread(
        target=_param_worker, args=(kind, names),
        name=f"param-{kind}", daemon=True,
    )
    _param_thread.start()
    return True, None


def _param_names_from_request(data):
    """Resolve a ``names``/``vehicles`` request body to a spec-name list.

    An empty body means "everything", which is what both toolbar buttons
    send.
    """
    names = data.get("names")
    if names:
        unknown = [n for n in names if n not in PARAM_SPECS_BY_NAME]
        if unknown:
            return None, f"Unknown parameter(s): {', '.join(unknown)}"
        return list(names), None

    vehicles = data.get("vehicles")
    if vehicles:
        unknown = [v for v in vehicles if v not in PARAM_VEHICLES]
        if unknown:
            return None, f"Unknown vehicle(s): {', '.join(unknown)}"
        return [s["name"] for s in PARAM_SPECS
                if s["vehicle"] in vehicles], None

    return [s["name"] for s in PARAM_SPECS], None


@app.route('/params', methods=['GET'])
def params_state():
    resp = jsonify(_param_snapshot())
    resp.headers['Cache-Control'] = 'no-store'
    return resp


@app.route('/params/targets', methods=['POST'])
def params_set_targets():
    """Persist operator-edited target values without touching a vehicle."""
    global param_targets
    data = request.get_json(silent=True) or {}
    incoming = data.get("targets")
    if not isinstance(incoming, dict) or not incoming:
        return jsonify({"success": False,
                        "message": "targets object required"}), 400

    unknown = [n for n in incoming if n not in PARAM_SPECS_BY_NAME]
    if unknown:
        return jsonify({"success": False,
                        "message": f"Unknown parameter(s): {', '.join(unknown)}"}), 400

    merged = dict(param_targets)
    merged.update(incoming)
    param_targets = _sanitize_param_targets(merged)
    _persist_config()
    return jsonify({"success": True, "targets": param_targets})


@app.route('/params/check', methods=['POST'])
def params_check():
    data = request.get_json(silent=True) or {}
    names, error = _param_names_from_request(data)
    if error:
        return jsonify({"success": False, "message": error}), 400
    started, message = _param_start_job("check", names)
    if not started:
        return jsonify({"success": False, "message": message}), 409
    return jsonify({"success": True, "queued": len(names)})


@app.route('/params/apply', methods=['POST'])
def params_apply():
    """Write the saved targets for the requested parameters.

    Targets come from the persisted config rather than the request body,
    so the UI has to save an edit before it can push it -- that keeps
    what the operator sees on screen and what lands on the autopilot from
    drifting apart.
    """
    data = request.get_json(silent=True) or {}
    names, error = _param_names_from_request(data)
    if error:
        return jsonify({"success": False, "message": error}), 400
    started, message = _param_start_job("apply", names)
    if not started:
        return jsonify({"success": False, "message": message}), 409
    return jsonify({"success": True, "queued": len(names)})


@app.route('/widget')
def widget():
    return app.send_static_file('widget.html')

@app.route('/telemetry', methods=['GET'])
def get_telemetry():
    try:
        depth = get_depth_data()
        vfr_data = get_vfr_hud_data()
        baro_data = get_baro_data()
        light_percentage = get_light_output()
        
        # Get BlueBoat position and altitude
        bb_lat, bb_lon, bb_alt = get_blueboat_gps_position()
        
        # Get towfish heading for offset calculation
        towfish_heading = get_towfish_heading()
        
        # Estimated towfish position: boat fix laid back per config
        gps_lat, gps_lon = offset_towed_position(bb_lat, bb_lon, towfish_heading)
        
        logger.info(f"Sending telemetry: depth={depth}, climb={vfr_data}, temp={baro_data}, lights={light_percentage}%, GPS=({gps_lat}, {gps_lon}), heading={towfish_heading}")
        
        response_data = {
            "success": True,
            "depth": round(depth, 1),
            "climb": round(vfr_data, 2),
            "temperature": round(baro_data, 1),
            "lights": light_percentage,
            "timestamp": datetime.now().strftime('%H:%M:%S')
        }
        
        # Add GPS data if available (offset position)
        if gps_lat is not None and gps_lon is not None:
            response_data["gps_lat"] = round(gps_lat, 6)
            response_data["gps_lon"] = round(gps_lon, 6)
        else:
            response_data["gps_lat"] = None
            response_data["gps_lon"] = None
        
        # Add altitude from BlueBoat
        response_data["altitude"] = round(bb_alt, 1) if bb_alt is not None else None
        
        # Add towfish heading
        response_data["towfish_heading"] = round(towfish_heading, 1) if towfish_heading is not None else None
        
        return jsonify(response_data)
    except Exception as e:
        logger.error(f"Error in telemetry endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

if __name__ == '__main__':
    # Kick off the USB probe before the Flask app starts so the first
    # /status / /config request after boot already has accurate mount
    # state.  The probe also handles drives that get hot-plugged after
    # the container is running.
    try:
        os.makedirs(LOCAL_RECORDING_DIR, exist_ok=True)
    except Exception as e:
        logger.warning(f"Could not create local recording dir: {e}")
    try:
        usb_storage.start_probe()
    except Exception as e:
        logger.warning(f"USB probe failed to start: {e}")
    try:
        _start_usb_health_watcher()
    except Exception as e:
        logger.warning(f"USB health watcher failed to start: {e}")

    # Fire tilt-down + zoom-Out once as soon as the extension is up so
    # the operator doesn't have to touch the widget just to prep the
    # camera. Runs in a background thread with bounded retries in case
    # mavlink2rest is still coming up.
    try:
        _kick_off_startup_optics()
    except Exception as e:
        logger.warning(f"Startup optics init failed to start: {e}")

    start_data_lake_server()
    app.run(host='0.0.0.0', port=5423)
