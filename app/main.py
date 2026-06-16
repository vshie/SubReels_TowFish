from flask import Flask, jsonify, request, send_file
import asyncio
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
import io
import piexif
from datetime import timezone
from websockets.exceptions import ConnectionClosed

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402  (after require_version)

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

isp_log_thread = None
stop_isp_log_thread = False
current_isp_log_file = None

current_events_file = None

ass_thread = None
stop_ass_thread = False
current_ass_file = None
ass_subtitle_counter = 0

# Subtitle timing reference. SRT/ASS entries are timestamped relative to
# this epoch and (re)scaled to the encoded video duration when a file is
# finalised. For manual recording it equals ``start_time`` for the whole
# session; in transect mode it is reset at every leg rollover so each
# per-leg ``.srt``/``.ass`` starts at 00:00.
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
CONFIG_FILE = "/app/videorecordings/videorecorder_config.json"
DEFAULT_TOW_VEHICLE_IP = "192.168.2.12"
DEFAULT_CONTAINER_FORMAT = "mp4"
VALID_CONTAINER_FORMATS = ("mp4", "mpegts")
DEFAULT_STREAM_PROTOCOL = "udp"
VALID_STREAM_PROTOCOLS = ("udp", "tcp")
DEFAULT_SNAPSHOT_URL = "http://192.168.2.10/cgi-bin/onesnap.cgi"
# What the automatic transect monitor captures per leg:
#   "video"     -> one RTSP .ts file per waypoint leg (+ per-leg SRT/ASS)
#   "timelapse" -> 2 Hz geotagged JPEGs into one subfolder per leg
DEFAULT_TRANSECT_CAPTURE_TYPE = "video"
VALID_TRANSECT_CAPTURE_TYPES = ("video", "timelapse")

def load_config():
    """Load persisted configuration from disk, returning defaults on failure."""
    defaults = {
        "tow_vehicle_ip": DEFAULT_TOW_VEHICLE_IP,
        "container_format": DEFAULT_CONTAINER_FORMAT,
        "stream_protocol": DEFAULT_STREAM_PROTOCOL,
        "snapshot_url": DEFAULT_SNAPSHOT_URL,
        "transect_capture_type": DEFAULT_TRANSECT_CAPTURE_TYPE,
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

# Towfish (ArduSub) telemetry is read through the local BlueOS host the
# extension runs on (host.docker.internal), same as depth/altitude/temp,
# rather than a hardcoded 192.168.2.2.
towfish_attitude_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/ATTITUDE'

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
    """Get yaw and roll from towfish ATTITUDE message. Returns dict with degrees."""
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

def get_towing_gps_position():
    """Get GPS position from BlueBoat with 4m offset behind towfish heading.
    Returns (lat, lon) or (None, None) on failure."""
    # Get BlueBoat position
    lat, lon, alt = get_blueboat_gps_position()
    if lat is None or lon is None:
        return (None, None)
    
    # Get towfish heading
    heading = get_towfish_heading()
    if heading is None:
        # If no heading available, return BlueBoat position without offset
        return (lat, lon)
    
    # Calculate offset position (4 meters behind towfish heading)
    offset_lat, offset_lon = calculate_offset_position(lat, lon, heading, 4.0)
    return (offset_lat, offset_lon)

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

def format_srt_timestamp(seconds):
    """Format seconds into SRT timestamp format (HH:MM:SS,mmm)"""
    hours = int(seconds // 3600)
    minutes = int((seconds % 3600) // 60)
    secs = int(seconds % 60)
    milliseconds = int((seconds - int(seconds)) * 1000)
    return f"{hours:02d}:{minutes:02d}:{secs:02d},{milliseconds:03d}"

# Global counter for SRT subtitle entries
srt_subtitle_counter = 0

def get_video_duration(video_path):
    """Get video duration in seconds using ffprobe"""
    try:
        cmd = [
            'ffprobe', '-v', 'error',
            '-show_entries', 'format=duration',
            '-of', 'default=noprint_wrappers=1:nokey=1',
            video_path
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        if result.returncode == 0:
            duration = float(result.stdout.strip())
            return duration
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

def update_srt_file():
    """Append position data to the active SRT, synced to video recording.

    Timestamps are relative to ``sidecar_epoch`` (reset per leg in
    transect mode), then rescaled to the encoded duration when the file
    is finalised. When tow-vehicle telemetry is *not fresh* (GPS fetch
    failed/timed out) the entry is written with EMPTY position values
    rather than carrying the last known fix forward -- gaps stay explicit
    and never get back-filled with stale coordinates.
    """
    global stop_srt_thread, srt_subtitle_counter

    srt_update_rate = 5  # Updates per second (5 Hz for position data)

    while not stop_srt_thread and recording:
        try:
            # Cheap bookkeeping under the lock: capture the file + epoch
            # and claim a sequence number atomically so a concurrent leg
            # rotation can't interleave a half-written entry.
            with _sidecar_lock:
                srt_path = current_srt_file_rtsp
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
                start_timestamp = format_srt_timestamp(elapsed)
                end_timestamp = format_srt_timestamp(elapsed + 1 / srt_update_rate)

                # Fresh fetch every iteration; (None, None, _) means the
                # tow vehicle is unreachable / data is stale.
                lat, lon, _ = get_blueboat_gps_position()
                heading = get_towfish_heading()
                towfish_alt = get_towfish_altitude()

                if lat is not None and lon is not None:
                    if heading is not None:
                        offset_lat, offset_lon = calculate_offset_position(lat, lon, heading, 4.0)
                    else:
                        offset_lat, offset_lon = lat, lon
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

            time.sleep(1 / srt_update_rate)
        except Exception as e:
            logger.error(f"Error updating SRT file: {str(e)}")
            time.sleep(1)

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

def _read_disk_free_mb():
    """Read free disk space on /app/videorecordings in MB."""
    try:
        stat = os.statvfs("/app/videorecordings")
        return round((stat.f_bavail * stat.f_frsize) / (1024 * 1024), 1)
    except Exception:
        return None

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

def _build_pipeline_description(rtsp_url, container_fmt, proto, mux_name="muxsink"):
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
    """
    if container_fmt == "mpegts":
        muxer_factory = "mpegtsmux"
    else:
        muxer_factory = "mp4mux"
    proto = proto if proto in VALID_STREAM_PROTOCOLS else DEFAULT_STREAM_PROTOCOL
    return (
        f"rtspsrc location={rtsp_url} is-live=true latency=5000 "
        f"protocols={proto} retry=5 timeout=5000000 "
        f"! rtph264depay wait-for-keyframe=true "
        f"! h264parse config-interval=-1 "
        # hauv-v2 leaky queue: absorbs RTSP jitter so a brief mux stall
        # never back-pressures the depayloader (which would otherwise
        # drop the RTP session). 30 s of headroom, no byte/buffer cap,
        # leak the oldest frames downstream if anything ever wedges.
        f"! queue max-size-time=30000000000 max-size-bytes=0 max-size-buffers=0 "
        f"leaky=downstream silent=true "
        f"! video/x-h264,stream-format=byte-stream,alignment=au "
        f"! splitmuxsink name={mux_name} max-size-time=0 "
        f"muxer-factory={muxer_factory} send-keyframe-requests=true "
        f"async-finalize=false"
    )

def _finalize_leg_sidecars(srt_path, ass_path, ts_path):
    """Rescale a closed leg's SRT/ASS to that leg's encoded duration.

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
            logger.info("Per-leg sidecars finalised for %s (%.2fs)",
                        os.path.basename(ts_path) if ts_path else "?", duration)
        else:
            logger.warning("Per-leg finalise: no duration for %s, sidecars left unscaled",
                           ts_path)
    except Exception:
        logger.exception("Per-leg sidecar finalise failed")


def _rotate_leg_sidecars(prev_ts_path, new_ts_path):
    """Swap SRT/ASS to a new leg file, finalising the previous leg's pair.

    Called from the ``splitmuxsink`` format-location callback (GStreamer
    streaming thread) when a new leg ``.ts`` opens. Kept cheap: creates
    the new empty sidecars, repoints the globals under ``_sidecar_lock``
    (resetting counters + ``sidecar_epoch`` so the new leg starts at
    00:00), and hands the just-closed pair to a background finaliser so
    ffprobe never blocks the streaming thread.
    """
    global current_srt_file_rtsp, current_ass_file
    global srt_subtitle_counter, ass_subtitle_counter, sidecar_epoch

    with _sidecar_lock:
        old_srt = current_srt_file_rtsp
        old_ass = current_ass_file

    new_srt = create_srt_file(new_ts_path)
    new_ass = create_ass_file(new_ts_path)

    with _sidecar_lock:
        current_srt_file_rtsp = new_srt
        current_ass_file = new_ass
        srt_subtitle_counter = 0
        ass_subtitle_counter = 0
        sidecar_epoch = datetime.now()

    threading.Thread(
        target=_finalize_leg_sidecars,
        args=(old_srt, old_ass, prev_ts_path),
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
                 container_fmt, proto, per_leg_sidecars=False):
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

def _decimal_deg_to_dms_rationals(deg):
    """Convert decimal degrees -> EXIF-style ((d,1),(m,1),(s*10000,10000))."""
    deg = abs(float(deg))
    d = int(deg)
    m_full = (deg - d) * 60.0
    m = int(m_full)
    s = (m_full - m) * 60.0
    return ((d, 1), (m, 1), (int(round(s * 10000)), 10000))

def _build_gps_exif_bytes(lat, lon, alt_m, heading_deg, ts_local, ts_utc):
    """Build piexif-encoded EXIF bytes embedding the towfish position.

    Mirrors what ``update_srt_file`` writes into the .srt:

    * ``lat`` / ``lon`` are the BlueBoat fix offset 4 m along the
      towfish heading (already computed by the caller).
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
    if not (have_gps or have_heading):
        return None

    gps_ifd = {}

    if have_gps:
        gps_ifd[piexif.GPSIFD.GPSLatitudeRef] = b'N' if lat >= 0 else b'S'
        gps_ifd[piexif.GPSIFD.GPSLatitude] = _decimal_deg_to_dms_rationals(lat)
        gps_ifd[piexif.GPSIFD.GPSLongitudeRef] = b'E' if lon >= 0 else b'W'
        gps_ifd[piexif.GPSIFD.GPSLongitude] = _decimal_deg_to_dms_rationals(lon)
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
        (int(round(ts_utc.second * 100)), 100),
    )
    gps_ifd[piexif.GPSIFD.GPSDateStamp] = ts_utc.strftime('%Y:%m:%d').encode('ascii')

    dt_str = ts_local.strftime('%Y:%m:%d %H:%M:%S').encode('ascii')
    exif_ifd = {
        piexif.ExifIFD.DateTimeOriginal: dt_str,
        piexif.ExifIFD.DateTimeDigitized: dt_str,
    }
    image_ifd = {
        piexif.ImageIFD.DateTime: dt_str,
        piexif.ImageIFD.Software: b"BlueOS-VideoRecorder Towfish",
    }

    try:
        return piexif.dump({"0th": image_ifd, "Exif": exif_ifd, "GPS": gps_ifd})
    except Exception:
        logger.exception("EXIF dump failed")
        return None

_TIMELAPSE_CSV_HEADER = [
    'timestamp', 'seq', 'jpg', 'size_bytes',
    'lat', 'lon', 'altitude_m', 'towfish_heading_deg',
    'depth_m', 'temperature_c',
    'snap_ms', 'telem_ms', 'sync_skew_ms',
]


class TimelapseSession:
    """Background thread that GETs JPEGs from the camera's snap CGI.

    Two layouts:
      * Manual timelapse (``per_leg=False``): all JPEGs + one
        ``telemetry.csv`` land directly in ``out_dir`` with sequential
        ``00001.jpg`` names.
      * Transect timelapse (``per_leg=True``): ``out_dir`` is the
        session root and each waypoint leg gets its own subfolder
        (``out_dir/wpNN/``) with its own ``telemetry.csv`` and a
        per-leg sequence counter. The monitor calls :meth:`set_leg`
        at entry and at every leg boundary. Until the first leg is
        set the loop captures nothing.
    """

    def __init__(self, snap_url, out_dir, per_leg=False):
        self._snap_url = snap_url
        self._base_dir = out_dir
        self._per_leg = per_leg
        self._stop_event = threading.Event()
        self._thread = None
        self.start_time = None
        self.snap_count = 0          # session-wide total (all legs)
        self.miss_count = 0
        self.last_snap_size_bytes = 0
        self.last_snap_path = None
        self.last_snap_at = None
        # Wall-clock gap (ms) between the most recent snap finishing
        # and its parallel telemetry fetch finishing. Surfaces the
        # tightness of snap/GPS sync on /status so a field operator
        # can spot mavlink2rest going slow without scraping the CSV.
        self.last_sync_skew_ms = -1.0

        # Current output target (swapped per leg under the lock). For
        # manual mode it is fixed to out_dir for the session lifetime.
        self._leg_lock = threading.Lock()
        self._leg_label = None
        if per_leg:
            self._leg_dir = None
            self._leg_csv = None
        else:
            self._leg_dir = out_dir
            self._leg_csv = os.path.join(out_dir, "telemetry.csv")
        self._leg_seq = 0            # per-leg (or per-session) counter
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
        os.makedirs(self._base_dir, exist_ok=True)
        # Manual mode writes its single CSV header up front; per-leg mode
        # defers until set_leg() opens the first leg subfolder.
        if not self._per_leg:
            self._write_csv_header(self._leg_csv)
        self.start_time = datetime.now()
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._loop, name="timelapse-loop", daemon=True,
        )
        self._thread.start()

    def set_leg(self, label):
        """Switch capture output to a new per-leg subfolder (per_leg only).

        Creates ``<base>/<label>/`` with a fresh ``telemetry.csv`` and
        resets the per-leg sequence counter so each leg's JPEGs start at
        ``00001.jpg``. Safe to call from the monitor thread while the
        capture loop runs.
        """
        if not self._per_leg:
            return None
        leg_dir = os.path.join(self._base_dir, label)
        os.makedirs(leg_dir, exist_ok=True)
        csv_path = os.path.join(leg_dir, "telemetry.csv")
        self._write_csv_header(csv_path)
        with self._leg_lock:
            self._leg_dir = leg_dir
            self._leg_csv = csv_path
            self._leg_seq = 0
            self._leg_label = label
            self.leg_count += 1
        return leg_dir

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

        Designed to be called from a worker thread that runs in parallel
        with ``_fetch_jpeg`` so the GPS / heading / altitude samples
        share the same wall-clock window as the camera shutter (the
        snap CGI takes ~200 ms, each mavlink2rest GET ~10-300 ms; in
        parallel they overlap).

        Returns a dict (always populated, with None values where data
        was unavailable) plus a ``fetch_ms`` field so the CSV can
        record how stale the telemetry was relative to the snap.
        """
        t0 = time.monotonic()
        try:
            bb_lat, bb_lon, bb_alt = get_blueboat_gps_position()
            heading = get_towfish_heading()
            if bb_lat is not None and bb_lon is not None and heading is not None:
                gps_lat, gps_lon = calculate_offset_position(
                    bb_lat, bb_lon, heading, 4.0,
                )
            else:
                gps_lat, gps_lon = bb_lat, bb_lon
            tow_alt = get_towfish_altitude()
            depth = get_depth_data()
            temp = get_baro_data()
        except Exception:
            logger.exception("TIMELAPSE telemetry fetch raised")
            bb_lat = bb_lon = bb_alt = None
            gps_lat = gps_lon = None
            heading = None
            tow_alt = None
            depth = None
            temp = None
        return {
            'bb_lat': bb_lat, 'bb_lon': bb_lon, 'bb_alt': bb_alt,
            'gps_lat': gps_lat, 'gps_lon': gps_lon,
            'heading': heading, 'tow_alt': tow_alt,
            'depth': depth, 'temp': temp,
            'fetch_ms': round((time.monotonic() - t0) * 1000, 1),
        }

    def _loop(self):
        next_fire = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()
            if now < next_fire:
                if self._stop_event.wait(timeout=next_fire - now):
                    break
            next_fire = max(next_fire + _TIMELAPSE_PERIOD_S, time.monotonic())

            # Per-leg mode: until the monitor opens the first leg there's
            # nowhere to write, so idle without burning the snap budget.
            with self._leg_lock:
                leg_ready = self._leg_dir is not None
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
                tow_alt = None
                depth = None
                temp = None
            else:
                bb_lat = telemetry['bb_lat']
                bb_lon = telemetry['bb_lon']
                bb_alt = telemetry['bb_alt']
                gps_lat = telemetry['gps_lat']
                gps_lon = telemetry['gps_lon']
                heading = telemetry['heading']
                tow_alt = telemetry['tow_alt']
                depth = telemetry['depth']
                temp = telemetry['temp']

            # Embed GPS+heading+timestamp into EXIF before writing so
            # the on-disk JPEG is self-describing (geotagged in any
            # standard map / photo viewer). Failure here is non-fatal:
            # the raw JPEG and CSV row still get written.
            try:
                exif_bytes = _build_gps_exif_bytes(
                    gps_lat, gps_lon, tow_alt, heading, ts_local, ts_utc,
                )
                if exif_bytes is not None:
                    # piexif.insert with raw bytes requires either a
                    # path or a BytesIO output buffer; using BytesIO
                    # keeps the rewrite in memory so we still write
                    # the file exactly once.
                    out_buf = io.BytesIO()
                    piexif.insert(exif_bytes, jpeg, out_buf)
                    jpeg = out_buf.getvalue()
            except Exception:
                logger.exception("TIMELAPSE EXIF insert failed; saving raw JPEG")

            # Capture the active leg target + claim a per-leg sequence
            # number atomically so a concurrent set_leg() can't split a
            # frame across two folders.
            with self._leg_lock:
                leg_dir = self._leg_dir
                leg_csv = self._leg_csv
                self._leg_seq += 1
                seq = self._leg_seq
            if leg_dir is None:
                self.miss_count += 1
                continue
            self.snap_count += 1
            filename = f"{seq:05d}.jpg"
            path = os.path.join(leg_dir, filename)
            try:
                with open(path, 'wb') as f:
                    f.write(jpeg)
            except Exception:
                logger.exception("TIMELAPSE write failed: %s", path)
                self.miss_count += 1
                continue
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
                with open(leg_csv, 'a', newline='') as f:
                    w = csv.writer(f)
                    w.writerow([
                        ts_local.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                        seq, filename, len(jpeg),
                        f"{gps_lat:.6f}" if gps_lat is not None else "",
                        f"{gps_lon:.6f}" if gps_lon is not None else "",
                        f"{bb_alt:.2f}" if bb_alt is not None else "",
                        f"{heading:.1f}" if heading is not None else "",
                        f"{depth:.2f}" if depth is not None else "",
                        f"{temp:.2f}" if temp is not None else "",
                        f"{snap_ms:.1f}",
                        f"{telem_ms:.1f}" if telem_ms >= 0 else "",
                        f"{sync_skew_ms:.1f}" if sync_skew_ms >= 0 else "",
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
        "name": "Video Recorder",
        "description": "Record video from connected cameras with telemetry subtitles",
        "icon": "mdi-video",
        "company": "Blue Robotics",
        "version": "0.5",
        "webpage": "https://github.com/bluerobotics/blueos-video-recorder",
        "api": "https://github.com/bluerobotics/BlueOS-docker"
    }
    '''

@app.route('/config', methods=['GET', 'POST'])
def config():
    global tow_vehicle_ip, container_format, stream_protocol, snapshot_url
    global transect_capture_type

    if request.method == 'POST':
        if mode != MODE_IDLE:
            return jsonify({"success": False,
                            "message": f"Cannot change config while {mode} active"}), 400
        # The transect monitor owns the recorder even while sitting in
        # its "waiting" state, so block capture-type changes then too.
        if _transect_monitor is not None and _transect_monitor.state != "disabled":
            return jsonify({"success": False,
                            "message": "Cannot change config while transect monitor is enabled"}), 400

        data = request.get_json(silent=True) or {}
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

        if not changed:
            return jsonify({"success": False, "message": "No valid fields provided"}), 400

        save_config({"tow_vehicle_ip": tow_vehicle_ip,
                      "container_format": container_format,
                      "stream_protocol": stream_protocol,
                      "snapshot_url": snapshot_url,
                      "transect_capture_type": transect_capture_type})
        logger.info(
            "Config updated: tow_vehicle_ip=%s, container_format=%s, "
            "stream_protocol=%s, snapshot_url=%s, transect_capture_type=%s",
            tow_vehicle_ip, container_format, stream_protocol, snapshot_url,
            transect_capture_type,
        )
        return jsonify({"success": True,
                        "tow_vehicle_ip": tow_vehicle_ip,
                        "container_format": container_format,
                        "stream_protocol": stream_protocol,
                        "snapshot_url": snapshot_url,
                        "transect_capture_type": transect_capture_type})

    resp = jsonify({
        "rtsp_endpoint": RTSP_ENDPOINT,
        "tow_vehicle_ip": tow_vehicle_ip,
        "container_format": container_format,
        "stream_protocol": stream_protocol,
        "snapshot_url": snapshot_url,
        "transect_capture_type": transect_capture_type,
    })
    resp.headers['Cache-Control'] = 'no-store'
    return resp

# ── Shared video-session lifecycle helpers ───────────────────────────────
# Used by both the manual ``/start`` & ``/stop`` Flask routes *and* the
# automatic ``TransectMonitor``. Both helpers assume the caller already
# holds ``_mode_lock`` so the transect monitor and HTTP routes can never
# race each other into a half-open state.
def _start_video_session(base_prefix, target_mode, initial_label=None,
                         per_leg_sidecars=False):
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
    own matching .srt/.ass, rotated automatically when splitmuxsink opens
    each new fragment.
    """
    global recording, start_time, sidecar_epoch
    global srt_thread, stop_srt_thread, current_srt_file_rtsp, current_video_file_rtsp, srt_subtitle_counter
    global isp_log_thread, stop_isp_log_thread, current_isp_log_file
    global current_events_file
    global ass_thread, stop_ass_thread, current_ass_file, ass_subtitle_counter
    global _session

    os.makedirs("/app/videorecordings", exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    ext = ".ts" if container_format == "mpegts" else ".mp4"

    base_filename = f"{base_prefix}_{timestamp}"
    label_suffix = f"_{initial_label}" if initial_label else ""
    # Sidecars are named off the *first* part path so the existing
    # post-processing helpers (adjust_srt_timing, etc.) keep their
    # historical pair-by-name relationship.
    anchor_path = os.path.join("/app/videorecordings",
                               f"{base_filename}_part00_00000{label_suffix}{ext}")
    current_video_file_rtsp = anchor_path

    current_srt_file_rtsp = create_srt_file(anchor_path)
    srt_subtitle_counter = 0
    current_isp_log_file = create_isp_log_file(anchor_path)
    current_ass_file = create_ass_file(anchor_path)
    ass_subtitle_counter = 0
    current_events_file = create_events_file(anchor_path)

    log_event("recording_starting",
              f"container={container_format} proto={stream_protocol} "
              f"mode={target_mode} prefix={base_prefix}")

    session = RecordingSession(
        rtsp_url=RTSP_ENDPOINT,
        out_dir="/app/videorecordings",
        base_filename=base_filename,
        ext=ext,
        container_fmt=container_format,
        proto=stream_protocol,
        per_leg_sidecars=per_leg_sidecars,
    )
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
        current_isp_log_file = None
        current_ass_file = None
        current_events_file = None
        raise

    _session = session
    _set_mode(target_mode)
    recording = True
    start_time = datetime.now()
    # Subtitle epoch starts with the session; transect leg rotations
    # reset it per leg via _rotate_leg_sidecars.
    sidecar_epoch = start_time

    stop_srt_thread = False
    srt_thread = threading.Thread(target=update_srt_file, daemon=True)
    srt_thread.start()

    stop_isp_log_thread = False
    isp_log_thread = threading.Thread(target=update_isp_log, daemon=True)
    isp_log_thread.start()

    stop_ass_thread = False
    ass_thread = threading.Thread(target=update_ass_file, daemon=True)
    ass_thread.start()

    log_event("recording_started", f"anchor={anchor_path}")
    logger.info("Recording started: anchor=%s", anchor_path)
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
    global isp_log_thread, stop_isp_log_thread, current_isp_log_file
    global current_events_file
    global ass_thread, stop_ass_thread, current_ass_file
    global _session

    log_event("recording_stopping", "Stop requested")

    video_anchor = current_video_file_rtsp
    srt_path = current_srt_file_rtsp
    ass_path = current_ass_file
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
    start_time = None
    current_srt_file_rtsp = None
    current_video_file_rtsp = None
    current_isp_log_file = None
    current_ass_file = None
    current_events_file = None
    _set_mode(MODE_IDLE)

    if isp_log_path and os.path.exists(isp_log_path):
        logger.info("ISP log file saved: %s", isp_log_path)
    if ass_path and os.path.exists(ass_path):
        logger.info("ASS telemetry file saved: %s", ass_path)
    if events_path and os.path.exists(events_path):
        logger.info("Events log saved: %s", events_path)

    if per_leg:
        # Transect: earlier legs were already finalised on rotation; only
        # the *final* leg's sidecars remain, rescaled to that one leg's
        # encoded duration (NOT the session sum). Done in the background
        # so /transect/disable and the monitor's exit return promptly.
        if (srt_path or ass_path) and last_leg_ts:
            threading.Thread(
                target=_finalize_leg_sidecars,
                args=(srt_path, ass_path, last_leg_ts),
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
        else:
            logger.warning(
                "Could not determine session video duration, "
                "subtitle timing not adjusted",
            )

    logger.info("Recording stopped successfully")

def _start_transect_timelapse(initial_label):
    """Stand up a per-leg TimelapseSession; flip ``mode`` to MODE_TRANSECT.

    Mirror of :func:`_start_video_session` for the image capture type.
    Returns the session root dir (``transect_<TS>/``), which doubles as
    the manifest's parent. Caller must hold ``_mode_lock``.
    """
    global _timelapse
    os.makedirs("/app/videorecordings", exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join("/app/videorecordings", f"transect_{timestamp}")

    session = TimelapseSession(snap_url=snapshot_url, out_dir=out_dir, per_leg=True)
    try:
        session.start()
        session.set_leg(initial_label)
    except Exception:
        logger.exception("Failed to start transect TimelapseSession")
        _set_mode(MODE_IDLE)
        _timelapse = None
        raise

    _timelapse = session
    _set_mode(MODE_TRANSECT)
    log_event("transect_timelapse_started", out_dir)
    logger.info("Transect timelapse started: %s (snap_url=%s)", out_dir, snapshot_url)
    return out_dir

def _stop_transect_timelapse():
    """Tear down the transect TimelapseSession; flip ``mode`` to IDLE."""
    global _timelapse
    session = _timelapse
    _timelapse = None
    if session is not None:
        session.stop()
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
        # parts, or manifest.ndjson inside the image session folder.
        if capture == "timelapse":
            self._manifest_path = os.path.join(anchor, "manifest.ndjson")
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
            # For images the "leg file" is the per-leg subfolder, and we
            # also record how many JPEGs landed in it.
            row["leg_dir"] = (os.path.basename(tl._leg_dir)
                              if tl is not None and tl._leg_dir else None)
            row["leg_images"] = tl._leg_seq if tl is not None else None
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
            if tl is not None and tl._leg_dir:
                current_leg_file = os.path.basename(tl._leg_dir)
                leg_images = tl._leg_seq
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
    global recording, start_time
    try:
        # Auto-clean if the watchdog thread died unexpectedly while video mode
        # was supposed to be active (e.g. uncaught exception in _run_one).
        if mode == MODE_VIDEO and (_session is None or not _session.is_alive()):
            logger.warning("Recording session is no longer alive; clearing mode")
            log_event("session_died", "RecordingSession watchdog exited unexpectedly")
            recording = False
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

        disk_free = _read_disk_free_mb()
        active_start = start_time if mode == MODE_VIDEO else (tl.start_time if (tl and mode == MODE_TIMELAPSE) else None)

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
            "disk_free_mb": disk_free,
            "gst_errors": gst_errors,
            "gst_warnings": gst_warnings,
            "file_stalls": file_stalls,
            "health": health,
            "container_format": container_format,
            "stream_protocol": stream_protocol,
            "timelapse": timelapse_block,
            # Per-poll snapshot of the automatic transect monitor. Null
            # when the monitor was never enabled; otherwise contains
            # the state machine state, last mission/nav telemetry, leg
            # count and per-event diagnostic strings (see
            # TransectMonitor.snapshot for the schema).
            "transect": (_transect_monitor.snapshot()
                         if _transect_monitor is not None else None),
        })
        resp.headers['Cache-Control'] = 'no-store'
        return resp
    except Exception as e:
        logger.error(f"Error in status endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/list', methods=['GET'])
def list_videos():
    """Return on-disk recordings, including timelapse session folders."""
    try:
        video_dir = "/app/videorecordings"
        if not os.path.exists(video_dir):
            os.makedirs(video_dir)

        videos = []
        timelapses = []
        for name in os.listdir(video_dir):
            full = os.path.join(video_dir, name)
            if os.path.isdir(full) and (name.startswith("timelapse_")
                                        or name.startswith("transect_")):
                # Count JPEGs recursively so transect image sessions
                # (which nest one wpNN/ subfolder per leg) report their
                # full frame total, not just the root folder.
                jpgs = 0
                try:
                    for _root, _dirs, _files in os.walk(full):
                        jpgs += sum(1 for f in _files if f.endswith('.jpg'))
                except Exception:
                    jpgs = 0
                if jpgs > 0 or name.startswith("timelapse_"):
                    timelapses.append({"name": name, "snap_count": jpgs})
            elif name.endswith(('.mp4', '.ts')):
                videos.append(name)
        videos.sort(reverse=True)
        timelapses.sort(key=lambda t: t["name"], reverse=True)
        return jsonify({"videos": videos, "timelapses": timelapses})
    except Exception as e:
        logger.error(f"Error in list endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/download/<path:filename>')
def download(filename):
    """Download any file under /app/videorecordings (incl. timelapse_*/<jpg>)."""
    try:
        full = os.path.realpath(os.path.join("/app/videorecordings", filename))
        # Path-traversal guard: never serve outside the videorecordings root.
        root = os.path.realpath("/app/videorecordings")
        if not full.startswith(root + os.sep):
            return jsonify({"success": False, "message": "Invalid path"}), 400
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
        video_dir = "/app/videorecordings"

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
            # Image transect: current leg subfolder name + frame count.
            tl = _timelapse
            if tl is not None and tl.last_snap_path:
                leg_dir = os.path.dirname(tl.last_snap_path)
                size = 0
                try:
                    for f in os.listdir(leg_dir):
                        full = os.path.join(leg_dir, f)
                        if os.path.isfile(full):
                            size += os.path.getsize(full)
                except Exception:
                    pass
                return jsonify({
                    "success": True,
                    "filename": os.path.basename(leg_dir),
                    "size_bytes": size,
                    "recording": True,
                    "mode": mode,
                    "snap_count": tl.snap_count,
                })

        if mode == MODE_TIMELAPSE:
            tl = _timelapse
            if tl is not None:
                folder = os.path.basename(os.path.dirname(tl.last_snap_path)) if tl.last_snap_path else None
                size = 0
                if tl.last_snap_path:
                    parent = os.path.dirname(tl.last_snap_path)
                    try:
                        for f in os.listdir(parent):
                            full = os.path.join(parent, f)
                            if os.path.isfile(full):
                                size += os.path.getsize(full)
                    except Exception:
                        pass
                return jsonify({
                    "success": True,
                    "filename": folder,
                    "size_bytes": size,
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
    global _timelapse
    with _mode_lock:
        if mode == MODE_TIMELAPSE:
            return jsonify({"success": False, "message": "Timelapse already running"}), 400
        if mode == MODE_VIDEO:
            return jsonify({"success": False,
                            "message": "Video recording is active; stop it first"}), 409
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            out_dir = os.path.join("/app/videorecordings", f"timelapse_{timestamp}")
            session = TimelapseSession(snap_url=snapshot_url, out_dir=out_dir)
            try:
                session.start()
            except Exception as e:
                logger.exception("Failed to start TimelapseSession")
                return jsonify({"success": False, "message": str(e)}), 500
            _timelapse = session
            _set_mode(MODE_TIMELAPSE)
            logger.info("Timelapse started: %s (snap_url=%s)", out_dir, snapshot_url)
            return jsonify({
                "success": True,
                "folder": os.path.basename(out_dir),
                "snapshot_url": snapshot_url,
            })
        except Exception as e:
            logger.exception("Error in /timelapse/start")
            return jsonify({"success": False, "message": str(e)}), 500

@app.route('/timelapse/stop', methods=['GET'])
def timelapse_stop():
    """Stop the active timelapse loop and return final stats."""
    global _timelapse
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
            return jsonify({"success": True,
                            "state": _transect_monitor.state,
                            "tow_vehicle_ip": tow_vehicle_ip})
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
        snap = _transect_monitor.snapshot()
        logger.info("Transect monitor disabled (legs=%d)", snap.get("leg_count", 0))
        return jsonify({"success": True, **snap})
    except Exception as e:
        logger.exception("Error in /transect/disable")
        return jsonify({"success": False, "message": str(e)}), 500

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
        
        # Get offset position (4m behind towfish heading)
        gps_lat, gps_lon = get_towing_gps_position()
        
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
    start_data_lake_server()
    app.run(host='0.0.0.0', port=5423)
