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
#   "video"      : RTSP H.265 RecordingSession is active
#   "timelapse"  : 2 Hz HTTP-snap TimelapseSession is active
# Mode transitions go through _mode_lock so the Flask routes can never
# leave the recorder in a half-started state across both modes.
# ---------------------------------------------------------------------------
MODE_IDLE = "idle"
MODE_VIDEO = "video"
MODE_TIMELAPSE = "timelapse"

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

# RTSP endpoint for H.265 video stream
RTSP_H265_ENDPOINT = "rtsp://admin:blue@192.168.2.10:554/stream_0"

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

def load_config():
    """Load persisted configuration from disk, returning defaults on failure."""
    defaults = {
        "tow_vehicle_ip": DEFAULT_TOW_VEHICLE_IP,
        "container_format": DEFAULT_CONTAINER_FORMAT,
        "stream_protocol": DEFAULT_STREAM_PROTOCOL,
        "snapshot_url": DEFAULT_SNAPSHOT_URL,
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

_cfg = load_config()
tow_vehicle_ip = _cfg["tow_vehicle_ip"]
container_format = _cfg["container_format"]
stream_protocol = _cfg["stream_protocol"]
snapshot_url = _cfg["snapshot_url"]

# Mavlink URLs (Towfish heading at 192.168.2.2)
towfish_attitude_url = 'http://192.168.2.2/mavlink2rest/mavlink/vehicles/1/components/1/messages/ATTITUDE'

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
    """Update ASS file with comprehensive labeled telemetry at 5 Hz."""
    global stop_ass_thread, current_ass_file, ass_subtitle_counter

    ass_update_rate = 5

    while not stop_ass_thread and recording and current_ass_file:
        try:
            if start_time:
                elapsed = (datetime.now() - start_time).total_seconds()
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
                tf_parts = f"TF {tf_yaw} {tf_roll} Dep:{tf_depth:.1f} Clm:{tf_climb:.2f} Tmp:{tf_temp:.1f}"

                bb_gps = f"{bb_lat:.6f},{bb_lon:.6f}" if bb_lat is not None else "--,--"
                bb_yaw = f"Yaw:{bb_att['yaw']:.1f}" if 'yaw' in bb_att else "Yaw:--"
                bb_pitch = f"Pitch:{bb_att['pitch']:.1f}" if 'pitch' in bb_att else "Pitch:--"
                bb_speed = f"Spd:{bb_spd:.1f}" if bb_spd is not None else "Spd:--"
                bb_parts = f"BB {bb_gps} {bb_yaw} {bb_pitch} {bb_speed}"

                text = f"{tf_parts} | {bb_parts}"

                ass_subtitle_counter += 1
                line = f"Dialogue: 0,{start_ts},{end_ts},Telem,,0,0,0,,{text}\n"

                with open(current_ass_file, 'a') as f:
                    f.write(line)

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
    """Update SRT file with position data for WebODM processing, synced to video recording"""
    global stop_srt_thread, current_srt_file_rtsp, start_time, srt_subtitle_counter
    
    srt_update_rate = 5  # Updates per second (5 Hz for position data)
    
    while not stop_srt_thread and recording and current_srt_file_rtsp:
        try:
            if start_time:
                elapsed = (datetime.now() - start_time).total_seconds()
                start_timestamp = format_srt_timestamp(elapsed)
                end_timestamp = format_srt_timestamp(elapsed + 1/srt_update_rate)
                
                # Get BlueBoat position and towfish heading
                lat, lon, _ = get_blueboat_gps_position()
                heading = get_towfish_heading()
                
                # Get towfish altitude (negative value when underwater)
                towfish_alt = get_towfish_altitude()
                
                if lat is not None and lon is not None:
                    # Calculate offset position if heading available
                    if heading is not None:
                        offset_lat, offset_lon = calculate_offset_position(lat, lon, heading, 4.0)
                    else:
                        offset_lat, offset_lon = lat, lon
                    
                    srt_subtitle_counter += 1
                    
                    # Format SRT entry for WebODM (ODM SRT parser expects latitude/longitude/altitude keys)
                    # Format: latitude: 37.123456 longitude: -122.123456 altitude: -5.2
                    srt_entry = (
                        f"{srt_subtitle_counter}\n"
                        f"{start_timestamp} --> {end_timestamp}\n"
                        f"latitude: {offset_lat:.6f} longitude: {offset_lon:.6f} altitude: {towfish_alt:.1f}\n"
                        f"\n"
                    )
                    
                    # Append to SRT file
                    with open(current_srt_file_rtsp, 'a') as f:
                        f.write(srt_entry)
                
            time.sleep(1/srt_update_rate)
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
# Key motivations (informed by today's tow-fish dive + the doris
# tony-video-h265 reference):
#
# * RTSP drops at 50 Mbit/s HEVC silently kill the recorder. The watchdog
#   here detects ERROR/EOS on the GStreamer bus *and* file-stall and
#   restarts the pipeline automatically; per-restart files are written
#   via ``splitmuxsink`` so we never lose the previously-recorded data.
# * The 1-hour MPEG-TS PTS offset that breaks VLC playback comes from
#   ``mpegtsmux`` reusing rtspsrc's first PTS. ``splitmuxsink`` opens a
#   fresh muxer per fragment (PTS resets to zero) so every part plays
#   straight away.
# * ``wait-for-keyframe=true`` + ``alignment=au`` + ``config-interval=-1``
#   guarantee every part starts at a decodable VPS+SPS+PPS+IDR.
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
    """Build the gst-parse_launch description for one RTSP H.265 session.

    Output is always written via splitmuxsink so we get fresh PTS per
    segment (VLC can play it) and the watchdog can roll a new file on
    each pipeline rebuild.
    """
    if container_fmt == "mpegts":
        muxer_factory = "mpegtsmux"
    else:
        muxer_factory = "mp4mux"
    proto = proto if proto in VALID_STREAM_PROTOCOLS else DEFAULT_STREAM_PROTOCOL
    return (
        f"rtspsrc location={rtsp_url} is-live=true latency=5000 "
        f"protocols={proto} retry=5 timeout=5000000 do-retransmission=false "
        f"! rtph265depay wait-for-keyframe=true "
        f"! h265parse config-interval=-1 "
        f"! video/x-h265,stream-format=byte-stream,alignment=au "
        f"! splitmuxsink name={mux_name} max-size-time=0 "
        f"muxer-factory={muxer_factory} send-keyframe-requests=true "
        f"async-finalize=false"
    )

class RecordingSession:
    """One logical recording (one /start ... /stop call).

    Owns a background thread that keeps a GStreamer pipeline alive,
    rebuilding it on RTSP drops or file stalls. Each rebuild produces
    a new on-disk part via ``splitmuxsink`` so we never lose what was
    already written.
    """

    def __init__(self, rtsp_url, out_dir, base_filename, ext,
                 container_fmt, proto):
        self._rtsp_url = rtsp_url
        self._out_dir = out_dir
        self._base_filename = base_filename  # "video_rtsp_<TS>"
        self._ext = ext  # ".ts" or ".mp4"
        self._container_fmt = container_fmt
        self._proto = proto

        self._stop_event = threading.Event()
        self._thread = None
        self._lock = threading.Lock()

        self._pipeline = None
        self._muxsink = None
        self._current_part = 0
        self.restart_count = 0
        self.last_pattern = None
        self.last_exit = None
        # First on-disk part (used by /filesize, sidecar timing, etc.)
        self.first_part_path = None
        # Diagnostic counters surfaced via /status
        self.gst_errors = 0
        self.gst_warnings = 0
        self.file_stalls = 0

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

    # ------------------------------------------------------------------
    # splitmuxsink format-location callback
    # ------------------------------------------------------------------
    def _on_format_location(self, _splitmux, fragment_id):
        """Compose the on-disk path for the next .ts/.mp4 fragment."""
        path = os.path.join(
            self._out_dir,
            f"{self._base_filename}_part{self._current_part:02d}_"
            f"{int(fragment_id):05d}{self._ext}",
        )
        self.last_pattern = path
        if self.first_part_path is None:
            self.first_part_path = path
        log_event("part_opened", path)
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

        On a live HEVC ``rtspsrc protocols=udp`` pipeline the
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

class TimelapseSession:
    """Background thread that GETs JPEGs from the camera's snap CGI."""

    def __init__(self, snap_url, out_dir):
        self._snap_url = snap_url
        self._out_dir = out_dir
        self._stop_event = threading.Event()
        self._thread = None
        self._csv_path = os.path.join(out_dir, "telemetry.csv")
        self.start_time = None
        self.snap_count = 0
        self.miss_count = 0
        self.last_snap_size_bytes = 0
        self.last_snap_path = None
        self.last_snap_at = None

    def start(self):
        if self._thread is not None:
            raise RuntimeError("TimelapseSession already started")
        os.makedirs(self._out_dir, exist_ok=True)
        # Telemetry CSV header so each .jpg can be geotagged later.
        try:
            with open(self._csv_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow([
                    'timestamp', 'seq', 'jpg', 'size_bytes',
                    'lat', 'lon', 'altitude_m', 'towfish_heading_deg',
                    'depth_m', 'temperature_c',
                ])
        except Exception:
            logger.exception("TIMELAPSE failed to write CSV header")
        self.start_time = datetime.now()
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._loop, name="timelapse-loop", daemon=True,
        )
        self._thread.start()

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

    def _loop(self):
        next_fire = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()
            if now < next_fire:
                if self._stop_event.wait(timeout=next_fire - now):
                    break
            next_fire = max(next_fire + _TIMELAPSE_PERIOD_S, time.monotonic())

            jpeg, reason = self._fetch_jpeg()
            ts_local = datetime.now()
            ts_utc = datetime.now(tz=timezone.utc)
            if jpeg is None:
                self.miss_count += 1
                logger.warning("TIMELAPSE snap miss (%s)", reason)
                continue

            # Fetch the telemetry block ONCE per snap and reuse it for
            # both EXIF embedding and the CSV sidecar; matches the
            # convention used by ``update_srt_file`` so a JPEG and the
            # SRT entry at the same wall-clock time agree exactly.
            try:
                bb_lat, bb_lon, bb_alt = get_blueboat_gps_position()
                heading = get_towfish_heading()
                if bb_lat is not None and bb_lon is not None and heading is not None:
                    gps_lat, gps_lon = calculate_offset_position(
                        bb_lat, bb_lon, heading, 4.0,
                    )
                else:
                    gps_lat, gps_lon = bb_lat, bb_lon
                tow_alt = get_towfish_altitude()  # negative when underwater
                depth = get_depth_data()
                temp = get_baro_data()
            except Exception:
                logger.exception("TIMELAPSE telemetry fetch failed")
                bb_lat = bb_lon = bb_alt = None
                gps_lat = gps_lon = None
                heading = None
                tow_alt = None
                depth = None
                temp = None

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

            self.snap_count += 1
            seq = self.snap_count
            filename = f"{seq:05d}.jpg"
            path = os.path.join(self._out_dir, filename)
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

            # CSV sidecar keeps its historical schema:
            #   altitude_m = BlueBoat GPS altitude above MSL (bb_alt)
            #   depth_m    = towfish depth (positive)
            try:
                with open(self._csv_path, 'a', newline='') as f:
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

    if request.method == 'POST':
        if mode != MODE_IDLE:
            return jsonify({"success": False,
                            "message": f"Cannot change config while {mode} active"}), 400

        data = request.get_json(silent=True) or {}
        changed = False

        new_ip = data.get('tow_vehicle_ip', '').strip()
        if new_ip:
            tow_vehicle_ip = new_ip
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
                      "snapshot_url": snapshot_url})
        logger.info(
            "Config updated: tow_vehicle_ip=%s, container_format=%s, "
            "stream_protocol=%s, snapshot_url=%s",
            tow_vehicle_ip, container_format, stream_protocol, snapshot_url,
        )
        return jsonify({"success": True,
                        "tow_vehicle_ip": tow_vehicle_ip,
                        "container_format": container_format,
                        "stream_protocol": stream_protocol,
                        "snapshot_url": snapshot_url})

    resp = jsonify({
        "rtsp_h265_endpoint": RTSP_H265_ENDPOINT,
        "tow_vehicle_ip": tow_vehicle_ip,
        "container_format": container_format,
        "stream_protocol": stream_protocol,
        "snapshot_url": snapshot_url,
    })
    resp.headers['Cache-Control'] = 'no-store'
    return resp

@app.route('/start', methods=['GET'])
def start():
    """Start an H.265 recording session.

    Mutually exclusive with timelapse mode (HTTP 409 if a timelapse
    is already running). The actual GStreamer pipeline lives in a
    background ``RecordingSession`` watchdog thread that auto-restarts
    on RTSP drops or file stalls; this handler only sets up sidecars
    and kicks the watchdog off.
    """
    global recording, start_time
    global srt_thread, stop_srt_thread, current_srt_file_rtsp, current_video_file_rtsp, srt_subtitle_counter
    global isp_log_thread, stop_isp_log_thread, current_isp_log_file
    global current_events_file
    global ass_thread, stop_ass_thread, current_ass_file, ass_subtitle_counter
    global _session

    with _mode_lock:
        if mode == MODE_VIDEO:
            return jsonify({"success": False, "message": "Already recording"}), 400
        if mode == MODE_TIMELAPSE:
            return jsonify({"success": False,
                            "message": "Timelapse is active; stop it first"}), 409
        try:
            os.makedirs("/app/videorecordings", exist_ok=True)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            ext = ".ts" if container_format == "mpegts" else ".mp4"

            base_filename = f"video_rtsp_{timestamp}"
            # Sidecars are named off the *first* part path so the existing
            # post-processing helpers (adjust_srt_timing, etc.) keep their
            # historical pair-by-name relationship.
            anchor_path = os.path.join("/app/videorecordings",
                                       f"{base_filename}_part00_00000{ext}")
            current_video_file_rtsp = anchor_path

            current_srt_file_rtsp = create_srt_file(anchor_path)
            srt_subtitle_counter = 0
            current_isp_log_file = create_isp_log_file(anchor_path)
            current_ass_file = create_ass_file(anchor_path)
            ass_subtitle_counter = 0
            current_events_file = create_events_file(anchor_path)

            log_event("recording_starting",
                      f"container={container_format} proto={stream_protocol}")

            session = RecordingSession(
                rtsp_url=RTSP_H265_ENDPOINT,
                out_dir="/app/videorecordings",
                base_filename=base_filename,
                ext=ext,
                container_fmt=container_format,
                proto=stream_protocol,
            )
            try:
                session.start()
            except Exception as e:
                logger.exception("Failed to start RecordingSession")
                _set_mode(MODE_IDLE)
                _session = None
                current_srt_file_rtsp = None
                current_video_file_rtsp = None
                current_isp_log_file = None
                current_ass_file = None
                current_events_file = None
                return jsonify({"success": False, "message": str(e)}), 500

            _session = session
            _set_mode(MODE_VIDEO)
            recording = True
            start_time = datetime.now()

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
            return jsonify({"success": True})
        except Exception as e:
            logger.exception("Error in /start")
            _set_mode(MODE_IDLE)
            _session = None
            recording = False
            start_time = None
            current_srt_file_rtsp = None
            current_video_file_rtsp = None
            current_isp_log_file = None
            current_ass_file = None
            current_events_file = None
            return jsonify({"success": False, "message": str(e)}), 500

@app.route('/stop', methods=['GET'])
def stop():
    """Stop the active RecordingSession and finalise sidecars.

    Sidecar timing is adjusted across the *sum of all part durations*
    so SRT/ASS scaling stays accurate even when the watchdog rebuilt
    the pipeline (RTSP drop) one or more times during the session.
    """
    global recording, start_time
    global srt_thread, stop_srt_thread, current_srt_file_rtsp, current_video_file_rtsp
    global isp_log_thread, stop_isp_log_thread, current_isp_log_file
    global current_events_file
    global ass_thread, stop_ass_thread, current_ass_file
    global _session

    with _mode_lock:
        if mode != MODE_VIDEO:
            return jsonify({"success": True, "message": "Not recording"})
        try:
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
            if session is not None:
                session.stop()
                # Prefer the actual first part path the splitmuxsink callback
                # observed; falls back to the anchor we composed at /start.
                if session.first_part_path:
                    video_anchor = session.first_part_path

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

            if video_anchor and srt_path and os.path.exists(srt_path):
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
            return jsonify({"success": True})
        except Exception as e:
            logger.exception("Error in /stop")
            _session = None
            recording = False
            start_time = None
            current_srt_file_rtsp = None
            current_video_file_rtsp = None
            current_isp_log_file = None
            current_ass_file = None
            current_events_file = None
            _set_mode(MODE_IDLE)
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
            "rtsp_h265_endpoint": RTSP_H265_ENDPOINT,
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
            if os.path.isdir(full) and name.startswith("timelapse_"):
                try:
                    jpgs = [f for f in os.listdir(full) if f.endswith('.jpg')]
                except Exception:
                    jpgs = []
                timelapses.append({
                    "name": name,
                    "snap_count": len(jpgs),
                })
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
