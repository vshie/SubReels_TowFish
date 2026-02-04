from flask import Flask, jsonify, request, send_file
import os
import subprocess
from datetime import datetime
import logging
import signal
import time
import shlex
import requests
import threading
import math

app = Flask(__name__)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Global variables
rtsp_process = None
recording = False
start_time = None
srt_thread = None
stop_srt_thread = False
current_srt_file_rtsp = None
current_video_file_rtsp = None

# Mavlink URLs (local vehicle)
ahrs2_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/AHRS2'
vfr_hud_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/VFR_HUD'
baro_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/SCALED_PRESSURE2'
rc_channels_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/RC_CHANNELS'

# Mavlink URLs (BlueBoat at 192.168.2.12)
blueboat_gps_url = 'http://192.168.2.12/mavlink2rest/mavlink/vehicles/1/components/1/messages/GLOBAL_POSITION_INT'

# Mavlink URLs (Towfish heading at 192.168.2.2)
towfish_attitude_url = 'http://192.168.2.2/mavlink2rest/mavlink/vehicles/1/components/1/messages/ATTITUDE'

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
    """Get GPS position from BlueBoat. Returns (lat, lon, alt) or (None, None, None) on failure."""
    try:
        response = requests.get(blueboat_gps_url, timeout=1)
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

def create_srt_file(video_path):
    """Create a new .srt file for WebODM position data"""
    srt_path = video_path.replace('.mp4', '.srt')
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
                    
                    # Format SRT entry for WebODM
                    # Format: lat: 37.123456, lon: -122.123456, alt: -5.2 (negative = underwater depth)
                    srt_entry = (
                        f"{srt_subtitle_counter}\n"
                        f"{start_timestamp} --> {end_timestamp}\n"
                        f"lat: {offset_lat:.6f}, lon: {offset_lon:.6f}, alt: {towfish_alt:.1f}\n"
                        f"\n"
                    )
                    
                    # Append to SRT file
                    with open(current_srt_file_rtsp, 'a') as f:
                        f.write(srt_entry)
                
            time.sleep(1/srt_update_rate)
        except Exception as e:
            logger.error(f"Error updating SRT file: {str(e)}")
            time.sleep(1)

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

@app.route('/start', methods=['GET'])
def start():
    global rtsp_process, recording, start_time, srt_thread, stop_srt_thread, current_srt_file_rtsp, current_video_file_rtsp, srt_subtitle_counter
    try:
        if recording:
            return jsonify({"success": False, "message": "Already recording"}), 400
            
        # Ensure the video directory exists
        os.makedirs("/app/videorecordings", exist_ok=True)
            
        # Add a small delay to allow cameras to initialize
        time.sleep(1)
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename_rtsp = f"video_rtsp_{timestamp}.mp4"
        filepath_rtsp = os.path.join("/app/videorecordings", filename_rtsp)
        
        # Save video filepath for post-processing
        current_video_file_rtsp = filepath_rtsp
        
        # Create SRT file for WebODM position data
        current_srt_file_rtsp = create_srt_file(filepath_rtsp)
        srt_subtitle_counter = 0  # Reset counter for new recording
        
        # Pipeline for RTSP H265 stream
        rtsp_pipeline = ("rtspsrc location=rtsp://admin:blue@192.168.2.10:554/stream_0 ! "
            "rtph265depay ! h265parse ! mp4mux ! "
            f"filesink location={filepath_rtsp}")

        rtsp_command = ["gst-launch-1.0", "-e"] + shlex.split(rtsp_pipeline)
        
        # Start RTSP recording process FIRST
        rtsp_started = False
        try:
            rtsp_process = subprocess.Popen(rtsp_command,
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE)
            
            logger.info(f"Starting RTSP recording with command: {' '.join(rtsp_command)}")
            
            if rtsp_process.poll() is not None:
                stdout, stderr = rtsp_process.communicate()
                logger.error(f"RTSP process failed to start. stdout: {stdout.decode()}, stderr: {stderr.decode()}")
                rtsp_process = None
            else:
                rtsp_started = True
                logger.info("RTSP recording process started")
                
                # Wait for GStreamer to connect and start receiving frames
                # This ensures video is actually recording before we start SRT timing
                time.sleep(2)
                logger.info("RTSP stream stabilized, starting SRT synchronization")
        except Exception as e:
            logger.error(f"Failed to start RTSP recording: {str(e)}")
            rtsp_process = None
        
        # Check if RTSP stream started successfully
        if not rtsp_started:
            logger.error("RTSP video stream failed to start")
            recording = False
            start_time = None
            current_srt_file_rtsp = None
            current_video_file_rtsp = None
            return jsonify({"success": False, "message": "RTSP video stream failed to start"}), 500
        
        # NOW set recording state and start_time AFTER video is confirmed recording
        # This ensures SRT timestamps are synchronized with actual video content
        recording = True
        start_time = datetime.now()
        
        # Start SRT file update thread - timestamps now match video timing
        stop_srt_thread = False
        srt_thread = threading.Thread(target=update_srt_file)
        srt_thread.daemon = True
        srt_thread.start()
        
        # Log SRT file generation
        logger.info(f"Started SRT position file generation (synced to video): {current_srt_file_rtsp}")
        logger.info("Recording started successfully with RTSP stream")
        
        return jsonify({"success": True})
    except Exception as e:
        logger.error(f"Error in start endpoint: {str(e)}")
        recording = False
        start_time = None
        if rtsp_process:
            try:
                rtsp_process.kill()
            except:
                pass
        rtsp_process = None
        current_srt_file_rtsp = None
        current_video_file_rtsp = None
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/stop', methods=['GET'])
def stop():
    global rtsp_process, recording, start_time, srt_thread, stop_srt_thread, current_srt_file_rtsp, current_video_file_rtsp
    try:
        if not recording:
            return jsonify({"success": True})
        
        # Save file paths before clearing globals
        video_path = current_video_file_rtsp
        srt_path = current_srt_file_rtsp
        
        # Stop SRT thread
        stop_srt_thread = True
        if srt_thread:
            srt_thread.join(timeout=2)
        
        # Stop RTSP recording process
        if rtsp_process:
            logger.info("Stopping RTSP recording process gracefully...")
            
            # Send SIGINT (Ctrl+C) to GStreamer for EOS
            rtsp_process.send_signal(signal.SIGINT)
            
            # Wait for the process to handle EOS
            try:
                rtsp_process.wait(timeout=7)
                logger.info("RTSP recording process stopped successfully")
            except subprocess.TimeoutExpired:
                logger.warning("RTSP process did not exit gracefully, force killing")
                rtsp_process.kill()
                rtsp_process.wait()
                logger.info("RTSP recording process force killed")
        
        recording = False
        start_time = None
        rtsp_process = None
        current_srt_file_rtsp = None
        current_video_file_rtsp = None
        
        # Post-processing: Adjust SRT timing to match actual video duration
        if video_path and srt_path and os.path.exists(video_path) and os.path.exists(srt_path):
            logger.info("Starting SRT timing adjustment post-processing...")
            
            # Wait for video file to be fully written and finalized
            time.sleep(3)
            
            # Get actual video duration
            video_duration = get_video_duration(video_path)
            if video_duration:
                logger.info(f"Video duration: {video_duration:.2f} seconds")
                adjust_srt_timing(srt_path, video_duration)
            else:
                logger.warning("Could not determine video duration, SRT timing not adjusted")
        
        logger.info("Recording process stopped successfully")
        return jsonify({"success": True})
    except Exception as e:
        logger.error(f"Error in stop endpoint: {str(e)}")
        recording = False
        start_time = None
        if rtsp_process:
            try:
                rtsp_process.kill()
            except:
                pass
        rtsp_process = None
        current_srt_file_rtsp = None
        current_video_file_rtsp = None
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/status', methods=['GET'])
def get_status():
    global rtsp_process, recording, start_time
    try:
        # Check if RTSP process has died and clean up
        if rtsp_process and rtsp_process.poll() is not None:
            logger.warning("RTSP recording process has died")
            try:
                rtsp_process.kill()
            except:
                pass
            rtsp_process = None
        
        # Stop recording if RTSP process is dead or None
        if not rtsp_process or rtsp_process.poll() is not None:
            if recording:
                logger.info("Recording process has stopped")
                recording = False
                start_time = None
            
        return jsonify({
            "recording": recording,
            "start_time": start_time.isoformat() if start_time else None,
            "rtsp_process_alive": rtsp_process and rtsp_process.poll() is None if rtsp_process else False
        })
    except Exception as e:
        logger.error(f"Error in status endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/list', methods=['GET'])
def list_videos():
    try:
        video_dir = "/app/videorecordings"
        if not os.path.exists(video_dir):
            os.makedirs(video_dir)
            
        videos = [f for f in os.listdir(video_dir) if f.endswith('.mp4')]
        videos.sort(reverse=True)  # Most recent first
        return jsonify({"videos": videos})
    except Exception as e:
        logger.error(f"Error in list endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/download/<filename>')
def download(filename):
    try:
        return send_file(
            os.path.join("/app/videorecordings", filename),
            as_attachment=True
        )
    except Exception as e:
        logger.error(f"Error in download endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

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
    app.run(host='0.0.0.0', port=5423)
