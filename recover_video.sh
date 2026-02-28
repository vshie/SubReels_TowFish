#!/bin/bash
#
# Recovery script for broken MP4 files (missing moov atom).
#
# Usage:
#   1. Connect to the same camera network
#   2. Run: ./recover_video.sh <broken_file.mp4>
#
# This script will:
#   a) Record a 5-second reference clip from the camera's RTSP stream
#   b) Use untrunc to rebuild the broken file's moov atom
#
# Prerequisites:
#   - ffmpeg installed (brew install ffmpeg)
#   - untrunc built at /tmp/untrunc/untrunc (already done)
#   - Camera reachable at 192.168.2.10

set -euo pipefail

RTSP_URL="rtsp://admin:blue@192.168.2.10:554/stream_0"
UNTRUNC="/tmp/untrunc/untrunc"
BROKEN_FILE="${1:?Usage: $0 <broken_file.mp4>}"

if [ ! -f "$BROKEN_FILE" ]; then
    echo "Error: File not found: $BROKEN_FILE"
    exit 1
fi

if [ ! -x "$UNTRUNC" ]; then
    echo "Error: untrunc not found at $UNTRUNC"
    echo "Build it: cd /tmp/untrunc && CPATH=/opt/homebrew/Cellar/ffmpeg/8.0_2/include LIBRARY_PATH=/opt/homebrew/Cellar/ffmpeg/8.0_2/lib make"
    exit 1
fi

REFERENCE="/tmp/recovery_reference_$(date +%s).mp4"

echo "=== Step 1: Recording 5-second reference clip from camera ==="
echo "    RTSP: $RTSP_URL"
ffmpeg -y -rtsp_transport tcp -i "$RTSP_URL" -t 5 -c copy "$REFERENCE" 2>&1 | tail -3

if [ ! -s "$REFERENCE" ]; then
    echo "Error: Failed to record reference clip. Is the camera reachable?"
    rm -f "$REFERENCE"
    exit 1
fi

echo ""
echo "=== Step 2: Verifying reference clip ==="
ffprobe -v error -show_entries stream=codec_name,width,height,r_frame_rate -of csv=p=0 "$REFERENCE"

echo ""
echo "=== Step 3: Running untrunc recovery ==="
DYLD_LIBRARY_PATH=/opt/homebrew/Cellar/ffmpeg/8.0_2/lib "$UNTRUNC" "$REFERENCE" "$BROKEN_FILE"

FIXED_FILE="${BROKEN_FILE}_fixed.mp4"
if [ -s "$FIXED_FILE" ]; then
    echo ""
    echo "=== Recovery complete! ==="
    echo "Fixed file: $FIXED_FILE"
    echo ""
    ffprobe -v error -show_entries format=duration,size -show_entries stream=codec_name,width,height,r_frame_rate -of default "$FIXED_FILE"
else
    echo ""
    echo "Error: Recovery failed - output file is empty or missing"
fi

rm -f "$REFERENCE"
