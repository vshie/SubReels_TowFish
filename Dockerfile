FROM ubuntu:20.04

# Avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install Python and minimal dependencies first
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install GStreamer dependencies in separate steps
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    gstreamer1.0-plugins-good \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    gstreamer1.0-plugins-bad \
    psmisc \
    && rm -rf /var/lib/apt/lists/*

# ffprobe reports each finished part's encoded duration, which is what the
# SRT/ASS/telemetry-CSV sidecars are rescaled against. Without it the
# sidecars keep their wall-clock timing and drift out of sync with the
# video, so the recorder needs it present rather than optional.
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Python GObject Introspection bindings so the recorder can drive
# GStreamer in-process via gi.repository.Gst (replaces gst-launch-1.0
# subprocess; needed for splitmuxsink format-location callbacks +
# bus-monitored auto-restart on RTSP drops).
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    python3-gi \
    python3-gi-cairo \
    gir1.2-gstreamer-1.0 \
    && rm -rf /var/lib/apt/lists/*

# USB filesystem support so the in-container ``mount`` can attach removable
# drives the BlueOS host hot-plugs into the towfish.  vfat (FAT32) is built
# into the kernel; exfat-utils + ntfs-3g cover the two other common formats
# operators use on field USB sticks.  exfat-fuse is intentionally installed
# alongside the userspace utilities even though usb_storage.py prefers the
# kernel exfat driver -- it's harmless when the kernel module is in use.
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    exfat-fuse \
    exfat-utils \
    ntfs-3g \
    && rm -rf /var/lib/apt/lists/*

# Create app directory
WORKDIR /app

# Copy app files
COPY app/ .

# Install Python dependencies
# - piexif: pure-python EXIF reader/writer used to stamp GPS lat/lon/altitude
#   and the towfish heading into every timelapse-mode JPEG.
RUN pip3 install flask requests websockets piexif

# Create directory for video recordings
RUN mkdir -p /app/videorecordings

# Set environment variables
ENV PYTHONUNBUFFERED=1
ENV FLASK_APP=main.py

# Expose ports
EXPOSE 5423 8765

LABEL version="1.0.0"

ARG IMAGE_NAME
LABEL permissions='\
{\
  "ExposedPorts": {\
    "5423/tcp": {},\
    "8765/tcp": {}\
  },\
  "HostConfig": {\
    "Binds": [\
      "/usr/blueos/extensions/subreels_towfish:/app/videorecordings",\
      "/dev/video2:/dev/video2",\
      "/dev:/dev",\
      "/mnt:/mnt:rshared"\
    ],\
    "ExtraHosts": ["host.docker.internal:host-gateway"],\
    "PortBindings": {\
      "5423/tcp": [\
        {\
          "HostPort": ""\
        }\
      ],\
      "8765/tcp": [\
        {\
          "HostPort": ""\
        }\
      ]\
    },\
    "NetworkMode": "host",\
    "Privileged": true\
  }\
}'

ARG AUTHOR
ARG AUTHOR_EMAIL
LABEL authors='[\
    {\
        "name": "Tony White",\
        "email": "tonywhite@bluerobotics.com"\
    }\
]'

ARG MAINTAINER
ARG MAINTAINER_EMAIL
LABEL company='\
{\
        "about": "Towed-body video survey for BlueOS: RTSP recording, geotagged 2 Hz stills, and mission-triggered transect capture driven by an ArduRover tow boat.",\
        "name": "Blue Robotics",\
        "email": "support@bluerobotics.com"\
    }'
LABEL type="tool"
LABEL tags='[\
    "video",\
    "recording",\
    "survey",\
    "towfish",\
    "ardusub",\
    "ardurover"\
]'

ARG REPO
ARG OWNER
LABEL readme='https://raw.githubusercontent.com/vshie/SubReels_TowFish/{tag}/README.md'
LABEL links='\
{\
        "source": "https://github.com/vshie/SubReels_TowFish",\
        "website": "https://bluerobotics.com",\
        "support": "mailto:support@bluerobotics.com"\
    }'
LABEL requirements="core >= 1.1"

# Mark /dev/video2 as a volume
VOLUME ["/dev/video2"]

ENTRYPOINT ["python3", "-u", "/app/main.py"]