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

# Python GObject Introspection bindings so the recorder can drive
# GStreamer in-process via gi.repository.Gst (replaces gst-launch-1.0
# subprocess; needed for splitmuxsink format-location callbacks +
# bus-monitored auto-restart on RTSP drops).
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    python3-gi \
    python3-gi-cairo \
    gir1.2-gstreamer-1.0 \
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

LABEL version="0.9"

ARG IMAGE_NAME
LABEL permissions='\
{\
  "ExposedPorts": {\
    "5423/tcp": {},\
    "8765/tcp": {}\
  },\
  "HostConfig": {\
    "Binds": [\
      "/usr/blueos/extensions/videorecorder:/app/videorecordings",\
      "/dev/video2:/dev/video2"\
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
        "about": "",\
        "name": "Blue Robotics",\
        "email": "support@bluerobotics.com"\
    }'
LABEL type="tool"

ARG REPO
ARG OWNER
LABEL readme=''
LABEL links='\
{\
        "source": ""\
    }'
LABEL requirements="core >= 1.1"

# Mark /dev/video2 as a volume
VOLUME ["/dev/video2"]

ENTRYPOINT ["python3", "-u", "/app/main.py"]