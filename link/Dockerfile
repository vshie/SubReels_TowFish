# SubReels: TowFish Link -- install on the tow boat's BlueOS, not the fish.
# Built from the repo root (see .github/workflows/deploy.yml), so paths are
# relative to it.
FROM python:3.11-alpine

COPY link/app.py link/index.html link/icon.png /app/
RUN mkdir -p /app/data

ENV PYTHONUNBUFFERED=1
EXPOSE 5424

LABEL version="0.99"

ARG IMAGE_NAME
LABEL permissions='\
{\
  "ExposedPorts": {\
    "5424/tcp": {}\
  },\
  "HostConfig": {\
    "Binds": [\
      "/usr/blueos/extensions/subreels_towfish_link:/app/data"\
    ],\
    "PortBindings": {\
      "5424/tcp": [\
        {\
          "HostPort": ""\
        }\
      ]\
    },\
    "NetworkMode": "host"\
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
        "about": "Install on the tow boat: adds the SubReels TowFish widget to the Cockpit of the boat, loaded from the towfish over the network.",\
        "name": "Blue Robotics",\
        "email": "support@bluerobotics.com"\
    }'
LABEL type="tool"
LABEL tags='[\
    "towfish",\
    "cockpit",\
    "widget"\
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

ENTRYPOINT ["python3", "-u", "/app/app.py"]
