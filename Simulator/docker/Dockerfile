# Towfish simulator image: ArduSub SITL built from source + the towsim runtime.
#
# The fish flew ArduSub V4.7.2 (fb773567). ARDUPILOT_REF selects the source;
# the build stage prints the commit it got so a mismatch is visible.
#
#   docker compose -f Simulator/docker/docker-compose.yml build

FROM ubuntu:22.04 AS build
ARG ARDUPILOT_REF=Sub-4.7.2
ARG EXPECTED_SHA=fb773567
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        git ca-certificates build-essential g++ python3 python3-pip python3-dev \
        libxml2-dev libxslt1-dev pkg-config \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir empy==3.3.4 pexpect future lxml pymavlink
RUN git clone --depth 1 --branch "${ARDUPILOT_REF}" --recurse-submodules --shallow-submodules \
        https://github.com/ArduPilot/ardupilot.git /ardupilot
WORKDIR /ardupilot
RUN echo "ardupilot $(git rev-parse --short=8 HEAD) (expected ${EXPECTED_SHA})" | tee /ardupilot_version.txt
RUN ./waf configure --board sitl && ./waf sub

FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip git ca-certificates \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir numpy pymavlink pyyaml matplotlib
COPY --from=build /ardupilot/build/sitl/bin/ardusub /usr/local/bin/ardusub
COPY --from=build /ardupilot_version.txt /ardupilot_version.txt
ENV ARDUSUB_BIN=/usr/local/bin/ardusub \
    TOWSIM_LOG_DIR=/logs \
    PYTHONUNBUFFERED=1
WORKDIR /sim
ENTRYPOINT ["python3", "-m", "towsim.run"]
