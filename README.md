# SubReels: TowFish

Towed-body photogrammetry survey extension for BlueOS. Second in the
[SubReels](https://github.com/vshie/SubReels) series.

SubReels: TowFish runs on the towed camera body (ArduSub) and coordinates with
an ArduRover tow boat on the surface. It records the towfish camera's RTSP
H.264 or H.265 stream, or captures geotagged 2 Hz stills, and can drive both
automatically from the boat's mission state so one file or image folder is
produced per survey, with numbering including waypoint leg information and time.
EXIF data (PIX4D formatting for Metashape ingestion) of images and a .csv file / subtitle file 
with similar information. 

## About the vehicle

The TowFish is a towed camera body rather than a free-swimming ROV or AUV. It
flies on a tether behind a BlueBoat — or any other surface towing platform — and
the two vehicles work as a pair. The boat contributes position and measures the depth of
water beneath it (via Ping sonar); the fish contributes its own pressure-depth measurement, attitude and imagery.
Coordinating that pair is what this extension exists to do!

Depth is flown against two references at once: a desired depth in the water
column, and the bottom depth under the boat. Working from both is what lets the
fish be held at a useful standoff over terrain that rises and falls, instead of
at a fixed depth below the surface that either loses the bottom or risks
striking it. Bottom depth comes from the boat's Ping sonar (`DISTANCE_SENSOR`
over `mavlink2rest`) and the fish's depth from its own barometer; the widget
shows the two side by side so the operator can fly the difference, with the
depth jog and ALT_HOLD on the same panel.

Every capture — video frame or still — is tagged with where the camera was and
where it was pointing: GPS position, altitude, towfish roll/pitch/yaw, and the
mount pitch derived from the tilt servo. That is what makes the imagery
usable for to-scale model output, rather than no scaling or scaling via known objects / positions in the model. 

### Position estimation

The fish is underwater and carries no GPS of its own, so its position is
inferred from the boat's. Today that inference is a **static offset**: 7 m
behind the boat's GPS fix, along the towfish's heading.

This is a deliberate first approximation, and it is the weakest link in the
geotagging chain — real layback varies with cable scope, tow speed and fish
depth, none of which a fixed distance accounts for. Planned work:

- a richer layback model driven by cable scope, speed and depth rather than a
  constant
- validating whichever model we adopt against an acoustic localization system
  carried on both the TowFish and the BlueBoat, so the estimate can be checked
  against a measured baseline instead of trusted on geometry alone

## Features

- **Transect auto capture** — watches the tow boat's mission over `mavlink2rest`
  and rolls a new file or image folder at every `MISSION_CURRENT.seq` change,
  keeping the RTSP pipeline alive across leg boundaries.
- **Geotagged stills** — 2 Hz JPEGs with GPS position, altitude and full towfish
  attitude written into EXIF/XMP, plus a `telemetry.csv` sidecar per session.
- **Telemetry subtitles** — SRT and ASS sidecars burned from live depth,
  heading, altitude and camera tilt.
- **USB storage with failover** — records to an attached USB drive when one is
  usable and falls back to the local SD mid-session if the drive disappears.
- **Cockpit MFD widget** — a compact black/green panel for field use with
  record, vehicle and optics control.
- **Survey parameter checker** — reads and writes the autopilot parameters that
  a tow survey depends on, on both vehicles, from the setup page.

## Interfaces

The extension serves two separate UIs.

### Setup console (`/`)

Pre-survey configuration only, at full page width:

- tow vehicle IP, recording storage, stream transport and camera snapshot URL
- recorder status, storage state and live telemetry
- the survey parameter checker
- a file browser over the recordings folder

### Cockpit MFD widget (`/widget`)

Everything used while a survey is running — record/timelapse/transect mode
selection, arm/disarm, flight mode, depth jog, camera tilt, zoom, focus and
white balance. Add it to Cockpit as a custom widget pointing at
`http://<vehicle>/"port number from Available Services for the extension"/widget`.

## Survey parameters

The setup console can check and correct the parameters a tow survey depends on.
Each target is editable and persisted, so the values below are a starting point
rather than a requirement — the right numbers shift with hull, tow point and sea
state.

**Tow boat (ArduRover)**

| Parameter | Default | Why |
| --- | --- | --- |
| `TURN_RADIUS` | 2.50 m | Round waypoints tightly enough that the towfish is not dragged across its own track. |
| `WP_PIVOT_ANGLE` | 0 deg | Disables pivot turns so the boat doesn't slow down/stop at every corner. |
| `WP_SPEED` | 0.8–1.1 m/s | Target speed on an AUTO mission leg.|
| `CRUISE_SPEED` | 0.8–1.1 m/s | Kept equal to `WP_SPEED` so the boat does not fight its own mission. |

**Towfish (ArduSub)**

| Parameter | Default | Why |
| --- | --- | --- |
| `ATC_ANG_RLL_P` | 0.00 | Leaves roll passive so the tow cable sets the fish attitude. |
| `ATC_RAT_RLL_D` | 0.0072 | Damps the roll oscillation the cable induces. |
| `ATC_RAT_RLL_FLTE` | 3 Hz | Roll rate error filter cutoff. |
| `ATC_RAT_RLL_FLTD` | 4 Hz | Roll rate derivative filter cutoff. |

Reads and writes go over `mavlink2rest` on each vehicle — the towfish through
`host.docker.internal`, the boat through the configured tow vehicle IP. A write
is only reported as successful once the autopilot echoes back a fresh
`PARAM_VALUE`, so a value that never reached the vehicle cannot look applied.

## Setup

### Prerequisites

- BlueOS on the towfish ([installation guide](https://blueos.cloud/docs/latest/usage/installation/))
- An ArduRover tow boat/gps instance reachable on the same network, running `mavlink2rest`
- A camera publishing an RTSP H.264/H265 stream with JPEG endpoint

### First run

1. Install the extension and open its page from the BlueOS sidebar.
2. Set **Tow Vehicle IP** to the boat's address (default `192.168.2.12`).
3. Set the camera to maximum quality. On a RadCam at `192.168.2.10`
   (`admin` / `blue`): Configuration → Video & Audio → resolution `3840x2160`,
   coding quality at maximum.
4. Choose a recording storage preference. USB is used when a drive with enough
   free space is mounted, otherwise recordings fall back to the local SD.
5. Run **Check All** in the survey parameters panel, adjust any targets, then
   **Apply Mismatched**.

## Storage layout

Recordings live on the host at `/usr/blueos/extensions/subreels_towfish`, mounted
into the container at `/app/videorecordings`, or under `Towfish/` on an attached
USB drive. Configuration persists in `subreels_towfish_config.json` alongside the
recordings.

## HTTP API

| Method | Path | Purpose |
| --- | --- | --- |
| GET | `/status` | Mode, health, storage, vehicle and optics state |
| GET | `/telemetry` | Depth, climb, temperature, lights, GPS, heading |
| GET/POST | `/config` | Read or update persisted configuration |
| GET | `/start`, `/stop` | Manual video recording |
| GET | `/timelapse/start`, `/timelapse/stop` | 2 Hz snapshot capture |
| GET/POST | `/transect/enable`, `/transect/disable` | Mission-triggered capture |
| GET | `/params` | Parameter specs, targets, last readings and job state |
| POST | `/params/check` | Read parameters from the vehicles |
| POST | `/params/apply` | Write saved targets to the vehicles |
| POST | `/params/targets` | Persist edited target values |
| GET | `/list`, `/download/<file>` | Browse and fetch recordings |

The service listens on port 5423, with a WebSocket on 8765 publishing recording
state to the Cockpit data lake.

## Development

```bash
docker compose up --build
```

CI builds and publishes the extension image on every push via
`.github/workflows/deploy.yml`, which needs `DOCKER_USERNAME` and
`DOCKER_PASSWORD` secrets plus the `MY_NAME`, `MY_EMAIL`, `ORG_NAME` and
`ORG_EMAIL` repository variables.

## License

See [LICENSE](LICENSE).
