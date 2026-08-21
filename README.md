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
depth jog and ALT_HOLD on the same panel. The **SURF TRACK** button on the
widget engages an extension-managed hold that closes that loop automatically,
using the boat's sonar as a one-layback lookahead so the fish begins its
vertical move before the terrain change arrives underneath it.

Every capture — video frame or still — is tagged with where the camera was and
where it was pointing: GPS position, altitude, towfish roll/pitch/yaw, and the
mount pitch derived from the tilt servo. That is what makes the imagery
usable for to-scale model output, rather than no scaling or scaling via known objects / positions in the model. 

### Position estimation

The fish is underwater and carries no GPS of its own, so its position is
inferred from the boat's: every geotag is the tow vehicle's fix pushed
backwards by a **static offset**, defaulting to 7 m. Both the distance and the
heading it is laid back along are set in the setup console and persist across
restarts.

The heading source matters whenever the fish and the boat are not aligned —
turns, crosswind, cross-current:

| Source | Use when |
| --- | --- |
| `towfish` (default) | the fish tracks straight behind and the boat is being pushed off its course |
| `boat` | the fish is yawing on the tether but the tow direction is steady |
| `average` | a compromise between the two, taken as a circular mean so it stays correct across north |

The console also carries a layback calculator: enter tether deployed and
average towfish depth and it solves the right triangle for the horizontal leg,
which is the offset from the tow point. A straight tether is the best case, so
that result is an upper bound — real cable sags and the fish rides closer in.

This is still a deliberate first approximation, and it remains the weakest link
in the geotagging chain, since real layback varies continuously with cable
scope, tow speed and depth. Planned work:

- a layback model that tracks scope, speed and depth live rather than holding a
  configured constant
- validating whichever model we adopt against an acoustic localization system
  carried on both the TowFish and the BlueBoat, so the estimate can be checked
  against a measured baseline instead of trusted on geometry alone

## Features

- **Transect auto capture** — watches the tow boat's mission over `mavlink2rest`
  and rolls a new file or image folder at every `MISSION_CURRENT.seq` change,
  keeping the RTSP pipeline alive across leg boundaries.
- **Geotagged stills** — 2 Hz JPEGs with GPS position, height above the seabed
  and full towfish attitude written into EXIF plus Pix4D-namespace XMP
  (`Camera:Yaw/Pitch/Roll` and `GPSXYAccuracy`/`GPSZAccuracy`, read by Agisoft
  Metashape, Pix4D and WebODM), RadCam IMX678 lens tags (`FocalLength`
  3.6–11 mm, 2.0 µm `FocalPlane` pitch, `FocalLengthIn35mmFilm`) so Metashape
  does not assume a 50 mm 35 mm-equivalent lens, and a `telemetry.csv`
  sidecar per session.
- **Height above the seabed** — the towfish has no altimeter, so altitude is
  `boat sonar − towfish depth − offset`, with the sounding replayed at the tow
  delay (layback ÷ boat speed) so both instruments describe the same patch of
  ground. The offset is set under TOW GEOMETRY and absorbs the sonar
  transducer's depth below the waterline. This is what lands in EXIF
  `GPSAltitude`; because it follows the bathymetry rather than a fixed datum,
  `GPSZAccuracy` is published alongside so the solver does not over-trust it.
- **SURF TRACK altitude hold** — an extension-managed flight mode alongside
  MANUAL, STABILIZE and ALT_HOLD that holds a set altitude over the bottom.
  The boat's Ping sonar leads the fish by the layback, so the loop uses the
  *instantaneous* sounding to command the depth the fish will need one tow
  delay from now — the fish starts moving before the terrain change arrives.
  Inside a configurable deadband it releases RC3 and lets the autopilot's
  own ALT_HOLD keep the fish put. When the sonar loses bottom lock, the boat
  drifts, the fish is at the surface or the autopilot leaves ALT_HOLD, the
  loop cleanly hands the stick back to ALT_HOLD until the conditions clear
  for a full 2 s. Operator UP/DOWN always wins outright: a hold briefly
  suspends the loop rather than disengaging it, so a brushed touchscreen
  button cannot silently kill the mode mid-survey.
- **Telemetry sidecars for video** — every recording gets a `*_telemetry.csv`
  at 5 Hz carrying the same fields as the stills path, so frames extracted
  later with `extract_geotagged_frames.py` come out with identical metadata.
  SRT and ASS subtitle sidecars are written alongside it for players.
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
- tow geometry: layback distance, heading source and the layback calculator
- the survey parameter checker
- a file browser over the recordings folder

### Cockpit MFD widget (`/widget`)

Everything used while a survey is running — record/timelapse/transect mode
selection, arm/disarm, flight mode, depth jog, camera tilt, zoom, focus and
white balance. Add it to Cockpit as a custom widget pointing at
`http://<vehicle>/"port number from Available Services for the extension"/widget`.

## ArduRover & ArduSub configuration

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

### Installation

#### Option A — Manual install in BlueOS (recommended)

Every push to `main` publishes a new image to Docker Hub via the `Deploy BlueOS
Extension Image` GitHub Action (see `.github/workflows/deploy.yml`). The action
prepends `blueos-` to the configured image name, so the published image is
`vshie/blueos-subreels_towfish`, not `vshie/subreels_towfish`.

1. Open BlueOS in your browser.
2. Go to **Extensions → Installed Extensions** and click the **+** button in
   the bottom-right.
3. Fill in the dialog:
   - **Extension Identifier:** `vshie.subreels_towfish`
   - **Extension Name:** `SubReels: TowFish`
   - **Docker image:** `vshie/blueos-subreels_towfish`
   - **Docker tag:** `main`
   - **Permissions:** copy and paste the JSON block below verbatim.

   ```json
   {
     "ExposedPorts": {
       "5423/tcp": {},
       "8765/tcp": {}
     },
     "HostConfig": {
       "Binds": [
         "/usr/blueos/extensions/subreels_towfish:/app/videorecordings",
         "/dev/video2:/dev/video2",
         "/dev:/dev",
         "/mnt:/mnt:rshared"
       ],
       "ExtraHosts": ["host.docker.internal:host-gateway"],
       "PortBindings": {
         "5423/tcp": [
           {
             "HostPort": ""
           }
         ],
         "8765/tcp": [
           {
             "HostPort": ""
           }
         ]
       },
       "NetworkMode": "host",
       "Privileged": true
     }
   }
   ```

4. Click **Create**. BlueOS pulls the image and starts the container.
5. Once it shows as running, open it from the BlueOS sidebar.

That block is identical to the `LABEL permissions` baked into the image. It
tells BlueOS to:

- **Bind `/usr/blueos/extensions/subreels_towfish` → `/app/videorecordings`**
  so recordings and `subreels_towfish_config.json` persist on the host and show
  up in the BlueOS file browser at
  `http://<host>:7777/files/extensions/subreels_towfish/`.
- **Bind `/dev` and `/mnt:rshared`** so USB drives the host hot-plugs appear
  inside the container and stay visible after a remount. Without the `rshared`
  propagation, a stick inserted after the container started would never show up.
- **Use host networking, privileged mode and `host.docker.internal`** so the
  container can reach `mavlink2rest` on the towfish, plus the tow boat across
  the network.
- **Expose 5423 and 8765** — the web UI and the Cockpit data-lake WebSocket.

The `/dev/video2` bind is inherited from the original video recorder and is not
used here, since this extension captures over RTSP. Drop that one line if your
towfish has no `/dev/video2`.

#### Option B — Build a local image and install from file

To test changes before they reach Docker Hub, build locally, export a `.tar`,
and upload it directly:

1. Build and save the image:

   ```bash
   git clone https://github.com/vshie/SubReels_TowFish.git
   cd SubReels_TowFish
   docker build -t blueos-subreels_towfish:local .
   docker save -o blueos-subreels_towfish-local.tar blueos-subreels_towfish:local
   ```

2. Copy the `.tar` to the machine with BlueOS open in a browser.
3. In BlueOS, go to **Extensions → Installed Extensions** and click the **+**
   button in the bottom-right.
4. Choose **Install from file** and select the `.tar` you just built.
5. When prompted, reuse the same permissions JSON from Option A.

### First run

1. Install the extension and open its page from the BlueOS sidebar.
2. Set **Tow Vehicle IP** to the boat's address (default `192.168.2.12`).
3. Set the camera to maximum quality. On a RadCam at `192.168.2.10`
   (`admin` / `blue`): Configuration → Video & Audio → resolution `3840x2160`,
   coding quality at maximum.
4. Choose a recording storage preference. USB is used when a drive with enough
   free space is mounted, otherwise recordings fall back to the local SD.
5. Set the tow geometry. Use the layback calculator if you know the tether
   length and working depth, and pick the heading source that matches how the
   fish tracks behind the boat.
6. Run **Check All** in the survey parameters panel, adjust any targets, then
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
| POST | `/vehicle/surftrack` | Enable/disable the altitude-over-bottom hold |
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
