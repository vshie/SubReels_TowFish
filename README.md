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
inferred from the boat's. Two models, chosen in the setup console:

- **Tether model** (default). From the tether length, the fish's depth and its
  compass heading. On a straight leg the fish sits
  `0.985 × √(L² − depth²) + 0.43 m` behind the boat's tow point: the tether
  runs almost straight, and the fish's body is 0.43 m behind its tow point.
  Through a turn the tether is modelled as a short chain of links, each
  dragging the next (2 links on an 8 m tether, 3 on 16 m, 4 on 25 m). So the
  fish cuts inside the boat's track, and coasts along its own heading when the
  boat turns back and the tether goes slack (`app/towed_position.py`).
  Scored against the simulator with realistic boat-GPS and compass error, the
  p95 error through hairpins is about 3 m on 8–16 m of tether and 5.5 m on
  25 m. A static 7 m offset was 7–21 m off there. Every photo's
  `GPSXYAccuracy` carries the model's own 1-sigma estimate, larger in turns,
  so photogrammetry down-weights turn frames. The model needs **tether
  length**; until that is set, or if the boat fix drops out, the static offset
  is used. SURF TRACK's sonar look-ahead uses the model's distance along the
  boat's track to the fish.
- **Static offset.** The boat's fix pushed backwards by a fixed distance
  (default 7 m) along a chosen heading.

**Line spacing** is read from the tow boat's mission over its mavlink2rest, or
worked out from the waypoints it flies, and can be set by hand instead. The
widget warns when the lines are tighter than the simulated minimum for the
tether: about 12 m on 8 m of tether, 35 m on 25 m.

For the static offset, the heading source matters whenever the fish and the
boat are not aligned — turns, crosswind, cross-current:

| Source | Use when |
| --- | --- |
| `towfish` (default) | the fish tracks straight behind and the boat is being pushed off its course |
| `boat` | the fish is yawing on the tether but the tow direction is steady |
| `average` | a compromise between the two, taken as a circular mean so it stays correct across north |

The console also carries a layback calculator for the static offset: enter the
tether deployed and the average towfish depth, and it solves the right triangle
for the horizontal leg.

Planned work:

- validating whichever model we adopt against an acoustic localization system
  carried on both the TowFish and the BlueBoat, so the estimate can be checked
  against a measured baseline instead of trusted on geometry alone

## Features

- **Transect auto capture** — watches the tow boat's mission over `mavlink2rest`
  and rolls a new file or image folder at every `MISSION_CURRENT.seq` change,
  keeping the RTSP pipeline alive across leg boundaries. Per-leg capture
  defaults to timelapse stills on every start; video is opt-in per survey.
  An armed transect survives a restart: if the Pi resets mid-survey, the
  monitor re-arms itself with the same capture type when the extension comes
  back, and only DISABLE TRANSECT clears it.
- **Geotagged stills** — 2 Hz JPEGs with GPS position, height above the seabed
  and full towfish attitude written into EXIF plus Pix4D-namespace XMP
  (`Camera:Yaw/Pitch/Roll` and `GPSXYAccuracy`/`GPSZAccuracy`, read by Agisoft
  Metashape, Pix4D and WebODM), BR 4k Cam / IMX678 lens tags (`FocalLength`
  3.6–11 mm, 2.0 µm `FocalPlane` pitch, `FocalLengthIn35mmFilm`) so Metashape
  does not assume a 50 mm 35 mm-equivalent lens, and a `telemetry.csv`
  sidecar per session (`telemetry_2.csv` if USB failover starts a new
  part on the SD card, so the two files can be copied together).
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
  - **Dive gates, never a climb.** Attitude never makes the loop command up.
    A slow boat, a stalled dive, wings at their down limit (SERVO5/6
    collective) or more than 35° of roll each refuse *down* commands and pin
    the commanded depth to the fish until they clear. Pitch drives no
    decision: the fish tows nose-down and dives steeper, so no pitch limit
    separates trouble from normal flight.
  - **Auto-righting** (its own toggle; runs in ALT_HOLD whether or not
    SURF TRACK is on). If the fish rolls past 60° for half a second it flies
    the pilot's drill in MANUAL straight away, in the corner, without waiting
    for the boat to straighten up: vertical stick at neutral (set in the
    config) plus roll stick toward upright the short way, centred once the
    fish is within 20°. Waiting cost depth: inverted in ALT_HOLD, the wings
    sit at full down, which flies the fish to the surface, and a neutral
    vertical stick leaves the wings their full differential for the roll
    snap. Once upright it switches to ALT_HOLD and descends gradually back
    to the depth the fish had when the boat reached the turn's first
    waypoint, easing off whenever the boat drops below 0.9 m/s. If the drill
    has not worked in 30 s it tries STABILIZE for up to 60 s. SURF TRACK
    holds while this runs and resumes afterwards only if it was already on.
    Any operator jog or mode change cancels it. It also counts tether twist
    (net full rolls, whoever is flying) for the widget and the log, with a
    reset button for after untwisting. The shore-launch lift guard (full up
    whenever the fish is armed in MANUAL) stands down while a rescue is
    flying, since the drill uses MANUAL on purpose; it takes over again for
    any other armed MANUAL.
  - **Logged in the BIN.** Controller state, dive gate, commanded depth,
    altitude and RC3 are sent to the fish as NAMED_VALUE_FLOAT (`ST_STATE`,
    `ST_GATE`, `ST_TGT`, `ST_ALT`, `ST_RC3`), so they land in the
    autopilot's own log as `NVAL` beside the attitude they explain.
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
selection, arm/disarm, flight mode, depth jog, camera tilt and white balance.
Cockpit lists it with the other extension widgets in edit mode, so adding it
is a drag onto a view:

- **Cockpit connected to the tow boat** (the normal setup): Cockpit only asks
  the BlueOS of the vehicle it is connected to, so install the small
  **SubReels: TowFish Link** extension on the boat as well. It advertises the
  widget with an iframe URL on the fish (default `192.168.2.2:5423`; change it
  on the link's own page in BlueOS, which also shows whether the fish is
  reachable). The widget then loads straight from the fish over the network.
  See [Tow boat: the Cockpit widget link](#tow-boat-the-cockpit-widget-link)
  for the install steps.
- **Cockpit connected to the towfish** (bench work): the TowFish extension
  advertises the widget itself, no link needed.

Without either, add a custom iframe widget pointing at
`http://<towfish>:5423/widget`.
Zoom and focus controls (plus their presets) are gated by the
`HAS_ZOOM_FOCUS` flag in `app/main.py` and hidden by default for the
fixed-lens 4k Cam; flip the flag to `True` to restore them for a
variable-zoom RadCam build.

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
| `ATC_ANG_RLL_P` | 1.5 | The only term that pushes the fish back to level. At 0 the cable sets roll on a straight line, but every hairpin turn on the 4 Nov 2026 tow rolled the fish inverted. The widget and setup console warn whenever the fish, or the saved target, is at 0. |
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

#### Tow boat: the Cockpit widget link

Cockpit only lists widgets from the BlueOS of the vehicle it is connected
to. In the field that is the tow boat, not the towfish, so the boat needs a
second, much smaller extension that tells its Cockpit where the towfish
widget is. It's a different image from the towfish extension above and goes
on the **boat**, not the fish. The same GitHub Action builds it from `link/`
on every push to `main`.

| | Towfish (above) | Tow boat link |
| --- | --- | --- |
| Install on | Towfish BlueOS | Tow boat BlueOS |
| Docker image | `vshie/blueos-subreels_towfish` | `vshie/blueos-subreels_towfish_link` |
| Port | 5423 (+ 8765) | 5424 |

1. Open the **boat's** BlueOS and go to **Extensions → Installed Extensions →
   +**.
2. Fill in the dialog:
   - **Extension Identifier:** `vshie.subreels_towfish_link`
   - **Extension Name:** `SubReels: TowFish Link`
   - **Docker image:** `vshie/blueos-subreels_towfish_link`
   - **Docker tag:** `main`
   - **Permissions:**

   ```json
   {
     "ExposedPorts": {
       "5424/tcp": {}
     },
     "HostConfig": {
       "Binds": [
         "/usr/blueos/extensions/subreels_towfish_link:/app/data"
       ],
       "PortBindings": {
         "5424/tcp": [
           {
             "HostPort": ""
           }
         ]
       },
       "NetworkMode": "host"
     }
   }
   ```

3. Click **Create**, then open **SubReels: TowFish Link** from the boat's
   BlueOS sidebar. It should report the towfish as reachable. If the towfish
   isn't at `192.168.2.2:5423`, enter its address there and click **Save**.
4. Connect Cockpit to the boat, open edit mode, and drag **SubReels TowFish**
   from the extension widgets onto a view. The widget loads directly from the
   towfish over the network.

When Cockpit is connected straight to the towfish (bench work), the link isn't
needed: the towfish extension lists the widget itself.

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
5. Set the tow geometry: enter the **tether length** (the tether model needs
   it), and the line spacing if the boat's mission can't be read. With the
   static offset instead, use the layback calculator and pick the heading
   source that matches how the fish tracks behind the boat.
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
| POST | `/vehicle/tilt`, `/vehicle/tilt-down` | Step the held camera tilt 3 µs of servo travel (~0.3°) up/down, or reset it to straight down |
| GET | `/params` | Parameter specs, targets, last readings and job state |
| POST | `/params/check` | Read parameters from the vehicles |
| POST | `/params/apply` | Write saved targets to the vehicles |
| POST | `/params/targets` | Persist edited target values |
| GET | `/list`, `/download/<file>` | Browse and fetch recordings |
| GET | `/cockpit_extras.json` | Widget entry for a Cockpit connected to the towfish |

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
