# Towfish simulator

Runs the towfish's real autopilot firmware, **ArduSub 4.7.2 (`Sub-4.7.2`, commit
`fb773567`)**, in SITL against a physics model of the towfish. The fish is built from
`towfishRevC.step` (see `towfish_geometry_report.md`) and towed along the boat's logged
GPS track from the 4 Nov 2026 test. The goal is to see whether that day's behaviour
(roll-outs and inversions in hairpin turns, clean straight legs) comes out of the physics
on its own, and then to try fixes against it.

The simulator is separate from the BlueOS extension. Nothing here is imported by
`app/`, the extension image copies only `app/`, and the root `.dockerignore` keeps this
folder out of the extension's build context.

## Quick start

The flight logs it replays are not in the repository. Put `towboat_00000110.BIN` and
`towfish_00000103.BIN` from the 4 Nov 2026 test in a `Log review/` folder at the repo
root (the container mounts it read-only), or point `TOWSIM_LOG_DIR` at wherever they
live.

```bash
# one-time: build ArduSub SITL from source + the python runtime (~10 min)
docker compose -f Simulator/docker/docker-compose.yml build

# a straight leg and a single 90-degree turn, autopilot alone in ALT_HOLD
docker compose -f Simulator/docker/docker-compose.yml run --rm --service-ports \
    towsim --start 10:28 --end 10:31 --pilot none

# the hairpin section, replaying the operator's logged mode changes and RC3
docker compose -f Simulator/docker/docker-compose.yml run --rm --service-ports \
    towsim --start 10:31:30 --end 10:48 --pilot replay

# score a run against the real fish (host or container)
python3 Simulator/analysis/compare.py Simulator/runs/<run-id>
```

While a run is going, QGroundControl or Cockpit can connect to `tcp://localhost:5762` to
watch the simulated fish.

To make the interactive 3D turn viewer, run with `--log-cable`, then export a time
range:

```bash
R="docker compose -f Simulator/docker/docker-compose.yml run --rm towsim --start 10:31:30 --end 10:35:30 --init-depth 5 --log-cable"
$R --pilot replay --run-id case1_friday                                    # as flown
$R --pilot none --param-file params/scenarios/roll_p_1_5.parm --run-id case2_rollp
$R --extension  --param-file params/scenarios/roll_p_1_5.parm --run-id case3_ext
python3 Simulator/analysis/export_viz.py --from 10:31:55 --to 10:35:00 \
    --case Simulator/runs/case1_friday --case Simulator/runs/case2_rollp --case Simulator/runs/case3_ext \
    --out Simulator/runs/viz_hairpin1/turn_viewer.html
```

(In zsh, run each command written out in full rather than through `$R`.)

`--extension` flies the extension's own `SurfTrackController` and `AutoRightController`,
imported unmodified from `app/main.py` (`towsim/extension_bridge.py`). Their depth,
attitude, mode and wing outputs come from SITL; the boat's sounding and speed come from
the log. The boat's mission progress, heading and bearing to its next waypoint come from
the track: the log's "Reached waypoint" times, or the corners of a synthetic hairpin. On
the logged track the bearing is taken as the direction to where the boat is at its next
"reached" event; that approximates ArduRover's target bearing, since waypoint positions
aren't logged. RC2/RC3 and mode commands go back to SITL. Everything runs on the sim
clock.

Useful `towsim.run` options:

| Option | Effect |
|---|---|
| `--set cable.length=25` | Override one `config/towfish.yaml` value for this run. Recorded in `meta.json` and shown in `compare.md`. |
| `--param-file my.parm` | Layer extra ArduSub parameters on top of the as-flown set, e.g. `ATC_ANG_RLL_P,1.5`. |
| `--init-depth 4` | Depth the fish is held at while the cable settles before `--start`. |
| `--spacing 12` | Replace the logged boat track with the window's first hairpin rebuilt at this lane spacing. Uses the logged approach heading and speed profile and the boat's 3.5 m `TURN_RADIUS`. The boat's sounding is still replayed by time. |
| `--turn-speed 1.25 --turn-speed-hold 35` | With `--spacing`: hold the boat at this speed through its turn, and for at least the hold time from the turn's start. The fish turns roughly layback / speed after the boat (~6 s on the 8 m tether). Speed changes stay within the boat's `ATC_ACCEL_MAX` / `ATC_DECEL_MAX`. |
| `--extension` | Fly the extension's own `SurfTrackController` and `AutoRightController` (implies `--pilot none`). |
| `--autoright-only` | Auto-righting on, SURF TRACK off: plain ALT_HOLD with the rescue watching. |
| `--ext-set autoright_vertical_pwm=1700` | Set an `app/main.py` module variable for this run (stick amounts, `surf_max_depth_m`, trigger constants). Recorded in `meta.json`. |
| `--straight 1.0` | Replace the boat track with a straight tow at this constant speed on the logged heading. Used for the max-depth study. |
| `--bottom-depth 12` | With `--extension`: a flat bottom at this depth instead of the logged sounding, so SURF TRACK holds a chosen depth. |
| `--sitl none` | Physics only, wings neutral. A smoke test for the model, not a result. |

## Studies (`analysis/sweep.py`)

`sweep.py` runs batches of the runs above, six docker containers at a time. It scores
each run from `truth.csv` and keeps `runs/sweep/results_<study>.json`; re-running skips finished
run IDs.

| Study | Command | What it finds |
|---|---|---|
| Righting drill | `sweep.py righting` | Best vertical stick and roll-stick amount for auto-righting. Ranked by time inverted, then depth kept. |
| Max depth | `sweep.py maxdepth` | Deepest the fish holds on a straight 1 m/s tow, per tether length (wings saturated down). |
| Line spacing | `sweep.py spacing --vertical-pwm V --roll-frac R` | Narrowest hairpin spacing with \|roll\| < 45° and no auto-righting, per tether length and working depth (4, 8 and 12 m). |

`analysis/position_eval.py` scores the extension's towfish position estimators
(`app/towed_position.py`, the exact code it ships) against every run with a cable log. The boat
tow point is cable node 0 and the truth is `truth.csv` north/east. The estimators get only what
the extension reads (boat fix, course and speed; fish depth and compass heading; the tether
length) at 5 Hz. Options add drifting boat-GPS error, compass noise and bias. Runs flown with
`--extension` also log the extension's live estimate as `est_n`, `est_e`, `est_unc` in
`truth.csv`.

The tether (`towsim/cable.py`) has a Kevlar strength member, so it is modelled as fixed-length
links that carry tension only: drag and wet weight move the nodes, then one exact
tridiagonal solve per physics step finds the link tensions that stop any taut link lengthening,
with an active-set pass for slack links. The fish end uses the fish's effective mass at its tow
point (rigid body plus added mass), so a slack-to-taut snap hits the fish at full strength.
`--set cable.model=spring` restores the original numerical spring (EA 5000 N) for comparison.
SITL's physics frames are shorter than `sim.physics_dt`, so they set the step.

`analysis/tether_statics.py` is a steady towed-cable solver. It takes the fish-end force
from the max-depth runs and gives depth for any tether length, with C_n, salt-water and
speed bands. `tests/test_physics.py` checks it against the lumped cable.

## How it fits together

```
towsim.run ── spawns ──> ardusub (SITL, JSON model, lockstep)
   │  <── servo PWM (UDP 9002) ──┤
   │  ── IMU/pose/velocity ──>   │     one clock: the physics drives SITL time
   │  ── MAVLink tcp:5760 ──>    │     arm, DO_SET_MODE, RC3 override (as GCS 255/240)
   ├─ boat_track:   boat GPS from towboat_00000110.BIN -> tow point (NED)
   ├─ cable:        inextensible lumped-mass tether (Kevlar: fixed-length links,
   │                tension only), normal + tangential drag, wet weight
   ├─ fish:         6-DOF rigid body + added mass, buoyancy at CB, gravity at CG
   ├─ hydro:        strip-theory wings/tail (servo PWM -> deflection), body drag
   └─ pilot_replay: operator's mode commands + RC3 from towfish_00000103.BIN
```

| File | What |
|---|---|
| `config/towfish.yaml` | Every physical constant, with its status and source. |
| `params/towfish_as_flown.parm` | The fish's parameters from the 4 Nov log (regenerate with `params/extract_params.py`). |
| `params/sitl_overrides.parm` | SITL-only replacements for missing hardware (no GPS, no leak sensor, ...). |
| `analysis/servo_map.py` | Checks the wing wiring in the config against the log. |
| `analysis/compare.py` | Straight-leg and turn scores, sim vs real, written to `compare.md` and `compare.png`. |
| `tests/test_physics.py` | Frames, foils, cable, rigid body and JSON-bridge checks. |
| `runs/<id>/` | `truth.csv` (physics state, 10 Hz), `meta.json`, `sitl.log`, `sitl.BIN`, `compare.*`. |

## Not cheating

The point is to find out whether the turn behaviour *emerges*. These rules keep the answer
honest:

1. **Constants have a source.** Every constant in `config/towfish.yaml` comes from the STEP
   model, the owner, the literature, or a stated derivation, and carries that status.
2. **What may be adjusted.** `estimate` / `ASSUMED` values may move only inside their
   `bounds`, and only:
   - when the owner measures them; or
   - against the **straight-leg** block of `compare.md`.
3. **Turns are held out.** The **turn** block judges the model and is never a target. If
   turns don't match, that is a finding about the physics, not something to tune away.
4. **What enters a run.** A run is fed only the boat's track, the operator's inputs (mode
   commands, RC3) and the as-flown parameters. The real fish's attitude and depth are read
   only by `compare.py`.
5. **Records.** Each run records its config hash, overrides and git SHA.

The wing wiring (`wings.port_servo` etc.) is `log-inferred`: it comes from MANUAL full-up
PWMs and the roll controller's output sign, not from turn behaviour. `servo_map.py`
re-derives it and fails if the config disagrees. Result: SERVO5 is port, and its PWM goes
*down* for more lift.

## Open items: the owner can supply these

Until these are measured, the sim runs on flagged estimates. The first two dominate turn
behaviour.

| # | What | Why it matters | How |
|---|---|---|---|
| 1 | Tether: effective drag coefficient | Sets the max depth on long tethers. Now a Blue Robotics Fathom, 7.5 mm, 8 m (owner, 5 Nov), neutral in fresh water. C_n is 1.2 (literature) with a 1.2–1.8 band for strumming. | A tow test reading tether angle at two speeds would pin C_n. |
| 2 | Tow rod: height of the tow point above the bar; rigid or pivoting | The lever arm for the cable's side pull: roll moment in a turn. Currently 100 mm, rigid. | Tape measure plus a photo. |
| 3 | CG height (and lateral offset) | Roll stiffness. With the housing contents on the tube axis, the CG lands **~1.5 mm above the CB**, so the fish is statically roll-unstable without control. Open-loop sims show a slow roll-off with a ~15 s time constant. | Hang test: hang from the tow point and photograph the trim. Roll test: let it rest in water and time the roll period. |
| 4 | Wing servo: degrees per µs, limits, which servo is port | Converts the autopilot's PWM into lift. Currently 0.09 °/µs, ±45 °. | Photo or video of a wing at 1100 / 1500 / 1900 µs. |
| 5 | Axis directions: is +X up and +Z starboard in the STEP? | Frame mapping. | Confirm from the model or a photo. |
| 6 | Water: salt or fresh; current at the site | Buoyancy (+86 g in salt vs neutral in fresh); through-water speed. | |

## First results (5 Nov 2026, config `ASSUMED` values as committed)

**`10:28–10:31`, `--pilot none`** (a straight leg plus the single 90° turn the real fish
survived):
- ArduSub held the simulated fish level and at depth through the straight leg and the
  90° turn.
- Max roll was 2°; the real fish reached 40° and did not invert.

**`10:31:30–10:48`, `--pilot replay`** (the hairpin section, operator inputs replayed
open-loop):
- **First hairpin (10:32, both fish starting at ~5.2 m): the failure emerges without
  any turn tuning.**
  - The fish climbs and ALT_HOLD drives the wing collective to its down limit (sim
    +20 s / real +16 s after the turn starts).
  - It then rolls past 90° (sim +29 s / real +19 s) and surfaces inverted.
- Every later hairpin also inverts in the sim, but from the wrong starting depth. The
  replayed operator inputs were timed for the real fish, and after the first upset the
  sim's state had diverged.
- **The main mismatch is a straight-leg one.** Like for like (both from ATT, with
  `AHRS_TRIM` applied):
  - the sim tows at about **−5° pitch vs −14° real** and needs ~0 µs of collective vs
    −60 µs real;
  - after a MANUAL/STABILIZE recovery the real fish glides back to ~4 m, while the sim
    levels out near 1 m.
  The ASSUMED tow-rod height (open item 2) and the CG height (open item 3) are the
  first things to measure. Both set the towing pitch moment, and both may be checked
  against straight legs.

## Known limitations

- Added-mass Coriolis/Munk terms are not modelled. Rigid-body Coriolis is.
- No free-surface wave or wake effects. Foils and elements simply lose force as they
  leave the water.
- The horizontal tail sits at the tube-axis height. The report gives only the cruciform's
  overall extents.
- Boat speed through the water is taken equal to GPS speed (no current unless
  `water.current_ned` is set).
- The boat's own motion (pitch/roll at the tow point) is not applied; only its track is.

## Phase 2 (planned)

Point the unmodified SubReels extension at the sim:
- a mavlink2rest container in front of SITL;
- a `boatsim` service replaying the boat's MAVLink stream (GPS, DISTANCE_SENSOR,
  MISSION_CURRENT).

This would exercise SURF TRACK and auto-righting closed-loop. It would also allow variation
scenarios: `ATC_ANG_RLL_P` 1.5, skip-line turns, different tow speeds.
