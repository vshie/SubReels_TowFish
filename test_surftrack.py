#!/usr/bin/env python3
"""Exercise main.py's surf-track controller without a vehicle.

The SurfTrackController's ``_tick`` is the whole loop -- everything the
thread does per tick lives there -- so driving it directly with stubbed
sensors is enough to cover the interesting behaviour: deadband,
proportional shape, direction sign, each hold reason, resume debounce,
slew-limit and the config clamps. Actually spinning up the thread would
just add sleep noise.

Run with: python3 test_surftrack.py
"""
from __future__ import annotations

import sys
import types
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "app"))

FAILURES: list[str] = []


def check(label, condition, detail=""):
    print(f"  {'ok  ' if condition else 'FAIL'} {label}"
          + (f" {detail}" if detail and not condition else ""))
    if not condition:
        FAILURES.append(label)


def stub(name, **attrs):
    mod = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(mod, key, value)
    sys.modules[name] = mod
    return mod


stub("flask", Flask=mock.MagicMock(), jsonify=mock.MagicMock(),
     request=mock.MagicMock(), send_file=mock.MagicMock())
stub("requests", get=mock.MagicMock(), post=mock.MagicMock(),
     exceptions=mock.MagicMock(),
     Session=mock.MagicMock(), adapters=mock.MagicMock())

gi = stub("gi", require_version=lambda *a, **k: None)
stub("gi.repository", Gst=mock.MagicMock())
gi.repository = sys.modules["gi.repository"]
stub("websockets", serve=mock.MagicMock())
stub("websockets.exceptions", ConnectionClosed=Exception)
sys.modules["websockets"].exceptions = sys.modules["websockets.exceptions"]
stub("usb_storage", get_base_dir=lambda: None, start_probe=lambda: None)

import main  # noqa: E402

# ---------------------------------------------------------------------------
# Fake sonar and command-capture harness
# ---------------------------------------------------------------------------


class FakeSonar:
    """Stands in for main._sonar_history during a tick.

    ``latest_speed_ms()`` and ``latest(max_age_s)`` are what the loop
    reads; ``depth_at_tow_delay`` is only used for the reported present
    altitude, so a static value there is fine for control tests.
    ``last_update_id`` returns a monotonic counter so the surf-track
    tick sees each call as a fresh sounding -- keeps the terrain half
    firing on every _tick just like the pre-split loop did, so existing
    assertions on _target_depth_m still apply.
    """

    def __init__(self, latest=(10.0, 0.5), speed=1.2, delayed=(10.5, 5.8, 0.2)):
        self._latest = latest
        self._speed = speed
        self._delayed = delayed
        self._id = 0

    def latest(self, max_age_s):
        if self._latest is None:
            return None
        depth, age = self._latest
        if age > max_age_s:
            return None
        return (depth, age)

    def latest_speed_ms(self):
        return self._speed

    def depth_at_tow_delay(self):
        return self._delayed

    def last_update_id(self):
        # Advance on every read so the loop treats each tick as a
        # fresh sounding. ``None`` when the previous ``latest`` was
        # None (nothing accepted into the ring buffer), mirroring
        # ``SonarHistory.last_update_id``.
        if self._latest is None:
            return None
        self._id += 1
        return self._id


def install_thrust_capture():
    """Replace main._set_thrust_command with a capture, returning it."""
    calls: list[tuple] = []

    def fake(direction, pwm, source):
        calls.append((direction, pwm, source))
        return True

    main._set_thrust_command = fake  # type: ignore[assignment]
    return calls


def install_thrust_operator_veto():
    """Simulate the operator owning the stick -- surf-track writes fail."""
    calls: list[tuple] = []

    def fake(direction, pwm, source):
        calls.append((direction, pwm, source))
        return source != "surftrack"  # surf-track blocked, operator ok

    main._set_thrust_command = fake  # type: ignore[assignment]
    return calls


def fresh_controller(sonar=None):
    """Return a controller in 'tracking' with sane globals + captured writes.

    Bypasses ``.enable`` so no thread is started; the harness drives
    ``_tick`` synchronously.
    """
    main._sonar_history = sonar if sonar is not None else FakeSonar()
    main.altitude_offset_m = 0.0
    main.surf_target_altitude_m = 4.0
    main.surf_deadband_m = 0.1
    main.surf_full_scale_error_m = 1.0
    main.surf_max_depth_m = 50.0
    main.tether_length_m = 0.0
    main.jog_up_pwm = 1600
    main.jog_down_pwm = 1400
    ctrl = main.SurfTrackController()
    ctrl.state = "tracking"
    ctrl.hold_reason = None
    ctrl._target_depth_m = None
    ctrl._raw_target_depth_m = None
    ctrl._last_sounding_id = None
    ctrl._last_tick_ts = None
    return ctrl


def default_vehicle(armed=True, mode=main.MODE_ALT_HOLD):
    return {"armed": armed, "custom_mode": mode, "mode_label": "ALT_HOLD",
            "depth_m": 5.0}


def drive(ctrl, *, fish_depth=5.5, sonar=None, vehicle=None,
          now=None, dt=0.5, attitude=None):
    """Run one _tick with the given inputs. Returns captured RC3 writes.

    ``attitude`` is ``None`` to leave ``get_towfish_attitude`` alone
    (defaults to an empty dict via the requests stub), or a dict like
    ``{"roll": 25.0, "pitch": 0.0}`` to inject a specific attitude
    into the recovery detector.
    """
    if sonar is not None:
        main._sonar_history = sonar
    if vehicle is None:
        vehicle = default_vehicle()
    calls = install_thrust_capture()
    tstart = now if now is not None else 100.0
    patches = [
        mock.patch.object(main.time, "monotonic", return_value=tstart),
        mock.patch.object(main, "_get_towfish_depth_m",
                          return_value=fish_depth),
        mock.patch.object(main, "get_vehicle_status_snapshot",
                          return_value=vehicle),
    ]
    if attitude is not None:
        patches.append(mock.patch.object(main, "get_towfish_attitude",
                                         return_value=attitude))
    with patches[0], patches[1], patches[2]:
        if len(patches) > 3:
            with patches[3]:
                ctrl._tick()
        else:
            ctrl._tick()
    return calls


# ---------------------------------------------------------------------------
# Deadband + proportional shape
# ---------------------------------------------------------------------------
print("deadband and proportional command")

# Bottom ahead 10 m, offset 0, target alt 4 m -> commanded depth = 6 m.
# Fish already at 6 m: error 0 -> deadband -> RC3 released, no command.
ctrl = fresh_controller(FakeSonar(latest=(10.0, 0.5)))
calls = drive(ctrl, fish_depth=6.0)
check("in-deadband release", calls == [(None, main.Z_PWM_NEUTRAL, "surftrack")],
      f"got {calls}")
check("in-deadband command is None", ctrl._last_command is None)

# 1.0 m error -> full authority. Bottom 10, target alt 4, so commanded
# depth converges to 6 m; pre-seed _target_depth_m past the slew ramp so
# these tests exercise the control law, not the slew (which has its own
# test below).
ctrl = fresh_controller(FakeSonar(latest=(10.0, 0.5)))
ctrl._target_depth_m = 6.0
calls = drive(ctrl, fish_depth=5.0)
check("full-scale error commands down at jog_down_pwm=1400",
      calls == [("down", main.jog_down_pwm, "surftrack")], f"got {calls}")

# Error just larger than deadband but well below full-scale should be
# clamped up to the SURF_MIN_FRAC floor rather than nearly-neutral.
# With the recovery-item tuning the floor is 0.08 and full-scale is
# wider, so the test uses a smaller deadband and a correspondingly
# small error to keep the "raw frac < floor" condition realisable.
ctrl = fresh_controller(FakeSonar(latest=(10.0, 0.5)))
ctrl._target_depth_m = 6.0
main.surf_deadband_m = 0.02
main.surf_full_scale_error_m = 1.0
calls = drive(ctrl, fish_depth=5.97)  # error 0.03 m, frac would be 0.03
check("small-error frac is floored at SURF_MIN_FRAC",
      len(calls) == 1 and calls[0][0] == "down"
      and calls[0][1] == round(main.Z_PWM_NEUTRAL
                               + (main.jog_down_pwm - main.Z_PWM_NEUTRAL)
                               * main.SURF_MIN_FRAC),
      f"got {calls}")

# Direction sign: fish deeper than target -> want to go up.
# Bottom 10, target 4, cmd depth 6. Fish at 7 -> error -1 -> up, at jog_up_pwm.
ctrl = fresh_controller(FakeSonar(latest=(10.0, 0.5)))
ctrl._target_depth_m = 6.0
calls = drive(ctrl, fish_depth=7.0)
check("fish deeper than commanded depth commands up",
      calls == [("up", main.jog_up_pwm, "surftrack")], f"got {calls}")

# Asymmetric authority: retune jog_up_pwm to 1700 before the tick and
# expect the tick to use the new number.
main.jog_up_pwm = 1700
ctrl = fresh_controller(FakeSonar(latest=(10.0, 0.5)))
main.jog_up_pwm = 1700
ctrl._target_depth_m = 6.0
calls = drive(ctrl, fish_depth=7.0)
check("mid-run change to jog_up_pwm takes effect next tick",
      calls == [("up", 1700, "surftrack")], f"got {calls}")

# Down authority also independent.
ctrl = fresh_controller(FakeSonar(latest=(10.0, 0.5)))
main.jog_up_pwm = 1700
main.jog_down_pwm = 1300
ctrl._target_depth_m = 6.0
calls = drive(ctrl, fish_depth=5.0)
check("asymmetric jog_down_pwm reaches the command",
      calls == [("down", 1300, "surftrack")], f"got {calls}")

# ---------------------------------------------------------------------------
# Wider deadband tolerates a bigger error before writing
# ---------------------------------------------------------------------------
print("\nreconfigurable deadband")
ctrl = fresh_controller(FakeSonar(latest=(10.0, 0.5)))
main.surf_deadband_m = 0.5
calls = drive(ctrl, fish_depth=6.3)  # error +0.3 m
check("error inside larger deadband releases",
      calls == [(None, main.Z_PWM_NEUTRAL, "surftrack")], f"got {calls}")

# ---------------------------------------------------------------------------
# Hold conditions
# ---------------------------------------------------------------------------
print("\nhold conditions and their reasons")

# fish_depth = None
ctrl = fresh_controller()
drive(ctrl, fish_depth=None)
check("no depth -> 'no_depth'", ctrl.state == "holding"
      and ctrl.hold_reason == "no_depth")

# fish_depth <= SURF_SURFACE_DEPTH_M
ctrl = fresh_controller()
drive(ctrl, fish_depth=0.3)
check("at surface -> 'at_surface'", ctrl.state == "holding"
      and ctrl.hold_reason == "at_surface")

# disarmed
ctrl = fresh_controller()
drive(ctrl, fish_depth=5.5, vehicle=default_vehicle(armed=False))
check("disarmed -> 'disarmed'", ctrl.state == "holding"
      and ctrl.hold_reason == "disarmed")

# mode not ALT_HOLD
ctrl = fresh_controller()
drive(ctrl, fish_depth=5.5, vehicle=default_vehicle(mode=main.MODE_STABILIZE))
check("mode != ALT_HOLD -> 'mode_not_alt_hold'",
      ctrl.state == "holding"
      and ctrl.hold_reason == "mode_not_alt_hold")

# sonar stale
ctrl = fresh_controller(FakeSonar(latest=None))
drive(ctrl, fish_depth=5.5)
check("no in-range sonar -> 'sonar_stale'",
      ctrl.state == "holding" and ctrl.hold_reason == "sonar_stale")

# not underway
ctrl = fresh_controller(FakeSonar(speed=0.05))
drive(ctrl, fish_depth=5.5)
check("boat drifting -> 'not_underway'",
      ctrl.state == "holding" and ctrl.hold_reason == "not_underway")

# geometry insane: bottom shallower than fish depth
ctrl = fresh_controller(FakeSonar(latest=(4.0, 0.5)))
drive(ctrl, fish_depth=5.5)
check("bottom above fish -> 'geometry_insane'",
      ctrl.state == "holding" and ctrl.hold_reason == "geometry_insane")

# Holding releases RC3 and records no command.
ctrl = fresh_controller(FakeSonar(latest=None))
calls = drive(ctrl, fish_depth=5.5)
check("holding releases surf-track RC3",
      calls == [(None, main.Z_PWM_NEUTRAL, "surftrack")], f"got {calls}")
check("holding clears the last commanded depth",
      ctrl._target_depth_m is None)

# ---------------------------------------------------------------------------
# Resume debounce: 2 s of good conditions before tracking resumes
# ---------------------------------------------------------------------------
print("\nresume debounce")
ctrl = fresh_controller(FakeSonar(latest=None))
drive(ctrl, fish_depth=5.5, now=100.0)  # -> holding sonar_stale
check("holding entered", ctrl.state == "holding")

# Conditions clear; first tick should stay in holding while debounce runs.
good_sonar = FakeSonar(latest=(10.0, 0.5))
drive(ctrl, fish_depth=6.0, sonar=good_sonar, now=101.0)
check("still holding partway through debounce", ctrl.state == "holding")
check("debounce clock is running",
      ctrl._hold_ok_since is not None)

# After SURF_RESUME_DEBOUNCE_S the loop promotes itself.
drive(ctrl, fish_depth=6.0, sonar=good_sonar,
      now=101.0 + main.SURF_RESUME_DEBOUNCE_S + 0.01)
check("promoted to tracking after debounce", ctrl.state == "tracking")

# A flicker back to bad resets the clock -- no cumulative credit.
ctrl = fresh_controller(FakeSonar(latest=None))
drive(ctrl, fish_depth=5.5, now=100.0)
drive(ctrl, fish_depth=6.0, sonar=good_sonar, now=101.5)
drive(ctrl, fish_depth=5.5, sonar=FakeSonar(latest=None), now=102.0)
drive(ctrl, fish_depth=6.0, sonar=good_sonar, now=102.5)
# Only 0.5 s of good time -- must still be holding.
check("bad flicker resets the debounce clock", ctrl.state == "holding")

# ---------------------------------------------------------------------------
# Slew limit on the commanded depth
# ---------------------------------------------------------------------------
print("\ntarget-depth slew limit")
# Bottom = 30, target alt = 4 -> raw_target = 26. Fish at 5. First tick's
# commanded depth starts from fish_depth (5) and can only step
# SURF_TARGET_SLEW_MPS * dt in that tick.
ctrl = fresh_controller(FakeSonar(latest=(30.0, 0.5)))
drive(ctrl, fish_depth=5.0, now=200.0)
first_target = ctrl._target_depth_m
# On the first tick dt defaults to SURF_TICK_S (no prior tick).
expected_first = 5.0 + main.SURF_TARGET_SLEW_MPS * main.SURF_TICK_S
check("first tick slews from fish depth by <= tick*slew",
      abs(first_target - expected_first) < 1e-6,
      f"got {first_target} expected {expected_first}")

# Second tick: 0.5 s later, another slew step (bounded, not 21 m).
drive(ctrl, fish_depth=5.0, now=200.5)
check("second tick continues the ramp",
      abs(ctrl._target_depth_m - (first_target
                                  + main.SURF_TARGET_SLEW_MPS * 0.5)) < 1e-6,
      f"got {ctrl._target_depth_m}")

# ---------------------------------------------------------------------------
# Config clamps
# ---------------------------------------------------------------------------
print("\nconfig clamps")
check("target 0.5 -> min 1.0",
      main._sanitize_surf_target_altitude_m(0.5)
      == main.SURF_TARGET_ALTITUDE_MIN_M)
check("target 9 -> max 6.0",
      main._sanitize_surf_target_altitude_m(9.0)
      == main.SURF_TARGET_ALTITUDE_MAX_M)
check("deadband 0.001 -> min",
      main._sanitize_surf_deadband_m(0.001)
      == main.SURF_DEADBAND_MIN_M)
check("deadband 10 -> max",
      main._sanitize_surf_deadband_m(10.0)
      == main.SURF_DEADBAND_MAX_M)
check("full-scale 0.05 -> min",
      main._sanitize_surf_full_scale_error_m(0.05)
      == main.SURF_FULL_SCALE_ERROR_MIN_M)
check("full-scale 100 -> max",
      main._sanitize_surf_full_scale_error_m(100.0)
      == main.SURF_FULL_SCALE_ERROR_MAX_M)
check("max depth 1 -> min",
      main._sanitize_surf_max_depth_m(1.0)
      == main.SURF_MAX_DEPTH_MIN_M)
check("max depth 999 -> max",
      main._sanitize_surf_max_depth_m(999.0)
      == main.SURF_MAX_DEPTH_MAX_M)
check("jog up 1500 clamped up to 1501",
      main._sanitize_jog_up_pwm(1500) == main.JOG_UP_PWM_MIN)
check("jog up 2000 clamped down to 1900",
      main._sanitize_jog_up_pwm(2000) == main.JOG_UP_PWM_MAX)
check("jog down 1500 clamped down to 1499",
      main._sanitize_jog_down_pwm(1500) == main.JOG_DOWN_PWM_MAX)
check("jog down 100 clamped up to 1100",
      main._sanitize_jog_down_pwm(100) == main.JOG_DOWN_PWM_MIN)
check("NaN falls back to the default (up)",
      main._sanitize_jog_up_pwm(float("nan"))
      == main.DEFAULT_JOG_UP_PWM)

# ---------------------------------------------------------------------------
# Operator veto: surf-track write refused, snapshot records suspension
# ---------------------------------------------------------------------------
print("\noperator veto")
ctrl = fresh_controller(FakeSonar(latest=(10.0, 0.5)))
# Preseed the target past the slew ramp so this tick actually issues a
# command (raw_target = 6, fish = 5, deadband = 0.1). Without the
# preseed the first-tick slew at SURF_TICK_S puts us inside deadband
# and the write we want to see vetoed never happens.
ctrl._target_depth_m = 6.0
ctrl._raw_target_depth_m = 6.0
calls = install_thrust_operator_veto()
with mock.patch.object(main.time, "monotonic", return_value=100.0), \
     mock.patch.object(main, "_get_towfish_depth_m", return_value=5.0), \
     mock.patch.object(main, "get_vehicle_status_snapshot",
                       return_value=default_vehicle()):
    ctrl._tick()
check("surf-track write was attempted",
      any(c[2] == "surftrack" and c[0] == "down" for c in calls))
check("controller notices the veto and clears last_command",
      ctrl._last_command is None)
check("last_event says 'suspended: operator'",
      ctrl.last_event == "suspended: operator")

# ---------------------------------------------------------------------------
# Depth clamp: max_depth caps the commanded raw target
# ---------------------------------------------------------------------------
print("\nmax-depth clamp on commanded target")
main.surf_max_depth_m = 8.0
ctrl = fresh_controller(FakeSonar(latest=(30.0, 0.5)))
main.surf_max_depth_m = 8.0
# Ramp for many ticks; the target must plateau at 8, never exceed.
now = 500.0
for i in range(200):
    drive(ctrl, fish_depth=5.0, sonar=FakeSonar(latest=(30.0, 0.5)),
          now=now + i * 0.5)
check("commanded depth clamped by surf_max_depth_m",
      ctrl._target_depth_m is not None and ctrl._target_depth_m <= 8.0 + 1e-6,
      f"got {ctrl._target_depth_m}")

# ---------------------------------------------------------------------------
# Tether ceiling: 0.7 * tether_length caps the commanded raw target
# ---------------------------------------------------------------------------
print("\ntether-length ceiling")
# tether = 5 m gives an effective ceiling of 0.7 * 5 = 3.5 m even
# though surf_max_depth_m is 50 m. Sonar reads 20 m ahead, target alt
# 4 m -> raw target 16 m -> must clamp to 3.5.
ctrl = fresh_controller(FakeSonar(latest=(20.0, 0.5)))
main.surf_max_depth_m = 50.0
main.tether_length_m = 5.0
# Ramp for many ticks so the slew catches up to the raw target.
now = 700.0
for i in range(200):
    drive(ctrl, fish_depth=3.0, sonar=FakeSonar(latest=(20.0, 0.5)),
          now=now + i * 0.5)
snap = ctrl.snapshot()
check("tether ceiling clamps _target_depth_m at 0.7 * tether",
      ctrl._target_depth_m is not None and ctrl._target_depth_m <= 3.5 + 1e-6,
      f"got _target_depth_m={ctrl._target_depth_m}")
check("snapshot exposes effective depth_ceiling_m",
      snap.get("depth_ceiling_m") == 3.5,
      f"got depth_ceiling_m={snap.get('depth_ceiling_m')}")
check("snapshot flags ceiling_limited",
      snap.get("ceiling_limited") is True)
# Undo global for the tests below so they use the plain surf_max_depth_m cap.
main.tether_length_m = 0.0

# ---------------------------------------------------------------------------
# Recovery entry on roll excursion, up-command at recovery frac
# ---------------------------------------------------------------------------
print("\nrecovery: roll excursion drives full up")
ctrl = fresh_controller(FakeSonar(latest=(10.0, 0.5)))
# Preseed a modest tracking state so the tick actually enters the
# _tick body at ``elif`` -> recovery rather than being swallowed by
# the ``hold_reason`` branch.
ctrl._target_depth_m = 6.0
ctrl._raw_target_depth_m = 6.0
calls = drive(ctrl, fish_depth=6.0, now=800.0,
              attitude={"roll": 25.0, "pitch": 0.0})
check("state transitions to 'recovering'", ctrl.state == "recovering",
      f"got state={ctrl.state}")
check("hold_reason records 'roll_excursion'",
      ctrl.hold_reason == "roll_excursion",
      f"got hold_reason={ctrl.hold_reason}")
check("recovery commands up",
      any(c[0] == "up" and c[2] == "surftrack" for c in calls),
      f"got {calls}")
# SURF_RECOVERY_UP_FRAC is 1.0 today -- pwm should hit the full up jog.
check("recovery up command is at recovery authority",
      any(c[0] == "up" and c[1] == main.jog_up_pwm for c in calls),
      f"got {calls}")

# ---------------------------------------------------------------------------
# Recovery: attitude clears after SURF_RECOVERY_CLEAR_S -> tracking + cooldown
# ---------------------------------------------------------------------------
print("\nrecovery: clearance exits with cooldown")
# Continue from the previous controller. Advance time past
# SURF_RECOVERY_CLEAR_S with attitude back inside limits.
t = 800.0
# Two ticks: one to seed _recovery_ok_since, another after the
# clearance window elapses.
drive(ctrl, fish_depth=6.0, now=t + 0.1,
      attitude={"roll": 0.0, "pitch": 0.0})
check("still recovering while clear-window unfinished",
      ctrl.state == "recovering",
      f"got state={ctrl.state}")
drive(ctrl, fish_depth=6.0, now=t + 0.1 + main.SURF_RECOVERY_CLEAR_S + 0.5,
      attitude={"roll": 0.0, "pitch": 0.0})
check("state returns to 'tracking' after clearance",
      ctrl.state == "tracking",
      f"got state={ctrl.state}")
check("recovery cooldown armed",
      ctrl._recovery_cooldown_until is not None
      and ctrl._recovery_cooldown_until > t,
      f"got cooldown={ctrl._recovery_cooldown_until}")
# Snapshot the cooldown from the same simulated clock the tick ran
# under so ``time.monotonic()`` still lies inside the cooldown
# window; otherwise real wall-clock time trivially clears it.
with mock.patch.object(main.time, "monotonic",
                       return_value=t + 0.1 + main.SURF_RECOVERY_CLEAR_S + 0.6):
    snap = ctrl.snapshot()
check("snapshot reports recovery_cooldown_s",
      isinstance(snap.get("recovery_cooldown_s"), (int, float))
      and snap["recovery_cooldown_s"] > 0,
      f"got {snap.get('recovery_cooldown_s')}")

# ---------------------------------------------------------------------------
# Recovery: pitch excursion also enters recovering
# ---------------------------------------------------------------------------
print("\nrecovery: pitch excursion also enters recovery")
ctrl = fresh_controller(FakeSonar(latest=(10.0, 0.5)))
ctrl._target_depth_m = 6.0
ctrl._raw_target_depth_m = 6.0
calls = drive(ctrl, fish_depth=6.0, now=900.0,
              attitude={"roll": 0.0, "pitch": 30.0})
check("pitch excursion -> recovering", ctrl.state == "recovering",
      f"got state={ctrl.state}")
check("pitch excursion sets hold_reason 'pitch_excursion'",
      ctrl.hold_reason == "pitch_excursion",
      f"got hold_reason={ctrl.hold_reason}")

# ---------------------------------------------------------------------------
# Speed gate: down refused when boat is stalled
# ---------------------------------------------------------------------------
print("\nspeed gate: down blocked when boat < 0.7 m/s")
sonar = FakeSonar(latest=(10.0, 0.5), speed=0.3)
ctrl = fresh_controller(sonar)
# Preseed so the tick would want to command down but the speed gate
# should refuse.
ctrl._target_depth_m = 6.0
ctrl._raw_target_depth_m = 6.0
calls = drive(ctrl, fish_depth=5.0, sonar=sonar, now=1000.0)
check("no down command written while too slow",
      not any(c[0] == "down" and c[2] == "surftrack" for c in calls),
      f"got {calls}")
# RC3 must still be released (deadband/no-command shape), and the
# snapshot must expose the reason for the operator.
check("RC3 released on speed gate",
      any(c == (None, main.Z_PWM_NEUTRAL, "surftrack") for c in calls),
      f"got {calls}")
check("_last_down_gate_reason == 'too_slow_to_dive'",
      ctrl._last_down_gate_reason == "too_slow_to_dive",
      f"got {ctrl._last_down_gate_reason}")

# ---------------------------------------------------------------------------
# Speed gate: up commands still allowed while stalled
# ---------------------------------------------------------------------------
print("\nspeed gate: up still allowed while stalled")
sonar = FakeSonar(latest=(10.0, 0.5), speed=0.3)
ctrl = fresh_controller(sonar)
# Preseed target depth shallower than fish so error < 0 -> up.
ctrl._target_depth_m = 3.0
ctrl._raw_target_depth_m = 3.0
calls = drive(ctrl, fish_depth=5.0, sonar=sonar, now=1100.0)
check("up command still issued despite low speed",
      any(c[0] == "up" and c[2] == "surftrack" for c in calls),
      f"got {calls}")

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILED: {FAILURES}")
    raise SystemExit(1)
print("all checks passed")
