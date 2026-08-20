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
     exceptions=mock.MagicMock())

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
    """

    def __init__(self, latest=(10.0, 0.5), speed=1.2, delayed=(10.5, 5.8, 0.2)):
        self._latest = latest
        self._speed = speed
        self._delayed = delayed

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
    main.jog_up_pwm = 1600
    main.jog_down_pwm = 1400
    ctrl = main.SurfTrackController()
    ctrl.state = "tracking"
    ctrl.hold_reason = None
    ctrl._target_depth_m = None
    ctrl._last_tick_ts = None
    return ctrl


def default_vehicle(armed=True, mode=main.MODE_ALT_HOLD):
    return {"armed": armed, "custom_mode": mode, "mode_label": "ALT_HOLD",
            "depth_m": 5.0}


def drive(ctrl, *, fish_depth=5.5, sonar=None, vehicle=None,
          now=None, dt=0.5):
    """Run one _tick with the given inputs. Returns captured RC3 writes."""
    if sonar is not None:
        main._sonar_history = sonar
    if vehicle is None:
        vehicle = default_vehicle()
    calls = install_thrust_capture()
    tstart = now if now is not None else 100.0
    with mock.patch.object(main.time, "monotonic", return_value=tstart), \
         mock.patch.object(main, "_get_towfish_depth_m",
                           return_value=fish_depth), \
         mock.patch.object(main, "get_vehicle_status_snapshot",
                           return_value=vehicle):
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
ctrl = fresh_controller(FakeSonar(latest=(10.0, 0.5)))
ctrl._target_depth_m = 6.0
calls = drive(ctrl, fish_depth=5.85)  # error 0.15 m, frac would be 0.15
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

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILED: {FAILURES}")
    raise SystemExit(1)
print("all checks passed")
