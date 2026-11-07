#!/usr/bin/env python3
"""Exercise main.py's auto-righting controller without a vehicle.

Same approach as test_surftrack.py: drive ``AutoRightController._tick``
directly with stubbed sensors and a captured RC/mode writer.

Run with: python3 test_autoright.py
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

main._persist_config = lambda: None  # type: ignore[assignment]


class FakeSonar:
    def __init__(self, speed=1.0):
        self.speed = speed

    def latest_speed_ms(self):
        return self.speed


class FakeWriter:
    def __init__(self):
        self.modes: list[int] = []

    def set_mode(self, mode):
        self.modes.append(mode)
        return True

    def named_value_float(self, name, value):
        return True


WRITER = FakeWriter()
main.get_default_writer = lambda: WRITER  # type: ignore[assignment]
main._surftrack.queue_named_values = lambda values: None  # type: ignore


def vehicle(mode=main.MODE_ALT_HOLD, armed=True):
    return {"armed": armed, "custom_mode": mode}


ALT, MAN, STAB = main.MODE_ALT_HOLD, main.MODE_MANUAL, main.MODE_STABILIZE


def fresh():
    WRITER.modes.clear()
    main.autoright_roll_min_depth_m = 0.0
    main.autoright_descent_start_frac = 0.0
    main.autoright_vertical_pwm = 1900
    main.autoright_roll_frac = 1.0
    main.tether_twist_turns = 0.0
    main.surf_max_depth_m = 50.0
    main.tether_length_m = 0.0
    main.jog_down_pwm = 1300
    main._sonar_history = FakeSonar()
    ctrl = main.AutoRightController()
    ctrl.state = "idle"
    return ctrl


def tick(ctrl, now, *, roll=0.0, depth=5.0, mode=ALT, armed=True,
         seq=None, speed=1.0, veto=False):
    """One tick. Returns the RC writes as (direction, pwm, source, roll_pwm)."""
    calls: list[tuple] = []

    def fake_thrust(direction, pwm, source, roll_pwm=None):
        calls.append((direction, pwm, source, roll_pwm))
        return not veto

    main._sonar_history.speed = speed
    # Force a nav poll every tick so the test controls timing directly.
    ctrl._nav_polled_at = None
    patches = [
        mock.patch.object(main.time, "monotonic", return_value=now),
        mock.patch.object(main, "_get_towfish_depth_m", return_value=depth),
        mock.patch.object(main, "get_towfish_attitude",
                          return_value={"roll": roll, "pitch": 0.0}),
        mock.patch.object(main, "get_vehicle_status_snapshot",
                          return_value=vehicle(mode, armed)),
        mock.patch.object(main, "_servo_output_message", return_value={}),
        mock.patch.object(main, "_tow_mission_seq", return_value=seq),
        mock.patch.object(main, "_set_thrust_command", fake_thrust),
    ]
    for p in patches:
        p.start()
    try:
        ctrl._tick()
    finally:
        for p in patches:
            p.stop()
    return calls


DWELL = main.SURF_INVERT_DWELL_S

# ---------------------------------------------------------------------------
print("trigger: shipped recipe")
check("acts at 60 deg", main.SURF_INVERT_ROLL_DEG == 60.0)
check("after 0.5 s", DWELL == 0.5)
check("vertical stick neutral by default", main.DEFAULT_AUTORIGHT_VERTICAL_PWM == 1500)
check("roll stick released within 20 deg", main.AUTORIGHT_ROLL_STICK_CUTOFF_DEG == 20.0)

print("\ntrigger: in the corner, without waiting for the boat")
ctrl = fresh()
tick(ctrl, 100.0, roll=70.0, depth=3.5, seq=5)
check("waiting while inside the dwell", ctrl.state == "waiting" and not WRITER.modes,
      f"state={ctrl.state} modes={WRITER.modes}")
tick(ctrl, 100.0 + DWELL - 0.1, roll=80.0, depth=3.4, seq=5)
check("not before the dwell", not WRITER.modes, f"got {WRITER.modes}")
tick(ctrl, 100.0 + DWELL + 0.1, roll=95.0, depth=3.3, seq=5)
check("MANUAL requested", WRITER.modes == [MAN], f"got {WRITER.modes}")
check("state manual", ctrl.state == "manual", f"got {ctrl.state}")

print("\ntrigger: no boat data needed")
ctrl = fresh()
tick(ctrl, 300.0, roll=-150.0, depth=0.3)
tick(ctrl, 300.0 + DWELL + 0.1, roll=-150.0, depth=0.3)
check("fires without a mission", WRITER.modes == [MAN], f"got {WRITER.modes}")

print("\ntrigger: a 50 deg bank is left to the roll controller")
ctrl = fresh()
for k in range(20):
    tick(ctrl, 350.0 + k * 0.5, roll=50.0, depth=3.5)
check("no rescue", not WRITER.modes and ctrl.state == "idle",
      f"state={ctrl.state} modes={WRITER.modes}")

print("\ntrigger: self-righted inside the dwell -> nothing")
ctrl = fresh()
tick(ctrl, 400.0, roll=0.0)
tick(ctrl, 404.0, roll=75.0)
tick(ctrl, 404.3, roll=40.0)
tick(ctrl, 406.0, roll=10.0)
check("no rescue", not WRITER.modes and ctrl.state == "idle",
      f"state={ctrl.state} modes={WRITER.modes}")

print("\ntrigger: never from an operator-flown mode")
ctrl = fresh()
for k in range(20):
    tick(ctrl, 500.0 + k, roll=150.0, depth=0.3, mode=MAN)
check("no rescue while the operator is in MANUAL", not WRITER.modes)


def start_rescue(t0=1000.0, roll=150.0):
    ctrl = fresh()
    tick(ctrl, t0, roll=roll, depth=0.3)
    tick(ctrl, t0 + DWELL + 0.1, roll=roll, depth=0.3)
    assert ctrl.state == "manual", ctrl.state
    return ctrl, t0 + DWELL + 0.5


# ---------------------------------------------------------------------------
print("\ndrill: vertical stick from the setting, roll stick the short way")
ctrl, t = start_rescue(roll=120.0)
main.autoright_vertical_pwm = 1750
calls = tick(ctrl, t, roll=120.0, depth=0.5, mode=MAN)
check("vertical PWM from the setting",
      calls and calls[-1][1] == 1750, f"got {calls}")
check("roll +120 -> RC2 low (roll left)",
      calls and calls[-1][3] == main.Z_PWM_NEUTRAL - 400, f"got {calls}")
calls = tick(ctrl, t + 0.5, roll=-178.0, depth=0.5, mode=MAN)
check("direction latched through the +-180 wrap (no chatter)",
      calls and calls[-1][3] == main.Z_PWM_NEUTRAL - 400, f"got {calls}")
calls = tick(ctrl, t + 1.0, roll=40.0, depth=0.5, mode=MAN)
check("roll stick still pushing at 40 deg",
      calls and calls[-1][3] == main.Z_PWM_NEUTRAL - 400, f"got {calls}")
calls = tick(ctrl, t + 1.5, roll=15.0, depth=0.5, mode=MAN)
check("roll stick centred under 20 deg, vertical kept",
      calls and calls[-1][3] == main.Z_PWM_NEUTRAL and calls[-1][1] == 1750,
      f"got {calls}")

ctrl2, t2 = start_rescue(t0=1500.0, roll=-120.0)
main.autoright_roll_frac = 0.5
calls = tick(ctrl2, t2, roll=-120.0, depth=0.5, mode=MAN)
check("roll -120 -> RC2 high (roll right), half stick",
      calls and calls[-1][3] == main.Z_PWM_NEUTRAL + 200, f"got {calls}")
main.autoright_roll_frac = 1.0

print("\ndrill: optional wait for depth before the roll stick")
ctrl3, t3 = start_rescue(t0=1600.0)
main.autoright_roll_min_depth_m = 1.5
calls = tick(ctrl3, t3, roll=170.0, depth=0.4, mode=MAN)
check("roll stick held while shallower than the set depth",
      calls and calls[-1][3] == main.Z_PWM_NEUTRAL, f"got {calls}")
calls = tick(ctrl3, t3 + 1.0, roll=170.0, depth=1.6, mode=MAN)
check("roll stick once deep enough",
      calls and calls[-1][3] == main.Z_PWM_NEUTRAL - 400, f"got {calls}")
ctrl3, t3 = start_rescue(t0=1700.0)
calls = tick(ctrl3, t3 + main.AUTORIGHT_ROLL_WAIT_MAX_S, roll=170.0, depth=0.4,
             mode=MAN)
check("never waits longer than the cap",
      calls and calls[-1][3] == main.Z_PWM_NEUTRAL - 400, f"got {calls}")
main.autoright_roll_min_depth_m = 0.0

print("\ndrill: upright 1 s -> RC2 centred, ALT_HOLD, descending")
t += 2.0
tick(ctrl, t, roll=5.0, depth=0.5, mode=MAN)
calls = tick(ctrl, t + main.SURF_RIGHTED_HOLD_S + 0.1, roll=5.0, depth=0.5,
             mode=MAN)
check("descending", ctrl.state == "descending", f"got {ctrl.state}")
check("ALT_HOLD requested", WRITER.modes[-1] == ALT, f"got {WRITER.modes}")
check("RC2 and RC3 centred on the hand-over",
      calls and calls[-1][1] == main.Z_PWM_NEUTRAL
      and calls[-1][3] == main.Z_PWM_NEUTRAL, f"got {calls}")

print("\ndescent: ramps in, eases off below 0.9 m/s, ends at the resume depth")
t += 2.0
pwms = []
for k in range(6):
    calls = tick(ctrl, t + k * 0.5, roll=2.0, depth=0.8)
    pwms += [c[1] for c in calls if c[0] == "down"]
check("down ramps in", len(pwms) >= 5
      and all(b < a for a, b in zip(pwms, pwms[1:])), f"got {pwms}")
frac = ctrl._descent_frac
tick(ctrl, t + 3.5, roll=2.0, depth=0.9, speed=0.8)
check("eases off when the boat slows", ctrl._descent_frac < frac)
check("no fresh turn entry -> default resume depth",
      ctrl._resume_depth_m == main.SURF_RESUME_DEPTH_M,
      f"got {ctrl._resume_depth_m}")
calls = tick(ctrl, t + 4.0, roll=2.0, depth=main.SURF_RESUME_DEPTH_M + 0.05)
check("rescued -> idle", ctrl.state == "idle"
      and (ctrl.snapshot()["last"] or {}).get("outcome") == "rescued",
      f"state={ctrl.state}")
check("release on the way out", calls and calls[-1][0] is None, f"got {calls}")

# ---------------------------------------------------------------------------
print("\nresume depth: fish depth when the boat reached the turn's first waypoint")
ctrl = fresh()
tick(ctrl, 2000.0, roll=0.0, depth=4.6, seq=6)
tick(ctrl, 2001.0, roll=0.0, depth=4.6, seq=7)   # first corner
tick(ctrl, 2005.0, roll=0.0, depth=4.2, seq=8)  # second corner
check("captured at the first corner", ctrl._turn_entry_depth_m == 4.6,
      f"got {ctrl._turn_entry_depth_m}")
tick(ctrl, 2015.0, roll=150.0, depth=0.4, seq=8)
tick(ctrl, 2018.0, roll=150.0, depth=0.4, seq=8)
tick(ctrl, 2021.0, roll=150.0, depth=0.4, seq=8)
check("rescue resumes to it", ctrl.state == "manual"
      and ctrl._resume_depth_m == 4.6, f"state={ctrl.state} "
      f"resume={ctrl._resume_depth_m}")

print("\nresume depth: fish already rolling at the waypoint -> last upright depth")
ctrl = fresh()
tick(ctrl, 2500.0, roll=5.0, depth=4.4, seq=6)
tick(ctrl, 2510.0, roll=60.0, depth=3.5, seq=6)
tick(ctrl, 2515.0, roll=110.0, depth=2.6, seq=7)
check("uses 4.4 m from before the roll", ctrl._turn_entry_depth_m == 4.4,
      f"got {ctrl._turn_entry_depth_m}")

print("\nresume depth: stale capture -> default; capped at the ceiling")
ctrl = fresh()
tick(ctrl, 3000.0, roll=0.0, depth=4.6, seq=6)
tick(ctrl, 3001.0, roll=0.0, depth=4.6, seq=7)
t = 3001.0 + main.AUTORIGHT_TURN_ENTRY_MAX_AGE_S + 10.0
tick(ctrl, t, roll=150.0, depth=0.4, seq=7)
tick(ctrl, t + 6.0, roll=150.0, depth=0.4, seq=7)
check("stale -> default", ctrl._resume_depth_m == main.SURF_RESUME_DEPTH_M,
      f"got {ctrl._resume_depth_m}")
main.surf_max_depth_m = 5.0
check("capped at the ceiling", main._resume_depth_for(9.0) == 5.0,
      f"got {main._resume_depth_for(9.0)}")

# ---------------------------------------------------------------------------
print("\nfallbacks: MANUAL timeout -> STABILIZE -> ALT_HOLD")
ctrl, t = start_rescue()
tick(ctrl, t, roll=150.0, depth=2.0, mode=MAN)
calls = tick(ctrl, t + main.SURF_RIGHTING_TIMEOUT_S + 0.1, roll=150.0,
             depth=3.0, mode=MAN)
check("STABILIZE requested", WRITER.modes[-1] == STAB and ctrl.state == "stabilize",
      f"state={ctrl.state} modes={WRITER.modes}")
t2 = t + main.SURF_RIGHTING_TIMEOUT_S + 0.5
calls = tick(ctrl, t2, roll=150.0, depth=3.0, mode=STAB)
check("sticks centred in STABILIZE", calls and calls[-1][0] is None,
      f"got {calls}")
tick(ctrl, t2 + 1.0, roll=5.0, depth=3.0, mode=STAB)
tick(ctrl, t2 + 2.5, roll=5.0, depth=3.0, mode=STAB)
check("upright in STABILIZE -> descending in ALT_HOLD",
      ctrl.state == "descending" and WRITER.modes[-1] == ALT,
      f"state={ctrl.state} modes={WRITER.modes}")

print("\nfallbacks: STABILIZE timeout gives up, stays in STABILIZE")
ctrl, t = start_rescue()
tick(ctrl, t, roll=150.0, depth=2.0, mode=MAN)
tick(ctrl, t + main.SURF_RIGHTING_TIMEOUT_S + 0.1, roll=150.0, mode=MAN)
t2 = t + main.SURF_RIGHTING_TIMEOUT_S + 0.5
tick(ctrl, t2, roll=150.0, mode=STAB)
tick(ctrl, t2 + main.SURF_STABILIZE_RIGHTING_TIMEOUT_S + 0.1, roll=150.0,
     mode=STAB)
check("rescue_failed -> idle", ctrl.state == "idle"
      and ctrl.snapshot()["last"]["outcome"] == "rescue_failed",
      f"state={ctrl.state}")
check("no further mode request", WRITER.modes[-1] == STAB, f"got {WRITER.modes}")
tick(ctrl, t2 + 70.0, roll=150.0, depth=0.3)
tick(ctrl, t2 + 80.0, roll=150.0, depth=0.3)
check("no re-trigger on the same inversion",
      ctrl.state in ("idle", "waiting") and WRITER.modes[-1] == STAB)

print("\ncancel: operator input or mode change")
ctrl, t = start_rescue()
tick(ctrl, t, roll=150.0, mode=MAN, veto=True)
check("operator input -> cancelled",
      ctrl.state == "idle" and ctrl.snapshot()["last"]["outcome"]
      == "rescue_cancelled", f"state={ctrl.state}")
ctrl, t = start_rescue()
tick(ctrl, t, roll=150.0, mode=MAN)
tick(ctrl, t + 0.5, roll=150.0, mode=STAB)
check("operator mode change -> cancelled",
      ctrl.state == "idle" and ctrl.snapshot()["last"]["outcome"]
      == "rescue_cancelled", f"state={ctrl.state}")

print("\nstandalone: never touches SURF TRACK")
main._surftrack.state = "disabled"
ctrl, t = start_rescue()
check("SURF TRACK still disabled", main._surftrack.state == "disabled")
check("is_active while rescuing", ctrl.is_active())

# ---------------------------------------------------------------------------
print("\ntether twist")
ctrl = fresh()
for k, r in enumerate([0, 90, 170, -170, -90, 0]):
    tick(ctrl, 5000.0 + k * 0.1, roll=float(r), mode=MAN)
check("+360 across the wrap counts +1", abs(main.tether_twist_turns - 1.0) < 1e-6,
      f"got {main.tether_twist_turns}")
ctrl = fresh()
for k, r in enumerate([0, -90, -170, 170, 90, 0]):
    tick(ctrl, 5100.0 + k * 0.1, roll=float(r), mode=MAN)
check("-360 counts -1", abs(main.tether_twist_turns + 1.0) < 1e-6,
      f"got {main.tether_twist_turns}")
ctrl = fresh()
for k, r in enumerate([0, 90, 170, 90, 0]):
    tick(ctrl, 5200.0 + k * 0.1, roll=float(r), mode=MAN)
check("to 170 and back counts 0", abs(main.tether_twist_turns) < 1e-6,
      f"got {main.tether_twist_turns}")
ctrl = fresh()
for k, r in enumerate([0, 60, 120, 60, 0]):
    tick(ctrl, 5300.0 + k * 0.1, roll=float(r), mode=MAN)
check("shortest-path recovery from +120 adds no turn",
      abs(main.tether_twist_turns) < 1e-6, f"got {main.tether_twist_turns}")
ctrl = fresh()
for k, r in enumerate([0, 90, 170, -170]):
    tick(ctrl, 5400.0 + k * 0.1, roll=float(r), mode=MAN, armed=False)
check("not counted while disarmed", main.tether_twist_turns == 0.0)
ctrl.reset_twist()
check("reset zeroes it", main.tether_twist_turns == 0.0)

# ---------------------------------------------------------------------------
print("\nlaunch-lift guard: stands down while a rescue flies MANUAL")


def guard_tick(guard, *, armed=True, mode=MAN):
    calls = []
    hb = {"armed": armed, "custom_mode": mode}
    with mock.patch.object(main, "_cached_heartbeat_snapshot", return_value=hb), \
         mock.patch.object(main, "_set_thrust_command",
                           side_effect=lambda d, p, source, roll_pwm=None:
                           calls.append((d, p, source)) or True):
        guard._tick()
    return calls


saved_autoright = main._autoright
ctrl, t = start_rescue(t0=6000.0, roll=120.0)
main._autoright = ctrl
guard = main.LaunchLiftGuard()
check("no full-up from the guard during a rescue in MANUAL",
      guard_tick(guard) == [] and guard.snapshot()["active"] is False)

# The shared RC3 path itself: the drill's neutral stick + roll stick holds.
main._thrust_source = None
main._thrust_deadline = None
main._thrust_operator_last_hit = None
with mock.patch.object(main.time, "monotonic", return_value=6001.0), \
     mock.patch.object(main, "_start_thrust_thread_if_needed"):
    ok_ar = main._set_thrust_command("up", 1500, source="autoright", roll_pwm=1100)
    ok_guard = main._set_thrust_command("up", main.Z_PWM_MAX, source="launch_guard")
check("auto-righting holds the channel", ok_ar is True)
check("launch guard refused while the rescue is active", ok_guard is False)
check("drill command intact (1500 + roll stick)",
      main._thrust_source == "autoright" and main._thrust_pwm == 1500
      and main._thrust_roll_pwm == 1100,
      f"got {main._thrust_source} {main._thrust_pwm} {main._thrust_roll_pwm}")

ctrl.state = "idle"     # rescue over (back in ALT_HOLD)
check("guard idle after the rescue ends in ALT_HOLD",
      guard_tick(guard, mode=ALT) == [])
calls = guard_tick(guard, mode=MAN)
check("guard takes over any other armed MANUAL",
      calls == [("up", main.Z_PWM_MAX, "launch_guard")], f"got {calls}")
main._autoright = saved_autoright
main._thrust_source = None
main._thrust_deadline = None
main._thrust_direction = None
main._thrust_roll_pwm = None

print()
if FAILURES:
    print(f"{len(FAILURES)} check(s) failed")
    sys.exit(1)
print("all checks passed")
