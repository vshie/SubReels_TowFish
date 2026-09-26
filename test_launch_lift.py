#!/usr/bin/env python3
"""Exercise the armed-MANUAL shore-launch lift guard without a vehicle."""
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
stub("photogrammetry_meta")

import main  # noqa: E402


def tick(guard, *, armed, mode):
    calls = []
    heartbeat = {
        "armed": armed,
        "custom_mode": mode,
        "mode_label": "MANUAL" if mode == main.MODE_MANUAL else "ALT_HOLD",
    }
    with mock.patch.object(main, "_cached_heartbeat_snapshot",
                           return_value=heartbeat), \
         mock.patch.object(
             main, "_set_thrust_command",
             side_effect=lambda direction, pwm, source: calls.append(
                 (direction, pwm, source)
             ) or True,
         ):
        guard._tick()
    return calls


print("\nlaunch lift condition")
guard = main.LaunchLiftGuard()
calls = tick(guard, armed=True, mode=main.MODE_MANUAL)
check("armed MANUAL commands up", calls == [
    ("up", main.Z_PWM_MAX, "launch_guard")
], f"got {calls}")
check("armed MANUAL uses the full RC3 endpoint",
      calls[0][1] == 1900, f"got {calls[0][1]}")
check("guard reports active", guard.snapshot()["active"] is True)

calls = tick(guard, armed=True, mode=main.MODE_MANUAL)
check("full-up command is refreshed", calls == [
    ("up", main.Z_PWM_MAX, "launch_guard")
], f"got {calls}")

print("\nrelease conditions")
calls = tick(guard, armed=True, mode=main.MODE_ALT_HOLD)
check("leaving MANUAL centers and releases", calls == [
    (None, main.Z_PWM_NEUTRAL, "launch_guard")
], f"got {calls}")
check("guard reports inactive after mode change",
      guard.snapshot()["active"] is False)

guard = main.LaunchLiftGuard()
check("disarmed MANUAL does not own RC3",
      tick(guard, armed=False, mode=main.MODE_MANUAL) == [])

tick(guard, armed=True, mode=main.MODE_MANUAL)
calls = tick(guard, armed=False, mode=main.MODE_MANUAL)
check("disarming centers and releases", calls == [
    (None, main.Z_PWM_NEUTRAL, "launch_guard")
], f"got {calls}")

print("\npriority")
main._thrust_source = "launch_guard"
main._thrust_deadline = 200.0
main._thrust_operator_last_hit = None
with mock.patch.object(main.time, "monotonic", return_value=100.0), \
     mock.patch.object(main, "_start_thrust_thread_if_needed"):
    accepted = main._set_thrust_command(
        "down", main.Z_PWM_MIN, source="operator",
    )
check("operator DOWN cannot defeat an active launch guard",
      accepted is False)

main._thrust_source = None
main._thrust_deadline = None
main._thrust_direction = None

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILED: {', '.join(FAILURES)}")
    sys.exit(1)
print("all launch lift checks passed")
