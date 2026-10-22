#!/usr/bin/env python3
"""Exercise the held camera tilt and the sonar confidence floor without a
vehicle.

Covers the widget tilt arrows' backend (step clamping, only remembering
an angle the autopilot accepted, boot reseating the held angle) and the
Ping confidence gate: a sounding is only stored or shown at or above
the 90 % floor, and one with no confidence at all is rejected too.

Run with: python3 test_tilt_and_sonar_gate.py
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
stub("photogrammetry_meta")

import main  # noqa: E402


def command(pitch, accepted=True):
    writer = mock.MagicMock()
    writer.mount_pitch.return_value = accepted
    with mock.patch.object(main, "get_default_writer", return_value=writer), \
         mock.patch.object(main, "_persist_config") as persist:
        ok = main._command_camera_tilt(pitch)
    sent = writer.mount_pitch.call_args[0][0] if writer.mount_pitch.called else None
    return ok, sent, persist.called


print("held camera tilt")
main.camera_tilt_deg = main.DEFAULT_CAMERA_TILT_DEG
check("defaults to straight down", main.camera_tilt_deg == -70.0)

ok, sent, persisted = command(main.camera_tilt_deg + main.camera_tilt_step_deg())
step = main.camera_tilt_step_deg()
check("3 us step is ~0.32 deg", abs(step - 3 * 140.0 / 1305) < 1e-9, f"step={step}")
check("up step commands -70 + one step", abs(sent - (-70.0 + step)) < 1e-9, f"sent={sent}")
check("accepted step is held", main.camera_tilt_deg == sent)
check("accepted step is persisted", persisted)

ok, sent, persisted = command(main.camera_tilt_deg + main.camera_tilt_step_deg(),
                              accepted=False)
check("rejected write reports failure", ok is False)
check("rejected write keeps the held angle", abs(main.camera_tilt_deg - (-70.0 + step)) < 1e-9)
check("rejected write is not persisted", not persisted)

ok, sent, _ = command(-200.0)
check("down past the limit clamps to -70", sent == -70.0, f"sent={sent}")
main.camera_tilt_deg = 69.9
ok, sent, _ = command(main.camera_tilt_deg + main.camera_tilt_step_deg())
check("up past the limit clamps to +70", sent == 70.0, f"sent={sent}")
ok, sent, _ = command(main.TILT_PITCH_DOWN_DEG)
check("TILT DOWN resets the hold", main.camera_tilt_deg == -70.0)

check("garbage config falls back to straight down",
      main._sanitize_camera_tilt_deg("nope") == -70.0)
check("saved config angle survives sanitizing",
      main._sanitize_camera_tilt_deg(-20) == -20.0)

main.camera_tilt_deg = -30.0
writer = mock.MagicMock()
writer.mount_pitch.return_value = True
with mock.patch.object(main, "get_default_writer", return_value=writer):
    main._startup_optics_worker()
check("boot reseats the held tilt, not straight down",
      writer.mount_pitch.call_args[0][0] == -30.0
      and not writer.tilt_down.called)


print("sonar confidence floor")


def classify(quality, distance=4.2, update="t1"):
    hist = main.SonarHistory()
    snap = {"distance_m": distance, "in_range": True,
            "signal_quality": quality, "last_update": update}
    return hist._classify_sample(snap)


accepted, reject = classify(95)
check("95 % is accepted", accepted is not None and reject is None)
accepted, reject = classify(90)
check("exactly 90 % is accepted", accepted is not None)
accepted, reject = classify(89)
check("89 % is rejected", accepted is None and reject == "low_quality")
accepted, reject = classify(None)
check("missing confidence is rejected", accepted is None and reject == "low_quality")


def vehicles(*publishers):
    tree = {}
    for sysid, compid, freq, quality in publishers:
        comp = tree.setdefault(str(sysid), {"components": {}})["components"]
        comp[str(compid)] = {"messages": {"DISTANCE_SENSOR": {
            "message": {"signal_quality": quality},
            "status": {"time": {"frequency": freq}},
        }}}
    return tree


picked = main._pick_fastest_distance_sensor(
    vehicles((1, 194, 2.8, 0), (2, 1, 0.97, 100)))
check("rated publisher beats a faster unrated one",
      picked is not None and picked[:2] == (2, 1), f"picked={picked}")
picked = main._pick_fastest_distance_sensor(
    vehicles((1, 194, 2.8, 98), (2, 1, 0.97, 100)))
check("fastest wins when both are rated",
      picked is not None and picked[:2] == (1, 194), f"picked={picked}")
picked = main._pick_fastest_distance_sensor(
    vehicles((1, 194, 2.8, 0), (2, 1, 0.97, 0)))
check("fastest still picked when none are rated",
      picked is not None and picked[:2] == (1, 194), f"picked={picked}")
check("no publishers -> None", main._pick_fastest_distance_sensor({}) is None)


if FAILURES:
    print(f"\n{len(FAILURES)} failure(s)")
    sys.exit(1)
print("\nall checks passed")
