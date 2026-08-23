#!/usr/bin/env python3
"""Exercise main.py's altitude path without a vehicle.

main.py cannot be imported normally outside the container (flask, gi,
websockets), so the hardware surface is stubbed here and the sonar buffer
is driven directly. This covers the parts that only exist at runtime and
so are out of reach of test_photogrammetry_meta.py: the layback-delayed
sounding lookup, its tolerance window, and the offset arithmetic.

Run with: python3 test_altitude_path.py
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
    print(f"  {'ok  ' if condition else 'FAIL'} {label}" + (f" {detail}" if detail and not condition else ""))
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

# gi insists on require_version before the repository import.
gi = stub("gi", require_version=lambda *a, **k: None)
stub("gi.repository", Gst=mock.MagicMock())
gi.repository = sys.modules["gi.repository"]
stub("websockets", serve=mock.MagicMock())
stub("websockets.exceptions", ConnectionClosed=Exception)
sys.modules["websockets"].exceptions = sys.modules["websockets.exceptions"]
stub("usb_storage", get_base_dir=lambda: None, start_probe=lambda: None)
# mavlink_params / mavlink_writer are pure-python and import fine.

import main  # noqa: E402


print("tow delay from layback and speed")
main.tow_offset_m = 7.0
main._sonar_history._speed_ms = 1.2
check("7 m at 1.2 m/s is ~5.8 s",
      abs(main._sonar_history.tow_delay_s() - 7.0 / 1.2) < 1e-9)
main._sonar_history._speed_ms = 0.02
check("drifting boat yields no delay rather than a huge one",
      main._sonar_history.tow_delay_s() is None)
main._sonar_history._speed_ms = 1.2
main.tow_offset_m = 0.0
check("zero layback needs no delay", main._sonar_history.tow_delay_s() == 0.0)


print("\ndelayed lookup over a sloping bottom")
main.tow_offset_m = 7.0
main._sonar_history._speed_ms = 1.2
delay = 7.0 / 1.2
# Bottom shoaling 0.5 m/s under the boat: the instantaneous reading and
# the one the fish is actually over differ by ~2.9 m.
now = 1000.0
main._sonar_history._samples.clear()
for i in range(200):
    t = now - 100.0 + i * 0.5
    main._sonar_history._samples.append((t, 20.0 - 0.5 * (t - (now - 100.0))))

with mock.patch.object(main.time, "monotonic", return_value=now):
    hit = main._sonar_history.depth_at_tow_delay()
check("a sounding was matched", hit is not None)
if hit:
    depth, used_delay, err = hit
    instantaneous = main._sonar_history._samples[-1][1]
    check("delay matches layback/speed", abs(used_delay - delay) < 1e-9)
    check("matched sample is close in time", err <= main._SONAR_MATCH_TOLERANCE_S)
    check("delayed reading differs from the instantaneous one",
          abs(depth - instantaneous) > 2.0,
          f"delayed={depth:.2f} instant={instantaneous:.2f}")
    check("delayed reading is the older, deeper bottom", depth > instantaneous)


print("\nstale buffer is refused, not extrapolated")
main._sonar_history._samples.clear()
main._sonar_history._samples.append((now - 90.0, 12.0))
with mock.patch.object(main.time, "monotonic", return_value=now):
    check("a sounding 84 s off target is rejected",
          main._sonar_history.depth_at_tow_delay() is None)
main._sonar_history._samples.clear()
with mock.patch.object(main.time, "monotonic", return_value=now):
    check("an empty buffer yields no altitude",
          main._sonar_history.depth_at_tow_delay() is None)


print("\naltitude arithmetic")
main._sonar_history._samples.clear()
main._sonar_history._samples.append((now - delay, 11.75))
with mock.patch.object(main.time, "monotonic", return_value=now):
    main.altitude_offset_m = 0.0
    alt, detail = main.compute_towfish_altitude(3.5)
    check("sonar 11.75 - depth 3.5 = 8.25", abs(alt - 8.25) < 1e-9, f"got {alt}")
    check("detail records the sounding used", detail["sonar_depth_m"] == 11.75)

    # A transducer 0.25 m below the waterline shortens every sounding.
    main.altitude_offset_m = 0.25
    alt_off, _ = main.compute_towfish_altitude(3.5)
    check("offset is subtracted", abs(alt_off - 8.0) < 1e-9, f"got {alt_off}")

    main.altitude_offset_m = 0.0
    none_alt, _ = main.compute_towfish_altitude(None)
    check("no depth means no altitude", none_alt is None)

    # Fish reads deeper than the bottom: the geometry disagrees with
    # itself, so publishing a negative camera height would be worse
    # than publishing nothing.
    bad, _ = main.compute_towfish_altitude(15.0)
    check("a non-physical altitude is withheld", bad is None)


print("\naccuracy priors reach the shared model")
xy, z = main.photogrammetry_meta.reference_accuracy_m(7.0, 8.25)
check("XY prior is metres-scale", 3.0 <= xy <= 10.0, f"got {xy}")
check("Z prior is sub-metre", z < 1.0, f"got {z}")


print()
if FAILURES:
    print(f"{len(FAILURES)} FAILED: {FAILURES}")
    raise SystemExit(1)
print("all checks passed")
