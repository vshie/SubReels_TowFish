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
    main._sonar_history._samples.append((t, 20.0 - 0.5 * (t - (now - 100.0)), None, None))

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
main._sonar_history._samples.append((now - 90.0, 12.0, None, None))
with mock.patch.object(main.time, "monotonic", return_value=now):
    check("a sounding 84 s off target is rejected",
          main._sonar_history.depth_at_tow_delay() is None)
main._sonar_history._samples.clear()
with mock.patch.object(main.time, "monotonic", return_value=now):
    check("an empty buffer yields no altitude",
          main._sonar_history.depth_at_tow_delay() is None)


print("\naltitude arithmetic")
main._sonar_history._samples.clear()
main._sonar_history._samples.append((now - delay, 11.75, None, None))
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


print("\naltitude at the fish's position (straight line or turn)")
import math  # noqa: E402

LAT0, LON0 = 21.3, -157.8
K_LAT = math.pi / 180.0 * main.towed_position.EARTH_R
K_LON = K_LAT * math.cos(math.radians(LAT0))


def ll(n, e):
    return (LAT0 + n / K_LAT, LON0 + e / K_LON)


def bed(n, e):
    return 10.0 + 0.5 * e          # seabed deepens 0.5 m per metre east


# A hairpin at 1 m/s: north up the line at e = 0, a half circle of radius 4
# m, back south at e = 8. One sounding every 0.5 s, stamped with where the
# boat was.
pts = [(n, 0.0) for n in [i * 0.5 for i in range(41)]]
pts += [(20.0 + 4.0 * math.sin(a), 4.0 - 4.0 * math.cos(a))
        for a in [k * math.pi / 25 for k in range(1, 26)]]
pts += [(20.0 - i * 0.5, 8.0) for i in range(1, 21)]
now = 2000.0
main._sonar_history._samples.clear()
for i, (n, e) in enumerate(pts):
    la, lo = ll(n, e)
    main._sonar_history._samples.append((now - (len(pts) - i) * 0.5, bed(n, e), la, lo))

hit = None
la, lo = ll(19.0, 0.2)                       # on the first leg
with mock.patch.object(main.time, "monotonic", return_value=now):
    hit = main._sonar_history.depth_at_position(la, lo, 2.0)
check("soundings near the point are used", hit is not None and abs(hit[0] - bed(19.0, 0.0)) < 0.05,
      f"got {hit}")
la, lo = ll(10.0, 30.0)
with mock.patch.object(main.time, "monotonic", return_value=now):
    check("nothing within range gives no sounding",
          main._sonar_history.depth_at_position(la, lo, 2.0) is None)

# The fish cutting inside the turn, 8 m behind the boat along the path:
# the model puts it near (21, 3); the time delay would replay the boat's
# own track 8 m back.
fish_n, fish_e = 21.5, 3.0
fla, flo = ll(fish_n, fish_e)
est = (now, 0.0, 0.0, {"lat": fla, "lon": flo, "uncertainty_m": 1.7})
main.tow_position_mode = "model"
main.tow_offset_m = 8.0
main.altitude_offset_m = 0.0
main._sonar_history._speed_ms = 1.0
with mock.patch.object(main.time, "monotonic", return_value=now), \
        mock.patch.object(main._towfish_position, "estimate", return_value=est), \
        mock.patch.object(main._towfish_position, "along_path_m", return_value=8.0):
    alt, detail = main.compute_towfish_altitude(3.0)
    main.altitude_lookup = "delay"
    alt_d, detail_d = main.compute_towfish_altitude(3.0)
    main.altitude_lookup = "position"
truth = bed(fish_n, fish_e) - 3.0
check("in the turn: source is the position model", detail["source"] == "position", f"got {detail}")
check("in the turn: within 0.8 m of the seabed under the fish (0.5 m/m cross-slope)",
      alt is not None and abs(alt - truth) < 0.8, f"got {alt} truth {truth:.2f}")
check("the time delay reads the boat's track instead",
      detail_d["source"] == "delay" and abs(alt_d - truth) > abs(alt - truth),
      f"delay {alt_d} position {alt} truth {truth:.2f}")
check("match distance reported", detail["match_m"] is not None and detail["match_m"] <= 3.4,
      f"got {detail['match_m']}")
with mock.patch.object(main.time, "monotonic", return_value=now), \
        mock.patch.object(main._towfish_position, "estimate", return_value=None):
    _, detail_n = main.compute_towfish_altitude(3.0)
check("no model estimate falls back to the delay", detail_n["source"] == "delay", f"got {detail_n}")
far = (now, 0.0, 0.0, {"lat": ll(60.0, 60.0)[0], "lon": ll(60.0, 60.0)[1], "uncertainty_m": 1.7})
with mock.patch.object(main.time, "monotonic", return_value=now), \
        mock.patch.object(main._towfish_position, "estimate", return_value=far):
    a_far, detail_f = main.compute_towfish_altitude(3.0)
check("fish over unsounded ground: the tow-delay sounding, flagged DELAY",
      a_far is not None and detail_f["source"] == "delay", f"got {detail_f}")
main._sonar_history._samples.clear()
with mock.patch.object(main.time, "monotonic", return_value=now), \
        mock.patch.object(main._towfish_position, "estimate", return_value=far):
    a_none, detail_none = main.compute_towfish_altitude(3.0)
check("nothing anywhere: no altitude, with a reason",
      a_none is None and "no sounding" in detail_none["reason"], f"got {detail_none}")

fix = {"lat": LAT0, "lon": LON0}
h = main.SonarHistory()
with mock.patch.object(main.time, "monotonic", return_value=now):
    h.sample_once({"distance_m": 6.0, "in_range": True, "signal_quality": 100, "last_update": 1}, 1.0, fix)
check("sampler stores the boat position with the sounding",
      list(h._samples) == [(now, 6.0, LAT0, LON0)], f"got {list(h._samples)}")


print("\naccuracy priors reach the shared model")
xy, z = main.photogrammetry_meta.reference_accuracy_m(7.0, 8.25)
check("XY prior is metres-scale", 3.0 <= xy <= 10.0, f"got {xy}")
check("Z prior is sub-metre", z < 1.0, f"got {z}")


print()
if FAILURES:
    print(f"{len(FAILURES)} FAILED: {FAILURES}")
    raise SystemExit(1)
print("all checks passed")
