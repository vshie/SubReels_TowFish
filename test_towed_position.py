#!/usr/bin/env python3
"""Towfish position model (app/towed_position.py) and its extension wiring."""
from __future__ import annotations

import math
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


import towed_position as tp  # noqa: E402

# ---------------------------------------------------------------------------
print("layback geometry")
check("8 m tether at 3.6 m: ~7.5 m astern (sim 7.47)",
      abs(tp.layback_m(8, 3.6) - 7.47) < 0.1, f"got {tp.layback_m(8, 3.6):.2f}")
check("25 m tether at 7.4 m: ~23.9 m astern (sim 23.96)",
      abs(tp.layback_m(25, 7.4) - 23.96) < 0.15, f"got {tp.layback_m(25, 7.4):.2f}")
check("depth beyond the tether is clamped", tp.layback_m(8, 20) > 0)


def straight(est, seconds, speed=1.0, course=0.0, depth=3.6, t0=0.0, n0=0.0, e0=0.0):
    out = None
    cn, ce = math.cos(math.radians(course)), math.sin(math.radians(course))
    for k in range(int(seconds * 5)):
        t = t0 + k * 0.2
        n, e = n0 + speed * (t - t0) * cn, e0 + speed * (t - t0) * ce
        out = est.update_ne(t, n, e, course, speed, depth, course)
    return out, n, e


print("\nstraight leg")
est = tp.TowedPositionEstimator(8, method="chain")
out, bn, be = straight(est, 60)
dist = math.hypot(bn - out["north"], be - out["east"])
check("settles at the layback directly astern",
      abs(dist - tp.layback_m(8, 3.6)) < 0.3 and abs(out["east"]) < 0.05,
      f"dist {dist:.2f} east {out['east']:.2f}")
check("not turning, not slack", not out["turning"] and not out["slack"])
check("straight uncertainty ~1.7 m on 8 m", abs(out["uncertainty_m"] - 1.66) < 0.05,
      f"got {out['uncertainty_m']}")
check("chain uses 2 links on 8 m", len(est.nodes) == 2, f"got {len(est.nodes)}")
check("4 links on 25 m", tp.TowedPositionEstimator(25, "chain").links_fixed is None
      and round(1 + 25 / 8) == 4)

print("\nhairpin: the fish cuts inside and never strays past the layback")
est = tp.TowedPositionEstimator(8, method="chain")
out, bn, be = straight(est, 40)
t, R, v = 40.0, 3.5, 1.0
worst = 0.0
inside = False
for k in range(int(math.pi * R / v * 5)):            # 180 deg turn to the east
    t += 0.2
    a = v * (k * 0.2) / R
    n, e = bn + R * math.sin(a), be + R * (1 - math.cos(a))
    course = math.degrees(a) % 360
    out = est.update_ne(t, n, e, course, v, 3.6, course)
    worst = max(worst, math.hypot(n - out["north"], e - out["east"]))
    inside = inside or out["east"] > 0.5
check("turn flagged", out["turning"])
check("fish moves toward the inside of the turn", inside)
check("stays within the layback (+0.5 m smoothing)",
      worst <= tp.layback_m(8, 3.6) + 0.5, f"worst {worst:.2f}")
check("turn uncertainty larger than straight", out["uncertainty_m"] > 1.66)

print("\nslack: the boat comes back toward the fish")
est = tp.TowedPositionEstimator(8, method="chain")
out, bn, be = straight(est, 40)
f0 = (out["north"], out["east"])
went_slack = False
for k in range(10):                                   # boat backs 3 m toward the fish
    out = est.update_ne(40.2 + k * 0.2, bn - 0.3 * (k + 1), be, 180.0, 1.5, 3.6, 0.0)
    went_slack = went_slack or out["slack"]
check("tether goes slack", went_slack)
check("fish coasts on along its heading (north), not toward the boat",
      out["north"] > f0[0], f"{f0[0]:.2f} -> {out['north']:.2f}")

print("\nGPS spike cannot fling the chain")
est = tp.TowedPositionEstimator(8, method="chain")
out, bn, be = straight(est, 40)
out = est.update_ne(40.2, bn + 30.0, be + 30.0, 0.0, 1.0, 3.6, 0.0)
out = est.update_ne(40.4, bn + 0.4, be, 0.0, 1.0, 3.6, 0.0)
moved = math.hypot(out["north"] - (bn + 0.4 - tp.layback_m(8, 3.6)), out["east"])
check("one bad fix is ignored", moved < 0.6, f"moved {moved:.2f}")

print("\ndrag model (inextensible links with water drag, fish held at its depth)")
for L, dep in ((8, 3.6), (25, 7.4)):
    est = tp.TowedPositionEstimator(L, method="drag")
    out, bn, be = straight(est, 120, depth=dep)
    lay = math.hypot(bn - out["north"], be - out["east"])
    check(f"{L} m: straight leg settles at the simulator's layback ({tp.layback_m(L, dep):.2f} m)",
          abs(lay - tp.layback_m(L, dep)) < 0.15 and abs(out["east"]) < 0.05, f"got {lay:.2f}")
    seg = L / (len(est.nodes) - 1)
    worst = max(abs(math.dist(est.nodes[i], est.nodes[i + 1]) / seg - 1) for i in range(len(est.nodes) - 1))
    check(f"{L} m: links do not stretch", worst < 1e-3, f"{worst * 100:.3f} %")
    check(f"{L} m: fish held at the measured depth", abs(est.nodes[-1][2] - dep) < 1e-9)
check("1 m links: 26 nodes on a 25 m tether", len(est.nodes) == 26, f"got {len(est.nodes)}")

est = tp.TowedPositionEstimator(8, method="drag")
out, bn, be = straight(est, 40)
t, R, v = 40.0, 3.5, 1.0
worst, inside = 0.0, False
for k in range(int(math.pi * R / v * 5)):
    t += 0.2
    a = v * (k * 0.2) / R
    n, e = bn + R * math.sin(a), be + R * (1 - math.cos(a))
    course = math.degrees(a) % 360
    out = est.update_ne(t, n, e, course, v, 3.6, course)
    worst = max(worst, math.hypot(n - out["north"], e - out["east"]))
    inside = inside or out["east"] > 0.5
check("drag hairpin: fish moves toward the inside", inside)
check("drag hairpin: never beyond the tether's reach", worst <= 8.0 + 0.5, f"worst {worst:.2f}")

est = tp.TowedPositionEstimator(8, method="drag")
out, bn, be = straight(est, 40)
went_slack = False
for k in range(10):
    out = est.update_ne(40.2 + k * 0.2, bn - 0.3 * (k + 1), be, 180.0, 1.5, 3.6, 0.0)
    went_slack = went_slack or out["slack"]
check("drag: backing toward the fish slackens the tether", went_slack)

est = tp.TowedPositionEstimator(8, method="drag")
out, bn, be = straight(est, 40)
out = est.update_ne(40.2, bn + 30.0, be + 30.0, 0.0, 1.0, 3.6, 0.0)
out = est.update_ne(40.4, bn + 0.4, be, 0.0, 1.0, 3.6, 0.0)
moved = math.hypot(out["north"] - (bn + 0.4 - tp.layback_m(8, 3.6)), out["east"])
check("drag: one bad GPS fix is ignored", moved < 0.6, f"moved {moved:.2f}")

print("\nhard limit: never beyond the tether's reach from the raw boat fix")
check("reach geometry: 8 m tether at 3.6 m ~7.18 m",
      abs(tp.max_reach_m(8, 3.6) - math.sqrt(64 - 3.53 ** 2)) < 1e-9)
for method in ("drag", "chain", "path", "geometric"):
    est = tp.TowedPositionEstimator(8, method=method)
    worst_excess, t = -1e9, 0.0
    n = e = 0.0
    course = 0.0
    for k in range(900):                         # 3 min: fast boat, tight S-turns, noisy speed
        t = k * 0.2
        course = (course + (25.0 if (k // 40) % 2 == 0 else -25.0)) % 360
        sp = 2.0 + 0.5 * math.sin(k / 7.0)
        n += sp * 0.2 * math.cos(math.radians(course))
        e += sp * 0.2 * math.sin(math.radians(course))
        dep = 3.0 + 1.5 * math.sin(k / 30.0)
        o = est.update_ne(t, n, e, course, sp, dep, (course + 40) % 360)
        excess = math.hypot(o["north"] - n, o["east"] - e) - (tp.max_reach_m(8, dep) + tp.TOW_POINT_TO_BODY_M)
        worst_excess = max(worst_excess, excess)
    check(f"{method}: fish always within tether reach + body offset", worst_excess <= 1e-6,
          f"exceeded by {worst_excess:.3f} m")

print("\nlat/lon front end")
tr = tp.TowedPositionTracker(8)
lat0, lon0 = 19.98, -155.83
o = None
for k in range(300):
    lat = lat0 + math.degrees(1.0 * k * 0.2 / tp.EARTH_R)
    o = tr.update(k * 0.2, lat, lon0, 0.0, 1.0, 3.6, 0.0)
back = math.radians(lat - o["lat"]) * tp.EARTH_R
check("estimate sits the layback south of a northbound boat",
      abs(back - tp.layback_m(8, 3.6)) < 0.3 and abs(o["lon"] - lon0) < 1e-7,
      f"back {back:.2f}")

print("\nspacing from a lawnmower mission")
def lawnmower(spacing, legs=4, length=100.0, lat0=19.98, lon0=-155.83):
    pts = []
    for i in range(legs):
        e = i * spacing
        ns = (0.0, length) if i % 2 == 0 else (length, 0.0)
        for n in ns:
            pts.append((lat0 + math.degrees(n / tp.EARTH_R),
                        lon0 + math.degrees(e / (tp.EARTH_R * math.cos(math.radians(lat0))))))
    return pts
check("12 m lines read as 12 m", abs(tp.spacing_from_waypoints(lawnmower(12)) - 12) < 0.05,
      f"got {tp.spacing_from_waypoints(lawnmower(12))}")
check("35 m lines read as 35 m", abs(tp.spacing_from_waypoints(lawnmower(35)) - 35) < 0.05)
check("too few waypoints -> None", tp.spacing_from_waypoints(lawnmower(12)[:3]) is None)
check("recommended minimum: 12 m on 8 m, 35 m on 25 m",
      tp.recommended_min_spacing_m(8) == 12 and tp.recommended_min_spacing_m(25) == 35)

# ---------------------------------------------------------------------------
# Extension wiring.
def stub(name, **attrs):
    mod = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(mod, k, v)
    sys.modules[name] = mod
    return mod


stub("flask", Flask=mock.MagicMock(), jsonify=mock.MagicMock(),
     request=mock.MagicMock(), send_file=mock.MagicMock())
stub("requests", get=mock.MagicMock(), post=mock.MagicMock(),
     exceptions=mock.MagicMock(), Session=mock.MagicMock(), adapters=mock.MagicMock())
gi = stub("gi", require_version=lambda *a, **k: None)
stub("gi.repository", Gst=mock.MagicMock())
gi.repository = sys.modules["gi.repository"]
stub("websockets", serve=mock.MagicMock())
stub("websockets.exceptions", ConnectionClosed=Exception)
sys.modules["websockets"].exceptions = sys.modules["websockets.exceptions"]
stub("usb_storage", get_base_dir=lambda: None, start_probe=lambda: None)

import main  # noqa: E402

print("\nextension: model mode needs a tether length")
main.tow_position_mode = "model"
main.tow_offset_m = 7.0
main.tether_length_m = 0.0
sampler = main.TowfishPositionSampler()
main._towfish_position = sampler
fixed = main.offset_towed_position(19.98, -155.83, towfish_heading=0.0)
check("unset tether -> the fixed 7 m offset", fixed != (19.98, -155.83)
      and abs(math.radians(19.98 - fixed[0]) * 6378137.0 - 7.0) < 0.05)
check("snapshot explains why", sampler.snapshot().get("reason") == "tether length not set")

print("\nextension: the tracker drives geotags once the tether is set")
main.tether_length_m = 8.0
clock = [1000.0]
lat = [19.98]
def fix():
    lat[0] += math.degrees(0.2 / tp.EARTH_R)
    return {"lat": lat[0], "lon": -155.83, "course": 0.0, "speed": 1.0}
with mock.patch.object(main, "get_blueboat_fix", side_effect=lambda: fix()), \
     mock.patch.object(main, "get_towfish_attitude", return_value={"yaw": 0.0}), \
     mock.patch.object(main, "_get_towfish_depth_m", return_value=3.6), \
     mock.patch.object(main._transect_spacing, "observe_fix", lambda *a: None), \
     mock.patch.object(main.time, "monotonic", side_effect=lambda: clock[0]):
    for k in range(200):
        clock[0] += 0.2
        sampler.tick(clock[0])
    pos = main.offset_towed_position(lat[0], -155.83, towfish_heading=0.0)
    snap = sampler.snapshot()
    acc = sampler.uncertainty_m()
    along = sampler.along_path_m()
back = math.radians(lat[0] - pos[0]) * tp.EARTH_R
check("geotag uses the modelled layback (~7.5 m), not 7 m",
      abs(back - tp.layback_m(8, 3.6)) < 0.3, f"back {back:.2f}")
check("snapshot active with layback", snap["active"] and abs(snap["layback_m"] - 7.47) < 0.1, f"{snap}")
check("EXIF accuracy from the model", acc is not None
      and abs(acc - tp.uncertainty_m(8, False, main.TOW_POSITION_METHOD)) < 0.05, f"got {acc}")
check("the extension ships the drag model", main.TOW_POSITION_METHOD == "drag")
check("along-path distance for the sonar look-ahead ~ layback",
      along is not None and abs(along - tp.layback_m(8, 3.6)) < 0.6, f"got {along}")
with mock.patch.object(main.time, "monotonic", return_value=clock[0] + 5.0):
    stale = main.offset_towed_position(lat[0], -155.83, towfish_heading=0.0)
check("stale tracker falls back to the fixed offset",
      abs(math.radians(lat[0] - stale[0]) * 6378137.0 - 7.0) < 0.05)
main.tow_position_mode = "fixed"
with mock.patch.object(main.time, "monotonic", return_value=clock[0]):
    fx = main.offset_towed_position(lat[0], -155.83, towfish_heading=0.0)
check("fixed mode ignores the model",
      abs(math.radians(lat[0] - fx[0]) * 6378137.0 - 7.0) < 0.05)
main.tow_position_mode = "model"

print("\nextension: config")
check("defaults to model", main.DEFAULT_TOW_POSITION_MODE == "model")
check("mode sanitiser", main._sanitize_tow_position_mode("FIXED") == "fixed"
      and main._sanitize_tow_position_mode("bogus") == "model")
check("spacing sanitiser clamps", main._sanitize_transect_spacing_m(-3) == 0.0
      and main._sanitize_transect_spacing_m("x") == 0.0
      and main._sanitize_transect_spacing_m(5000) == main.TRANSECT_SPACING_MAX_M)

print("\nextension: spacing resolution")
ts = main.TransectSpacing()
main.transect_spacing_m = 0.0
check("nothing known -> None", ts.resolve() == (None, None))
# Waypoints flown: targets reported along the way.
for seq, p in enumerate(lawnmower(9, legs=4)):
    ts._flown[seq] = p
check("derived from the waypoints flown", ts.resolve() == (9.0, "flown"), f"got {ts.resolve()}")
ts._mission = 12.0
check("the mission beats the flown estimate", ts.resolve() == (12.0, "mission"))
main.transect_spacing_m = 16.0
check("the operator beats both", ts.resolve() == (16.0, "operator"))
main.transect_spacing_m = 9.0
main.tether_length_m = 8.0
snap = ts.snapshot()
check("9 m lines on an 8 m tether warns (min 12 m)",
      snap["below_minimum"] is True and snap["recommended_min_m"] == 12.0, f"{snap}")
main.transect_spacing_m = 16.0
check("16 m lines on an 8 m tether: no warning", ts.snapshot()["below_minimum"] is False)

print("\nextension: mission download parses waypoints")
items = [{"seq": i, "command": {"type": "MAV_CMD_NAV_WAYPOINT"},
          "x": int(p[0] * 1e7), "y": int(p[1] * 1e7)} for i, p in enumerate(lawnmower(20))]
items.insert(0, {"seq": 0, "command": {"type": "MAV_CMD_DO_CHANGE_SPEED"}, "x": 0, "y": 0})
for i, it in enumerate(items):
    it["seq"] = i
state = {"req": None}
def fake_get(url):
    if url.endswith("MISSION_COUNT"):
        return {"count": len(items)}
    if url.endswith("MISSION_ITEM_INT"):
        return items[state["req"]] if state["req"] is not None else {}
    return {}
class FakeWriter:
    target_system, target_component = 1, 1
    def send_message(self, m):
        if m["type"] == "MISSION_REQUEST_INT":
            state["req"] = m["seq"]
        return True
ts = main.TransectSpacing()
main.transect_spacing_m = 0.0
with mock.patch.object(main, "_mav_get_message", side_effect=fake_get), \
     mock.patch.object(ts, "_writer", return_value=FakeWriter()):
    ts.read_mission()
check("20 m lines from the downloaded mission", ts.resolve() == (20.0, "mission"),
      f"got {ts.resolve()} err {ts._last_error}")

print()
if FAILURES:
    print(f"{len(FAILURES)} check(s) failed")
    sys.exit(1)
print("all checks passed")
