#!/usr/bin/env python3
"""Terrain profiles and the boat's Ping model (towsim/terrain.py).

Run with: python3 Simulator/tests/test_terrain.py
"""
from __future__ import annotations

import math

from _check import check, finish
from towsim import terrain as T

print("slope profile")
for s, want in ((0, 6.0), (59, 6.0), (90, 4.5), (120, 3.0), (150, 3.0), (210, 4.5), (300, 6.0)):
    check(f"s={s} m -> {want} m", abs(T.slope_profile(s) - want) < 1e-9, f"got {T.slope_profile(s)}")
check("5 % grade", abs((T.slope_profile(70) - T.slope_profile(80)) / 10 - 0.05) < 1e-9)

print("\ncoral head")
check("7 m bottom away from the head", T.coral_profile(100, 0) == 7.0)
check("4 m flat top, 5 m across", T.coral_profile(150, 0) == 4.0 and T.coral_profile(152.5, 0) == 4.0
      and T.coral_profile(150, 2.4) == 4.0)
check("sides taper over 1 m", abs(T.coral_profile(153.0, 0) - 5.5) < 1e-9
      and T.coral_profile(153.5, 0) == 7.0)

print("\nframes")
ter = T.Terrain.from_spec("coral", [10.0, 20.0], math.radians(90))   # heading east
check("along-track distance runs along the heading", abs(ter.along_cross(10.0, 170.0)[0] - 150) < 1e-9)
check("depth_at follows the track", ter.depth_at(10.0, 170.0) == 4.0 and ter.depth_at(10.0, 100.0) == 7.0)
check("flat:<d>", T.Terrain.from_spec("flat:8.5", [0, 0], 0.0).depth_at(3, 4) == 8.5)

print("\nPing beam (first return)")
ter = T.Terrain.from_spec("coral", [0.0, 0.0], 0.0)
edge = T.CORAL_S - T.CORAL_TOP_R - T.CORAL_SIDE                   # 146.5 m: base of the head
r_beam = 7.0 * math.tan(math.radians(T.BEAM_HALF_ANGLE_DEG))      # ~1.9 m at 7 m
check("flat bottom reads the depth", abs(ter.first_return(100.0, 0.0) - 7.0) < 1e-9)
early = ter.first_return(edge - 0.5 * r_beam, 0.0)
check("sees the head before the boat is over it", early < 6.9, f"got {early:.2f}")
check("not before the beam reaches it", ter.first_return(edge - 1.2 * r_beam, 0.0) == 7.0)
check("reads the top when over it", abs(ter.first_return(150.0, 0.0) - 4.0) < 1e-9)

print("\nPing packets")
ping = T.PingModel(T.Terrain.from_spec("flat:6", [0, 0], 0.0), seed=1)
ids, good, bad = set(), 0, 0
for k in range(6000):                                             # 600 s at 10 Hz
    pk = ping.update(k * 0.1, 0.0, 0.0)
    if pk["last_update"] in ids:
        continue
    ids.add(pk["last_update"])
    if pk["signal_quality"] >= 90:
        good += 1
        check_ok = abs(pk["distance_m"] - 6.0) < 0.05
        if not check_ok:
            check("good packet near the depth", False, f"got {pk}")
    else:
        bad += 1
        if pk["distance_m"] < 30:
            check("bad packet is garbage the extension's range cap rejects", False, f"got {pk}")
rate = (good + bad) / 600
check("packet rate near the logged ~5 Hz", 3.5 < rate < 6.5, f"got {rate:.2f} Hz")
check("some low-quality packets, as logged", bad > 0, f"got {bad}")

finish("terrain")
