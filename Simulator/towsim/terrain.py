"""Seabed shapes along a straight tow, and the boat's Ping looking at them.

Terrain specs (``--terrain``):

  slope        6 m flat for 60 m, down a 5 % grade to 3 m over 60 m, 3 m flat
               for 60 m, back to 6 m over 60 m, then 6 m flat
  coral        7 m bottom with a round, flat-topped head on the track at
               150 m: 3 m high (top at 4 m), 5 m across the top, sides
               tapering to the bottom over 1 m
  flat:<d>     flat bottom at d metres

Distance is measured along the boat's track from where the boat is at
``--start`` (s = 0), so the fish meets each feature one layback after
the boat does.

The Ping model replays the 4 Nov boat log's own packet timing: the
sequence of new RFND values (5.2 Hz, with its gaps) and which of them
had quality >= 90. Good packets report the slant range to the nearest
seabed inside the 30 deg beam (first return) plus 5 mm noise; the rest
report the kind of garbage the log shows (78-96 m at quality < 90), so
the extension's own quality gate has to drop them.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

BEAM_HALF_ANGLE_DEG = 15.0      # Ping1D: 30 deg beam (datasheet; not measured)
NOISE_M = 0.005                 # 4 Nov RFND, quality >= 90: ~5 mm
GOOD_QUALITY = 100
BAD_QUALITY = 50
FALLBACK_PERIOD_S = 0.19        # 5.2 Hz, if the log is not available


@dataclass
class Terrain:
    kind: str
    origin: np.ndarray           # boat N, E at s = 0
    heading: np.ndarray          # unit N, E along the track
    flat_depth: float = 6.0

    @classmethod
    def from_spec(cls, spec: str, origin, heading_rad: float) -> "Terrain":
        kind, _, arg = spec.partition(":")
        if kind not in ("slope", "coral", "flat"):
            raise ValueError(f"unknown terrain {spec!r}")
        h = np.array([math.cos(heading_rad), math.sin(heading_rad)])
        return cls(kind, np.asarray(origin[:2], dtype=float), h,
                   float(arg) if kind == "flat" else 6.0)

    def along_cross(self, n, e):
        d = np.array([n, e], dtype=float) - self.origin
        s = float(d @ self.heading)
        c = float(d[0] * -self.heading[1] + d[1] * self.heading[0])
        return s, c

    def depth_sc(self, s: float, c: float = 0.0) -> float:
        if self.kind == "flat":
            return self.flat_depth
        if self.kind == "slope":
            return slope_profile(s)
        return coral_profile(s, c)

    def depth_at(self, n: float, e: float) -> float:
        return self.depth_sc(*self.along_cross(n, e))

    def first_return(self, n: float, e: float, draft: float = 0.0) -> float:
        """Slant range to the nearest seabed inside the beam, from a
        transducer at (n, e) and ``draft`` below the surface."""
        s0, c0 = self.along_cross(n, e)
        below = self.depth_sc(s0, c0) - draft
        best = below
        rmax = max(below, 0.0) * math.tan(math.radians(BEAM_HALF_ANGLE_DEG))
        if rmax <= 0:
            return best
        for frac in (0.25, 0.5, 0.75, 1.0):
            r = rmax * frac
            for k in range(16):
                a = 2 * math.pi * k / 16
                ds, dc = r * math.cos(a), r * math.sin(a)
                h = self.depth_sc(s0 + ds, c0 + dc) - draft
                # Only returns inside the cone count: the slant to a point
                # r out is inside the beam if r <= h tan(15 deg).
                if h > 0 and r <= h * math.tan(math.radians(BEAM_HALF_ANGLE_DEG)) + 1e-9:
                    best = min(best, math.hypot(r, h))
        return best


def slope_profile(s: float) -> float:
    if s < 60:
        return 6.0
    if s < 120:
        return 6.0 - 3.0 * (s - 60) / 60
    if s < 180:
        return 3.0
    if s < 240:
        return 3.0 + 3.0 * (s - 180) / 60
    return 6.0


CORAL_S = 150.0
CORAL_BASE = 7.0
CORAL_TOP = 4.0
CORAL_TOP_R = 2.5
CORAL_SIDE = 1.0


def coral_profile(s: float, c: float) -> float:
    r = math.hypot(s - CORAL_S, c)
    if r <= CORAL_TOP_R:
        return CORAL_TOP
    if r >= CORAL_TOP_R + CORAL_SIDE:
        return CORAL_BASE
    return CORAL_TOP + (CORAL_BASE - CORAL_TOP) * (r - CORAL_TOP_R) / CORAL_SIDE


def logged_ping_events():
    """[(interval_s, ok)] for each new RFND value in the 4 Nov boat log,
    ok = quality >= 90; None when the log is not available."""
    try:
        from .logs import BOAT_BIN, read_bin
        r = read_bin(BOAT_BIN, {"RFND": ["Dist", "Quality"]})["RFND"]
    except Exception:
        return None
    t = np.asarray(r["t"])
    d = np.asarray(r["Dist"], dtype=float)
    q = np.asarray(r["Quality"], dtype=float)
    new = np.r_[True, np.diff(d) != 0]
    t, q = t[new], q[new]
    dt = np.diff(t)
    return list(zip(dt.tolist(), (q[1:] >= 90).tolist()))


class PingModel:
    """Packets on the logged schedule, starting at a seeded offset."""

    def __init__(self, terrain: Terrain, seed: int = 0, draft: float = 0.0):
        self.terrain = terrain
        self.draft = draft
        self.rng = np.random.default_rng(seed)
        ev = logged_ping_events()
        self.events = ev if ev else [(FALLBACK_PERIOD_S, True)]
        self.i = int(self.rng.integers(len(self.events)))
        self.next_t = None
        self.packet = None          # {"distance_m", "signal_quality", "last_update", ...}
        self.count = 0

    def update(self, t: float, n: float, e: float):
        """Advance to time t with the transducer at (n, e)."""
        if self.next_t is None:
            self.next_t = t
        while t >= self.next_t:
            dt, ok = self.events[self.i % len(self.events)]
            self.i += 1
            self.count += 1
            if ok:
                dist = self.terrain.first_return(n, e, self.draft) + self.rng.normal(0, NOISE_M)
                self.packet = {"distance_m": round(dist, 3), "signal_quality": GOOD_QUALITY,
                               "in_range": True, "last_update": self.count}
            else:
                self.packet = {"distance_m": round(float(self.rng.uniform(78, 96)), 3),
                               "signal_quality": BAD_QUALITY, "in_range": True,
                               "last_update": self.count}
            self.next_t += max(dt, 0.02)
        return self.packet
