"""Lumped-mass tow cable in NED.

Node 0 is the boat's tow point (prescribed), node N the fish's tow point
(prescribed by the fish each step; the cable returns the force it exerts
there). Segments are axial springs that cannot push, with quadratic normal
and tangential drag and wet weight. Integration is semi-implicit Euler at
the physics step.
"""
from __future__ import annotations

import math

import numpy as np

from .frames import Config


class Cable:
    def __init__(self, cfg: Config, rho: float):
        self.n = int(cfg["cable.segments"])
        self.length = float(cfg["cable.length"])
        self.l0 = self.length / self.n
        d = float(cfg["cable.diameter"]) / 1000.0
        self.d = d
        self.rho = rho
        self.node_mass = (float(cfg["cable.linear_mass"])
                          + rho * math.pi * d * d / 4.0) * self.l0
        self.k = float(cfg["cable.ea"]) / self.l0
        self.c = 2.0 * float(cfg["cable.damping_ratio"]) * math.sqrt(self.k * self.node_mass)
        self.wet = float(cfg["cable.wet_weight"]) * self.l0
        self.cdn = float(cfg["cable.cd_normal"])
        self.cdt = float(cfg["cable.cd_tangent"])
        self.x = np.zeros((self.n + 1, 3))
        self.v = np.zeros((self.n + 1, 3))
        self.tension = np.zeros(self.n)

    def lay_straight(self, top: np.ndarray, bottom: np.ndarray, vel: np.ndarray):
        """Initial layout: evenly spaced on the straight chord, moving at vel."""
        f = np.linspace(0.0, 1.0, self.n + 1)[:, None]
        self.x = top[None, :] * (1 - f) + bottom[None, :] * f
        self.v = np.repeat(vel[None, :], self.n + 1, axis=0)

    def step(self, dt: float, top_pos, top_vel, fish_pos, fish_vel,
             current: np.ndarray) -> np.ndarray:
        """Advance the free nodes; return the force on the fish (NED, N)."""
        x, v = self.x, self.v
        x[0], v[0] = top_pos, top_vel
        x[-1], v[-1] = fish_pos, fish_vel

        seg = x[1:] - x[:-1]
        ln = np.linalg.norm(seg, axis=1) + 1e-12
        u = seg / ln[:, None]
        stretch = ln - self.l0
        rel_v = np.sum((v[1:] - v[:-1]) * u, axis=1)
        t = np.where(stretch > 0.0, self.k * stretch + self.c * rel_v, 0.0)
        t = np.maximum(t, 0.0)
        self.tension = t
        ft = t[:, None] * u  # pulls node i toward i+1

        # Drag on each segment from the water velocity relative to it.
        w = current[None, :] - 0.5 * (v[1:] + v[:-1])
        wt = np.sum(w * u, axis=1)[:, None] * u
        wn = w - wt
        # Segments above the surface (boat end) see no water drag.
        mid_z = 0.5 * (x[1:, 2] + x[:-1, 2])
        wet = np.clip(mid_z / 0.02 + 0.5, 0.0, 1.0)[:, None]
        fd = 0.5 * self.rho * wet * (
            self.cdn * self.d * ln[:, None] * np.linalg.norm(wn, axis=1)[:, None] * wn
            + self.cdt * math.pi * self.d * ln[:, None] * np.linalg.norm(wt, axis=1)[:, None] * wt)

        f = np.zeros_like(x)
        f[:-1] += ft + 0.5 * fd
        f[1:] += -ft + 0.5 * fd
        f[:, 2] += self.wet

        v[1:-1] += f[1:-1] / self.node_mass * dt
        x[1:-1] += v[1:-1] * dt
        # Force the last segment applies to the fish: tension toward the
        # cable, plus that segment's share of drag at the fish end.
        return -ft[-1] + 0.5 * fd[-1]

    @property
    def fish_end_tension(self) -> float:
        return float(self.tension[-1])
