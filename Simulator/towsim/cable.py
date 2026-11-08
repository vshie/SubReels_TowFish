"""Lumped-mass tow cable in NED.

Node 0 is the boat's tow point (prescribed), node N the fish's tow point.

Two models (``cable.model``):

* ``inextensible`` (default) -- the Fathom tether has a Kevlar strength
  member and does not stretch. Each segment is a fixed-length link that can
  carry tension but never push. Drag (normal and tangential, quadratic) and
  wet weight move the nodes; then one velocity-level solve finds the link
  tensions that stop every taut link from lengthening. The links couple only
  to their neighbours, so that is a tridiagonal system, solved exactly each
  step, with an active-set pass that drops any link that would have to push
  (it is slack). The fish end uses the fish's effective inverse mass at its
  tow point (rigid body + added mass), so a slack-to-taut snap hits the fish
  as hard as the physics says. A small position correction removes drift.
* ``spring`` -- the original model: axial springs (``cable.ea``) that cannot
  push, explicit semi-implicit Euler. Kept for comparison; its stiffness is
  numerical, chosen for a 1 ms step, not the tether's.
"""
from __future__ import annotations

import math

import numpy as np

from .frames import Config

POS_CORRECTION = 0.2      # fraction of a link's over-length removed per step


class Cable:
    def __init__(self, cfg: Config, rho: float):
        try:
            self.model = str(cfg["cable.model"])
        except Exception:
            self.model = "inextensible"
        self.n = int(cfg["cable.segments"])
        self.length = float(cfg["cable.length"])
        self.l0 = self.length / self.n
        d = float(cfg["cable.diameter"]) / 1000.0
        self.d = d
        self.rho = rho
        self.node_mass = (float(cfg["cable.linear_mass"])
                          + rho * math.pi * d * d / 4.0) * self.l0
        if self.model == "spring":
            self.k = float(cfg["cable.ea"]) / self.l0
            self.c = 2.0 * float(cfg["cable.damping_ratio"]) * math.sqrt(self.k * self.node_mass)
        self.wet = float(cfg["cable.wet_weight"]) * self.l0
        self.cdn = float(cfg["cable.cd_normal"])
        self.cdt = float(cfg["cable.cd_tangent"])
        self.x = np.zeros((self.n + 1, 3))
        self.v = np.zeros((self.n + 1, 3))
        self.tension = np.zeros(self.n)
        self.lam = np.zeros(self.n)        # last link impulses (N s)

    def lay_straight(self, top: np.ndarray, bottom: np.ndarray, vel: np.ndarray):
        """Initial layout: evenly spaced on the straight chord, moving at vel."""
        f = np.linspace(0.0, 1.0, self.n + 1)[:, None]
        self.x = top[None, :] * (1 - f) + bottom[None, :] * f
        self.v = np.repeat(vel[None, :], self.n + 1, axis=0)

    # -- shared: drag -----------------------------------------------------
    def _drag(self, current):
        x, v = self.x, self.v
        seg = x[1:] - x[:-1]
        ln = np.linalg.norm(seg, axis=1) + 1e-12
        u = seg / ln[:, None]
        w = current[None, :] - 0.5 * (v[1:] + v[:-1])
        wt = np.sum(w * u, axis=1)[:, None] * u
        wn = w - wt
        # Segments above the surface (boat end) see no water drag.
        mid_z = 0.5 * (x[1:, 2] + x[:-1, 2])
        wet = np.clip(mid_z / 0.02 + 0.5, 0.0, 1.0)[:, None]
        fd = 0.5 * self.rho * wet * (
            self.cdn * self.d * ln[:, None] * np.linalg.norm(wn, axis=1)[:, None] * wn
            + self.cdt * math.pi * self.d * ln[:, None] * np.linalg.norm(wt, axis=1)[:, None] * wt)
        return fd, u, ln

    # -- spring model -----------------------------------------------------
    def step(self, dt: float, top_pos, top_vel, fish_pos, fish_vel,
             current: np.ndarray) -> np.ndarray:
        """Advance the free nodes with both ends prescribed; return the force
        on the fish end (NED, N). Used directly by tests and the jig."""
        if self.model != "spring":
            drag_end = self.predict(dt, top_pos, top_vel, fish_pos, fish_vel, current)
            imp = self.solve(dt, np.asarray(fish_vel, dtype=float), None)
            self.advance(dt, fish_pos, fish_vel)
            return imp / dt + drag_end
        x, v = self.x, self.v
        x[0], v[0] = top_pos, top_vel
        x[-1], v[-1] = fish_pos, fish_vel

        fd, u, ln = self._drag(current)
        stretch = ln - self.l0
        rel_v = np.sum((v[1:] - v[:-1]) * u, axis=1)
        t = np.where(stretch > 0.0, self.k * stretch + self.c * rel_v, 0.0)
        t = np.maximum(t, 0.0)
        self.tension = t
        ft = t[:, None] * u  # pulls node i toward i+1

        f = np.zeros_like(x)
        f[:-1] += ft + 0.5 * fd
        f[1:] += -ft + 0.5 * fd
        f[:, 2] += self.wet

        v[1:-1] += f[1:-1] / self.node_mass * dt
        x[1:-1] += v[1:-1] * dt
        return -ft[-1] + 0.5 * fd[-1]

    # -- inextensible model -------------------------------------------------
    def predict(self, dt, top_pos, top_vel, fish_pos, fish_vel, current):
        """Drag and wet weight on the free nodes. Returns the drag share the
        last segment puts on the fish end (an external force on the fish)."""
        x, v = self.x, self.v
        x[0], v[0] = top_pos, top_vel
        x[-1], v[-1] = fish_pos, fish_vel
        fd, _, _ = self._drag(current)
        f = np.zeros_like(x)
        f[:-1] += 0.5 * fd
        f[1:] += 0.5 * fd
        f[:, 2] += self.wet
        v[1:-1] += f[1:-1] / self.node_mass * dt
        return 0.5 * fd[-1]

    def solve(self, dt, fish_vel, fish_k):
        """Link tensions for this step.

        ``fish_vel`` is the fish tow point's (predicted) NED velocity and
        ``fish_k`` its 3x3 effective inverse mass in NED (None while the fish
        is held on the jig: infinite mass). Updates the free nodes' velocity
        and returns the impulse (NED, N s) the tether applies to the fish.
        """
        x, v = self.x, self.v
        n = self.n
        v[-1] = fish_vel
        seg = x[1:] - x[:-1]
        ln = np.linalg.norm(seg, axis=1) + 1e-12
        u = seg / ln[:, None]
        vs = np.sum((v[1:] - v[:-1]) * u, axis=1)          # separation speed
        C = ln - self.l0
        # Target separation speed: remove a share of any over-length; a slack
        # link may close up to exactly its length this step.
        vt = np.where(C > 0.0, -POS_CORRECTION * C / dt, -C / dt)
        winv = np.full(n + 1, 1.0 / self.node_mass)
        winv[0] = 0.0                                       # boat: prescribed
        wfish = float(u[-1] @ fish_k @ u[-1]) if fish_k is not None else 0.0
        winv[-1] = wfish
        cos_next = np.sum(u[1:] * u[:-1], axis=1)           # u_i . u_{i+1}

        active = vs > vt                                    # would over-stretch
        lam = np.zeros(n)
        for _ in range(8):
            a = np.zeros(n); b = np.zeros(n); c = np.zeros(n); r = np.zeros(n)
            for i in range(n):
                if not active[i]:
                    b[i] = 1.0
                    continue
                b[i] = -(winv[i] + winv[i + 1])
                if i > 0 and active[i - 1]:
                    a[i] = winv[i] * cos_next[i - 1]
                if i < n - 1 and active[i + 1]:
                    c[i] = winv[i + 1] * cos_next[i]
                r[i] = vt[i] - vs[i]
            lam = _thomas(a, b, c, r)
            neg = active & (lam < 0.0)
            if neg.any():                                   # would push: slack
                active &= ~neg
                continue
            # Check inactive links are not now over-stretching.
            dv = self._apply(lam, u, winv)
            vs_new = np.sum(((v[1:] + dv[1:]) - (v[:-1] + dv[:-1])) * u, axis=1)
            newly = (~active) & (vs_new > vt + 1e-9)
            if not newly.any():
                break
            active |= newly
        lam = np.maximum(lam, 0.0) * active
        self.lam = lam
        self.tension = lam / dt
        dv = self._apply(lam, u, winv)
        v[1:-1] += dv[1:-1]
        return -lam[-1] * u[-1]                             # impulse on the fish

    def _apply(self, lam, u, winv):
        """Velocity changes from link impulses (fish end handled by caller)."""
        imp = lam[:, None] * u                              # on node i toward i+1
        dv = np.zeros_like(self.v)
        dv[:-1] += imp * winv[:-1, None]
        dv[1:] -= imp * winv[1:, None]
        return dv

    def advance(self, dt, fish_pos, fish_vel):
        """Integrate the free nodes once the fish has its new state."""
        self.x[1:-1] += self.v[1:-1] * dt
        self.x[-1], self.v[-1] = fish_pos, fish_vel

    @property
    def fish_end_tension(self) -> float:
        return float(self.tension[-1])


def _thomas(a, b, c, r):
    """Solve a tridiagonal system (a: sub, b: main, c: super diagonals)."""
    n = len(b)
    cp = np.zeros(n); rp = np.zeros(n)
    cp[0] = c[0] / b[0]
    rp[0] = r[0] / b[0]
    for i in range(1, n):
        m = b[i] - a[i] * cp[i - 1]
        cp[i] = c[i] / m if i < n - 1 else 0.0
        rp[i] = (r[i] - a[i] * rp[i - 1]) / m
    out = np.zeros(n)
    out[-1] = rp[-1]
    for i in range(n - 2, -1, -1):
        out[i] = rp[i] - cp[i] * out[i + 1]
    return out
