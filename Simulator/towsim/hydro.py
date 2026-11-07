"""Strip-theory hydrodynamics for the towfish.

Every lifting surface (both servo wings, the cruciform tail) is cut into
spanwise strips. Each strip sees the local water velocity including the
body's rotation, so roll/pitch/yaw damping and sideslip effects come out of
the geometry instead of being assigned. Bluff parts (housing, top bar, tube
clamp) are slender elements with cross-flow drag. Nothing here knows about
turns, the log or ArduSub.

Section convention: span axis `s`, chord axis `c` (pointing to the leading
edge), "up" normal n = -(c x s). With water velocity w relative to the strip
(in-plane part w2), alpha = atan2(w2.n, -w2.c); lift acts along w2 x s and
drag along w2. A positive (nose-up) wing deflection adds to alpha.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .frames import Config, step_to_body

GROUP_FIXED, GROUP_PORT, GROUP_STBD = 0, 1, 2


@dataclass
class Strips:
    pos: np.ndarray        # (N,3) body m, quarter-chord point
    span: np.ndarray       # (N,3) unit span axis
    chord: np.ndarray      # (N,3) unit chord axis (toward leading edge)
    c: np.ndarray          # (N,) chord length m
    ds: np.ndarray         # (N,) span width m
    slope: np.ndarray      # (N,) finite-span lift slope 1/rad
    ar: np.ndarray         # (N,)
    group: np.ndarray      # (N,) GROUP_*
    incidence: np.ndarray  # (N,) rad

    @property
    def area(self):
        return self.c * self.ds

    @property
    def normal(self):
        return -np.cross(self.chord, self.span)


@dataclass
class Elements:
    pos: np.ndarray        # (M,3) body m, segment centre
    length: np.ndarray     # (M,) m along body x
    width_h: np.ndarray    # (M,) horizontal extent m (faces vertical flow)
    width_v: np.ndarray    # (M,) vertical extent m (faces lateral flow)
    cd: np.ndarray         # (M,)


def lift_slope(ar: float) -> float:
    return 2 * math.pi * ar / (ar + 2.0)


def build_geometry(cfg: Config, cg_mm: np.ndarray) -> tuple[Strips, Elements]:
    rows = []  # (pos_mm, span_body, chord_body, c_m, ds_m, ar, group, inc)
    x_axis = np.array([1.0, 0.0, 0.0])
    y_axis = np.array([0.0, 1.0, 0.0])
    z_axis = np.array([0.0, 0.0, 1.0])

    # Wings: interpolate the measured stations onto equal-width strips.
    st = cfg["wings.stations"]
    zs = np.array([s["z"] for s in st])
    les = np.array([s["le"] for s in st])
    tes = np.array([s["te"] for s in st])
    n = int(cfg["wings.strips_per_wing"])
    edges = np.linspace(zs[0], zs[-1], n + 1)
    ar_w = float(cfg["foil.ar_wing"])
    inc = math.radians(float(cfg["wings.incidence"]))
    x_w = float(cfg["wings.chord_line_x"])
    for side, group in ((1.0, GROUP_STBD), (-1.0, GROUP_PORT)):
        for a, b in zip(edges[:-1], edges[1:]):
            zm = 0.5 * (a + b)
            le = float(np.interp(zm, zs, les))
            te = float(np.interp(zm, zs, tes))
            chord_mm = te - le
            yq = le + 0.25 * chord_mm
            rows.append(([x_w, yq, side * zm], y_axis, x_axis, chord_mm / 1000,
                         (b - a) / 1000, ar_w, group, inc))

    # Tail: horizontal halves on the tube axis height (ASSUMED x = 0, the
    # report gives only the cruciform's overall extents), vertical above and
    # below.
    tc = float(cfg["tail.chord"])
    yq = float(cfg["tail.y_le"]) + 0.25 * tc
    hs = float(cfg["tail.half_span_h"])
    k = int(cfg["tail.strips"])
    ar_h = float(cfg["foil.ar_tail_h"])
    ar_v = float(cfg["foil.ar_tail_v"])
    for side in (1.0, -1.0):
        e = np.linspace(0.0, hs, k + 1)
        for a, b in zip(e[:-1], e[1:]):
            rows.append(([0.0, yq, side * 0.5 * (a + b)], y_axis, x_axis,
                         tc / 1000, (b - a) / 1000, ar_h, GROUP_FIXED, 0.0))
    for lo, hi in ((0.0, float(cfg["tail.v_top"])), (float(cfg["tail.v_bottom"]), 0.0)):
        e = np.linspace(lo, hi, k + 1)
        for a, b in zip(e[:-1], e[1:]):
            # Vertical surface: span along body z (down).
            rows.append(([0.5 * (a + b), yq, 0.0], z_axis, x_axis,
                         tc / 1000, abs(b - a) / 1000, ar_v, GROUP_FIXED, 0.0))

    pos = step_to_body(np.array([r[0] for r in rows]), cg_mm)
    strips = Strips(
        pos=pos,
        span=np.array([r[1] for r in rows], dtype=float),
        chord=np.array([r[2] for r in rows], dtype=float),
        c=np.array([r[3] for r in rows]),
        ds=np.array([r[4] for r in rows]),
        slope=np.array([lift_slope(r[5]) for r in rows]),
        ar=np.array([r[5] for r in rows], dtype=float),
        group=np.array([r[6] for r in rows], dtype=int),
        incidence=np.array([r[7] for r in rows]),
    )

    eps = []
    for el in cfg["body.elements"]:
        segs = int(el["segments"])
        ys = np.linspace(el["y0"], el["y1"], segs + 1)
        for a, b in zip(ys[:-1], ys[1:]):
            eps.append(([el["x"], 0.5 * (a + b), el["z"]], (b - a) / 1000,
                        el["width_h"] / 1000, el["width_v"] / 1000, el["cd_cross"]))
    elements = Elements(
        pos=step_to_body(np.array([e[0] for e in eps]), cg_mm),
        length=np.array([e[1] for e in eps]),
        width_h=np.array([e[2] for e in eps]),
        width_v=np.array([e[3] for e in eps]),
        cd=np.array([e[4] for e in eps]),
    )
    return strips, elements


class Foil:
    """Lift/drag coefficients, attached-to-separated blend, any alpha."""

    def __init__(self, cfg: Config):
        self.alpha_s = math.radians(float(cfg["foil.stall_alpha"]))
        self.cd0 = float(cfg["foil.cd0"])
        self.e = float(cfg["foil.oswald"])
        self.cd90 = float(cfg["foil.flat_plate_cd90"])
        self.blend = float(cfg["foil.stall_blend"])

    def coeffs(self, alpha: np.ndarray, slope: np.ndarray, ar: np.ndarray):
        a = np.arctan2(np.sin(alpha), np.cos(alpha))  # wrap to [-pi, pi]
        cl_att = slope * a
        cd_att = self.cd0 + cl_att ** 2 / (math.pi * self.e * ar)
        sa, ca = np.sin(a), np.cos(a)
        cl_sep = self.cd90 * sa * ca
        cd_sep = self.cd0 + self.cd90 * sa * sa
        x = np.clip((np.abs(a) - self.alpha_s) / self.blend, -50.0, 50.0)
        sigma = 1.0 / (1.0 + np.exp(-x))
        cl = (1 - sigma) * cl_att + sigma * cl_sep
        cd = (1 - sigma) * np.minimum(cd_att, cd_sep + 0.5) + sigma * cd_sep
        return cl, cd


def strip_forces(strips: Strips, foil: Foil, rho: float, v: np.ndarray,
                 omega: np.ndarray, water_body: np.ndarray,
                 deflection: np.ndarray, wet: np.ndarray):
    """Total body force and moment (about the CG) from all strips.

    `deflection` is per strip (rad, nose-up), `wet` the submerged fraction
    per strip (0 in air).
    """
    w = water_body[None, :] - (v[None, :] + np.cross(omega[None, :], strips.pos))
    ws = np.sum(w * strips.span, axis=1)
    w2 = w - ws[:, None] * strips.span
    sp2 = np.sum(w2 * w2, axis=1)
    speed = np.sqrt(sp2) + 1e-9
    wc = np.sum(w2 * strips.chord, axis=1)
    wn = np.sum(w2 * strips.normal, axis=1)
    alpha = np.arctan2(wn, -wc) + deflection + strips.incidence
    cl, cd = foil.coeffs(alpha, strips.slope, strips.ar)
    lift_dir = np.cross(w2, strips.span)
    lift_dir /= (np.linalg.norm(lift_dir, axis=1)[:, None] + 1e-12)
    drag_dir = w2 / speed[:, None]
    q = 0.5 * rho * sp2 * strips.area * wet
    f = q[:, None] * (cl[:, None] * lift_dir + cd[:, None] * drag_dir)
    m = np.cross(strips.pos, f)
    return f.sum(axis=0), m.sum(axis=0), alpha


def element_forces(el: Elements, rho: float, v: np.ndarray, omega: np.ndarray,
                   water_body: np.ndarray, wet: np.ndarray):
    """Cross-flow drag on slender elements (axial drag is lumped separately)."""
    w = water_body[None, :] - (v[None, :] + np.cross(omega[None, :], el.pos))
    cross = w.copy()
    cross[:, 0] = 0.0
    mag = np.linalg.norm(cross, axis=1)
    f = np.zeros_like(w)
    k = 0.5 * rho * el.cd * el.length * mag * wet
    f[:, 1] = k * el.width_v * cross[:, 1]
    f[:, 2] = k * el.width_h * cross[:, 2]
    m = np.cross(el.pos, f)
    return f.sum(axis=0), m.sum(axis=0)


def added_mass_matrix(strips: Strips, el: Elements, rho: float,
                      housing_volume_m3: float, scale: float) -> np.ndarray:
    """6x6 added mass (about the CG) from strip/element contributions.

    Each lifting strip adds pi/4 rho c^2 ds along its normal; each element
    adds rho*pi/4*width^2*length in the two cross directions. Surge gets
    0.1 rho V of the housing. Couplings appear through r x n. Added-mass
    Coriolis/Munk terms are not modelled (README limitations).
    """
    A = np.zeros((6, 6))
    for i in range(len(strips.c)):
        n = strips.normal[i]
        ma = math.pi / 4 * rho * strips.c[i] ** 2 * strips.ds[i]
        g = np.concatenate([n, np.cross(strips.pos[i], n)])
        A += ma * np.outer(g, g)
    for i in range(len(el.length)):
        for d, width in ((np.array([0.0, 1.0, 0.0]), el.width_v[i]),
                         (np.array([0.0, 0.0, 1.0]), el.width_h[i])):
            ma = math.pi / 4 * rho * width ** 2 * el.length[i]
            g = np.concatenate([d, np.cross(el.pos[i], d)])
            A += ma * np.outer(g, g)
    A[0, 0] += 0.1 * rho * housing_volume_m3
    return scale * A
