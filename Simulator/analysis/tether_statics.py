#!/usr/bin/env python3
"""Steady straight-tow tether shape: how deep the fish gets for any tether length.

Two-dimensional towed-cable equilibrium in the vertical plane, in the
boat's frame (water flowing aft at the tow speed U). Integrating from the
fish (s = 0) to the boat:

    d(T t)/ds = -f(phi)        T tension, t = (cos phi, sin phi) toward the boat
    f = normal drag  1/2 rho d Cn |v_n| v_n
      + tangential drag  1/2 rho pi d Ct |v_t| v_t
      + wet weight  (0, -w)
    dx/ds = cos phi,  dz/ds = sin phi

The boundary condition is the fish's pull on the cable, T0 t0 = (D, N): its
drag D and its net down-force N (wings at full down, minus buoyancy). It
does not depend on the tether's length -- the wings' maximum down-force is
fixed -- so a single integration gives the fish's depth below the tow
point for every tether length at once: depth(L) = z(L) - z(0).

(D, N) comes from SITL runs (`--from-run`): the median tension and
direction of the tether's last segment while the fish holds its deepest
depth. It scales with U^2 for the speed band (an approximation: buoyancy
does not scale).

Usage:
    python3 Simulator/analysis/tether_statics.py --from-run Simulator/runs/sw_B1_L8
    python3 Simulator/analysis/tether_statics.py --D 7.0 --N 9.0 --Lmax 100 --json out.json
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from towsim.frames import Config  # noqa: E402

RHO_FRESH = 1000.0


def tether_shape(D: float, N: float, *, U: float = 1.0, d: float, cn: float, ct: float,
                 w: float = 0.0, rho: float = RHO_FRESH, Lmax: float = 100.0,
                 ds: float = 0.01, return_force: bool = False):
    """Integrate fish -> boat. Returns (s, x, z) arrays; z up, from the fish.

    With ``return_force`` also the tension vector (Fx forward, Fz up) at each s.
    """
    n = int(Lmax / ds) + 1
    s = np.linspace(0.0, Lmax, n)
    x = np.zeros(n)
    z = np.zeros(n)
    F = np.array([D, N], dtype=float)
    Fs = np.zeros((n, 2))
    Fs[0] = F
    kn = 0.5 * rho * d * cn
    kt = 0.5 * rho * math.pi * d * ct
    vrel = np.array([-U, 0.0])

    def force(F):
        T = math.hypot(*F)
        t = F / T if T > 1e-9 else np.array([1.0, 0.0])
        vt = (vrel @ t) * t
        vn = vrel - vt
        f = kn * np.linalg.norm(vn) * vn + kt * np.linalg.norm(vt) * vt
        f = f + np.array([0.0, -w])
        return f, t

    for i in range(1, n):
        # Midpoint (RK2) step on F, then the shape.
        f1, t1 = force(F)
        Fm = F - 0.5 * ds * f1
        f2, t2 = force(Fm)
        F = F - ds * f2
        Fs[i] = F
        x[i] = x[i - 1] + ds * t2[0]
        z[i] = z[i - 1] + ds * t2[1]
        if math.hypot(*F) < 1e-6:
            # Slack: no further shape is meaningful.
            x[i:], z[i:] = x[i], z[i]
            break
    if return_force:
        return s, x, z, Fs
    return s, x, z


def depth_curve(D, N, **kw):
    """Fish depth below the tow point vs tether length."""
    s, x, z = tether_shape(D, N, **kw)
    return s, z  # z(L) - z(0) with z(0) = 0


def fish_end_force(run: Path, window_s: float = 60.0):
    """(D, N, depth, tension) from the last `window_s` of a straight-tow run."""
    truth = list(csv.DictReader(open(run / "truth.csv")))
    cable = list(csv.DictReader(open(run / "cable.csv")))
    t_end = float(truth[-1]["t_sim"])
    keep = [i for i, r in enumerate(truth)
            if float(r["t_sim"]) >= t_end - window_s and i < len(cable)]
    n_nodes = (len(cable[0]) - 1) // 3
    Ds, Ns, depths, tens = [], [], [], []
    for i in keep:
        r, c = truth[i], cable[i]
        a = np.array([float(c[f"{k}{n_nodes - 2}"]) for k in "ned"])
        b = np.array([float(c[f"{k}{n_nodes - 1}"]) for k in "ned"])
        u = a - b                       # fish end -> next node (toward the boat)
        u /= np.linalg.norm(u)
        T = float(r["tension_n"])
        horiz = math.hypot(u[0], u[1])
        Ds.append(T * horiz)            # forward pull balances the fish's drag
        Ns.append(T * -u[2])            # upward pull balances its down-force
        depths.append(float(r["depth"]))
        tens.append(T)
    return (float(np.median(Ds)), float(np.median(Ns)),
            float(np.median(depths)), float(np.median(tens)))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from-run", type=Path, action="append", default=[],
                    help="straight-tow run(s) to take the fish-end force from")
    ap.add_argument("--D", type=float, help="fish drag at 1 m/s (N)")
    ap.add_argument("--N", type=float, help="fish net down-force at 1 m/s (N)")
    ap.add_argument("--U", type=float, default=1.0)
    ap.add_argument("--cn", type=float)
    ap.add_argument("--w", type=float, help="tether wet weight N/m (+ down)")
    ap.add_argument("--Lmax", type=float, default=100.0)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args(argv)

    cfg = Config()
    d = float(cfg["cable.diameter"]) / 1000.0
    cn = args.cn if args.cn is not None else float(cfg["cable.cd_normal"])
    ct = float(cfg["cable.cd_tangent"])
    w = args.w if args.w is not None else float(cfg["cable.wet_weight"])
    attach = float(cfg["cable.boat_attach_height"])

    runs = []
    for r in args.from_run:
        D, N, dep, T = fish_end_force(r)
        meta = json.loads((r / "meta.json").read_text())
        L = next(float(x[1]) for x in meta["config"] if x[0] == "cable.length")
        runs.append({"run": r.name, "L": L, "D": D, "N": N, "depth": dep, "tension": T})
        print(f"{r.name}: L={L:g} m  fish depth {dep:.2f} m  D={D:.2f} N  N={N:.2f} N  T={T:.2f} N")
    if args.D is not None:
        D, N = args.D, args.N
    elif runs:
        D = float(np.median([r["D"] for r in runs]))
        N = float(np.median([r["N"] for r in runs]))
    else:
        raise SystemExit("give --from-run or --D/--N")
    scale = (args.U / 1.0) ** 2
    s, z = depth_curve(D * scale, N * scale, U=args.U, d=d, cn=cn, ct=ct, w=w,
                       Lmax=args.Lmax)
    depth = z - attach        # below the surface; the tow point is `attach` above it
    step = int(round(1.0 / (s[1] - s[0])))
    gain = np.gradient(depth[::step])
    out = {"D": D, "N": N, "U": args.U, "cn": cn, "ct": ct, "w": w, "d": d,
           "L": s[::step].round(2).tolist(), "depth": depth[::step].round(3).tolist(),
           "gain_per_m": gain.round(4).tolist(), "runs": runs}
    for L in (8, 12, 16, 20, 25, 30, 40, 50, 60, 80, 100):
        if L <= args.Lmax:
            k = int(L / (s[1] - s[0]))
            print(f"  L={L:4d} m  depth {depth[k]:6.2f} m  (+{gain[min(L, len(gain) - 1)]:.3f} m per extra m)")
    if args.json:
        args.json.write_text(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
