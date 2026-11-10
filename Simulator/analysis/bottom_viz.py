#!/usr/bin/env python3
"""Side-view replay of the bottom-following runs, as one HTML page.

For each terrain (slope, coral) and tether (8 m, 25 m) it takes the run
flown with the shipped settings (variant all_ps40: the three SURF TRACK
fixes and pilot speed caps 40/30) and the 4 Nov code (variant base) for
comparison, and writes frames every 0.5 s: the tether's node positions
(cable.csv), the fish's depth, roll and pitch, SURF TRACK's commanded
depth and state, and the boat's latest Ping sounding. Positions are
projected onto distance along the track (s), the terrain's own axis.

    python3 Simulator/analysis/bottom_viz.py  ->  runs/report/bottom_viewer.html
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

SIM = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SIM))
from towsim.terrain import coral_profile, slope_profile  # noqa: E402

RUNS = SIM / "runs"
PAGE = RUNS / "report" / "bottom_viewer.html"
TEMPLATE = Path(__file__).resolve().parent / "bottom_viz_template.html"
STEP_S = 0.5
CASES = [("slope", 8), ("slope", 25), ("coral", 8), ("coral", 25)]


def rows(run: Path):
    return [r for r in csv.DictReader(open(run / "truth.csv")) if r.get("jig", "0") == "0" and r.get("s_fish")]


def along_axis(tr):
    """s = a*north + b*east + c, fitted from the fish's logged s."""
    A = np.array([[float(r["north"]), float(r["east"]), 1.0] for r in tr])
    s = np.array([float(r["s_fish"]) for r in tr])
    coef, *_ = np.linalg.lstsq(A, s, rcond=None)
    return coef


def f(x, d=2):
    return round(float(x), d)


def case(terrain, L):
    ship = RUNS / f"sw_BF_{terrain}_L{L}_all_ps40"
    base = RUNS / f"sw_BF_{terrain}_L{L}_base"
    tr = rows(ship)
    a, b, c = along_axis(tr)
    t_rel = float(tr[0]["t_sim"])
    cab = {}
    with open(ship / "cable.csv") as fh:
        rd = csv.reader(fh)
        next(rd)
        for row in rd:
            cab[round(float(row[0]), 1)] = row
    frames, next_t, sonar = [], t_rel, None
    for r in tr:
        if r.get("sonar_depth") and r.get("sonar_q") and float(r["sonar_q"]) >= 90:
            sonar = float(r["sonar_depth"])
        t = float(r["t_sim"])
        if t < next_t:
            continue
        next_t += STEP_S
        row = cab.get(round(t, 1)) or cab.get(round(t - 0.1, 1))
        nodes = []
        if row:
            v = np.array(row[1:], dtype=float).reshape(-1, 3)
            keep = list(range(0, len(v), 2 if len(v) > 14 else 1))
            if keep[-1] != len(v) - 1:
                keep.append(len(v) - 1)
            for i in keep:
                n, e, d = v[i]
                nodes += [f(a * n + b * e + c), f(max(d, -0.1))]
        tgt = r.get("surf_target")
        frames.append([f(t - t_rel, 1), f(r["s_boat"]), f(r["s_fish"]), f(r["depth"]),
                       f(r["roll"], 1), f(r["pitch"], 1), f(tgt) if tgt else None,
                       f(sonar) if sonar is not None else None, r.get("surf", ""), nodes])
    btr = rows(base)
    ghost = []
    last = -1e9
    for r in btr:
        s = float(r["s_fish"])
        if s - last >= 0.5:
            ghost += [f(s), f(r["depth"])]
            last = s
    return {"terrain": terrain, "L": L, "frames": frames, "base": ghost}


def main() -> int:
    xs = np.arange(-40, 340.25, 0.25)
    data = {"cases": [case(t, L) for t, L in CASES],
            "terrain": {"s": [f(x) for x in xs],
                        "slope": [f(slope_profile(x), 3) for x in xs],
                        "coral": [f(coral_profile(x, 0.0), 3) for x in xs]}}
    page = TEMPLATE.read_text().replace("/*DATA*/null", json.dumps(data, separators=(",", ":")))
    PAGE.parent.mkdir(parents=True, exist_ok=True)
    PAGE.write_text(page)
    print("wrote", PAGE, f"{len(page) / 1e6:.2f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
