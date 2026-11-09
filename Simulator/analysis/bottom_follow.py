#!/usr/bin/env python3
"""How well SURF TRACK followed the seabed in a terrain run.

Reads a run's truth.csv (needs --terrain) and scores it against the terrain
it flew over. Distances are along the boat's track from the boat at --start
(s_fish is the fish's).

    python3 Simulator/analysis/bottom_follow.py runs/<id> [...]

Ideal depth = bottom under the fish - target altitude, clamped to SURF
TRACK's own envelope [SURF_SURFACE_DEPTH_M, surf_max_depth_m].
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

TARGET_ALT = 4.0
SURFACE = 0.5
MAX_DEPTH = 5.0
SETTLE_M = 0.3


def load(run: Path) -> dict:
    rows = [r for r in csv.DictReader(open(run / "truth.csv")) if r.get("jig", "0") == "0"]
    rows = [r for r in rows if r.get("s_fish")]

    def col(k):
        return np.array([float(r[k]) if r.get(k) not in (None, "") else np.nan for r in rows])

    d = {k: col(k) for k in ("t_sim", "depth", "roll", "bottom_fish", "alt_true", "s_fish",
                             "s_boat", "surf_target", "surf_raw", "sonar_depth", "boat_speed",
                             "rc3")}
    d["surf"] = [r.get("surf", "") for r in rows]
    d["meta"] = json.loads((run / "meta.json").read_text())
    return d


def ideal(bottom):
    return np.clip(bottom - TARGET_ALT, SURFACE, MAX_DEPTH)


def best_lag(t, depth, want, max_lag=40.0):
    """Seconds by which the fish's depth trails the ideal (minimum RMS)."""
    best = (np.inf, 0.0)
    for lag in np.arange(0.0, max_lag + 0.01, 0.5):
        w = np.interp(t - lag, t, want)
        m = t >= t[0] + lag
        rms = float(np.sqrt(np.mean((depth[m] - w[m]) ** 2)))
        if rms < best[0]:
            best = (rms, float(lag))
    return best[1], best[0]


def first(mask):
    i = np.flatnonzero(mask)
    return int(i[0]) if len(i) else None


def common(d) -> dict:
    resc = np.array([s.startswith("rescue") for s in d["surf"]])
    hold = np.array(["holding" in s for s in d["surf"]])
    reasons = {}
    for s in d["surf"]:
        if s.startswith("holding/"):
            k = s.split("/")[1]
            reasons[k] = reasons.get(k, 0) + 1
    return {"min_clearance_m": round(float(np.nanmin(d["alt_true"])), 2),
            "max_roll_deg": round(float(np.nanmax(np.abs(d["roll"]))), 1),
            "rescues": int(np.sum(np.diff(resc.astype(int)) == 1)),
            "holding_s": round(float(hold.sum() * 0.1), 1),
            "hold_reasons_s": {k: round(v * 0.1, 1) for k, v in sorted(reasons.items())}}


def score_slope(d) -> dict:
    t, z, b, s = d["t_sim"], d["depth"], d["bottom_fish"], d["s_fish"]
    want = ideal(b)
    out = common(d)
    reach = b - TARGET_ALT >= SURFACE + 0.3          # 4 m separation is possible
    settled = s >= 0
    m = reach & settled
    err = d["alt_true"][m] - TARGET_ALT
    out.update({"alt_err_med_m": round(float(np.median(err)), 2),
                "alt_err_p95_abs_m": round(float(np.percentile(np.abs(err), 95)), 2)})
    lag, rms = best_lag(t[settled], z[settled], want[settled])
    out.update({"lag_s": lag, "rms_at_lag_m": round(rms, 2)})
    sh = (s >= 120) & (s <= 180)
    out.update({"shallow_depth_med_m": round(float(np.median(z[sh])), 2) if sh.any() else None,
                "shallow_at_surface_s": round(float(np.sum(z[sh] <= 0.6) * 0.1), 1),
                "shallow_clearance_min_m": round(float(np.min(d["alt_true"][sh])), 2) if sh.any() else None})
    # Diving back: the bottom is 6 m again past s = 240; want 2 m.
    i0 = first(s >= 240)
    if i0 is None:
        out["dive_back"] = "not reached"
    else:
        j = first((np.arange(len(t)) >= i0) & (np.abs(z - 2.0) <= SETTLE_M))
        out["dive_back_s"] = round(float(t[j] - t[i0]), 1) if j is not None else None
        out["dive_back"] = "yes" if j is not None else "no"
        out["end_depth_m"] = round(float(z[-1]), 2)
    # Up-slope (6 -> 3): did the climb keep 4 m until the surface clamp?
    up = (s >= 60) & (s <= 120)
    out["upslope_clearance_min_m"] = round(float(np.min(d["alt_true"][up])), 2) if up.any() else None
    return out


def score_coral(d, head_s=150.0, top_r=2.5, side=1.0) -> dict:
    t, z, s = d["t_sim"], d["depth"], d["s_fish"]
    out = common(d)
    pre = (s >= 100) & (s <= 130)
    z0 = float(np.median(z[pre])) if pre.any() else float(z[0])
    over = np.abs(s - head_s) <= top_r
    near = np.abs(s - head_s) <= top_r + side
    out.update({"approach_depth_m": round(z0, 2),
                "depth_over_head_max_m": round(float(z[over].max()), 2) if over.any() else None,
                "clearance_over_head_min_m": round(float(np.min(d["alt_true"][near])), 2) if near.any() else None,
                "clearance_no_action_m": round(4.0 - z0, 2)})
    k = first((s >= 100) & (z <= z0 - 0.2))
    out["climb_start_s_to_head_edge_m"] = round(float(s[k] - (head_s - top_r - side)), 1) if k is not None else None
    j = first((s >= head_s) & (z <= z0 - 0.2))
    past = s >= head_s + top_r + side
    kk = first(past)
    if kk is not None:
        back = first((np.arange(len(t)) >= kk) & (np.abs(z - z0) <= SETTLE_M))
        out["depth_regained_s"] = round(float(t[back] - t[kk]), 1) if back is not None else None
        out["end_depth_m"] = round(float(z[-1]), 2)
    out["at_surface_s"] = round(float(np.sum(z <= 0.6) * 0.1), 1)
    _ = j
    return out


def score(run: Path) -> dict:
    d = load(run)
    kind = d["meta"]["args"].get("terrain", "")
    if kind.startswith("slope"):
        return score_slope(d)
    if kind.startswith("coral"):
        return score_coral(d)
    return common(d)


def main(argv) -> int:
    for a in argv:
        print(a, json.dumps(score(Path(a)), indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
