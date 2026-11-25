#!/usr/bin/env python3
"""Run the righting, max-depth and line-spacing studies as batches of sim runs.

Each study builds `towsim.run` jobs, runs them through docker compose six
at a time, scores each from its truth.csv and records everything in
runs/sweep/results_<study>.json (re-running skips finished run IDs).

    python3 Simulator/analysis/sweep.py righting            # study C
    python3 Simulator/analysis/sweep.py maxdepth            # B1, plateau search included
    python3 Simulator/analysis/sweep.py spacing --vertical-pwm 1700 --roll-min-depth 1.5 --descent-start 0.5  # B2

Scores:
  righting  time inverted (|roll| > 90 deg), shallowest depth from the trigger
            to the end, time to upright and back to the turn-entry depth,
            STABILIZE needed, net tether twist.
  maxdepth  median depth over the last 60 s of a straight tow at 1.0 m/s.
  spacing   pass = |roll| < 45 deg from the boat's turn to +90 s and no
            auto-righting.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import subprocess
import sys
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

SIM = Path(__file__).resolve().parent.parent
REPO = SIM.parent
sys.path.insert(0, str(SIM))
from analysis.compare import boat_turns  # noqa: E402
from towsim.boat_track import parse_time  # noqa: E402
from towsim.logs import load_boat, utc_offset  # noqa: E402

RUNS = SIM / "runs"
RESULTS_DIR = RUNS / "sweep"
COMPOSE = ["docker", "compose", "-f", str(SIM / "docker" / "docker-compose.yml"),
           "run", "--rm", "towsim"]
WORKERS = 6
ROLL_P = ["--param-file", "params/scenarios/roll_p_1_5.parm"]
HAIRPIN = ["--start", "10:31:30", "--end", "10:34:30"]
SPACINGS = [7.6, 9, 12, 16, 20, 25, 30, 40]
LOCK = threading.Lock()


# -- bookkeeping ----------------------------------------------------------
def load_results() -> dict:
    """All studies' results. Each study writes its own file, so two
    studies can run at once without overwriting each other."""
    res = {}
    for f in sorted(RESULTS_DIR.glob("results_*.json")):
        res.update(json.loads(f.read_text()))
    return res


def save_result(run_id: str, entry: dict) -> None:
    f = RESULTS_DIR / f"results_{entry.get('study', 'misc')}.json"
    with LOCK:
        res = json.loads(f.read_text()) if f.exists() else {}
        res[run_id] = entry
        RESULTS_DIR.mkdir(parents=True, exist_ok=True)
        f.write_text(json.dumps(res, indent=1))


def run_job(run_id: str, argv: list[str], score, info: dict) -> dict:
    res = load_results()
    if run_id in res and "score" in res[run_id]:
        return res[run_id]
    log = RUNS / "sweep" / f"{run_id}.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    cmd = COMPOSE + argv + ["--run-id", run_id]
    with log.open("w") as f:
        rc = subprocess.call(cmd, cwd=REPO, stdout=f, stderr=subprocess.STDOUT)
    entry = {"argv": argv, **info, "exit": rc}
    try:
        if rc != 0:
            raise RuntimeError(f"run exited {rc}")
        entry["score"] = score(RUNS / run_id)
    except Exception as e:  # keep going; the log says why
        entry["error"] = repr(e)
    save_result(run_id, entry)
    print(f"  {run_id}: {entry.get('score', entry.get('error'))}", flush=True)
    return entry


def run_batch(jobs) -> list[dict]:
    with ThreadPoolExecutor(WORKERS) as ex:
        return list(ex.map(lambda j: run_job(*j), jobs))


def truth(run: Path) -> dict:
    rows = list(csv.DictReader(open(run / "truth.csv")))
    rows = [r for r in rows if r.get("jig", "0") == "0"]
    out = {k: np.array([float(r[k]) for r in rows])
           for k in ("t_sim", "t_boat", "depth", "roll", "tension_n")}
    out["surf"] = [r.get("surf", "") for r in rows]
    out["mode"] = [r.get("sitl_mode", "") for r in rows]
    tw = [r.get("ar_twist", "") for r in rows]
    out["ar_twist"] = np.array([float(x) if x else 0.0 for x in tw])
    return out


def turn_start(run: Path) -> float:
    meta = json.loads((run / "meta.json").read_text())
    boat = load_boat()
    turns = boat_turns(boat, meta["t_start_boat"], meta["t_end_boat"])
    return turns[0][0]


# -- study C: the righting drill -----------------------------------------
# Ran (5 Nov) with the trigger that waited for the boat's new leg + 3 s;
# that trigger has since been replaced by study C3's winner, so a rerun
# now flies the corner trigger with these stick recipes.
def score_righting(run: Path) -> dict:
    d = truth(run)
    t, roll, depth = d["t_boat"], d["roll"], d["depth"]
    rescue = np.array([s.startswith("rescue") for s in d["surf"]])
    inv = np.abs(roll) > 90
    out = {"inverted_s": round(float(inv.sum() * 0.1), 1),
           "max_roll": round(float(np.abs(roll).max()), 0),
           "rescues": int(np.sum(np.diff(rescue.astype(int)) == 1)),
           "stabilize": any("stabilize" in s for s in d["surf"]),
           "end_depth": round(float(depth[-1]), 2),
           "end_mode": d["mode"][-1],
           # Net twist from the truth attitude, and the extension's own count.
           "twist_truth": round(float(np.sum(np.angle(np.exp(1j * np.radians(np.diff(roll))))) / (2 * math.pi)), 2),
           "twist_counter": round(float(d["ar_twist"][-1]), 2)}
    if rescue.any():
        k0 = int(np.argmax(rescue))
        out["trigger_depth"] = round(float(depth[k0]), 2)
        out["min_depth_after"] = round(float(depth[k0:].min()), 2)
        up = np.where((np.arange(len(t)) > k0) & (np.abs(roll) < 20))[0]
        out["to_upright_s"] = round(float(t[up[0]] - t[k0]), 1) if len(up) else None
        done = np.where((np.arange(len(t)) > k0) & ~rescue)[0]
        out["to_resume_s"] = round(float(t[done[0]] - t[k0]), 1) if len(done) else None
    else:
        out["min_depth_after"] = round(float(depth.min()), 2)
    return out


RIGHTING_GRID = [(v, md, ds) for v in (1900, 1750, 1600, 1500)
                 for md in (0.0, 1.5) for ds in (0.0, 0.5)]
RIGHTING_TRACKS = {"fri": [], "sp9": ["--spacing", "9"]}


def righting_job(track, v, md, ds, extra_set=(), tag="", rep=1):
    # SITL lockstep is not bit-exact (inversion times jitter ~0.5 s), so a
    # recipe gets more than one run per track; rep > 1 is a replicate.
    rep_tag = f"_r{rep}" if rep > 1 else ""
    rid = f"sw_C_{track}_V{v}_M{int(md * 10):02d}_S{int(ds * 10):02d}{tag}{rep_tag}"
    argv = ["--start", "10:31:30", "--end", "10:35:30", "--init-depth", "5", "--log-cable",
            "--extension", *ROLL_P, *RIGHTING_TRACKS[track],
            "--ext-set", f"autoright_vertical_pwm={v}",
            "--ext-set", f"autoright_roll_min_depth_m={md}",
            "--ext-set", f"autoright_descent_start_frac={ds}"]
    for kv in extra_set:
        argv += ["--ext-set", kv]
    return (rid, argv, score_righting,
            {"study": "righting", "track": track, "vertical_pwm": v, "roll_min_depth": md,
             "descent_start": ds, "trigger": tag.strip("_") or None, "rep": rep})


def study_righting(args) -> None:
    run_batch([righting_job(trk, *g, rep=r) for r in range(1, args.reps + 1)
               for trk in RIGHTING_TRACKS for g in RIGHTING_GRID])
    print("best:", rank_righting())


def rank_righting():
    """Owner's objective: least time inverted, then most depth kept."""
    res = load_results()
    agg = {}
    for e in res.values():
        if e.get("study") != "righting" or e.get("trigger") or "score" not in e:
            continue
        k = (e["vertical_pwm"], e["roll_min_depth"], e["descent_start"])
        agg.setdefault(k, []).append(e["score"])
    if not agg:
        return None

    def key(k):
        sc = agg[k]
        return (sum(s["inverted_s"] for s in sc), sum(s["rescues"] for s in sc),
                -sum(s["min_depth_after"] for s in sc))
    return min(agg, key=key)


# -- study C3: act in the corner, while the fish is still deep -------------
# Waiting for the boat to straighten up costs ~4 m: depth hold keeps the
# wings at full down, which flies an inverted fish to the surface. Here the
# drill starts as soon as the roll passes THRESH (no boat wait), and the
# vertical stick spans full up .. half down: with the collective off its
# stop the wings keep differential for the roll snap (owner, 5 Nov).
EARLY_GRID = [(th, v) for th in (60, 90) for v in (1900, 1700, 1500, 1300)]


def early_job(track, th, v, rep=1):
    rep_tag = f"_r{rep}" if rep > 1 else ""
    rid = f"sw_C3_{track}_T{th}_V{v}{rep_tag}"
    argv = ["--start", "10:31:30", "--end", "10:35:30", "--init-depth", "5", "--log-cable",
            "--extension", *ROLL_P, *RIGHTING_TRACKS[track],
            "--ext-set", f"autoright_vertical_pwm={v}",
            "--ext-set", "autoright_roll_min_depth_m=0.0",
            "--ext-set", "autoright_descent_start_frac=0.0",
            "--ext-set", f"SURF_INVERT_ROLL_DEG={float(th)}",
            "--ext-set", "SURF_INVERT_DWELL_S=0.5",
            "--ext-set", "AUTORIGHT_ROLL_STICK_CUTOFF_DEG=20.0"]
    return (rid, argv, score_righting,
            {"study": "early", "track": track, "thresh": th, "vertical_pwm": v, "rep": rep})


def study_early(args) -> None:
    run_batch([early_job(trk, th, v, rep=r) for r in range(1, args.reps + 1)
               for trk in RIGHTING_TRACKS for th, v in EARLY_GRID])


# -- B1b: max depth with a fixed partial-down stick ------------------------
# STABILIZE, roll stabilised, RC3 held: the collective is set by hand
# (1400 = 25 % down .. 1100 = full down). The deepest *stable* steady depth
# over the stick settings is the tether's max depth; the setting that holds
# it shows how much collective can be spent before roll authority runs out.
FIXED_STICKS = (1400, 1300, 1200, 1100)
FIXED_LENGTHS = (8, 12, 16, 20, 25, 30, 40, 50, 60)


def fixed_job(L, pwm):
    seg = int(round(L)) if L <= 40 else int(round(L / 2))
    end = "10:36:30" if L < 40 else "10:38:30"
    rid = f"sw_B1F_L{L:g}_P{pwm}"
    argv = ["--start", "10:31:30", "--end", end, "--straight", "1.0", "--fixed-stick", str(pwm),
            "--init-depth", "4", "--warmup", "50", "--log-cable", *ROLL_P,
            "--set", f"cable.length={L}", "--set", f"cable.segments={seg}"]
    return (rid, argv, score_maxdepth, {"study": "fixed", "L": L, "pwm": pwm})


def study_fixed(args) -> None:
    lengths = args.lengths or FIXED_LENGTHS
    run_batch([fixed_job(L, p) for L in lengths for p in FIXED_STICKS])


# -- B1: max depth vs tether length --------------------------------------
# The fish is flown by the extension as in the field: SURF TRACK over a flat
# bottom far below anything reachable, no depth cap and no tether ceiling,
# so it keeps commanding down until its own wings_saturated / down_stall
# gates stop it. Releasing the fish deep in plain ALT_HOLD does not work:
# both wings pin at full down, nothing is left for roll, and it rolls over
# on a straight leg (sw_B1_*_althold runs).
MAXDEPTH_EXT = ["--extension", "--bottom-depth", "60",
                "--ext-set", "surf_max_depth_m=50", "--ext-set", "tether_length_m=0.0"]


def score_maxdepth(run: Path) -> dict:
    d = truth(run)
    t = d["t_sim"]
    last = t >= t[-1] - 60
    settle = t >= t[-1] - 120
    gated = [s for s, k in zip(d["surf"], last) if k]
    return {"depth": round(float(np.median(d["depth"][last])), 3),
            "depth_spread": round(float(np.ptp(d["depth"][last])), 3),
            "tension": round(float(np.median(d["tension_n"][last])), 2),
            "max_roll": round(float(np.abs(d["roll"][settle]).max()), 1),
            "inverted_s": round(float(np.sum(np.abs(d["roll"]) > 90) * 0.1), 1),
            "rescues": int(np.sum(np.diff(np.array([s.startswith("rescue") for s in d["surf"]], int)) == 1)),
            "saturated_frac": round(sum("wings_saturated" in s for s in gated) / max(1, len(gated)), 2),
            "stable": bool(np.abs(d["roll"][settle]).max() < 45)}


def maxdepth_job(L: float, extra_set=(), tag=""):
    seg = int(round(L)) if L <= 40 else int(round(L / 2))
    end = "10:37:30" if L < 40 else "10:39:30"
    rid = f"sw_B1_L{L:g}{tag}".replace(".", "p")
    argv = ["--start", "10:31:30", "--end", end, "--straight", "1.0", "--pilot", "none",
            "--init-depth", "3", "--warmup", "55", "--log-cable", *ROLL_P, *MAXDEPTH_EXT,
            "--set", f"cable.length={L}", "--set", f"cable.segments={seg}"]
    for kv in extra_set:
        argv += ["--set", kv]
    return (rid, argv, score_maxdepth, {"study": "maxdepth", "L": L, "tag": tag})


def study_maxdepth(args) -> None:
    lengths = [8, 12, 16, 20, 25, 30, 40]
    run_batch([maxdepth_job(L) for L in lengths])
    # Plateau search: keep lengthening until two successive depths agree to 0.25 m.
    for L in (50, 60, 80, 100):
        depths = depth_by_length()
        Ls = sorted(depths)
        if len(Ls) >= 2 and abs(depths[Ls[-1]] - depths[Ls[-2]]) < 0.25:
            break
        run_batch([maxdepth_job(L)])
    depths = depth_by_length()
    Ls = sorted(depths)
    plateau = None
    for a, b in zip(Ls, Ls[1:]):
        if abs(depths[b] - depths[a]) < 0.25:
            plateau = (depths[a] + depths[b]) / 2
            break
    if plateau is None:
        print("no plateau inside 100 m; see the statics curve for the trend")
        return
    # Bisect the shortest tether within 0.25 m of the plateau.
    short = [L for L in Ls if depths[L] < plateau - 0.25]
    hi = min(L for L in Ls if depths[L] >= plateau - 0.25)
    if not short:
        print(f"plateau {plateau:.2f} m already at the shortest tether tried ({hi} m)")
        return
    lo = max(L for L in short if L < hi) if any(L < hi for L in short) else hi
    while hi - lo > 2:
        mid = round((lo + hi) / 2)
        run_batch([maxdepth_job(mid)])
        if depth_by_length()[mid] >= plateau - 0.25:
            hi = mid
        else:
            lo = mid
    print(f"plateau {plateau:.2f} m; shortest tether within 0.25 m: {hi} m (bracket {lo}-{hi})")


def depth_by_length(tag=""):
    res = load_results()
    return {e["L"]: e["score"]["depth"] for e in res.values()
            if e.get("study") == "maxdepth" and e.get("tag", "") == tag and "score" in e
            and e["score"].get("stable", True)}


# -- B2: minimum spacing ---------------------------------------------------
def score_spacing(run: Path) -> dict:
    d = truth(run)
    ts = turn_start(run)
    w = (d["t_boat"] >= ts) & (d["t_boat"] <= ts + 90)
    rescue = any(s.startswith("rescue") for s, k in zip(d["surf"], w) if k)
    max_roll = float(np.abs(d["roll"][w]).max())
    return {"pass": bool(max_roll < 45 and not rescue), "max_roll": round(max_roll, 1),
            "rescue": rescue, "slack_s": round(float(np.sum(d["tension_n"][w] < 1.0) * 0.1), 1),
            "min_depth": round(float(d["depth"][w].min()), 2),
            "max_depth": round(float(d["depth"][w].max()), 2)}


# The corner drill that won study C3: act at 60 deg held 0.5 s, no boat
# wait, vertical stick neutral, roll stick released at 20 deg.
CORNER_RECIPE = ["--ext-set", "SURF_INVERT_ROLL_DEG=60.0", "--ext-set", "SURF_INVERT_DWELL_S=0.5",
                 "--ext-set", "AUTORIGHT_ROLL_STICK_CUTOFF_DEG=20.0",
                 "--ext-set", "autoright_vertical_pwm=1500",
                 "--ext-set", "autoright_roll_min_depth_m=0.0",
                 "--ext-set", "autoright_descent_start_frac=0.0"]
SPACING_LENGTHS = (8, 16, 25, 40, 50, 60)


def spacing_job(L, D, S, args=None):
    seg = int(round(L)) if L <= 40 else int(round(L / 2))
    rid = f"sw_B2_L{L:g}_D{D:g}_S{S:g}".replace(".", "p")
    warm = round(15 + D / 0.3 + 40)
    end = "10:34:30" if L < 40 else "10:35:30"     # long tethers lag the boat
    # Flat bottom 4 m under D: SURF TRACK's default 4 m altitude holds D.
    argv = ["--start", "10:31:30", "--end", end, "--init-depth", str(D), "--warmup", str(warm),
            "--extension", *ROLL_P, "--spacing", str(S), "--bottom-depth", str(D + 4),
            "--set", f"cable.length={L}", "--set", f"cable.segments={seg}",
            "--ext-set", f"surf_max_depth_m={D + 1}", "--ext-set", "tether_length_m=0.0",
            *CORNER_RECIPE]
    return (rid, argv, score_spacing, {"study": "spacing", "L": L, "D": D, "S": S})


def max_depth_by_length(pwm=1200):
    """Deepest stable steady depth per tether (fixed-stick study, 75 % down)."""
    res = load_results()
    return {e["L"]: e["score"]["depth"] for e in res.values()
            if e.get("study") == "fixed" and e.get("pwm") == pwm and "score" in e
            and e["score"].get("stable")}


def study_spacing(args) -> None:
    depths = max_depth_by_length()
    lengths = sorted(set(args.lengths or SPACING_LENGTHS))
    cells = [(L, D) for L in lengths for D in (4, 8, 12)
             if L in depths and D <= depths[L] - 0.5]
    skipped = [(L, D) for L in lengths for D in (4, 8, 12) if (L, D) not in cells]
    print("cells:", cells, "\nunreachable:", skipped)
    # Binary search over the grid per cell; all cells advance together each batch.
    state = {c: [0, len(SPACINGS) - 1, None] for c in cells}   # lo idx, hi idx, best pass idx
    while True:
        jobs, probes = [], {}
        for c, (lo, hi, best) in state.items():
            if lo > hi:
                continue
            mid = (lo + hi) // 2
            probes[c] = mid
            jobs.append(spacing_job(c[0], c[1], SPACINGS[mid], args))
        if not jobs:
            break
        results = run_batch(jobs)
        for (c, mid), e in zip(probes.items(), results):
            lo, hi, best = state[c]
            if e.get("score", {}).get("pass"):
                state[c] = [lo, mid - 1, mid]
            else:
                state[c] = [mid + 1, hi, best]
    # Confirm the step below the minimum fails (non-monotonic check).
    jobs = []
    for c, (_, _, best) in state.items():
        if best is not None and best > 0:
            jobs.append(spacing_job(c[0], c[1], SPACINGS[best - 1], args))
    run_batch(jobs)
    for c, (_, _, best) in sorted(state.items()):
        print(f"L={c[0]:g} D={c[1]:g}: min spacing "
              f"{SPACINGS[best] if best is not None else '> 40'} m")


# -- B3: full spacing grid on the short tethers (owner, 6 Nov) -------------
# Every spacing on one list, so each cell's pass/fail is visible rather than
# just the binary-search probes, with the cable logged for the 3D viewer.
# Scored from the first corner to 90 s after the second (wide crosslinks put
# the second corner up to ~55 s after the first).
GRID_CELLS = [(8, 4), (16, 4), (25, 4), (25, 8), (25, 9.4)]
GRID_SPACINGS = [7.6, 9, 12, 16, 20, 25, 30, 35, 40]
GRID_WIDE = {25: [50]}
# Extra tight spacings per cell (owner, 6 Nov: 9.4 m on 25 m = that tether's
# deepest steady depth at 75 % down).
GRID_EXTRA = {(25, 9.4): [8]}


def second_corner(run: Path) -> float:
    from towsim.boat_track import load_track, synthetic_turn_track
    meta = json.loads((run / "meta.json").read_text())
    a = meta["args"]
    boat = load_boat()
    t_start, t_end = meta["t_start_boat"], meta["t_end_boat"]
    t_turn, _, dh = boat_turns(boat, t_start, t_end)[0]
    tr = synthetic_turn_track(load_track(t_start, 0.1), t_start - float(a["warmup"]) - 5, t_turn,
                              1.0 if dh > 0 else -1.0, float(a["spacing"]))
    return tr.wp_times[1]


def _first_hold(t, ok, start, hold_s):
    """First time >= start from which `ok` stays true for hold_s, else None."""
    since = None
    for tk, k in zip(t, ok):
        if tk < start:
            continue
        if k:
            since = tk if since is None else since
            if tk - since >= hold_s:
                return since
        else:
            since = None
    return None


def score_grid(run: Path) -> dict:
    """How badly the turn upsets the fish, and how long it takes to recover.

    Upset: |roll| past 45 deg. Level again: within 20 deg for 5 s. Back at
    depth: within 0.5 m of the depth held before the turn for 5 s. Times
    count from the first corner to 90 s after the second; recovery is
    followed to the end of the run.
    """
    d = truth(run)
    t, roll, depth = d["t_boat"], np.abs(d["roll"]), d["depth"]
    ts = turn_start(run)
    spacing = json.loads((run / "meta.json").read_text())["args"].get("spacing")
    # The logged hairpin's second corner follows its first within seconds.
    t2 = second_corner(run) if spacing not in (None, "None") else ts + 15
    w = (t >= ts) & (t <= t2 + 90)
    pre = depth[(t >= ts - 20) & (t < ts)]
    hold = float(np.median(pre)) if len(pre) else float(depth[w][0])
    rescue = any(s.startswith("rescue") for s, k in zip(d["surf"], w) if k)
    up = roll > 45
    starts = np.flatnonzero(np.diff(np.r_[0, (up & w).astype(int)]) == 1)
    out = {"pass": bool(roll[w].max() < 45 and not rescue), "max_roll": round(float(roll[w].max()), 1),
           "rescue": rescue,
           "not_level_s": round(float(np.sum((roll > 20) & w) * 0.1), 1),
           "upset_s": round(float(np.sum(up & w) * 0.1), 1),
           "inverted_s": round(float(np.sum((roll > 90) & w) * 0.1), 1),
           "upsets": int(len(starts)),
           "slack_s": round(float(np.sum(d["tension_n"][w] < 1.0) * 0.1), 1),
           "hold_depth": round(hold, 2),
           "min_depth": round(float(depth[w].min()), 2),
           "second_corner_s": round(t2 - ts, 1),
           "recovered": None, "level_after_s": None, "depth_after_s": None}
    if len(starts):
        t_up = float(t[starts[0]])
        last_up = float(t[np.flatnonzero(up & w)[-1]])
        lvl = _first_hold(t, roll < 20, last_up, 5.0)
        dep = _first_hold(t, (np.abs(depth - hold) < 0.5) & (roll < 20), last_up, 5.0)
        out["first_upset_s"] = round(t_up - ts, 1)
        out["level_after_s"] = None if lvl is None else round(lvl - t_up, 1)
        out["depth_after_s"] = None if dep is None else round(dep - t_up, 1)
        out["recovered"] = lvl is not None
    return out


def rescore_grid(args) -> None:
    res = load_results()
    for rid, e in res.items():
        if e.get("study") == "grid" and (RUNS / rid / "truth.csv").exists() and e.get("exit") == 0:
            e["score"] = score_grid(RUNS / rid)
            save_result(rid, e)
    study_grid_summary()


def grid_job(L, D, S):
    rid, argv, _, info = spacing_job(L, D, S)
    rid = rid.replace("sw_B2_", "sw_B3_")
    i = argv.index("--end")
    argv[i + 1] = "10:35:30"
    return (rid, argv + ["--log-cable"], score_grid, {**info, "study": "grid"})


def study_grid(args) -> None:
    run_batch([grid_job(L, D, S) for L, D in GRID_CELLS
               for S in sorted(GRID_SPACINGS + GRID_WIDE.get(L, []) + GRID_EXTRA.get((L, D), []))])
    study_grid_summary()


def study_grid_summary() -> None:
    res = load_results()
    for L, D in GRID_CELLS:
        row = sorted((e["S"], e["score"]["pass"]) for e in res.values()
                     if e.get("study") == "grid" and e["L"] == L and e["D"] == D and "score" in e)
        print(f"L={L} D={D}: " + "  ".join(f"{S:g}{'+' if ok else '-'}" for S, ok in row))


# -- B4: steady approach first (owner, 6 Nov) ------------------------------
# The B3 runs released the fish from the jig only 48 s before the turn, so
# deep cells turned while still settling. B4 starts at 10:29:30, after the
# logged 90 deg turn at 10:29, giving ~2.8 min of straight approach at the
# boat's logged speeds; the hold tests fly 5 min at a constant 1 m/s.
STEADY_CELLS = [(25, 8), (25, 9.4)]


def score_steady(run: Path) -> dict:
    """score_grid plus how well the approach held depth (last 60 s before the turn)."""
    out = score_grid(run)
    d = truth(run)
    ts = turn_start(run)
    pre = (d["t_boat"] >= ts - 60) & (d["t_boat"] < ts)
    before = d["t_boat"] < ts
    resc = np.array([s.startswith("rescue") for s in d["surf"]])
    out.update({"approach_depth_med": round(float(np.median(d["depth"][pre])), 2),
                "approach_depth_min": round(float(d["depth"][pre].min()), 2),
                "approach_max_roll": round(float(np.abs(d["roll"][pre]).max()), 1),
                "approach_rescues": int(np.sum(np.diff(resc[before].astype(int)) == 1))})
    return out


def steady_job(L, D, S):
    rid, argv, _, info = grid_job(L, D, S)
    rid = rid.replace("sw_B3_", "sw_B4_")
    argv[argv.index("--start") + 1] = "10:29:30"
    return (rid, argv, score_steady, {**info, "study": "steady"})


def score_hold(run: Path) -> dict:
    d = truth(run)
    t = d["t_boat"]
    last = t >= t[-1] - 180
    resc = np.array([s.startswith("rescue") for s in d["surf"]])
    return {"depth_med": round(float(np.median(d["depth"][last])), 2),
            "depth_min": round(float(d["depth"][last].min()), 2),
            "depth_max": round(float(d["depth"][last].max()), 2),
            "max_roll": round(float(np.abs(d["roll"][last]).max()), 1),
            "not_level_s": round(float(np.sum(np.abs(d["roll"][last]) > 20) * 0.1), 1),
            "rescues": int(np.sum(np.diff(resc.astype(int)) == 1))}


def hold_job(L, D):
    seg = int(round(L)) if L <= 40 else int(round(L / 2))
    rid = f"sw_H_L{L:g}_D{D:g}_U1".replace(".", "p")
    argv = ["--start", "10:31:30", "--end", "10:36:30", "--straight", "1.0",
            "--init-depth", str(D), "--warmup", str(round(15 + D / 0.3 + 40)), "--log-cable",
            "--extension", *ROLL_P, "--bottom-depth", str(D + 4),
            "--set", f"cable.length={L}", "--set", f"cable.segments={seg}",
            "--ext-set", f"surf_max_depth_m={D + 1}", "--ext-set", "tether_length_m=0.0"]
    return (rid, argv, score_hold, {"study": "hold", "L": L, "D": D})


def study_steady(args) -> None:
    jobs = [hold_job(L, D) for L, D in STEADY_CELLS]
    jobs += [steady_job(L, D, S) for L, D in STEADY_CELLS
             for S in sorted(GRID_SPACINGS + GRID_WIDE.get(L, []) + GRID_EXTRA.get((L, D), []))]
    run_batch(jobs)


# -- DZ: THR_DZ 0 check (owner, 6 Nov) -----------------------------------
# With ArduSub's 100 us stick deadzone, SURF TRACK's small proportional
# commands did nothing and the fish settled ~0.5 m short. The extension
# now enforces THR_DZ 0; re-fly the hold tests and a few turns with it.
ROLL_P_DZ0 = ["--param-file", "params/scenarios/roll_p_1_5_dz0.parm"]


def _dz0(job):
    rid, argv, score, info = job
    argv = list(argv)
    i = argv.index("--param-file")
    argv[i + 1] = ROLL_P_DZ0[1]
    return (rid + "_dz0", argv, score, {**info, "study": info["study"] + "_dz0"})


def study_dz0(args) -> None:
    jobs = [_dz0(hold_job(L, D)) for L, D in [(25, 4), (25, 8), (25, 9.4)]]
    jobs += [_dz0(grid_job(L, D, S)) for L, D, S in
             [(8, 4, 12), (16, 4, 25), (25, 4, 35), (25, 8, 35)]]
    run_batch(jobs)


# -- JOG: SURF TRACK full-scale down command (owner, 6 Nov) ---------------
# jog_down_pwm sets both the operator's jog-down button and SURF TRACK's
# full-scale down command. Flown at 1300 on 4 Nov, defaults to 1400; does
# full stick (1100) help, with THR_DZ 0?
def _jog(job, pwm):
    rid, argv, score, info = _dz0(job)
    return (f"{rid}_jog{pwm}", list(argv) + ["--ext-set", f"jog_down_pwm={pwm}"], score,
            {**info, "study": info["study"] + "_jog", "jog_down_pwm": pwm})


def study_jog(args) -> None:
    base = [hold_job(25, 4), hold_job(25, 8),
            grid_job(8, 4, 12), grid_job(25, 4, 35), grid_job(25, 8, 35)]
    run_batch([_jog(j, p) for p in (1100, 1400) for j in base])


# -- DT: physics step check (owner, 6 Nov) ---------------------------------
# SITL runs at 200 Hz (5 ms frames); physics substeps at sim.physics_dt.
# 5 ms is unstable for 1 m cable segments; does 2.5 ms match 1 ms?
def study_dt(args) -> None:
    def dt(job):
        rid, argv, score, info = job
        return (rid + "_dt25", list(argv) + ["--set", "sim.physics_dt=0.0025"], score,
                {**info, "study": info["study"] + "_dt"})
    smoke = ("smoke_shipped", ["--start", "10:31:30", "--end", "10:33:45", "--init-depth", "5",
                               "--extension", *ROLL_P, "--log-cable"], score_righting,
             {"study": "smoke"})
    run_batch([dt(smoke), dt(_dz0(hold_job(25, 8))), dt(_dz0(grid_job(16, 4, 25)))])


# -- DZS: which THR_DZ to ship (owner, 6 Nov) -----------------------------
# THR_DZ 0 let SURF TRACK's small corrections act, but the 8 m tether at
# 12 m lines went from clean (100) to upset in 3 of 3 runs. Try 0/25/50/100
# on that turn, Friday's logged hairpin, the hard deep turn and a hold;
# turns twice each for run-to-run jitter.
DZ_VALUES = (0, 25, 50, 100)


def _with_dz(job, dz, rep=1):
    rid, argv, score, info = job
    argv = list(argv)
    argv[argv.index("--param-file") + 1] = f"params/scenarios/roll_p_1_5_dz{dz}.parm"
    tag = f"_DZ{dz}" + (f"_r{rep}" if rep > 1 else "")
    return (rid + tag, argv, score, {**info, "study": "dzsel", "dz": dz, "rep": rep,
                                       "case": rid})


def study_dzsel(args) -> None:
    friday = ("sw_DZ_fri", ["--start", "10:31:30", "--end", "10:35:30", "--init-depth", "5",
                            "--extension", *ROLL_P, "--log-cable"], score_grid, {"study": "dzsel"})
    turns = [grid_job(8, 4, 12), friday, grid_job(25, 8, 35)]
    jobs = [_with_dz(hold_job(25, 4), dz) for dz in DZ_VALUES]
    jobs += [_with_dz(j, dz, r) for r in (1, 2) for j in turns for dz in DZ_VALUES]
    run_batch(jobs)


# -- TP: tow point station along the bar (owner, 6 Nov) --------------------
# The tow point (top of the rod, 100 mm above the bar -- ASSUMED) sits at
# y = -326, 6 in back from the bar's front tip and 432 mm ahead of the CG.
# Slide it along the bar and fly the same cases. The as-flown THR_DZ 100.
TOWPOINT_Y = (-479.6, -400.0, -326.0, -250.0, -150.0, -85.3, 0.0, 105.8)


def score_hold_trim(run: Path) -> dict:
    out = score_hold(run)
    rows = [r for r in csv.DictReader(open(run / "truth.csv")) if r.get("jig", "0") == "0"]
    t = np.array([float(r["t_boat"]) for r in rows])
    last = t >= t[-1] - 180
    pitch = np.array([float(r["pitch"]) for r in rows])[last]
    out.update({"pitch_med": round(float(np.median(pitch)), 1),
                "pitch_spread": round(float(np.ptp(pitch)), 1)})
    return out


def _towpt(job, y):
    rid, argv, score, info = job
    if score is score_hold:
        score = score_hold_trim
    tag = f"_TP{y:g}".replace(".", "p").replace("-", "m")
    # Owner's CG (on the tube centreline) with the 200 g nose weight modelled.
    sets, _ = _ballast_sets("nose")
    return (rid + tag, list(argv) + sets + ["--set", f"tow_point.pos=[178.1,{y},0.0]"], score,
            {**info, "study": "towpt", "tow_y": y, "case": rid})


def study_towpt(args) -> None:
    friday = ("sw_TP_fri", ["--start", "10:31:30", "--end", "10:35:30", "--init-depth", "5",
                            "--extension", *ROLL_P, "--log-cable"], score_grid, {"study": "towpt"})
    cases = [hold_job(8, 4), friday, grid_job(8, 4, 12), grid_job(25, 8, 35)]
    run_batch([_towpt(c, y) for y in TOWPOINT_Y for c in cases])


# -- BAL: 200 g nose weight moved under the tube (owner, 6 Nov) -----------
# Owner: CG is on the tube centreline today, with a 200 g weight on the
# bar's nose tip; it could move to the lowest point under the tube, which
# lowers the CG (hydrostatic roll righting) and moves it aft. The weight is
# modelled as its own mass; the housing remainder shifts so the measured
# total and (for the baseline) the balanced-in-air CG are kept.
BALLAST_KG = 0.2
BALLAST_NOSE = (78.0, -470.0)            # top of the bar, at its front tip (x, y mm)
BALLAST_UNDER_X = -66.0                  # under the 56 mm-radius tube, ~20 mm thick
BALLAST_CASES = {"nose": BALLAST_NOSE, "under_front": (BALLAST_UNDER_X, -60.0),
                 "under_cg": (BALLAST_UNDER_X, 105.8)}


def _ballast_sets(where):
    from towsim.frames import Config
    cfg = Config()
    comps = [dict(c) for c in cfg["mass.components"]]
    total = float(cfg["mass.total"])
    y_cg0, x_cg0 = float(cfg["cg.y_mm"]), 0.0           # owner: on the centreline
    h = next(c for c in comps if c["name"] == "housing")
    # Remainder placed so the baseline (weight on the nose) balances at the
    # measured CG; it stays there when the weight moves.
    m_h = h["mass"] - BALLAST_KG
    h["mass"] = round(m_h, 4)
    h["pos"] = [round((total * x_cg0 - BALLAST_KG * BALLAST_NOSE[0]
                       - sum(c["mass"] * c["pos"][0] for c in comps if c is not h)) / m_h, 2),
                round((total * y_cg0 - BALLAST_KG * BALLAST_NOSE[1]
                       - sum(c["mass"] * c["pos"][1] for c in comps if c is not h)) / m_h, 2),
                0.0]
    x_w, y_w = BALLAST_CASES[where]
    comps.append({"name": "ballast", "mass": BALLAST_KG, "pos": [x_w, y_w, 0.0], "shape": "point"})
    m = sum(c["mass"] for c in comps)
    cg_x = sum(c["mass"] * c["pos"][0] for c in comps) / m
    cg_y = sum(c["mass"] * c["pos"][1] for c in comps) / m
    return ["--set", "mass.components=" + json.dumps(comps, separators=(",", ":")),
            "--set", f"cg.x_mm={cg_x:.2f}", "--set", f"cg.y_mm={cg_y:.2f}"], (cg_x, cg_y)


def study_ballast(args) -> None:
    friday = ("sw_BAL_fri", ["--start", "10:31:30", "--end", "10:35:30", "--init-depth", "5",
                             "--extension", *ROLL_P, "--log-cable"], score_grid, {"study": "ballast"})
    cases = [hold_job(8, 4), friday, grid_job(8, 4, 12), grid_job(25, 8, 35)]
    jobs = []
    for where in BALLAST_CASES:
        sets, cg = _ballast_sets(where)
        print(f"{where}: CG x {cg[0]:.1f} mm (up), y {cg[1]:.1f} mm (aft)")
        for rid, argv, score, info in cases:
            jobs.append((f"{rid}_BAL{where}", list(argv) + sets,
                         score_hold_trim if score is score_hold else score,
                         {**info, "study": "ballast", "ballast": where, "case": rid,
                          "cg": [round(cg[0], 1), round(cg[1], 1)]}))
    run_batch(jobs)


# -- GEO: collar along the rail, tube within the collar (owner, 6 Nov) ------
# The tube sits in a ring clamp that also carries the wing pivots; the clamp
# slides along the aluminium rail and the tube slides in the clamp. Model
# frame stays on the clamp/wings:
#   collar_aft c  -- the clamp (tube + wings) moves aft along the rail, so
#                    everything on the rail (tow point, bar, tail, nose
#                    weight) moves forward by c relative to the body;
#   tube_aft t    -- the tube (housing mass, its buoyancy, the dome's drag)
#                    moves aft by t through the clamp.
# Today: c = 0, t = 0 (clamp 2/3 of the tube length from its back end).
HOUSING_VOL_CM3 = 2411.78       # of 3225.5: the housing's share of the buoyancy
RAIL_VOL_CM3 = 100.0            # bar + tail fin, rough
COLLAR_AFT = (-150.0, -75.0, 75.0, 150.0)
TUBE_AFT = (-100.0, -50.0, 50.0, 90.0)   # clamp must stay on the 267 mm tube


def _geometry_sets(c=0.0, t=0.0, ballast="nose"):
    from towsim.frames import Config
    cfg = Config()
    sets, _ = _ballast_sets(ballast)
    comps = json.loads(sets[1].split("=", 1)[1])
    for k in comps:
        on_rail = k["name"] in ("bar", "tail_fin") or (k["name"] == "ballast" and ballast == "nose")
        if on_rail:
            k["pos"][1] = round(k["pos"][1] - c, 2)
        elif k["name"] in ("housing", "ballast"):     # under-tube weight rides with the tube
            k["pos"][1] = round(k["pos"][1] + t, 2)
    elems = [dict(e) for e in cfg["body.elements"]]
    for e in elems:
        if e["name"] == "bar":
            e["y0"], e["y1"] = round(e["y0"] - c, 2), round(e["y1"] - c, 2)
        elif e["name"] == "housing":
            e["y0"], e["y1"] = round(e["y0"] + t, 2), round(e["y1"] + t, 2)
    vol = float(cfg["buoyancy.volume"])
    cb = list(cfg["buoyancy.cb"])
    cb[1] = round(cb[1] + t * HOUSING_VOL_CM3 / vol - c * RAIL_VOL_CM3 / vol, 2)
    tow = list(cfg["tow_point.pos"])
    tow[1] = round(tow[1] - c, 2)
    m = sum(k["mass"] for k in comps)
    cg_x = sum(k["mass"] * k["pos"][0] for k in comps) / m
    cg_y = sum(k["mass"] * k["pos"][1] for k in comps) / m
    j = lambda v: json.dumps(v, separators=(",", ":"))
    return ["--set", "mass.components=" + j(comps), "--set", f"cg.x_mm={cg_x:.2f}",
            "--set", f"cg.y_mm={cg_y:.2f}", "--set", "body.elements=" + j(elems),
            "--set", "buoyancy.cb=" + j(cb), "--set", "tow_point.pos=" + j(tow),
            "--set", f"tail.y_le={float(cfg['tail.y_le']) - c:.2f}",
            "--set", f"body.axial_center_y={float(cfg['body.axial_center_y']) + t / 2:.2f}"], \
        {"cg_y": round(cg_y, 1), "cb_y": cb[1], "tow_y": tow[1]}


def study_geometry(args) -> None:
    friday = ("sw_GEO_fri", ["--start", "10:31:30", "--end", "10:35:30", "--init-depth", "5",
                             "--extension", *ROLL_P, "--log-cable"], score_grid, {"study": "geometry"})
    cases = [hold_job(8, 4), friday, grid_job(8, 4, 12), grid_job(25, 8, 35)]
    configs = [(c, 0.0) for c in COLLAR_AFT] + [(0.0, t) for t in TUBE_AFT]
    jobs = []
    for c, t in configs:
        sets, g = _geometry_sets(c, t)
        print(f"collar_aft {c:+g} tube_aft {t:+g}: {g}", flush=True)
        tag = f"_C{c:+g}_T{t:+g}".replace("+", "p").replace("-", "m").replace(".", "")
        for rid, argv, score, info in cases:
            jobs.append((rid + tag, list(argv) + sets,
                         score_hold_trim if score is score_hold else score,
                         {**info, "study": "geometry", "collar_aft": c, "tube_aft": t,
                          "case": rid, **g}))
    run_batch(jobs)


# -- RCP: candidate recipes, repeated (owner, 6 Nov) ----------------------
# Combine what helped in the single-run sweeps, fly each twice, then map the
# spacing for the 8 m and 25 m tethers with the winner.
RECIPES = {"today": (0.0, 0.0, "nose"), "collar75": (75.0, 0.0, "nose"),
           "under": (0.0, 0.0, "under_front"), "collar75_under": (75.0, 0.0, "under_front"),
           "tube90": (0.0, 90.0, "nose"), "tube90_under": (0.0, 90.0, "under_front")}


def _recipe_job(job, name, rep):
    rid, argv, score, info = job
    c, t, ballast = RECIPES[name]
    sets, g = _geometry_sets(c, t, ballast)
    tag = f"_R{name}" + (f"_r{rep}" if rep > 1 else "")
    return (rid + tag, list(argv) + sets, score_hold_trim if score is score_hold else score,
            {**info, "study": "recipe", "recipe": name, "rep": rep, "case": rid, **g})


def study_recipe(args) -> None:
    friday = ("sw_RCP_fri", ["--start", "10:31:30", "--end", "10:35:30", "--init-depth", "5",
                             "--extension", *ROLL_P, "--log-cable"], score_grid, {"study": "recipe"})
    cases = [friday, grid_job(8, 4, 9), grid_job(8, 4, 12), grid_job(25, 8, 25), grid_job(25, 8, 35)]
    names = args.recipes or list(RECIPES)
    run_batch([_recipe_job(j, n, r) for r in (1, 2) for n in names for j in cases])


def study_recipe_spacing(args) -> None:
    name = args.recipe
    cells = [(8, 4), (25, 8), (25, 4)]
    run_batch([_recipe_job(grid_job(L, D, S), name, 1) for L, D in cells
               for S in sorted(GRID_SPACINGS + GRID_WIDE.get(L, []))])


# -- INX: the inextensible (Kevlar) tether (owner, 7 Nov) ------------------
# Re-run the owner's cases on the fixed-length, tension-only tether, with
# the recommended layout (weight under the front of the tube), the shipped
# extension settings, and the tether length set so the extension's own
# position estimate is logged (est_n/est_e/est_unc).
INX_HOLD_TARGETS = {8: (4.0, 4.5, 5.0, 5.5), 25: (7.0, 7.5, 8.0, 8.5, 9.0)}
INX_WIDE = {8: 16, 25: 35}
INX_TIGHT = 8


def _with_tether(argv, L):
    argv = list(argv)
    if "tether_length_m=0.0" in argv:
        argv[argv.index("tether_length_m=0.0")] = f"tether_length_m={float(L)}"
    return argv


def _inx(job, L, prefix, study, extra=(), rep=1, cable="inextensible"):
    rid, argv, score, info = _recipe_job(job, "under", 1)
    rid = rid.replace("_Runder", "").replace("sw_H_", prefix).replace("sw_B4_", prefix)
    if rep > 1:
        rid += f"_r{rep}"
    argv = _with_tether(argv, L) + ["--set", f"cable.model={cable}"] + list(extra)
    return (rid, argv, score, {**info, "study": study, "cable": cable, "rep": rep, "L": L})


def inx_hold_job(L, D):
    return _inx(hold_job(L, D), L, "sw_IA_", "inx_hold")


def study_inx_hold(args) -> None:
    run_batch([inx_hold_job(L, D) for L, ts in INX_HOLD_TARGETS.items() for D in ts])
    print(inx_max_depths())


def inx_max_depths():
    """Deepest target each tether held steady and level: roll < 20 deg, no
    rescues, depth within +-0.3 m, over the last 3 min of the hold test."""
    res = load_results()
    best = {}
    for e in res.values():
        if e.get("study") != "inx_hold" or "score" not in e:
            continue
        s = e["score"]
        ok = s["max_roll"] < 20 and s["rescues"] == 0 and s["depth_max"] - s["depth_min"] <= 0.6
        if ok and e["D"] > best.get(e["L"], (0, None))[0]:
            best[e["L"]] = (e["D"], s["depth_med"])
    return best


def study_inx_turns(args) -> None:
    maxd = inx_max_depths()
    jobs = []
    for L in (8, 25):
        if L not in maxd:
            print(f"no steady max depth for {L} m yet; run inx-hold first")
            continue
        for D in sorted({4.0, maxd[L][0]}):
            for S in (INX_TIGHT, INX_WIDE[L]):
                for r in (1, 2):
                    jobs.append(_inx(steady_job(L, D, S), L, "sw_IB_", "inx_turns", rep=r))
                jobs.append(_inx(steady_job(L, D, S), L, "sw_IC_", "inx_turns_spring", cable="spring"))
    run_batch(jobs)


# -- RATES: climb and dive the fish can actually do (owner, 8 Nov) --------
# Straight tows with a scripted vertical stick, so the rate is measured
# rather than assumed. STABILIZE shows the wing's own limit (the stick is
# the collective); ALT_HOLD shows what the depth controller delivers at
# each stick and pilot speed cap. Up and down steps alternate so the fish
# stays inside the tether's depth range. No extension: nothing rescues.
RATE_STEPS = {
    "alt_hold": [(1500, 0), (1700, 40), (1500, 52), (1300, 70), (1500, 90),
                 (1800, 110), (1500, 122), (1200, 140), (1500, 162), (1900, 182),
                 (1500, 192), (1100, 210), (1500, 232), (2000, 252), (1500, 260),
                 (1000, 278), (1500, 298)],
    "stabilize": [(1500, 0), (1700, 30), (1500, 38), (1300, 56), (1500, 66),
                  (1900, 84), (1500, 90), (1100, 108), (1500, 116), (1500, 134)],
}
RATE_CAPS = {"25_20": "params/scenarios/roll_p_1_5.parm",
             "40_30": "params/scenarios/roll_p_1_5_ps40_30.parm",
             "60_40": "params/scenarios/roll_p_1_5_ps60_40.parm",
             "100_60": "params/scenarios/roll_p_1_5_ps100_60.parm"}
RATE_INIT_DEPTH = {8: 3.0, 25: 5.0}


def score_rates(run: Path) -> dict:
    """One row per stick step: rate fitted after a 2 s settle (+ = deeper),
    the fastest 3 s rate inside it, depth range and peak roll."""
    rows = list(csv.DictReader(open(run / "truth.csv")))
    rows = [r for r in rows if r.get("jig", "0") == "0"]
    t = np.array([float(r["t_sim"]) for r in rows])
    z = np.array([float(r["depth"]) for r in rows])
    roll = np.array([float(r["roll"]) for r in rows])
    rc3 = np.array([int(float(r["rc3"])) for r in rows])
    s5 = np.array([float(r["servo5"]) for r in rows])
    s6 = np.array([float(r["servo6"]) for r in rows])
    coll = ((s5 - 1500) + (1500 - s6)) / 2
    steps = []
    i = 0
    while i < len(t):
        j = i
        while j < len(t) and rc3[j] == rc3[i]:
            j += 1
        if rc3[i] != 1500 and t[j - 1] - t[i] >= 4:
            m = slice(i, j)
            tt, zz = t[m], z[m]
            k = tt >= tt[0] + 2.0
            rate = float(np.polyfit(tt[k], zz[k], 1)[0]) if k.sum() > 5 else float("nan")
            peak = 0.0
            for a in range(len(tt)):
                b = np.searchsorted(tt, tt[a] + 3.0)
                if b < len(tt):
                    r = (zz[b] - zz[a]) / (tt[b] - tt[a])
                    if abs(r) > abs(peak):
                        peak = r
            steps.append({"pwm": int(rc3[i]), "dur_s": round(float(tt[-1] - tt[0]), 1),
                          "rate_mps": round(rate, 3), "peak3_mps": round(float(peak), 3),
                          "depth_start": round(float(zz[0]), 2), "depth_end": round(float(zz[-1]), 2),
                          "depth_min": round(float(zz.min()), 2), "depth_max": round(float(zz.max()), 2),
                          "coll_med_us": round(float(np.median(coll[m])), 0),
                          "coll_p95_us": round(float(np.percentile(np.abs(coll[m]), 95)), 0),
                          "roll_peak_deg": round(float(np.abs(roll[m]).max()), 1)})
        i = j
    return {"steps": steps, "roll_peak_deg": round(float(np.abs(roll).max()), 1),
            "depth_min": round(float(z.min()), 2), "depth_max": round(float(z.max()), 2)}


def rates_job(L, v, mode, cap="25_20", depth=None, rep=1):
    D = depth if depth is not None else RATE_INIT_DEPTH[L]
    prog = RATE_STEPS[mode]
    end_s = prog[-1][1] + 20
    rid = f"sw_RT_{mode[:2]}_L{L:g}_V{v:g}_C{cap}_D{D:g}".replace(".", "p")
    if rep > 1:
        rid += f"_r{rep}"
    seg = int(round(L))
    sets, g = _geometry_sets(*RECIPES["under"])
    end = 10 * 3600 + 31 * 60 + 30 + end_s
    argv = ["--start", "10:31:30", "--end", f"{end // 3600}:{end % 3600 // 60:02d}:{end % 60:02d}",
            "--straight", str(v),
            "--init-depth", str(D), "--warmup", str(round(15 + D / 0.3 + 40)),
            "--param-file", RATE_CAPS[cap],
            "--stick-steps", ",".join(f"{p}@{s}" for p, s in prog), "--stick-mode", mode,
            "--set", f"cable.length={L}", "--set", f"cable.segments={seg}",
            "--set", "cable.model=inextensible", *sets]
    return (rid, argv, score_rates, {"study": "rates", "L": L, "v": v, "mode": mode,
                                     "cap": cap, "D": D, "rep": rep})


def study_rates(args) -> None:
    jobs = [rates_job(L, v, "stabilize") for L in (8, 25) for v in (0.8, 1.0, 1.2)]
    jobs += [rates_job(L, v, "alt_hold", cap) for L in (8, 25) for v in (0.8, 1.0, 1.2)
             for cap in (args.caps or ("25_20", "40_30", "60_40"))]
    run_batch(jobs)


# -- BOTTOM: does SURF TRACK follow the seabed? (owner, 8 Nov) ------------
# Straight 1 m/s tows over towsim/terrain.py's slope (6 -> 3 -> 6 m) and
# coral head (7 m bottom, 3 m high, 5 m across), Kevlar tether, weight
# under the tube, the shipped extension with the tether length set so the
# position model and its tow delay are live. Variants are --ext-set
# overrides of app/main.py names, so a fix is tried before it is coded.
BOTTOM = {"slope": {"init": 2.0, "dur": 360}, "coral": {"init": 3.0, "dur": 260}}


def score_bottom(run: Path) -> dict:
    from analysis.bottom_follow import score
    return score(run)


def bottom_job(terrain, L, variant="base", ext=(), rep=1):
    b = BOTTOM[terrain]
    seg = int(round(L))
    sets, _ = _geometry_sets(*RECIPES["under"])
    end = 10 * 3600 + 31 * 60 + 30 + b["dur"] + int(L)
    rid = f"sw_BF_{terrain}_L{L:g}_{variant}" + (f"_r{rep}" if rep > 1 else "")
    argv = ["--start", "10:31:30", "--end", f"{end // 3600}:{end % 3600 // 60:02d}:{end % 60:02d}",
            "--straight", "1.0", "--terrain", terrain, "--ping-seed", str(rep),
            "--init-depth", str(b["init"]), "--warmup", str(round(15 + b["init"] / 0.3 + 40)),
            "--extension", *ROLL_P, "--log-cable",
            "--set", f"cable.length={L}", "--set", f"cable.segments={seg}",
            "--set", "cable.model=inextensible", *sets,
            "--ext-set", f"tether_length_m={float(L)}"]
    for kv in ext:
        argv += ["--ext-set", kv]
    if variant in BOTTOM_PARAMS:
        argv[argv.index(ROLL_P[1])] = BOTTOM_PARAMS[variant]
    return (rid, argv, score_bottom, {"study": "bottom", "terrain": terrain, "L": L,
                                      "variant": variant, "ext": list(ext), "rep": rep})


BOTTOM_VARIANTS = {
    "base": (),
    "look": ("surf_lookahead=shallowest",),
    "resume": ("surf_resume_from_surface=true",),
    "dz": ("surf_stick_skip_deadzone=true",),
    "all": ("surf_lookahead=shallowest", "surf_resume_from_surface=true",
            "surf_stick_skip_deadzone=true"),
}
# Commanded-depth slew matched to what the shipped caps achieve at the
# jog PWMs (rates study: ~0.18 m/s up at 1800, ~0.09 m/s down at 1300).
BOTTOM_VARIANTS["all_slew"] = BOTTOM_VARIANTS["all"] + (
    "SURF_TARGET_SLEW_UP_MPS=0.18", "SURF_TARGET_SLEW_DOWN_MPS=0.09")
# Variants flown with a different scenario .parm (pilot speed caps).
BOTTOM_PARAMS = {"all_ps40": RATE_CAPS["40_30"]}
BOTTOM_VARIANTS["all_ps40"] = BOTTOM_VARIANTS["all"]


def study_bottom(args) -> None:
    names = args.variants or ["base"]
    jobs = [bottom_job(tr, L, n, BOTTOM_VARIANTS[n], r) for n in names
            for tr in ("slope", "coral") for L in (8, 25) for r in range(1, args.reps + 1)]
    if "base" in names:
        jobs.append(bottom_job("coral", 8, "jog1600", ("jog_up_pwm=1600",)))
    run_batch(jobs)


# -- ALTITUDE: seabed under the fish, straight and through a turn (9 Nov) --
# The extension now reads the towfish's altitude from the boat's soundings
# at the position model's estimate of the fish, falling back to the
# sounding one tow delay ago. Each run logs both (alt_pos, alt_delay)
# against the truth (alt_true) over a seabed that slopes across the lanes
# of a hairpin, so the inside of the turn is on different ground from the
# boat's track. Shipped settings: 40/30 caps, SURF TRACK fixes on.
# Deepening toward the second lane (the logged hairpin turns left), so the
# fish has water under it on both legs.
ALT_PLANE = "plane:8:0.15:270"
ALT_SPACING = {8: 8, 25: 35}


def score_altitude(run: Path) -> dict:
    rows = [r for r in csv.DictReader(open(run / "truth.csv"))
            if r.get("jig", "0") == "0" and r.get("alt_true")]
    tb = np.array([float(r["t_boat"]) for r in rows])
    true = np.array([float(r["alt_true"]) for r in rows])
    num = lambda k: np.array([float(r[k]) if r.get(k) else np.nan for r in rows])  # noqa: E731
    pos, dly = num("alt_pos"), num("alt_delay")
    src = np.array([r.get("alt_pos_src", "") for r in rows])
    meta = json.loads((run / "meta.json").read_text())
    if meta["args"].get("spacing") not in (None, "None"):
        ts = turn_start(run)
        L = float(re.search(r"_L(\d+)", run.name).group(1))
        windows = {"straight": tb < ts - 5, "turn": (tb >= ts) & (tb <= ts + 40 + L)}
    else:
        windows = {"straight": tb >= tb[0] + 30}
    out = {}
    for name, m in windows.items():
        for key, a in (("pos", pos), ("delay", dly)):
            ok = m & np.isfinite(a)
            err = np.abs(a[ok] - true[ok])
            out[f"{name}_{key}_p50"] = round(float(np.median(err)), 2) if ok.any() else None
            out[f"{name}_{key}_p95"] = round(float(np.percentile(err, 95)), 2) if ok.any() else None
            out[f"{name}_{key}_coverage"] = round(float(ok.sum() / max(1, m.sum())), 2)
        out[f"{name}_pos_from_model"] = round(float(np.mean(src[m] == "position")), 2) if m.any() else None
    return out


def altitude_jobs():
    jobs = []
    for L in (8, 25):
        rid, argv, _, info = _inx(steady_job(L, 4.0, ALT_SPACING[L]), L, "sw_AL_", "altitude")
        argv = list(argv)
        i = argv.index("--bottom-depth")
        argv[i:i + 2] = ["--terrain", ALT_PLANE]
        argv[argv.index(ROLL_P[1])] = RATE_CAPS["40_30"]
        jobs.append((rid + "_p270", argv, score_altitude, {**info, "study": "altitude", "case": "hairpin"}))
        rid, argv, _, info = bottom_job("slope", L, "alt")
        argv = list(argv)
        argv[argv.index(ROLL_P[1])] = RATE_CAPS["40_30"]
        jobs.append((rid.replace("sw_BF_", "sw_AL_"), argv, score_altitude,
                     {**info, "study": "altitude", "case": "slope"}))
    return jobs


def study_altitude(args) -> None:
    run_batch(altitude_jobs())


# -- BRIDLE: the tow point as built (owner, 19 Nov) -------------------------
# The fish has always towed from a two-leg bridle (config tow_point.mode
# bridle), not the rod top the model used. First check which of the two
# reproduces the 4 Nov straight-leg trim (operator replay, the fish as
# flown: weight on the nose, flown parameters), then re-fly the current
# layout's baseline with the bridle: hold depth and the hairpins on 8 m and
# 25 m, weight under the tube, 40/30 caps, the shipped extension.
REPLAY_WINDOW = ["--start", "10:24:00", "--end", "10:36:00", "--init-depth", "3"]


def score_trim(run: Path) -> dict:
    from analysis.compare import load_real, load_sim, straight_metrics
    meta = json.loads((run / "meta.json").read_text())
    t0, t1 = meta["t_start_boat"], meta["t_end_boat"]
    sim = load_sim(run, meta)
    t1 = min(t1, float(sim["t"][-1]))
    boat, real = load_real(t0, t1)
    turns = boat_turns(boat, t0, t1 - 30)
    out = {}
    for name, d in (("real", real), ("sim", sim)):
        s = straight_metrics(d["t"], d["pitch"], d["depth"], d["coll"], turns, t0, t1) or {}
        for k in ("pitch_median", "depth_median", "collective_median_us", "samples"):
            out[f"{name}_{k}"] = s.get(k)
    return out


def bridle_jobs(reps=2):
    jobs = []
    nose, _ = _ballast_sets("nose")
    for mode in ("rigid", "bridle"):
        jobs.append((f"sw_BR_replay_{mode}", REPLAY_WINDOW + nose + ["--set", f"tow_point.mode={mode}"],
                     score_trim, {"study": "bridle", "case": "replay", "mode": mode}))
    caps = RATE_CAPS["40_30"]

    def recap(job, prefix):
        rid, argv, score, info = job
        argv = list(argv)
        argv[argv.index(ROLL_P[1])] = caps
        rid = prefix + rid.split("_", 2)[2]
        return (rid, argv, score, {**info, "study": "bridle"})

    for L, ds in ((8, (4.0, 4.5)), (25, (7.0, 8.0))):
        for D in ds:
            jobs.append(recap(inx_hold_job(L, D), "sw_BR_H"))
    for L, S in ((8, 8), (25, 35)):
        for r in range(1, reps + 1):
            jobs.append(recap(_inx(steady_job(L, 4.0, S), L, "sw_IB_", "inx_turns", rep=r), "sw_BR_T"))
    return jobs


def study_bridle(args) -> None:
    run_batch(bridle_jobs())


# -- SPINE: two stacked 1 m carbon tubes instead of the channel (19 Nov) ---
# Owner: 6.1 mm OD / 4.5 mm ID carbon tubes (~21 g/m), flooded, the lower
# one where the channel was, the upper one `sep` mm above it (<= 150),
# joined by 3D-printed couplers to each other, the wing-servo collar and
# the tail fin. No bridle: the tether clamps to the top of the upper tube
# at station `tow_y`. The spine starts 25 mm ahead of that clamp and runs
# 1000 mm; the tail fin's trailing edge sits 5 mm from its end. Tube,
# wings, collar and the 200 g weight under the tube are as today.
SPINE_LEN = 1000.0
SPINE_TUBE_KG = 0.021                    # per 1 m tube: 13.3 mm^2 x ~1.58 g/cm^3
SPINE_TUBE_CM3 = 13.3
SPINE_OD = 6.1
SPINE_LO_X = 71.7                        # where the channel's centreline was
COUPLER_KG = 0.008                       # ASSUMED per coupler, owner to weigh
COUPLER_CM3 = 6.5
CHANNEL_CM3 = 73.0                       # 197 g of aluminium
SPINE_SEPS = (25, 50, 100, 150)
SPINE_TOW_YS = (-99.0, -250.0)           # bridle apex station; further forward


def _spine_sets(sep, tow_y):
    from towsim.frames import Config
    cfg = Config()
    sets, _ = _ballast_sets("under_front")
    comps = [c for c in json.loads(sets[1].split("=", 1)[1]) if c["name"] != "bar"]
    front = tow_y - 25.0
    end = front + SPINE_LEN
    mid = (front + end) / 2
    hi_x = SPINE_LO_X + sep
    x_c = (SPINE_LO_X + hi_x) / 2
    y_le = end - 5.0 - float(cfg["tail.chord"])
    for c in comps:
        if c["name"] == "tail_fin":
            c["pos"][1] = round(y_le + 45.4, 2)
    comps += [{"name": "spine_lo", "mass": SPINE_TUBE_KG, "pos": [SPINE_LO_X, mid, 0.0],
               "shape": "rod_y", "length": SPINE_LEN},
              {"name": "spine_hi", "mass": SPINE_TUBE_KG, "pos": [hi_x, mid, 0.0],
               "shape": "rod_y", "length": SPINE_LEN}]
    coupler_ys = (tow_y, 30.0, (30.0 + y_le) / 2, y_le + 45.0)
    comps += [{"name": f"coupler{i}", "mass": COUPLER_KG, "pos": [x_c, y, 0.0], "shape": "point"}
              for i, y in enumerate(coupler_ys)]
    m = sum(c["mass"] for c in comps)
    cg_x = sum(c["mass"] * c["pos"][0] for c in comps) / m
    cg_y = sum(c["mass"] * c["pos"][1] for c in comps) / m
    elems = [dict(e) for e in cfg["body.elements"] if e["name"] != "bar"]
    for name, x in (("spine_lo", SPINE_LO_X), ("spine_hi", hi_x)):
        elems.append({"name": name, "x": x, "z": 0.0, "y0": front, "y1": end, "width_h": SPINE_OD,
                      "width_v": SPINE_OD, "cd_cross": 1.2, "segments": 8})
    for i, y in enumerate(coupler_ys):
        elems.append({"name": f"coupler{i}", "x": x_c, "z": 0.0, "y0": y - 7.5, "y1": y + 7.5,
                      "width_h": 10.0, "width_v": sep + 12.0, "cd_cross": 1.2, "segments": 1})
    # Buoyancy: the channel's volume out, the tubes' and couplers' in.
    vol0 = float(cfg["buoyancy.volume"])
    cb0 = list(cfg["buoyancy.cb"])
    parts = [(-CHANNEL_CM3, [SPINE_LO_X, 130.0]), (SPINE_TUBE_CM3, [SPINE_LO_X, mid]),
             (SPINE_TUBE_CM3, [hi_x, mid])] + [(COUPLER_CM3, [x_c, y]) for y in coupler_ys]
    vol = vol0 + sum(v for v, _ in parts)
    cb = [round((vol0 * cb0[k] + sum(v * p[k] for v, p in parts)) / vol, 2) for k in (0, 1)] + [cb0[2]]
    tow = [round(hi_x + SPINE_OD / 2, 2), tow_y, 0.0]
    j = lambda v: json.dumps(v, separators=(",", ":"))  # noqa: E731
    out = ["--set", "mass.components=" + j(comps), "--set", f"mass.total={m:.4f}",
           "--set", f"cg.x_mm={cg_x:.2f}", "--set", f"cg.y_mm={cg_y:.2f}",
           "--set", "body.elements=" + j(elems), "--set", f"buoyancy.volume={vol:.1f}",
           "--set", "buoyancy.cb=" + j(cb), "--set", "tow_point.mode=rigid",
           "--set", "tow_point.pos=" + j(tow), "--set", f"tail.y_le={y_le:.2f}"]
    geo = {"mass_kg": round(m, 4), "cg_x": round(cg_x, 2), "cg_y": round(cg_y, 2), "cb": cb,
           "tow": tow, "spine_front_y": front, "tail_le_y": round(y_le, 2),
           "tow_above_cg_mm": round(tow[0] - cg_x, 1), "cb_above_cg_mm": round(cb[0] - cg_x, 1)}
    return out, geo


def _spine(job, sep, tow_y, prefix, rep=1):
    """A baseline (bridle) job re-flown with the spine geometry instead."""
    rid, argv, score, info = job
    argv = list(argv)
    # Drop the channel recipe's geometry; keep everything else.
    keep, i = [], 0
    while i < len(argv):
        if argv[i] == "--set" and argv[i + 1].split("=", 1)[0] in (
                "mass.components", "cg.x_mm", "cg.y_mm", "body.elements", "buoyancy.cb",
                "tow_point.pos", "tail.y_le", "body.axial_center_y"):
            i += 2
            continue
        keep.append(argv[i])
        i += 1
    sets, geo = _spine_sets(sep, tow_y)
    tag = f"_SP{sep:g}_Y{-tow_y:g}".replace(".", "p") + (f"_r{rep}" if rep > 1 else "")
    rid = prefix + rid.split("_", 2)[2] + tag
    return (rid, keep + sets, score, {**info, "study": "spine", "sep": sep, "tow_y": tow_y,
                                      "rep": rep, "geo": geo})


def spine_jobs(seps=SPINE_SEPS, tow_ys=SPINE_TOW_YS, reps=1):
    base = [j for j in bridle_jobs(reps=1) if "replay" not in j[0]]
    cases = [j for j in base if j[0] in ("sw_BR_HL8_D4p5_U1", "sw_BR_TL8_D4_S8", "sw_BR_TL25_D4_S35")]
    return [_spine(j, s, y, "sw_SP_", r) for s in seps for y in tow_ys for j in cases
            for r in range(1, reps + 1)]


def study_spine(args) -> None:
    run_batch(spine_jobs(reps=args.reps if args.reps else 1))


def study_spine2(args) -> None:
    """Bracket the first sweep's optimum (12 and 75 mm) on its cases, and
    check every small separation on a tighter 25 m hairpin and a deep hold."""
    caps = RATE_CAPS["40_30"]
    tight = _inx(steady_job(25, 4.0, 25), 25, "sw_IB_", "inx_turns")
    tight = (tight[0].replace("sw_IB_", "sw_BR_T"), [caps if a == ROLL_P[1] else a for a in tight[1]],
             tight[2], {**tight[3], "study": "bridle"})
    deep = next(j for j in bridle_jobs(reps=1) if j[0] == "sw_BR_HL25_D8_U1")
    jobs = spine_jobs(seps=(12, 75))
    jobs += [_spine(j, s, y, "sw_SP_") for s in (12, 25, 50, 75) for y in SPINE_TOW_YS
             for j in (tight, deep)]
    jobs.append(tight)                     # the bridle on the tighter hairpin, for reference
    run_batch(jobs)


# -- TAILS: the underslung V as built, against a cross and other Vs (20 Nov) --
# Every tail has an 87 mm chord and the fin's 44 g. A cross is centred on
# the spine (the tubes' midpoint; the channel's centreline on today's fish)
# with four 77.5 mm arms: the same 270 cm^2 as the V. The 37 deg V is the
# usual V-tail sizing for the same pitch and yaw volumes as a cross.
SPINE_MID_X = SPINE_LO_X + 25.0          # between the tubes at 50 mm spacing
SPINE_TOP_X = SPINE_LO_X + 50.0


def _v(root_x, angle, span):
    ar = round(2 * span / 87.0, 2)
    return [{"root_x": root_x, "root_z": 0.0, "angle_deg": -angle, "span": span, "ar": ar},
            {"root_x": root_x, "root_z": 0.0, "angle_deg": -(180 - angle), "span": span, "ar": ar}]


def _cross(centre_x, arm=77.5):
    return [{"root_x": centre_x, "root_z": 0.0, "angle_deg": a, "span": arm, "ar": round(2 * arm / 87.0, 2)}
            for a in (0.0, 90.0, 180.0, -90.0)]


TAILS = {
    "v48": lambda spine: _v(SPINE_LO_X, 48.0, 155.0),
    "cross": lambda spine: _cross(SPINE_MID_X if spine else SPINE_LO_X),
    "vup": lambda spine: _v(SPINE_TOP_X if spine else SPINE_LO_X, -48.0, 155.0),
    "v37": lambda spine: _v(SPINE_LO_X, 37.0, 150.0),
}


def _with_tail(job, name, spine=True, prefix=None):
    rid, argv, score, info = job
    panels = TAILS[name](spine)
    argv = list(argv)
    cx = sum(p["root_x"] + 0.5 * p["span"] * math.sin(math.radians(p["angle_deg"])) for p in panels) / len(panels)
    comps = None
    for i, a in enumerate(argv):
        if a.startswith("mass.components="):
            comps = json.loads(a.split("=", 1)[1])
            for c in comps:
                if c["name"] == "tail_fin":
                    c["pos"][0] = round(cx, 2)
            argv[i] = "mass.components=" + json.dumps(comps, separators=(",", ":"))
    if comps is not None:                  # the CG height follows the fin's mass
        m = sum(c["mass"] for c in comps)
        cg_x = sum(c["mass"] * c["pos"][0] for c in comps) / m
        for i, a in enumerate(argv):
            if a.startswith("cg.x_mm="):
                argv[i] = f"cg.x_mm={cg_x:.2f}"
    argv += ["--set", "tail.chord=87.0", "--set", "tail.panels=" + json.dumps(panels, separators=(",", ":"))]
    rid = (prefix + rid.split("_", 2)[2] if prefix else rid) + f"_T{name}"
    return (rid, argv, score, {**info, "study": "tails", "tail": name, "spine": spine})


def study_tails(args) -> None:
    caps = RATE_CAPS["40_30"]
    tight = _inx(steady_job(25, 4.0, 25), 25, "sw_IB_", "inx_turns")
    tight = ("sw_BR_TL25_D4_S25", [caps if a == ROLL_P[1] else a for a in tight[1]], tight[2],
             {**tight[3], "study": "bridle"})
    base = {j[0]: j for j in bridle_jobs(reps=1)}
    cases = [base["sw_BR_HL8_D4p5_U1"], base["sw_BR_HL25_D8_U1"], base["sw_BR_TL8_D4_S8"],
             base["sw_BR_TL25_D4_S35"], tight]
    jobs = []
    # Today's fish (channel, bridle) with the V as built, and the replay trim check.
    jobs += [_with_tail(j, "v48", spine=False, prefix="sw_TB_") for j in cases]
    jobs += [_with_tail(base[f"sw_BR_replay_{m}"], "v48", spine=False, prefix="sw_TB_")
             for m in ("rigid", "bridle")]
    # The spine at 50 mm, clamp at 25 mm, with each tail.
    for name in TAILS:
        jobs += [_with_tail(_spine(j, 50, -99.0, "sw_SP_"), name, prefix="sw_TS_") for j in cases]
    # Re-check the spacing with the V as built.
    jobs += [_with_tail(_spine(j, s, -99.0, "sw_SP_"), "v48", prefix="sw_TS_")
             for s in (25, 75) for j in cases[1:]]
    run_batch(jobs)


# -- LENGTH: how short can the fish be? (owner, 24 Nov) --------------------
# Upright V tail only (the best tail on the 1 m spine), scaled in area by k
# (span and chord x sqrt(k)), 3 mm PETG printed as the owner prints it:
# 3 perimeters, 10 % gyroid, ~3 top and bottom layers of 0.2 mm, so ~1.2 mm
# of solid skin plus 1.8 mm of 10 % core -- ~1.38 mm of plastic per panel
# (both panels ~47 g, close to the CAD fin's 44 g). The core is taken as
# flooded, so the tail displaces only its plastic. Three layouts:
#   spine  -- the 50 mm twin-tube spine, clamp at y -99 on the top tube,
#             tail quarter chord `arm` behind today's spine CG; the spine
#             ends 5 mm past the tail, so its length follows the arm
#   endcap -- no spine: a hoop on top of the pressure tube's front, the V
#             on a bracket on top of the tube just behind the end cap
#   hoop   -- no spine, no tail: only the hoop, at `hoop_y`
# The 200 g weight stays where it is; the CG moves with the tail (owner).
PETG_G_CM3 = 1.27
TAIL_T_MM = 3.0
TAIL_SOLID_MM = 1.2 + 1.8 * 0.10        # plastic per panel: skins + 10 % gyroid core
SPINE_CG_Y = 137.8                       # today's 1 m spine CG station
HOOP_X = 71.0                            # top of the 112 mm tube + a ~15 mm hoop
LENGTH_ARMS = (668, 550, 450, 350, 250)
LENGTH_KS = (1.0, 1.5, 2.0)
ENDCAP_KS = (1.0, 2.0, 3.0)
HOOP_YS = (-40.0, 30.0, 138.0)


def _compact_sets(layout="spine", arm=668.0, k=1.0, hoop_y=-40.0):
    from towsim.frames import Config
    cfg = Config()
    sets, _ = _ballast_sets("under_front")
    comps = [c for c in json.loads(sets[1].split("=", 1)[1]) if c["name"] not in ("bar", "tail_fin")]
    elems = [dict(e) for e in cfg["body.elements"] if e["name"] != "bar"]
    vol0, cb0 = float(cfg["buoyancy.volume"]), list(cfg["buoyancy.cb"])
    # The channel's and the CAD fin's volume out (the fin's from its 44 g of PETG).
    parts = [(-CHANNEL_CM3, [SPINE_LO_X, 130.0]), (-0.0444 * 1000 / PETG_G_CM3, [14.1, 721.5])]
    chord, span = 87.0 * math.sqrt(k), 155.0 * math.sqrt(k)
    panels, y_le, front, end = [], None, None, None
    if layout == "spine":
        tow = [round(SPINE_TOP_X + SPINE_OD / 2, 2), -99.0]
        front = -124.0
        y_le = SPINE_CG_Y + arm - 0.25 * chord
        end = y_le + chord + 5.0
        L = end - front
        mid = (front + end) / 2
        for name, x in (("spine_lo", SPINE_LO_X), ("spine_hi", SPINE_TOP_X)):
            comps.append({"name": name, "mass": round(SPINE_TUBE_KG * L / 1000, 5), "pos": [x, mid, 0.0],
                          "shape": "rod_y", "length": L})
            elems.append({"name": name, "x": x, "z": 0.0, "y0": front, "y1": end, "width_h": SPINE_OD,
                          "width_v": SPINE_OD, "cd_cross": 1.2, "segments": max(2, int(L / 125))})
            parts.append((SPINE_TUBE_CM3 * L / 1000, [x, mid]))
        cys = [-99.0, 30.0, y_le + chord / 2]
        if L >= 500:
            cys.append((30.0 + y_le) / 2)
        for i, y in enumerate(cys):
            comps.append({"name": f"coupler{i}", "mass": COUPLER_KG, "pos": [SPINE_MID_X, y, 0.0], "shape": "point"})
            elems.append({"name": f"coupler{i}", "x": SPINE_MID_X, "z": 0.0, "y0": y - 7.5, "y1": y + 7.5,
                          "width_h": 10.0, "width_v": 62.0, "cd_cross": 1.2, "segments": 1})
            parts.append((COUPLER_CM3, [SPINE_MID_X, y]))
        panels = _v(SPINE_TOP_X, -48.0, span)
    else:
        tow = [HOOP_X, -40.0 if layout == "endcap" else hoop_y]
        comps.append({"name": "hoop", "mass": COUPLER_KG, "pos": [HOOP_X - 5, tow[1], 0.0], "shape": "point"})
        if layout == "endcap":
            y_le = 192.0                                   # 10 mm behind the end-cap face
            comps.append({"name": "bracket", "mass": COUPLER_KG, "pos": [62.0, y_le + chord / 2, 0.0], "shape": "point"})
            panels = _v(66.0, -48.0, span)
    if panels:
        area_cm2 = 2 * span * chord / 100.0
        vol = area_cm2 * TAIL_SOLID_MM / 10.0          # plastic only; the core floods
        cx = sum(p["root_x"] + 0.5 * p["span"] * math.sin(math.radians(p["angle_deg"])) for p in panels) / len(panels)
        comps.append({"name": "tail_fin", "mass": round(vol * PETG_G_CM3 / 1000, 4),
                      "pos": [round(cx, 2), round(y_le + chord / 2, 2), 0.0], "shape": "point"})
        parts.append((vol, [cx, y_le + chord / 2]))
    m = sum(c["mass"] for c in comps)
    cg_x = sum(c["mass"] * c["pos"][0] for c in comps) / m
    cg_y = sum(c["mass"] * c["pos"][1] for c in comps) / m
    vol_t = vol0 + sum(v for v, _ in parts)
    cb = [round((vol0 * cb0[i] + sum(v * p[i] for v, p in parts)) / vol_t, 2) for i in (0, 1)] + [cb0[2]]
    j = lambda v: json.dumps(v, separators=(",", ":"))  # noqa: E731
    out = ["--set", "mass.components=" + j(comps), "--set", f"mass.total={m:.4f}",
           "--set", f"cg.x_mm={cg_x:.2f}", "--set", f"cg.y_mm={cg_y:.2f}",
           "--set", "body.elements=" + j(elems), "--set", f"buoyancy.volume={vol_t:.1f}",
           "--set", "buoyancy.cb=" + j(cb), "--set", "tow_point.mode=rigid",
           "--set", "tow_point.pos=" + j(tow + [0.0]), "--set", f"tail.chord={chord:.2f}",
           "--set", f"tail.y_le={(y_le if y_le is not None else 700.0):.2f}",
           "--set", "tail.panels=" + j(panels)]
    tail_qc = (y_le + 0.25 * chord) if y_le is not None else None
    geo = {"layout": layout, "arm_nominal": arm if layout == "spine" else None, "k": k if panels else 0.0,
           "hoop_y": tow[1] if layout != "spine" else None,
           "length_mm": round((end - front) if layout == "spine" else (182.0 + 85.3 + (chord + 15 if panels else 0)), 1),
           "spine_mm": round(end - front, 1) if layout == "spine" else 0.0,
           "tail_area_cm2": round(2 * span * chord / 100.0, 1) if panels else 0.0,
           "tail_mass_g": round(comps[-1]["mass"] * 1000, 1) if panels else 0.0,
           "arm_actual_mm": round(tail_qc - cg_y, 1) if tail_qc is not None else None,
           "tow_ahead_of_cg_mm": round(cg_y - tow[1], 1), "tow_above_cg_mm": round(tow[0] - cg_x, 1),
           "mass_kg": round(m, 4), "cg": [round(cg_x, 2), round(cg_y, 2)], "cb": cb,
           "buoyant_g": round(vol_t * 1.025 - m * 1000)}
    return out, geo


GEOMETRY_KEYS = ("mass.components", "mass.total", "cg.x_mm", "cg.y_mm", "body.elements", "buoyancy.cb",
                 "buoyancy.volume", "tow_point.pos", "tow_point.mode", "tail.y_le", "tail.chord",
                 "tail.panels", "body.axial_center_y")


def _with_geometry(job, sets, geo, rid):
    _, argv, score, info = job
    keep, i = [], 0
    while i < len(argv):
        if argv[i] == "--set" and argv[i + 1].split("=", 1)[0] in GEOMETRY_KEYS:
            i += 2
            continue
        keep.append(argv[i])
        i += 1
    return (rid, keep + sets, score, {**info, "study": "length", "geo": geo})


def length_configs(stage="A"):
    cfgs = [("spine", dict(arm=a, k=k)) for a in LENGTH_ARMS for k in LENGTH_KS]
    cfgs += [("endcap", dict(k=k)) for k in ENDCAP_KS]
    cfgs += [("hoop", dict(hoop_y=y)) for y in HOOP_YS]
    return cfgs


def _cfg_tag(layout, kw):
    if layout == "spine":
        return f"sp_A{kw['arm']:g}_K{kw['k']:g}".replace(".", "p")
    if layout == "endcap":
        return f"ec_K{kw['k']:g}".replace(".", "p")
    return f"hp_Y{kw['hoop_y']:g}".replace("-", "m").replace(".", "p")


def score_hold_len(run: Path) -> dict:
    """score_hold_trim plus heading wander about the tow line (last 180 s,
    first half against second half so a growing yaw oscillation shows)."""
    out = score_hold_trim(run)
    rows = [r for r in csv.DictReader(open(run / "truth.csv")) if r.get("jig", "0") == "0"]
    t = np.array([float(r["t_boat"]) for r in rows])
    yaw = np.unwrap(np.radians([float(r["yaw"]) for r in rows]))
    last = t >= t[-1] - 180
    y = np.degrees(yaw[last] - np.median(yaw[last]))
    h = len(y) // 2
    out.update({"yaw_std": round(float(np.std(y)), 2), "yaw_std_1": round(float(np.std(y[:h])), 2),
                "yaw_std_2": round(float(np.std(y[h:])), 2)})
    return out


DEPTH_DIVE_S = {8: 35, 25: 50}


def score_depth_change(run: Path) -> dict:
    rows = [r for r in csv.DictReader(open(run / "truth.csv")) if r.get("jig", "0") == "0"]
    t = np.array([float(r["t_sim"]) for r in rows])
    z = np.array([float(r["depth"]) for r in rows])
    roll = np.array([float(r["roll"]) for r in rows])
    pitch = np.array([float(r["pitch"]) for r in rows])
    rc3 = np.array([int(float(r["rc3"])) for r in rows])
    # The climb holds full up long enough to reach the surface, where the fish
    # (no extension, so no auto-righting) wallows; judge the transit only,
    # while it is below 1 m, and report the surface separately.
    below = z >= 1.0
    tr = below & np.isin(rc3, (1200, 1800))
    out = {"max_roll": round(float(np.abs(roll[tr]).max()), 1) if tr.any() else None,
           "upsets": int(np.sum(np.diff((np.abs(roll[tr]) > 45).astype(int)) == 1)),
           "surface_max_roll": round(float(np.abs(roll[~below]).max()), 1) if (~below).any() else None,
           "depth_max": round(float(z.max()), 2)}
    for name, pwm in (("dive", 1200), ("climb", 1800)):
        m = rc3 == pwm
        if m.sum() < 20:
            continue
        tt, zz = t[m], z[m]
        a, b = tt[0] + 0.1 * (tt[-1] - tt[0]), tt[0] + 0.9 * (tt[-1] - tt[0])
        k = (tt >= a) & (tt <= b)
        out[f"{name}_rate"] = round(float(np.polyfit(tt[k], zz[k], 1)[0]), 3)
        out[f"{name}_from"] = round(float(zz[0]), 2)
        out[f"{name}_to"] = round(float(zz[-1]), 2)
        mb = m & below
        out[f"{name}_max_roll"] = round(float(np.abs(roll[mb]).max()), 1) if mb.any() else None
        out[f"{name}_pitch_spread"] = round(float(np.ptp(pitch[mb])), 1) if mb.any() else None
    return out


def depth_job(L, sets, geo, tag):
    """1 m -> the tether's max depth (stick 1200) -> back to 1 m (stick 1800),
    ALT_HOLD, straight at 1 m/s, no extension: an upset is not rescued."""
    T = DEPTH_DIVE_S[L]
    prog = [(1500, 0), (1200, 20), (1500, 20 + T), (1800, 50 + T), (1500, 50 + 2 * T)]
    end = 10 * 3600 + 31 * 60 + 30 + 75 + 2 * T
    argv = ["--start", "10:31:30", "--end", f"{end // 3600}:{end % 3600 // 60:02d}:{end % 60:02d}",
            "--straight", "1.0", "--init-depth", "1.0", "--warmup", "58",
            "--param-file", RATE_CAPS["40_30"],
            "--stick-steps", ",".join(f"{p}@{s}" for p, s in prog), "--stick-mode", "alt_hold",
            "--set", f"cable.length={L}", "--set", f"cable.segments={int(round(L))}",
            "--set", "cable.model=inextensible", *sets]
    return (f"sw_LN_DL{L}_{tag}", argv, score_depth_change,
            {"study": "length", "case": f"depth_L{L}", "geo": geo, "L": L})


def length_jobs(stage="A", only=None, configs=None):
    base = {j[0]: j for j in bridle_jobs(reps=1)}
    caps = RATE_CAPS["40_30"]
    tight = _inx(steady_job(25, 4.0, 25), 25, "sw_IB_", "inx_turns")
    tight = ("sw_BR_TL25_D4_S25", [caps if a == ROLL_P[1] else a for a in tight[1]], tight[2],
             {**tight[3], "study": "bridle"})
    holds = [(base["sw_BR_HL8_D4p5_U1"], "HL8"), (base["sw_BR_HL25_D8_U1"], "HL25")]
    turns = [(base["sw_BR_TL8_D4_S8"], "TL8S8"), (tight, "TL25S25")]
    jobs = []
    for layout, kw in (configs or length_configs()):
        tag = _cfg_tag(layout, kw)
        if only is not None and tag not in only:
            continue
        sets, geo = _compact_sets(layout, **kw)
        if stage == "A":
            for j, c in holds:
                r = _with_geometry(j, sets, geo, f"sw_LN_{c}_{tag}")
                jobs.append((r[0], r[1], score_hold_len, {**r[3], "case": c, "cfg": tag}))
        else:
            for j, c in turns:
                r = _with_geometry(j, sets, geo, f"sw_LN_{c}_{tag}")
                jobs.append((*r[:3], {**r[3], "case": c, "cfg": tag}))
            for L in (8, 25):
                d = depth_job(L, sets, geo, tag)
                jobs.append((d[0], d[1], d[2], {**d[3], "cfg": tag}))
    return jobs


LENGTH_C = [("spine", dict(arm=350, k=1.0)), ("spine", dict(arm=300, k=1.0)), ("spine", dict(arm=300, k=1.5)),
            ("spine", dict(arm=250, k=1.5)), ("spine", dict(arm=250, k=2.0)), ("spine", dict(arm=668, k=1.0))]


def length_jobs_c():
    """Boundary check: the configs around the shortest pass, through every
    stage A/B case plus a wider 25 m hairpin and holds at 0.8 / 1.2 m/s."""
    base = {j[0]: j for j in bridle_jobs(reps=1)}
    jobs = []
    for layout, kw in LENGTH_C:
        tag = _cfg_tag(layout, kw)
        sets, geo = _compact_sets(layout, **kw)
        jobs += length_jobs("A", configs=[(layout, kw)]) + length_jobs("B", configs=[(layout, kw)])
        r = _with_geometry(base["sw_BR_TL25_D4_S35"], sets, geo, f"sw_LN_TL25S35_{tag}")
        jobs.append((*r[:3], {**r[3], "case": "TL25S35", "cfg": tag}))
        for hj, c in ((base["sw_BR_HL8_D4p5_U1"], "HL8"), (base["sw_BR_HL25_D8_U1"], "HL25")):
            for v in (0.8, 1.2):
                r = _with_geometry(hj, sets, geo, f"sw_LN_{c}V{v:g}_{tag}".replace(".", "p"))
                argv = list(r[1])
                argv[argv.index("--straight") + 1] = str(v)
                jobs.append((r[0], argv, score_hold_len, {**r[3], "case": f"{c}V{v:g}", "cfg": tag}))
    return jobs


def hold_ok(s):
    """Steady straight line: level, within +-0.3 m, no pitch or growing yaw wander."""
    return (s["max_roll"] < 20 and s["rescues"] == 0 and s["depth_max"] - s["depth_min"] <= 0.6
            and s.get("pitch_spread", 0) < 12 and s.get("yaw_std_2", 0) <= 1.5 * s.get("yaw_std_1", 0) + 1.0)


def study_length(args) -> None:
    stage = (args.stage or "A").upper()
    if stage == "C":
        run_batch(length_jobs_c())
        return
    if stage == "A":
        run_batch(length_jobs("A"))
        return
    res = load_results()
    passed = []
    for layout, kw in length_configs():
        tag = _cfg_tag(layout, kw)
        hs = [res.get(f"sw_LN_{c}_{tag}", {}).get("score") for c in ("HL8", "HL25")]
        if all(h and hold_ok(h) for h in hs):
            passed.append(tag)
    print("passed stage A:", passed, flush=True)
    run_batch(length_jobs("B", only=set(passed)))


# -- PITCH: why the 1 m fish pitches on a straight line (owner, 25 Nov) ----
# With a perfectly steady boat the fish still oscillates ~+-1.5 deg at an
# 8.9 s period on both tethers, the wings moving with it: the depth hold
# and the wings' pitching moment feeding each other. Lock the wings, soften
# the depth controller, and add the boat's measured speed wobble.
PITCH_PARAMS = {"base": RATE_CAPS["40_30"],
                "soft": "params/scenarios/roll_p_1_5_ps40_30_soft.parm",
                "softer": "params/scenarios/roll_p_1_5_ps40_30_softer.parm"}


def score_pitch(run: Path) -> dict:
    out = score_hold_len(run)
    rows = [r for r in csv.DictReader(open(run / "truth.csv")) if r.get("jig", "0") == "0"]
    t = np.array([float(r["t_boat"]) for r in rows])
    last = t >= t[-1] - 180
    p = np.array([float(r["pitch"]) for r in rows])[last]
    c = 0.5 * (np.array([float(r["defl_port_deg"]) for r in rows]) + np.array([float(r["defl_stbd_deg"]) for r in rows]))[last]
    z = np.array([float(r["depth"]) for r in rows])[last]
    f = np.fft.rfftfreq(len(p), 0.1)
    P = np.abs(np.fft.rfft(p - p.mean())) ** 2
    band = f > 0.03
    out.update({"pitch_std": round(float(p.std()), 2), "pitch_period_s": round(float(1 / f[band][np.argmax(P[band])]), 1),
                "collective_std_deg": round(float(c.std()), 2), "depth_std": round(float(z.std()), 3)})
    return out


def pitch_jobs():
    base = {j[0]: j for j in bridle_jobs(reps=1)}
    sets, geo = _compact_sets("spine", arm=668, k=1.0)
    jobs = []
    for L, hk in ((25, "sw_BR_HL25_D8_U1"), (8, "sw_BR_HL8_D4p5_U1")):
        for name, wob, extra in (("base", 0.0, []), ("lock", 0.0, ["--fixed-stick", "1500"]),
                                 ("soft", 0.0, []), ("softer", 0.0, []),
                                 ("base", 0.05, []), ("soft", 0.05, []), ("softer", 0.05, [])):
            r = _with_geometry(base[hk], sets, geo, f"sw_PI_L{L}_{name}" + ("_wob" if wob else ""))
            argv = list(r[1])
            argv[argv.index("--param-file") + 1] = PITCH_PARAMS["base" if name == "lock" else name]
            if wob:
                argv += ["--speed-wobble", str(wob)]
            jobs.append((r[0], argv + extra, score_pitch,
                         {**r[3], "study": "pitch", "case": name, "wobble": wob, "L": L}))
    return jobs


def study_pitch(args) -> None:
    run_batch(pitch_jobs())


# -- TETHER20: max depth on the owner's new 20 m tether (25 Nov) -----------
# The 1 m spine (50 mm tubes, clamp at 25 mm, upright V as printed), 40/30
# caps, 1 m/s straight. Two answers: the deepest target SURF TRACK holds
# steady (as inx_max_depths on the 8 m / 25 m tethers), and the deepest the
# fish reaches with the stick held full down from 1 m.
DEPTH_DIVE_S[20] = 60
T20_TARGETS = (6.0, 6.5, 7.0, 7.5, 8.0, 8.5)


def tether20_jobs(L=20, speed=1.0):
    """Holds at each target and a full-down dive, at `speed` m/s (1.0 keeps
    the original run ids; other speeds get a _V suffix)."""
    sets, geo = _compact_sets("spine", arm=668, k=1.0)
    sfx = "" if speed == 1.0 else f"_V{speed:g}".replace(".", "p")
    jobs = []
    for D in T20_TARGETS + ((9.0, 9.5) if speed < 1.0 else ()):
        rid, argv, score, info = inx_hold_job(L, D)
        argv = list(argv)
        argv[argv.index("--param-file") + 1] = RATE_CAPS["40_30"]
        argv[argv.index("--straight") + 1] = str(speed)
        r = _with_geometry((rid, argv, score, info), sets, geo, f"sw_T20_H_D{D:g}".replace(".", "p") + sfx)
        jobs.append((r[0], r[1], score_hold_len, {**r[3], "study": "tether20", "case": "hold", "D": D,
                                                  "L": L, "speed": speed}))
    d = depth_job(L, sets, geo, "sp_A668_K1")
    argv = list(d[1])
    argv[argv.index("--straight") + 1] = str(speed)
    jobs.append((d[0].replace("sw_LN_", "sw_T20_") + sfx, argv, d[2],
                 {**d[3], "study": "tether20", "case": "dive", "speed": speed}))
    return jobs


def study_tether20(args) -> None:
    run_batch(tether20_jobs(speed=args.speed or 1.0))
    res = load_results()
    sp = args.speed or 1.0
    held = [e["D"] for e in res.values() if e.get("study") == "tether20" and e.get("case") == "hold"
            and e.get("speed", 1.0) == sp and "score" in e and hold_ok(e["score"])]
    print(f"deepest held steady on 20 m at {sp} m/s:", max(held) if held else None)


def pitchfix_jobs():
    """What the softer depth controller costs: the 1 m spine's hairpins, depth
    changes and the coral dodge, with the as-flown and the softest gains."""
    sets, geo = _compact_sets("spine", arm=668, k=1.0)
    cfg = [("spine", dict(arm=668, k=1.0))]
    jobs = []
    for name in ("base", "softer"):
        for j in length_jobs("B", configs=cfg):
            argv = list(j[1])
            argv[argv.index("--param-file") + 1] = PITCH_PARAMS[name]
            jobs.append((j[0].replace("sw_LN_", "sw_PF_") + f"_{name}", argv, j[2],
                         {**j[3], "study": "pitchfix", "gains": name}))
        for L in (8, 25):
            rid, argv, score, info = bottom_job("coral", L, "pf")
            argv = list(argv)
            argv[argv.index(ROLL_P[1])] = PITCH_PARAMS[name]
            r = _with_geometry((rid, argv, score, info), sets, geo, f"sw_PF_coral_L{L}_{name}")
            jobs.append((r[0], r[1], score, {**r[3], "study": "pitchfix", "case": f"coral_L{L}", "gains": name}))
    return jobs


def study_pitchfix(args) -> None:
    run_batch(pitchfix_jobs())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("study", choices=["righting", "early", "maxdepth", "fixed", "spacing", "grid", "rescore-grid", "steady", "dz0", "jog", "dt", "dzsel", "towpt", "ballast", "geometry", "recipe", "recipe-spacing", "inx-hold", "inx-turns", "rates", "bottom", "altitude", "bridle", "spine", "spine2", "tails", "length", "pitch", "tether20", "pitchfix", "rank"])
    ap.add_argument("--vertical-pwm", type=int, default=1900)
    ap.add_argument("--roll-min-depth", type=float, default=0.0)
    ap.add_argument("--descent-start", type=float, default=0.0)
    ap.add_argument("--lengths", type=float, nargs="*")
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--recipes", nargs="*", help="recipe study: names from RECIPES")
    ap.add_argument("--recipe", default="today", help="recipe-spacing: which recipe")
    ap.add_argument("--speed", type=float, help="tether20: tow speed (m/s), default 1.0")
    ap.add_argument("--stage", help="length: A (holds), B (turns and depth changes)")
    ap.add_argument("--variants", nargs="*", help="bottom: names from BOTTOM_VARIANTS")
    ap.add_argument("--caps", nargs="*", help="rates: pilot speed caps from RATE_CAPS")
    ap.add_argument("--reps", type=int, default=2, help="runs per recipe and track (righting)")
    args = ap.parse_args(argv)
    global WORKERS
    WORKERS = args.workers
    {"righting": study_righting, "early": study_early, "grid": study_grid, "rescore-grid": rescore_grid, "steady": study_steady, "dz0": study_dz0, "jog": study_jog, "dt": study_dt, "dzsel": study_dzsel, "towpt": study_towpt, "ballast": study_ballast, "geometry": study_geometry, "recipe": study_recipe,
     "recipe-spacing": study_recipe_spacing, "inx-hold": study_inx_hold,
     "inx-turns": study_inx_turns, "rates": study_rates, "bottom": study_bottom, "altitude": study_altitude, "bridle": study_bridle, "spine": study_spine, "spine2": study_spine2, "tails": study_tails, "length": study_length, "pitch": study_pitch, "tether20": study_tether20, "pitchfix": study_pitchfix, "fixed": study_fixed,
     "maxdepth": study_maxdepth,
     "spacing": study_spacing, "rank": lambda a: print(rank_righting())}[args.study](args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
