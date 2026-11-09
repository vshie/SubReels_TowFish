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


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("study", choices=["righting", "early", "maxdepth", "fixed", "spacing", "grid", "rescore-grid", "steady", "dz0", "jog", "dt", "dzsel", "towpt", "ballast", "geometry", "recipe", "recipe-spacing", "inx-hold", "inx-turns", "rates", "bottom", "rank"])
    ap.add_argument("--vertical-pwm", type=int, default=1900)
    ap.add_argument("--roll-min-depth", type=float, default=0.0)
    ap.add_argument("--descent-start", type=float, default=0.0)
    ap.add_argument("--lengths", type=float, nargs="*")
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--recipes", nargs="*", help="recipe study: names from RECIPES")
    ap.add_argument("--recipe", default="today", help="recipe-spacing: which recipe")
    ap.add_argument("--variants", nargs="*", help="bottom: names from BOTTOM_VARIANTS")
    ap.add_argument("--caps", nargs="*", help="rates: pilot speed caps from RATE_CAPS")
    ap.add_argument("--reps", type=int, default=2, help="runs per recipe and track (righting)")
    args = ap.parse_args(argv)
    global WORKERS
    WORKERS = args.workers
    {"righting": study_righting, "early": study_early, "grid": study_grid, "rescore-grid": rescore_grid, "steady": study_steady, "dz0": study_dz0, "jog": study_jog, "dt": study_dt, "dzsel": study_dzsel, "towpt": study_towpt, "ballast": study_ballast, "geometry": study_geometry, "recipe": study_recipe,
     "recipe-spacing": study_recipe_spacing, "inx-hold": study_inx_hold,
     "inx-turns": study_inx_turns, "rates": study_rates, "bottom": study_bottom, "fixed": study_fixed,
     "maxdepth": study_maxdepth,
     "spacing": study_spacing, "rank": lambda a: print(rank_righting())}[args.study](args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
