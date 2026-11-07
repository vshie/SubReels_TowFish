#!/usr/bin/env python3
"""Score the extension's towfish position estimators against the simulator.

For every run with a cable log: the boat tow point is cable node 0, the
fish's true position is truth.csv north/east. Each estimator in
``app/towed_position.py`` (the code the extension ships) is fed only what
the extension can read -- boat position, course and speed, fish depth and
compass heading, the tether length -- at the extension's sample rate, and
its error against the truth is binned by straight leg vs turn.

    python3 Simulator/analysis/position_eval.py                # all runs
    python3 Simulator/analysis/position_eval.py --gps-noise 1.0 --yaw-noise 5 --yaw-bias 3
    python3 Simulator/analysis/position_eval.py --json Simulator/runs/report/position_eval.json
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT.parent / "app"))
from towed_position import METHODS, TowedPositionEstimator  # noqa: E402

RUNS = ROOT / "runs"
TURN_DEG = 15.0          # boat course change that counts as a turn...
TURN_WINDOW_S = 40.0     # ...within this window (covers the turn and its aftermath)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def load_run(run: Path, rate_hz: float):
    meta = json.loads((run / "meta.json").read_text())
    L = next((float(c[1]) for c in meta["config"] if c[0] == "cable.length"), None)
    truth = list(csv.DictReader(open(run / "truth.csv")))
    cable = list(csv.reader(open(run / "cable.csv")))[1:]
    n = min(len(truth), len(cable))
    step = max(1, int(round(10.0 / rate_hz)))        # truth is 10 Hz
    rows = []
    for i in range(0, n, step):
        r, c = truth[i], cable[i]
        if r.get("jig", "0") != "0":
            continue
        rows.append({"t": float(r["t_sim"]), "bn": float(c[1]), "be": float(c[2]),
                     "fn": float(r["north"]), "fe": float(r["east"]),
                     "depth": float(r["depth"]), "yaw": float(r["yaw"]),
                     "tension": float(r["tension_n"])})
    return L, rows


def boat_kinematics(rows, span_s=1.0):
    """Course and speed from the boat's own fixes, like a GPS would give."""
    j = 0
    for i, r in enumerate(rows):
        while rows[j]["t"] < r["t"] - span_s:
            j += 1
        a = rows[j]
        dt = r["t"] - a["t"]
        if dt > 0.2:
            dn, de = r["bn"] - a["bn"], r["be"] - a["be"]
            r["speed"] = math.hypot(dn, de) / dt
            r["course"] = math.degrees(math.atan2(de, dn)) % 360 if r["speed"] > 0.1 else None
        else:
            r["speed"], r["course"] = 0.0, None
    # Turn flag: course changed by more than TURN_DEG within the window.
    last = None
    hist = []
    for r in rows:
        if r["course"] is not None:
            hist.append((r["t"], r["course"]))
        while hist and hist[0][0] < r["t"] - TURN_WINDOW_S:
            hist.pop(0)
        r["turn"] = bool(hist) and max(abs(wrap180(c - hist[-1][1])) for _, c in hist) > TURN_DEG


def evaluate(run: Path, args, rng):
    L, rows = load_run(run, args.rate)
    if not L or len(rows) < 100:
        return None
    boat_kinematics(rows)
    bias = args.yaw_bias
    out = {}
    for spec in specs(args):
        m, _, arg = spec.partition(":")
        kw = {}
        if arg.startswith("len"):
            kw["link_len_m"] = float(arg[3:])
        elif arg:
            kw["links"] = int(arg)
        est = TowedPositionEstimator(L, method=m, fixed_offset_m=args.fixed_offset, **kw)
        errs = {"straight": [], "turn": [], "slack": []}
        # GPS error: slowly wandering (first-order Gauss-Markov, like an EKF
        # solution) plus a little white jitter.
        wn = we = 0.0
        a = math.exp(-1.0 / (args.rate * args.gps_tau)) if args.gps_tau > 0 else 0.0
        q = args.gps_noise * math.sqrt(max(0.0, 1 - a * a))
        for k, r in enumerate(rows):
            wn = a * wn + rng.gauss(0, q) if args.gps_noise else 0.0
            we = a * we + rng.gauss(0, q) if args.gps_noise else 0.0
            gn = wn + (rng.gauss(0, args.gps_jitter) if args.gps_jitter else 0.0)
            ge = we + (rng.gauss(0, args.gps_jitter) if args.gps_jitter else 0.0)
            yaw = r["yaw"] + bias + (rng.gauss(0, args.yaw_noise) if args.yaw_noise else 0.0)
            o = est.update_ne(r["t"], r["bn"] + gn, r["be"] + ge, r["course"], r["speed"],
                              r["depth"], yaw)
            if r["t"] - rows[0]["t"] < 20:
                continue
            e = math.hypot(o["north"] - r["fn"], o["east"] - r["fe"])
            errs["turn" if r["turn"] else "straight"].append(e)
            if r["tension"] < 1.0:
                errs["slack"].append(e)
        out[spec] = errs
    return L, out


def specs(args):
    base = [m for m in METHODS if m != "chain"]
    return (base + [f"chain:{n}" for n in args.links]
            + [f"chain:len{x:g}" for x in args.link_len])


def pct(v, q):
    if not v:
        return None
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", default="sw_*", help="glob under Simulator/runs")
    ap.add_argument("--rate", type=float, default=5.0, help="extension sample rate, Hz")
    ap.add_argument("--fixed-offset", type=float, default=7.0)
    ap.add_argument("--gps-noise", type=float, default=0.0, help="boat GPS noise, m (1 sigma)")
    ap.add_argument("--gps-tau", type=float, default=30.0, help="GPS error correlation time, s")
    ap.add_argument("--gps-jitter", type=float, default=0.0, help="white GPS jitter, m")
    ap.add_argument("--yaw-noise", type=float, default=0.0, help="fish compass noise, deg")
    ap.add_argument("--yaw-bias", type=float, default=0.0, help="fish compass bias, deg")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--links", type=int, nargs="*", default=[], help="chain link counts to try")
    ap.add_argument("--link-len", type=float, nargs="*", default=[8.0],
                    help="metres of tether per extra chain link to try")
    ap.add_argument("--json", type=Path)
    args = ap.parse_args(argv)
    rng = random.Random(args.seed)

    pooled = {}      # (L, method, segment) -> errors
    nruns = 0
    for run in sorted(RUNS.glob(args.runs)):
        if not (run / "cable.csv").exists():
            continue
        r = evaluate(run, args, rng)
        if r is None:
            continue
        nruns += 1
        L, out = r
        for m, errs in out.items():
            for seg, v in errs.items():
                pooled.setdefault((L, m, seg), []).extend(v)

    table = []
    for L in sorted({k[0] for k in pooled}):
        for m in specs(args):
            row = {"L": L, "method": m}
            for seg in ("straight", "turn", "slack"):
                v = pooled.get((L, m, seg), [])
                row[seg] = {"n": len(v), "median": pct(v, 0.5), "p95": pct(v, 0.95), "max": pct(v, 1.0)}
            table.append(row)

    print(f"{nruns} runs; error = horizontal distance from the fish's true position (m)")
    print(f"{'tether':>7} {'method':>10} | {'straight med/p95':>17} | {'turn med/p95/max':>22} | {'slack med/p95':>14}")
    f = lambda x: "  -  " if x is None else f"{x:5.2f}"
    for row in table:
        s, t, k = row["straight"], row["turn"], row["slack"]
        print(f"{row['L']:>6g}m {row['method']:>10} | {f(s['median'])} / {f(s['p95'])}   | "
              f"{f(t['median'])} / {f(t['p95'])} / {f(t['max'])} | {f(k['median'])} / {f(k['p95'])}")
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps({"args": vars(args) | {"json": str(args.json)},
                                         "runs": nruns, "table": table}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
