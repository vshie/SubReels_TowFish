#!/usr/bin/env python3
"""Collect the bottom-following study into one JSON for the report page.

Reads runs/rates/log_rates.json (analysis/rates_from_log.py), the sweep's
results_rates.json and results_bottom.json, and each bottom run's
truth.csv (profiles downsampled to 0.5 m of track).

    python3 Simulator/analysis/bottom_report.py
        ->  runs/report/bottom_data.json and runs/report/bottom_following.html
            (bottom_report_template.html with the data embedded)
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

SIM = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SIM))
from analysis.bottom_follow import load  # noqa: E402
from towsim.terrain import coral_profile, slope_profile  # noqa: E402

RUNS = SIM / "runs"
OUT = RUNS / "report" / "bottom_data.json"
PROFILE_VARIANTS = ("base", "all", "all_ps40")
PAGE = RUNS / "report" / "bottom_following.html"
TEMPLATE = Path(__file__).resolve().parent / "bottom_report_template.html"


def profile(run: Path, step=0.5):
    d = load(run)
    s = d["s_fish"]
    out = {"s": [], "depth": [], "target": [], "alt": []}
    edges = np.arange(np.floor(s.min()), s.max(), step)
    idx = np.searchsorted(s, edges)
    for i in idx:
        if i >= len(s):
            continue
        out["s"].append(round(float(s[i]), 2))
        out["depth"].append(round(float(d["depth"][i]), 3))
        tg = d["surf_target"][i]
        out["target"].append(None if np.isnan(tg) else round(float(tg), 3))
        out["alt"].append(round(float(d["alt_true"][i]), 3))
    return out


def main() -> int:
    data = {"log_rates": json.loads((RUNS / "rates" / "log_rates.json").read_text())}
    data["log_rates"].pop("segments", None)
    rates = json.loads((RUNS / "sweep" / "results_rates.json").read_text())
    data["sim_rates"] = [{k: e[k] for k in ("mode", "L", "v", "cap")} | {"score": e["score"]}
                         for e in rates.values() if "score" in e]
    bottom = json.loads((RUNS / "sweep" / "results_bottom.json").read_text())
    data["bottom"] = []
    for rid, e in sorted(bottom.items()):
        if "score" not in e:
            continue
        row = {k: e[k] for k in ("terrain", "L", "variant", "rep")} | {"run": rid, "score": e["score"]}
        if e["rep"] == 1 and e["variant"] in PROFILE_VARIANTS:
            row["profile"] = profile(RUNS / rid)
        data["bottom"].append(row)
    xs = np.arange(-30, 330.5, 0.5)
    data["terrain"] = {"slope": [round(slope_profile(x), 3) for x in xs],
                       "coral": [round(coral_profile(x, 0.0), 3) for x in xs],
                       "s": xs.round(2).tolist()}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(data, separators=(",", ":")))
    page = TEMPLATE.read_text().replace("/*DATA*/null", json.dumps(data, separators=(",", ":")))
    PAGE.write_text(page)
    print("wrote", OUT, "and", PAGE, f"{len(page) / 1e3:.0f} kB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
