#!/usr/bin/env python3
"""Build the minimum-length report page from runs/sweep/results_length.json.

    python3 Simulator/analysis/length_report.py  ->  runs/report/towfish_length.html
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

SIM = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SIM))
import analysis.sweep as sw  # noqa: E402

RES = SIM / "runs" / "sweep" / "results_length.json"
OUT = SIM / "runs" / "report" / "towfish_length.html"
ARMS = (668, 550, 450, 350, 300, 250)
KS = (1.0, 1.5, 2.0)


def load():
    r = json.loads(RES.read_text())
    rows = {}
    for e in r.values():
        if "score" in e:
            rows.setdefault(e["cfg"], {})[e["case"]] = e["score"]
    return rows


def verdict(d):
    """(state, worst hairpin roll, note). state: pass / marginal / fail / none."""
    if "HL8" not in d:
        return "none", None, ""
    fails = []
    for c in ("HL8", "HL25"):
        if not sw.hold_ok(d[c]):
            fails.append("straight hold")
    turns = [d[c] for c in ("TL8S8", "TL25S25", "TL25S35") if c in d]
    worst = max((t["max_roll"] for t in turns), default=None)
    if any(t["upsets"] or t["rescue"] for t in turns):
        fails.append("hairpin upset")
    for c in ("depth_L8", "depth_L25"):
        if c in d and (d[c]["upsets"] or (d[c]["max_roll"] or 0) >= 45):
            fails.append("depth change")
    if fails:
        return "fail", worst, ", ".join(sorted(set(fails)))
    speed = [c for c in ("HL8V0.8", "HL8V1.2", "HL25V0.8", "HL25V1.2") if c in d]
    slow = [c for c in speed if not sw.hold_ok(d[c])]
    if slow:
        p = max(d[c]["pitch_spread"] for c in slow)
        return "marginal", worst, f"pitch wanders {p:.0f}° at 0.8 m/s on 25 m"
    return "pass", worst, "boundary-tested" if speed else ""


def main() -> int:
    rows = load()
    cells = []
    for arm in ARMS:
        row = []
        for k in KS:
            tag = sw._cfg_tag("spine", {"arm": arm, "k": k})
            if tag not in rows:
                row.append(None)
                continue
            _, geo = sw._compact_sets("spine", arm=arm, k=k)
            st, worst, note = verdict(rows[tag])
            row.append({"state": st, "worst": worst, "note": note, "spine": geo["spine_mm"],
                        "area": geo["tail_area_cm2"]})
        _, g1 = sw._compact_sets("spine", arm=arm, k=1.0)
        cells.append({"arm": arm, "spine": g1["spine_mm"], "row": row})
    other = []
    for layout, kw, name in (("endcap", {"k": 1.0}, "V on the end cap, 1× (270 cm²)"),
                             ("endcap", {"k": 2.0}, "V on the end cap, 2× (540 cm²)"),
                             ("endcap", {"k": 3.0}, "V on the end cap, 3× (810 cm²)"),
                             ("hoop", {"hoop_y": -40.0}, "Hoop at the front of the tube, no tail"),
                             ("hoop", {"hoop_y": 30.0}, "Hoop over the collar, no tail"),
                             ("hoop", {"hoop_y": 138.0}, "Hoop over the CG, no tail")):
        d = rows.get(sw._cfg_tag(layout, kw), {})
        out = []
        for c, lab in (("HL8", "8 m tether"), ("HL25", "25 m tether")):
            s = d.get(c)
            if not s:
                continue
            if sw.hold_ok(s):
                out.append(f"{lab}: steady")
            elif s["max_roll"] > 90:
                out.append(f"{lab}: rolls over ({s['max_roll']:.0f}°)")
            elif s["rescues"]:
                out.append(f"{lab}: needed auto-righting (peak roll {s['max_roll']:.0f}°)")
            else:
                out.append(f"{lab}: roll {s['max_roll']:.0f}°, pitch wanders {s['pitch_spread']:.0f}°")
        other.append({"name": name, "result": "; ".join(out)})
    rates = {}
    for L in (8, 25):
        s = rows.get("sp_A668_K1", {}).get(f"depth_L{L}", {})
        rates[L] = (s.get("dive_rate"), s.get("climb_rate"), s.get("depth_max"))
    data = {"cells": cells, "other": other, "rates": rates}
    page = (Path(__file__).resolve().parent / "length_report_template.html").read_text()
    OUT.write_text(page.replace("/*DATA*/null", json.dumps(data)))
    print("wrote", OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
