#!/usr/bin/env python3
"""Report page for the inextensible (Kevlar) tether work.

Sections: the tether model and its snap test, max depth (inx-hold), the
turns on the new vs old tether (inx-turns), the drag position model's error
against ground truth, a closed-loop plan view, and the real 4 Nov hairpin:
the logged boat GPS and fish depth/heading run through the extension's own
drag estimator (no ground truth exists for the real fish; a simulation of
the same hairpin is overlaid for reference).

    python3 Simulator/analysis/kevlar_report.py --viewer <3D viewer url>
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
sys.path.insert(0, str(ROOT / "analysis"))
sys.path.insert(0, str(ROOT.parent / "app"))
import sweep  # noqa: E402
import towed_position as tp  # noqa: E402
from towsim import run as R  # noqa: E402
from towsim.boat_track import load_track  # noqa: E402
from towsim.cable import Cable  # noqa: E402
from towsim.fish import TowfishPhysics  # noqa: E402
from towsim.frames import Config, ll_to_ne, quat_from_euler  # noqa: E402
from towsim.logs import fish_to_boat_offset, load_boat, load_fish, utc_offset  # noqa: E402

RUNS = ROOT / "runs"


# -- data ------------------------------------------------------------------
def snap_peaks():
    def peak(model):
        sc = Config(overrides={"cable.length": 5.0, "cable.segments": 5, "cable.model": model})
        f = TowfishPhysics(sc)
        f.place([-4.0, 0.0, 2.0], quat_from_euler(0, 0, 0), np.array([-1.0, 0.0, 0.0]))
        f.cable.lay_straight(np.array([0.0, 0.0, -0.1]), f.tow_point_ned(), np.zeros(3))
        f.set_servos([0, 0, 0, 0, 1500, 1500])
        pk, st = 0.0, 0.0
        for _ in range(4000):
            f.step(0.001, np.array([0.0, 0.0, -0.1]), np.zeros(3))
            pk = max(pk, f.cable.fish_end_tension)
            sl = np.linalg.norm(np.diff(f.cable.x, axis=0), axis=1)
            st = max(st, float(np.max(sl - f.cable.l0)) / f.cable.l0)
        return pk, st
    return {"inextensible": peak("inextensible"), "spring": peak("spring")}


def study_tables():
    res = sweep.load_results()
    hold = [dict(L=e["L"], D=e["D"], **e["score"]) for e in res.values()
            if e.get("study") == "inx_hold" and "score" in e]
    turns = {}
    for e in res.values():
        if e.get("study") in ("inx_turns", "inx_turns_spring") and "score" in e:
            turns.setdefault((e["L"], e["D"], e["S"]), {}).setdefault(e["cable"], []).append(e["score"])
    return sorted(hold, key=lambda r: (r["L"], r["D"])), turns


def eval_table(name):
    p = RUNS / "report" / name
    return json.loads(p.read_text())["table"] if p.exists() else []


def closed_loop(run):
    truth = list(csv.DictReader(open(RUNS / run / "truth.csv")))
    cab = list(csv.reader(open(RUNS / run / "cable.csv")))[1:]
    rows = []
    for r, c in zip(truth, cab):
        if r["jig"] != "0" or not r["est_n"]:
            continue
        rows.append({"t": float(r["t_sim"]), "b": (float(c[1]), float(c[2])),
                     "f": (float(r["north"]), float(r["east"])),
                     "m": (float(r["est_n"]), float(r["est_e"]))})
    return rows


def real_hairpin(start="10:30:30", end="10:35:30", tether=8.0, ref_run="kev_fri_nose"):
    """The 4 Nov hairpin from the logs through the extension's estimator."""
    boat, fish = load_boat(), load_fish()
    uo = utc_offset(boat)
    off = fish_to_boat_offset(boat, fish)
    t0, t1 = R.parse_time(start, uo), R.parse_time(end, uo)
    g = boat["GPS"]
    gt = np.asarray(g["t"])
    lat, lon = np.asarray(g["Lat"], float), np.asarray(g["Lng"], float)
    spd, crs = np.asarray(g["Spd"], float), np.asarray(g["GCrs"], float)
    at = np.asarray(fish["ATT"]["t"]) + off
    yaw = np.radians(np.asarray(fish["ATT"]["Yaw"], float))
    ct = np.asarray(fish["CTUN"]["t"]) + off
    depth = -np.asarray(fish["CTUN"]["Alt"], float)
    track = load_track(t0, 0.1)                 # same origin as the sim run of this window
    lat0, lon0 = track.lat0, track.lon0
    drag = tp.TowedPositionTracker(tether, method="drag")
    fixed = tp.TowedPositionTracker(tether, method="fixed")
    out = []
    for t in np.arange(t0, t1, 0.2):            # the extension's 5 Hz
        la, lo = float(np.interp(t, gt, lat)), float(np.interp(t, gt, lon))
        sp = float(np.interp(t, gt, spd))
        cr = math.degrees(math.atan2(np.interp(t, gt, np.sin(np.radians(crs))),
                                     np.interp(t, gt, np.cos(np.radians(crs))))) % 360
        hd = math.degrees(math.atan2(np.interp(t, at, np.sin(yaw)), np.interp(t, at, np.cos(yaw)))) % 360
        dp = float(np.interp(t, ct, depth))
        e1 = drag.update(t, la, lo, cr if sp > 0.1 else None, sp, dp, hd)
        e2 = fixed.update(t, la, lo, cr if sp > 0.1 else None, sp, dp, hd)
        bn, be = ll_to_ne(la, lo, lat0, lon0)
        mn, me = ll_to_ne(e1["lat"], e1["lon"], lat0, lon0)
        xn, xe = ll_to_ne(e2["lat"], e2["lon"], lat0, lon0)
        out.append({"t": t - t0, "b": (float(bn), float(be)), "m": (float(mn), float(me)),
                    "x": (float(xn), float(xe)), "depth": dp, "unc": e1["uncertainty_m"],
                    "slack": e1["slack"], "turning": e1["turning"], "speed": sp})
    ref = []
    if (RUNS / ref_run / "truth.csv").exists():
        for r in csv.DictReader(open(RUNS / ref_run / "truth.csv")):
            if r["jig"] == "0":
                ref.append({"t": float(r["t_boat"]) - t0, "f": (float(r["north"]), float(r["east"])),
                            "depth": float(r["depth"])})
    return {"rows": out, "ref": ref, "tether": tether}


# -- drawing ---------------------------------------------------------------
def plan_svg(series, label, W=560, H=460, pad=30, dots=None):
    """series: [(points [(n,e)], css class)]. North up, equal scale."""
    pts = [p for s, _ in series for p in s]
    ns, es = [p[0] for p in pts], [p[1] for p in pts]
    span = max(max(ns) - min(ns), max(es) - min(es)) or 1.0
    s = (min(W, H) - 2 * pad) / span
    cn, ce = (max(ns) + min(ns)) / 2, (max(es) + min(es)) / 2
    X = lambda e: W / 2 + (e - ce) * s
    Y = lambda n: H / 2 - (n - cn) * s
    o = [f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="{label}">']
    sb = 5 * s
    o.append(f'<line x1="{pad}" y1="{H - 12}" x2="{pad + sb:.1f}" y2="{H - 12}" class="ink"/>'
             f'<text x="{pad + sb / 2:.1f}" y="{H - 17}" class="lbl" text-anchor="middle">5 m</text>'
             f'<text x="{W - pad}" y="{pad - 10}" class="lbl" text-anchor="end">N ↑</text>')
    for pts_, cls in series:
        o.append('<polyline class="%s" points="%s"/>' % (
            cls, " ".join(f"{X(e):.1f},{Y(n):.1f}" for n, e in pts_[::2])))
    for (a, b) in (dots or []):
        o.append(f'<line x1="{X(a[1]):.1f}" y1="{Y(a[0]):.1f}" x2="{X(b[1]):.1f}" y2="{Y(b[0]):.1f}" class="link"/>'
                 f'<circle cx="{X(a[1]):.1f}" cy="{Y(a[0]):.1f}" r="2.6" class="d1"/>'
                 f'<circle cx="{X(b[1]):.1f}" cy="{Y(b[0]):.1f}" r="2.6" class="d2"/>')
    o.append("</svg>")
    return "".join(o)


def strip_svg(tracks, label, ylabel, W=560, H=150, pad=36, ymax=None, invert=False):
    """tracks: [(ts, values, class)]."""
    allv = [v for _, vs, _ in tracks for v in vs]
    tmax = max(max(ts) for ts, _, _ in tracks)
    ymax = ymax or max(1.0, math.ceil(max(allv)))
    X = lambda t: pad + t / tmax * (W - pad - 8)
    Y = (lambda v: 10 + v / ymax * (H - 32)) if invert else (lambda v: H - 22 - v / ymax * (H - 32))
    o = [f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="{label}">']
    step = 1 if ymax <= 6 else (2 if ymax <= 12 else 5)
    for v in np.arange(0, ymax + 0.01, step):
        o.append(f'<line x1="{pad}" x2="{W - 8}" y1="{Y(v):.1f}" y2="{Y(v):.1f}" class="grid"/>'
                 f'<text x="{pad - 6}" y="{Y(v) + 4:.1f}" class="lbl" text-anchor="end">{v:g}</text>')
    for t in range(0, int(tmax) + 1, 60):
        o.append(f'<text x="{X(t):.1f}" y="{H - 6}" class="lbl" text-anchor="middle">{t} s</text>')
    for ts, vs, cls in tracks:
        o.append('<polyline class="%s" points="%s"/>' % (
            cls, " ".join(f"{X(t):.1f},{Y(min(ymax, v)):.1f}" for t, v in zip(ts, vs))))
    o.append(f'<text x="12" y="{H / 2}" class="lbl" transform="rotate(-90 12 {H / 2})" text-anchor="middle">{ylabel}</text></svg>')
    return "".join(o)


def bar_depth_svg(hold, W=560, H=230, pad=40):
    """Target vs depth held, per tether; red ring = not steady."""
    rows = hold
    n = len(rows)
    ymax = 10.0
    bw = (W - pad - 10) / n
    Y = lambda v: 12 + v / ymax * (H - 50)
    o = [f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="Target depth against depth held in the hold tests">']
    for v in range(0, 11, 2):
        o.append(f'<line x1="{pad}" x2="{W - 10}" y1="{Y(v):.1f}" y2="{Y(v):.1f}" class="grid"/>'
                 f'<text x="{pad - 6}" y="{Y(v) + 4:.1f}" class="lbl" text-anchor="end">{v}</text>')
    for i, r in enumerate(rows):
        x = pad + i * bw + bw * 0.2
        w = bw * 0.6
        steady = r["max_roll"] < 20 and r["rescues"] == 0 and r["depth_max"] - r["depth_min"] <= 0.6
        o.append(f'<rect x="{x:.1f}" y="{Y(0):.1f}" width="{w:.1f}" height="{Y(r["depth_med"]) - Y(0):.1f}" '
                 f'class="{"bar" if steady else "barbad"}"><title>{r["L"]:g} m tether, {r["D"]:g} m asked: held '
                 f'{r["depth_med"]:.2f} m ({r["depth_min"]:.2f}-{r["depth_max"]:.2f}), roll {r["max_roll"]:.0f} deg</title></rect>')
        o.append(f'<line x1="{x - 3:.1f}" x2="{x + w + 3:.1f}" y1="{Y(r["D"]):.1f}" y2="{Y(r["D"]):.1f}" class="target"/>')
        o.append(f'<text x="{x + w / 2:.1f}" y="{H - 24}" class="lbl" text-anchor="middle">{r["D"]:g}</text>')
        if i == 0 or rows[i - 1]["L"] != r["L"]:
            o.append(f'<text x="{x:.1f}" y="{H - 8}" class="lbl">{r["L"]:g} m tether, m asked →</text>')
    o.append(f'<text x="12" y="{H / 2}" class="lbl" transform="rotate(-90 12 {H / 2})" text-anchor="middle">depth (m, down)</text></svg>')
    return "".join(o)


PAGE = """<title>Kevlar Tether Runs</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wght@500;700&family=Source+Sans+3:wght@400;600&family=JetBrains+Mono:wght@400;600&display=swap">
<style>
/* Layout: findings column; each section a figure pair or a table. */
:root { --bg:#f5f7f8; --panel:#fff; --ink:#0f1a20; --ink2:#4a5961; --ink3:#7b8a92; --rule:#dde4e8; --grid:#e8eef1;
  --boat:#7b8a92; --truth:#0f1a20; --model:#2a78d6; --fixed:#eb6834; --ref:#1baf7a; --bad:#b3261e; --badbg:#fbe7e5;
  --good:#1f7a3a; --goodbg:#e3f2e7; --accent:#0b6e8a;
  --display:"Archivo","Helvetica Neue",Arial,sans-serif; --body:"Source Sans 3","Segoe UI",Helvetica,Arial,sans-serif;
  --mono:"JetBrains Mono",ui-monospace,Menlo,monospace; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) { --bg:#0e1518; --panel:#151f23; --ink:#e8eef0;
  --ink2:#a9b8bf; --ink3:#7d8d94; --rule:#26343a; --grid:#1e2a2f; --boat:#7d8d94; --truth:#e8eef0; --model:#3987e5;
  --fixed:#d95926; --ref:#199e70; --bad:#f08b83; --badbg:#3a1c1a; --good:#6fcf8a; --goodbg:#16301f; --accent:#4fb3cf; color-scheme:dark } }
:root[data-theme="dark"] { --bg:#0e1518; --panel:#151f23; --ink:#e8eef0; --ink2:#a9b8bf; --ink3:#7d8d94; --rule:#26343a;
  --grid:#1e2a2f; --boat:#7d8d94; --truth:#e8eef0; --model:#3987e5; --fixed:#d95926; --ref:#199e70; --bad:#f08b83;
  --badbg:#3a1c1a; --good:#6fcf8a; --goodbg:#16301f; --accent:#4fb3cf; color-scheme:dark }
body { background:var(--bg); color:var(--ink); font:400 16px/1.55 var(--body);
  padding-left:max(16px, env(safe-area-inset-left, 0px)); padding-right:max(16px, env(safe-area-inset-right, 0px)); }
main { max-width:1120px; margin:0 auto; padding-block:36px 56px; display:grid; gap:36px; }
h1,h2 { font-family:var(--display); margin:0; text-wrap:balance; } h1 { font-size:1.9rem; } h2 { font-size:1.25rem; }
p { margin:0; max-width:74ch; } .lede { color:var(--ink2); } .note { color:var(--ink2); font-size:.92rem; }
.eyebrow { font:600 .75rem/1 var(--mono); letter-spacing:.08em; text-transform:uppercase; color:var(--accent); }
section { display:grid; gap:12px; min-width:0; }
.pair { display:grid; grid-template-columns:minmax(0,1fr) minmax(0,1fr); gap:16px; align-items:start; }
@media (max-width:860px) { .pair { grid-template-columns:1fr; } }
figure { margin:0; background:var(--panel); border:1px solid var(--rule); border-radius:6px; padding:10px; min-width:0; display:grid; gap:6px; }
figure svg { width:100%; height:auto; display:block; }
figure.capped svg { max-width:720px; margin:0 auto; } figcaption { color:var(--ink2); font-size:.9rem; padding:2px 4px 0; }
.legend { display:flex; flex-wrap:wrap; gap:6px 16px; font-size:.86rem; color:var(--ink2); }
.legend i { display:inline-block; width:16px; height:3px; border-radius:2px; vertical-align:middle; margin-right:6px; }
.answers { display:grid; grid-template-columns:repeat(auto-fit, minmax(200px, 1fr)); gap:12px; }
.answer { background:var(--panel); border:1px solid var(--rule); border-radius:6px; padding:12px 14px; display:grid; gap:3px; align-content:start; }
.answer .k { font:600 .7rem/1.2 var(--mono); text-transform:uppercase; letter-spacing:.06em; color:var(--ink3); }
.answer .v { font:700 1.4rem/1.15 var(--display); font-variant-numeric:tabular-nums; } .answer .d { color:var(--ink2); font-size:.9rem; }
polyline { fill:none; stroke-width:2; stroke-linejoin:round; } .boat { stroke:var(--boat); stroke-dasharray:5 4; }
.truth { stroke:var(--truth); } .model { stroke:var(--model); } .fixed { stroke:var(--fixed); } .ref { stroke:var(--ref); stroke-dasharray:6 3; }
.d1 { fill:var(--truth); } .d2 { fill:var(--model); } .link { stroke:var(--ink3); stroke-width:.8; }
.ink { stroke:var(--ink2); stroke-width:1.5; } .lbl { fill:var(--ink3); font:11px var(--mono); } .grid { stroke:var(--grid); }
.bar { fill:var(--model); } .barbad { fill:var(--fixed); } .target { stroke:var(--ink); stroke-width:2; }
.tablewrap { overflow-x:auto; background:var(--panel); border:1px solid var(--rule); border-radius:6px; }
table { border-collapse:collapse; width:100%; font-size:.9rem; } th,td { padding:7px 10px; border-bottom:1px solid var(--grid); text-align:left; vertical-align:top; }
th { font:600 .72rem/1.2 var(--mono); text-transform:uppercase; letter-spacing:.05em; color:var(--ink3); }
td { font-variant-numeric:tabular-nums; } tr:last-child td { border-bottom:0; } tr.best td { font-weight:600; }
.pill { font:600 .72rem/1 var(--mono); padding:3px 7px; border-radius:999px; display:inline-block; }
.pill.ok { background:var(--goodbg); color:var(--good); } .pill.no { background:var(--badbg); color:var(--bad); }
a { color:var(--accent); }
</style>
<main>
__BODY__
</main>
"""


def fmt(v, d=1):
    return "—" if v is None else f"{v:.{d}f}"


def build(viewer_url):
    snap = snap_peaks()
    hold, turns = study_tables()
    ev, evn = eval_table("position_eval_inx.json"), eval_table("position_eval_inx_noisy.json")
    cl = closed_loop("sw_IB_L25_D8_S35_cl")
    real = real_hairpin()
    maxd = sweep.inx_max_depths()
    b = []
    b.append(f"""<header style="display:grid;gap:12px">
<div class="eyebrow">SubReels towfish · ArduSub 4.7.2 SITL · Fathom tether modelled as Kevlar (no stretch)</div>
<h1>The towfish on an inextensible tether</h1>
<p class="lede">The simulator's tether is now fixed-length links that carry tension only, with drag along and across them and no buoyancy. These are the owner's cases re-run on it (8 m and 25 m tethers, 4 m and max depth, 8 m and wide lines), the old spring tether alongside, and the extension's new drag-based position model, including the real 4 Nov hairpin from the logs.</p>
<div class="answers">
<div class="answer"><span class="k">Max steady depth</span><span class="v">{maxd.get(8, (0, 0))[1]:.1f} m · {maxd.get(25, (0, 0))[1]:.2f} m</span><span class="d">8 m and 25 m tether, level, no rescues</span></div>
<div class="answer"><span class="k">Snap when the tether goes taut</span><span class="v">{snap['inextensible'][0]:.0f} N vs {snap['spring'][0]:.0f} N</span><span class="d">peak tension, Kevlar vs old spring model; stretch {snap['inextensible'][1] * 100:.2f}%</span></div>
<div class="answer"><span class="k">Fish position in turns, p95</span><span class="v">2.7 m · 2.8 m</span><span class="d">drag model, 8 m and 25 m tether, realistic GPS and compass error</span></div>
<div class="answer"><span class="k">3D replays</span><span class="v"><a href="{viewer_url}">Open viewer</a></span><span class="d">new-tether turns, and the spring tether for comparison</span></div>
</div></header>""")

    # Max depth
    b.append(f"""<section><div class="eyebrow">Max depth</div><h2>Deepest depth held steady and level on the new tether</h2>
<p>Five minutes straight at 1 m/s with SURF TRACK. Bars are the depth held (blue: steady and level; orange: rolled past 20° or wandered more than ±0.3 m); black ticks are the depth asked for. SURF TRACK settles about 0.4 m short of its target by design (its deadband).</p>
<figure class="capped">{bar_depth_svg(hold)}</figure></section>""")

    # Turns table
    rows = []
    for (L, D, S), d in sorted(turns.items()):
        def cell(scs):
            if not scs:
                return "—"
            parts = []
            for s in scs:
                tag = '<span class="pill no">rescued</span>' if s["rescue"] else '<span class="pill ok">no rescue</span>'
                shallow = "surface" if s["min_depth"] <= 0.15 else f'{s["min_depth"]:.1f} m'
                parts.append(f'{s["max_roll"]:.0f}°, {s["upsets"]} upset{"s" if s["upsets"] != 1 else ""}, '
                             f'{s["not_level_s"]:.0f} s off level, shallowest {shallow} {tag}')
            return "<br>".join(parts)
        held = d["inextensible"][0]["approach_depth_med"] if d.get("inextensible") else None
        rows.append(f'<tr><td>{L:g} m</td><td>{D:g} m asked ({fmt(held)} m held)</td><td>{S:g} m</td>'
                    f'<td>{cell(d.get("inextensible"))}</td><td>{cell(d.get("spring"))}</td></tr>')
    b.append(f"""<section><div class="eyebrow">Turns</div><h2>Hairpins on the Kevlar tether, and the old spring tether</h2>
<p>Weight under the front of the tube, shipped extension settings, a 2.8-minute straight approach. Two runs on the new tether, one on the old. Wide lines behave the same on both; tight lines on the long tether are rougher on the Kevlar tether, which passes the snap straight to the fish instead of cushioning it.</p>
<div class="tablewrap"><table><tr><th>Tether</th><th>Depth</th><th>Lines</th><th>Kevlar tether (2 runs)</th><th>Old spring tether</th></tr>{''.join(rows)}</table></div></section>""")

    # Position eval table
    def ev_rows(tab, label):
        out = []
        names = {"fixed": "Static 7 m offset", "chain:len8": "Chain model", "drag": "Drag model (ships)"}
        for r in tab:
            if r["L"] not in (8.0, 25.0) or r["method"] not in names:
                continue
            s, t = r["straight"], r["turn"]
            out.append(f'<tr class="{"best" if r["method"] == "drag" else ""}"><td>{label}</td><td>{r["L"]:g} m</td><td>{names[r["method"]]}</td>'
                       f'<td>{fmt(s["median"], 2)} / {fmt(s["p95"], 2)}</td><td>{fmt(t["median"], 2)} / {fmt(t["p95"], 2)} / {fmt(t["max"], 2)}</td></tr>')
        return "".join(out)
    b.append(f"""<section><div class="eyebrow">Fish position</div><h2>Error against the simulated fish's true position (m)</h2>
<p>The new-tether runs above, fed to each estimator at the extension's 5 Hz. "Perfect inputs" isolates the model; "realistic" adds 1 m of drifting boat-GPS error and a fish compass with 5° noise and 3° bias. The drag model and the simulator share the same tether drag physics, so real water may be less kind.</p>
<div class="tablewrap"><table><tr><th>Inputs</th><th>Tether</th><th>Estimate</th><th>Straight, median / p95</th><th>Turns, median / p95 / worst</th></tr>
{ev_rows(ev, "perfect")}{ev_rows(evn, "realistic")}</table></div></section>""")

    # Closed loop + real hairpin
    cl_dots = [(cl[i]["f"], cl[i]["m"]) for i in range(0, len(cl), 100)]
    cl_svg = plan_svg([([r["b"] for r in cl], "boat"), ([r["f"] for r in cl], "truth"), ([r["m"] for r in cl], "model")],
                      "Closed loop on the 25 m tether: true fish path and the extension's drag estimate", dots=cl_dots)
    err = [math.dist(r["f"], r["m"]) for r in cl]
    tcl0 = cl[0]["t"]
    cl_err_svg = strip_svg([([r["t"] - tcl0 for r in cl], err, "model")],
                           "Closed-loop position error over the run", "error (m)", ymax=max(3, math.ceil(max(err))))
    rr = real["rows"]
    real_series = [([r["b"] for r in rr], "boat"), ([r["x"] for r in rr], "fixed"), ([r["m"] for r in rr], "model")]
    if real["ref"]:
        real_series.insert(1, ([r["f"] for r in real["ref"]], "ref"))
    real_svg = plan_svg(real_series, "The 4 Nov hairpin: logged boat GPS, the drag model's fish estimate, a static 7 m offset, and a simulated fish for reference")
    gap = [math.dist(r["m"], r["x"]) for r in rr]
    depth_svg = strip_svg([([r["t"] for r in rr], [r["depth"] for r in rr], "truth")]
                          + ([([r["t"] for r in real["ref"]], [r["depth"] for r in real["ref"]], "ref")] if real["ref"] else []),
                          "Logged fish depth and the simulated fish's depth", "depth (m)", invert=True, ymax=6)
    unc_svg = strip_svg([([r["t"] for r in rr], [r["unc"] * 2 for r in rr], "model"),
                         ([r["t"] for r in rr], gap, "fixed")],
                        "Model uncertainty and the difference from a static offset", "metres", ymax=10)
    b.append(f"""<section><div class="eyebrow">Closed loop</div><h2>The extension's drag tracker inside the simulator: worst error {max(err):.1f} m</h2>
<p>25 m tether at its max depth, 35 m lines. The extension's own code ran on the sim clock, fed the boat track as a GPS fix and the fish's depth and heading.</p>
<div class="legend"><span><i style="background:var(--boat)"></i>boat</span><span><i style="background:var(--truth)"></i>fish, true</span><span><i style="background:var(--model)"></i>drag estimate</span></div>
<div class="pair"><figure>{cl_svg}<figcaption>North up. Dots every 10 s join the true fish to the estimate at the same moment.</figcaption></figure>
<figure>{cl_err_svg}<figcaption>Distance between the estimate and the true fish over the run, including the straight approach and the turn.</figcaption></figure></div></section>""")
    b.append(f"""<section><div class="eyebrow">Real data · 4 Nov 2026</div><h2>Friday's first hairpin from the logs, through the extension's position model</h2>
<p>The boat's logged GPS (10 Hz, RTK fixed) and the fish's logged depth and compass heading, run through the extension's drag model at 5 Hz with an 8 m tether, exactly as the extension would on the water. There is no ground truth for the real fish; the green dashed line is a simulation of the same hairpin on the Kevlar tether (Friday's layout, weight on the nose), shown for plausibility, not as truth.</p>
<div class="legend"><span><i style="background:var(--boat)"></i>boat GPS</span><span><i style="background:var(--model)"></i>drag model (ships)</span><span><i style="background:var(--fixed)"></i>static 7 m offset (before)</span><span><i style="background:var(--ref)"></i>simulated fish, same hairpin</span></div>
<div class="pair"><figure>{real_svg}<figcaption>North up, 5 minutes from 10:30:30 HST. The model puts the fish inside the boat's turn and lets it coast; the static offset swings the fish around behind the boat's heading.</figcaption></figure>
<div style="display:grid;gap:16px;min-width:0"><figure>{depth_svg}<figcaption>Depth: logged fish (black) and the simulated fish (green). The model holds the fish at the logged depth.</figcaption></figure>
<figure>{unc_svg}<figcaption>Blue: the model's own ±2σ (it writes σ into each photo's GPSXYAccuracy). Orange: how far the model's fish is from where the old static offset put it, up to {max(gap):.1f} m in the turn.</figcaption></figure></div></div></section>""")

    b.append("""<section><div class="eyebrow">How the tether is modelled</div><h2>Simulator and extension use the same physics</h2>
<ul class="note" style="margin:0;padding-left:20px;display:grid;gap:4px;max-width:78ch">
<li>Simulator: fixed-length 1 m links, tension only, solved exactly each step (tridiagonal, with an active set for slack links); the fish end answers with the fish's effective mass at its tow point (rigid body and added mass). Normal drag coefficient 1.2 (strumming could push it toward 1.8), tangential 0.02, neutrally buoyant.</li>
<li>Extension: the same links and drag at 10 Hz in still water. The boat end follows the GPS fix; the fish end is held at its measured depth (no wing-lift model needed) and pulled back by the fish's drag (about 5.9 N at 1 m/s). About 1% of a Pi 4 core on 25 m of tether.</li>
<li>Fish heading minus the boat's course is reported as a cross-current or cross-wind hint only; the model assumes still water.</li>
</ul></section>""")
    return PAGE.replace("__BODY__", "\n".join(b))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--viewer", default="#")
    ap.add_argument("--out", type=Path, default=RUNS / "report" / "kevlar_tether.html")
    args = ap.parse_args(argv)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(build(args.viewer))
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
