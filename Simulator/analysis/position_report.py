#!/usr/bin/env python3
"""Plan-view comparison of the towfish position estimate through hairpins.

Reads runs flown with --extension (which log the extension's live estimate
as est_n/est_e) and the position_eval JSON tables, and writes one HTML page:
boat track, the fish's true path, the static 7 m offset and the tether model.

    python3 Simulator/analysis/position_report.py --run pos_fri_L8 --run pos_L25_D8_S35
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RUNS = ROOT / "runs"


def load(run, fixed_offset=7.0, before_s=25.0, after_s=95.0):
    meta = json.loads((RUNS / run / "meta.json").read_text())
    L = next(float(c[1]) for c in meta["config"] if c[0] == "cable.length")
    truth = list(csv.DictReader(open(RUNS / run / "truth.csv")))
    cable = list(csv.reader(open(RUNS / run / "cable.csv")))[1:]
    rows = []
    for r, c in zip(truth, cable):
        if r["jig"] != "0" or not r.get("est_n"):
            continue
        bn, be = float(c[1]), float(c[2])
        yaw = math.radians(float(r["yaw"]))
        rows.append({"t": float(r["t_sim"]), "b": (bn, be),
                     "f": (float(r["north"]), float(r["east"])),
                     "m": (float(r["est_n"]), float(r["est_e"])),
                     "x": (bn - fixed_offset * math.cos(yaw), be - fixed_offset * math.sin(yaw)),
                     "unc": float(r["est_unc"])})
    # Turn = first big change in the boat's course.
    def course(i, j):
        (n1, e1), (n2, e2) = rows[i]["b"], rows[j]["b"]
        return math.degrees(math.atan2(e2 - e1, n2 - n1))
    t_turn = rows[0]["t"]
    c0 = course(0, 10)
    for i in range(10, len(rows) - 10):
        if abs((course(i, i + 10) - c0 + 180) % 360 - 180) > 20:
            t_turn = rows[i]["t"]
            break
    rows = [r for r in rows if t_turn - before_s <= r["t"] <= t_turn + after_s]
    for r in rows:
        r["em"] = math.dist(r["m"], r["f"])
        r["ex"] = math.dist(r["x"], r["f"])
    return {"run": run, "L": L, "t_turn": t_turn, "rows": rows}


def svg_plan(case, W=520, H=420, pad=28):
    rows = case["rows"][::2]
    pts = [p for r in rows for p in (r["b"], r["f"], r["m"], r["x"])]
    ns, es = [p[0] for p in pts], [p[1] for p in pts]
    span = max(max(ns) - min(ns), max(es) - min(es)) or 1.0
    s = min((W - 2 * pad), (H - 2 * pad)) / span
    cn, ce = (max(ns) + min(ns)) / 2, (max(es) + min(es)) / 2
    X = lambda e: W / 2 + (e - ce) * s
    Y = lambda n: H / 2 - (n - cn) * s
    path = lambda key: " ".join(f"{X(r[key][1]):.1f},{Y(r[key][0]):.1f}" for r in rows)
    o = [f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="Plan view of the {case["L"]:g} m tether hairpin: '
         f'boat track, true fish path, static offset and tether model estimates.">']
    # 5 m scale bar
    sb = 5 * s
    o.append(f'<line x1="{pad}" y1="{H - 12}" x2="{pad + sb:.1f}" y2="{H - 12}" class="ink"/>'
             f'<text x="{pad + sb / 2:.1f}" y="{H - 16}" class="lbl" text-anchor="middle">5 m</text>')
    o.append(f'<text x="{W - pad}" y="{pad - 8}" class="lbl" text-anchor="end">N ↑</text>')
    for key, cls in (("b", "boat"), ("x", "fixed"), ("f", "truth"), ("m", "model")):
        o.append(f'<polyline points="{path(key)}" class="{cls}"/>')
    # time ticks every 10 s along the truth and model
    t0 = rows[0]["t"]
    last = -99
    for r in rows:
        if r["t"] - t0 - last >= 10:
            last = r["t"] - t0
            for key, cls in (("f", "truthd"), ("m", "modeld")):
                o.append(f'<circle cx="{X(r[key][1]):.1f}" cy="{Y(r[key][0]):.1f}" r="2.6" class="{cls}"/>')
            o.append(f'<line x1="{X(r["f"][1]):.1f}" y1="{Y(r["f"][0]):.1f}" x2="{X(r["m"][1]):.1f}" '
                     f'y2="{Y(r["m"][0]):.1f}" class="link"/>')
    o.append("</svg>")
    return "".join(o)


def svg_error(case, W=520, H=170, pad=34):
    rows = case["rows"]
    t0 = case["t_turn"]
    tmin, tmax = rows[0]["t"] - t0, rows[-1]["t"] - t0
    ymax = max(5.0, math.ceil(max(max(r["ex"], r["em"]) for r in rows) / 5) * 5)
    X = lambda t: pad + (t - tmin) / (tmax - tmin) * (W - pad - 10)
    Y = lambda e: H - 22 - e / ymax * (H - 34)
    o = [f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="Position error over time through the turn.">']
    for v in range(0, int(ymax) + 1, 5 if ymax <= 20 else 10):
        o.append(f'<line x1="{pad}" x2="{W - 10}" y1="{Y(v):.1f}" y2="{Y(v):.1f}" class="grid"/>'
                 f'<text x="{pad - 6}" y="{Y(v) + 4:.1f}" class="lbl" text-anchor="end">{v}</text>')
    o.append(f'<line x1="{X(0):.1f}" x2="{X(0):.1f}" y1="10" y2="{H - 22}" class="turnline"/>'
             f'<text x="{X(0) + 4:.1f}" y="20" class="lbl">boat turns</text>')
    for t in range(int(math.ceil(tmin / 20) * 20), int(tmax) + 1, 20):
        o.append(f'<text x="{X(t):.1f}" y="{H - 6}" class="lbl" text-anchor="middle">{t:+d} s</text>')
    band = " ".join(f"{X(r['t'] - t0):.1f},{Y(min(ymax, 2 * r['unc'])):.1f}" for r in rows)
    o.append(f'<polygon points="{X(rows[0]["t"] - t0):.1f},{Y(0):.1f} {band} {X(rows[-1]["t"] - t0):.1f},{Y(0):.1f}" class="bandf"/>')
    for key, cls in (("ex", "fixed"), ("em", "model")):
        pts = " ".join(f"{X(r['t'] - t0):.1f},{Y(min(ymax, r[key])):.1f}" for r in rows)
        o.append(f'<polyline points="{pts}" class="{cls}"/>')
    o.append(f'<text x="12" y="{H / 2}" class="lbl" transform="rotate(-90 12 {H / 2})" text-anchor="middle">error (m)</text>')
    o.append("</svg>")
    return "".join(o)


def table_html(path):
    if not path.exists():
        return ""
    t = json.loads(path.read_text())["table"]
    keep = {"fixed": "Static 7 m offset (today)", "chain:len8": "Tether model"}
    f = lambda v: "—" if v is None else f"{v:.1f}"
    rows = []
    for r in t:
        if r["L"] not in (8.0, 16.0, 25.0) or r["method"] not in keep:
            continue
        rows.append(f'<tr class="{"best" if r["method"] != "fixed" else ""}"><td>{r["L"]:g} m</td><td>{keep[r["method"]]}</td>'
                    f'<td>{f(r["straight"]["median"])} / {f(r["straight"]["p95"])}</td>'
                    f'<td>{f(r["turn"]["median"])} / {f(r["turn"]["p95"])} / {f(r["turn"]["max"])}</td></tr>')
    return ('<div class="tablewrap"><table><tr><th>Tether</th><th>Estimate</th><th>Straight legs, median / p95</th>'
            '<th>Turns, median / p95 / worst</th></tr>' + "".join(rows) + "</table></div>")


PAGE = """<title>Towfish Position Model</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wght@500;700&family=Source+Sans+3:wght@400;600&family=JetBrains+Mono:wght@400;600&display=swap">
<style>
/* Layout: findings column; each hairpin as plan view + error strip side by side. */
:root { --bg:#f6f8f9; --panel:#fff; --ink:#0f1a20; --ink2:#4a5961; --ink3:#7b8a92; --rule:#dde4e8; --grid:#e8eef1;
  --boat:#7b8a92; --truth:#0f1a20; --fixed:#eb6834; --model:#2a78d6; --band:rgba(42,120,214,.12); --accent:#0b6e8a;
  --display:"Archivo","Helvetica Neue",Arial,sans-serif; --body:"Source Sans 3","Segoe UI",Helvetica,Arial,sans-serif;
  --mono:"JetBrains Mono",ui-monospace,Menlo,monospace; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) { --bg:#0e1518; --panel:#151f23; --ink:#e8eef0;
  --ink2:#a9b8bf; --ink3:#7d8d94; --rule:#26343a; --grid:#1e2a2f; --boat:#7d8d94; --truth:#e8eef0; --fixed:#d95926;
  --model:#3987e5; --band:rgba(57,135,229,.18); --accent:#4fb3cf; color-scheme:dark } }
:root[data-theme="dark"] { --bg:#0e1518; --panel:#151f23; --ink:#e8eef0; --ink2:#a9b8bf; --ink3:#7d8d94; --rule:#26343a;
  --grid:#1e2a2f; --boat:#7d8d94; --truth:#e8eef0; --fixed:#d95926; --model:#3987e5; --band:rgba(57,135,229,.18);
  --accent:#4fb3cf; color-scheme:dark }
body { background:var(--bg); color:var(--ink); font:400 16px/1.55 var(--body);
  padding-left:max(16px, env(safe-area-inset-left, 0px)); padding-right:max(16px, env(safe-area-inset-right, 0px)); }
main { max-width:1100px; margin:0 auto; padding-block:36px 56px; display:grid; gap:32px; }
h1,h2 { font-family:var(--display); margin:0; text-wrap:balance; } h1 { font-size:1.9rem; } h2 { font-size:1.25rem; }
p { margin:0; max-width:72ch; } .lede { color:var(--ink2); }
.eyebrow { font:600 .75rem/1 var(--mono); letter-spacing:.08em; text-transform:uppercase; color:var(--accent); }
.case { display:grid; grid-template-columns:minmax(0,1fr) minmax(0,1fr); gap:16px; align-items:start; }
@media (max-width:820px) { .case { grid-template-columns:1fr; } }
figure { margin:0; background:var(--panel); border:1px solid var(--rule); border-radius:6px; padding:10px; min-width:0; }
figure svg { width:100%; height:auto; display:block; } figcaption { color:var(--ink2); font-size:.9rem; padding:6px 4px 0; }
.legend { display:flex; flex-wrap:wrap; gap:6px 16px; font-size:.88rem; color:var(--ink2); }
.legend i { display:inline-block; width:16px; height:3px; border-radius:2px; vertical-align:middle; margin-right:6px; }
polyline { fill:none; stroke-width:2; stroke-linejoin:round; } .boat { stroke:var(--boat); stroke-dasharray:5 4; }
.truth { stroke:var(--truth); } .fixed { stroke:var(--fixed); } .model { stroke:var(--model); }
.truthd { fill:var(--truth); } .modeld { fill:var(--model); } .link { stroke:var(--ink3); stroke-width:.8; }
.ink { stroke:var(--ink2); stroke-width:1.5; } .lbl { fill:var(--ink3); font:11px var(--mono); }
.grid { stroke:var(--grid); } .turnline { stroke:var(--ink3); stroke-dasharray:3 3; } .bandf { fill:var(--band); stroke:none; }
.tablewrap { overflow-x:auto; background:var(--panel); border:1px solid var(--rule); border-radius:6px; }
table { border-collapse:collapse; width:100%; font-size:.92rem; } th,td { padding:7px 10px; border-bottom:1px solid var(--grid); text-align:left; }
th { font:600 .72rem/1.2 var(--mono); text-transform:uppercase; letter-spacing:.05em; color:var(--ink3); }
td { font-variant-numeric:tabular-nums; } tr.best td { font-weight:600; } tr:last-child td { border-bottom:0; }
</style>
<main>
  <header style="display:grid;gap:10px">
    <div class="eyebrow">SubReels towfish · position estimate · ArduSub SITL with tether physics</div>
    <h1>Where the towfish really is in a hairpin</h1>
    <p class="lede">The extension now places the fish from the tether length, its depth and heading, and follows it through turns with a chain model of the tether. Blue is that estimate, live from the extension's own code inside the simulator; orange is the old static 7 m offset; black is where the fish actually was.</p>
    <div class="legend"><span><i style="background:var(--boat)"></i>boat track</span><span><i style="background:var(--truth)"></i>fish, true</span>
      <span><i style="background:var(--model)"></i>tether model</span><span><i style="background:var(--fixed)"></i>static 7 m offset</span>
      <span><i style="background:var(--band);height:8px"></i>model's own ±2σ</span></div>
  </header>
  __CASES__
  <section style="display:grid;gap:12px">
    <div class="eyebrow">All simulated runs</div>
    <h2>Error in metres, with realistic boat-GPS and compass error</h2>
    <p>403 simulated runs on 8&ndash;60 m tethers, 7.6&ndash;50 m hairpins. Boat GPS drifts with 1 m error; the fish compass has 5° noise and a 3° bias.</p>
    __TABLE__
  </section>
</main>
"""


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", action="append", required=True)
    ap.add_argument("--table", type=Path, default=RUNS / "report" / "position_eval_noisy.json")
    ap.add_argument("--out", type=Path, default=RUNS / "report" / "position_model.html")
    args = ap.parse_args(argv)
    cases = []
    for run in args.run:
        c = load(run)
        rows = [r for r in c["rows"] if r["t"] >= c["t_turn"]]
        mx_m = max(r["em"] for r in rows)
        mx_x = max(r["ex"] for r in rows)
        title = f'{c["L"]:g} m tether'
        cases.append(f'''<section style="display:grid;gap:12px"><div class="eyebrow">{title}</div>
<h2>{title}: worst error after the turn {mx_m:.1f} m with the model, {mx_x:.1f} m with the static offset</h2>
<div class="case"><figure>{svg_plan(c)}<figcaption>Plan view, north up. Dots every 10 s on the true path and the model, joined by thin lines.</figcaption></figure>
<figure>{svg_error(c)}<figcaption>Distance from the fish's true position. The shaded band is twice the model's own uncertainty, which it writes into each photo's GPSXYAccuracy.</figcaption></figure></div></section>''')
    html = PAGE.replace("__CASES__", "\n".join(cases)).replace("__TABLE__", table_html(args.table))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(html)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
