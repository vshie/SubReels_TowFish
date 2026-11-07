#!/usr/bin/env python3
"""Build the tether / depth / line-spacing report page from the sweep results.

Reads runs/sweep/results_*.json (studies "fixed", "spacing", "early",
"righting") and runs/sweep/statics_P*.json (tether_statics.py --json), and
writes one self-contained HTML page.

    python3 Simulator/analysis/sweep_report.py --out Simulator/runs/report/tether_report.html
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "analysis"))
from sweep import RESULTS_DIR, SPACINGS, load_results  # noqa: E402

STICKS = {1200: "75 % down", 1300: "50 % down", 1400: "25 % down"}
CORNER_VIEWER = "https://claude.ai/artifact/EB4rb4icZ7r92re7iGheKV"
SPACING_VIEWER = "https://claude.ai/artifact/2UouzXC7iyUhNuLsJWWiHh"


def gather() -> dict:
    res = load_results()
    fixed = [{"L": e["L"], "pwm": e["pwm"], "depth": e["score"]["depth"],
              "stable": e["score"]["stable"], "tension": e["score"]["tension"]}
             for e in res.values() if e.get("study") == "fixed" and "score" in e]
    statics = {}
    for pwm in STICKS:
        f = RESULTS_DIR / f"statics_P{pwm}.json"
        if f.exists():
            s = json.loads(f.read_text())
            statics[pwm] = [[L, d] for L, d in zip(s["L"], s["depth"]) if 4 <= L <= 100]

    cells = {}
    for e in res.values():
        if e.get("study") != "spacing" or "score" not in e:
            continue
        sc = e["score"]
        cells.setdefault((e["L"], e["D"]), []).append(
            {"S": e["S"], "pass": sc["pass"], "max_roll": sc["max_roll"],
             "slack_s": sc["slack_s"], "min_depth": sc["min_depth"]})
    spacing = []
    for (L, D), probes in sorted(cells.items()):
        probes.sort(key=lambda p: p["S"])
        passes = [p["S"] for p in probes if p["pass"]]
        fails = [p["S"] for p in probes if not p["pass"]]
        best = min(passes) if passes else None
        # A failure wider than the narrowest pass means the cell is not monotonic.
        odd = bool(best is not None and any(s > best for s in fails))
        tight = max((s for s in fails if best is None or s < best), default=None)
        spacing.append({"L": L, "D": D, "min_S": best, "fail_below": tight,
                        "max_tested": max(p["S"] for p in probes), "nonmonotonic": odd,
                        "probes": probes})

    grid = {}
    for e in res.values():
        if e.get("study") == "grid" and "score" in e:
            grid.setdefault(f'{e["L"]}_{e["D"]}', []).append({"S": e["S"], **e["score"]})
    for v in grid.values():
        v.sort(key=lambda r: r["S"])

    def agg(rows):
        return {"n": len(rows),
                "inverted_s": round(float(np.mean([r["inverted_s"] for r in rows])), 1),
                "worst_inverted_s": round(float(max(r["inverted_s"] for r in rows)), 1),
                "upright_s": round(float(np.mean([r.get("to_upright_s") or 0 for r in rows])), 1),
                "min_depth": round(float(np.mean([r["min_depth_after"] for r in rows])), 2),
                "rescues": round(float(np.mean([r["rescues"] for r in rows])), 1),
                "twist": round(float(max(abs(r["twist_truth"]) for r in rows)), 2)}

    corner = {}
    for e in res.values():
        if e.get("study") == "early" and "score" in e:
            corner.setdefault((e["thresh"], e["vertical_pwm"]), []).append(e["score"])
    corner_rows = [{"thresh": k[0], "pwm": k[1], **agg(v)} for k, v in corner.items()]
    corner_rows.sort(key=lambda r: (r["inverted_s"], r["rescues"]))
    old = [e["score"] for e in res.values()
           if e.get("study") == "righting" and "score" in e and not e.get("trigger")]
    return {"fixed": fixed, "statics": {str(k): v for k, v in statics.items()},
            "spacings": SPACINGS, "spacing": spacing, "grid": grid, "corner": corner_rows,
            "old_rule": agg(old) if old else None, "viewer": CORNER_VIEWER, "spacing_viewer": SPACING_VIEWER}


PAGE = r"""<title>Tether, Depth and Line Spacing</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wght@500;700&family=Source+Sans+3:wght@400;600&family=JetBrains+Mono:wght@400;600&display=swap">
<style>
/* Layout: one reading column of findings, each finding a chart + its table; data in mono. */
:root {
  --bg: #f6f8f9; --panel: #ffffff; --ink: #0f1a20; --ink-2: #4a5961; --ink-3: #7b8a92;
  --rule: #dde4e8; --grid: #e8eef1; --accent: #0b6e8a;
  --s1: #2a78d6; --s2: #eb6834; --s3: #1baf7a; --s4: #eda100; --s5: #e87ba4;
  --good: #1f7a3a; --bad: #b3261e; --good-bg: #e3f2e7; --bad-bg: #fbe7e5;
  --display: "Archivo", "Helvetica Neue", Arial, sans-serif;
  --body: "Source Sans 3", "Segoe UI", Helvetica, Arial, sans-serif;
  --mono: "JetBrains Mono", ui-monospace, Menlo, Consolas, monospace;
}
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg: #0e1518; --panel: #151f23; --ink: #e8eef0; --ink-2: #a9b8bf; --ink-3: #7d8d94;
  --rule: #26343a; --grid: #1e2a2f; --accent: #4fb3cf;
  --s1: #3987e5; --s2: #d95926; --s3: #199e70; --s4: #c98500; --s5: #d55181;
  --good: #6fcf8a; --bad: #f08b83; --good-bg: #16301f; --bad-bg: #3a1c1a; color-scheme: dark } }
:root[data-theme="dark"] {
  --bg: #0e1518; --panel: #151f23; --ink: #e8eef0; --ink-2: #a9b8bf; --ink-3: #7d8d94;
  --rule: #26343a; --grid: #1e2a2f; --accent: #4fb3cf;
  --s1: #3987e5; --s2: #d95926; --s3: #199e70; --s4: #c98500; --s5: #d55181;
  --good: #6fcf8a; --bad: #f08b83; --good-bg: #16301f; --bad-bg: #3a1c1a; color-scheme: dark }
body { background: var(--bg); color: var(--ink); font: 400 16px/1.55 var(--body);
  padding-left: max(16px, env(safe-area-inset-left, 0px)); padding-right: max(16px, env(safe-area-inset-right, 0px)); }
main { max-width: 960px; margin: 0 auto; padding-block: 40px 64px; display: grid; gap: 40px; }
h1, h2 { font-family: var(--display); text-wrap: balance; margin: 0; letter-spacing: -0.01em; }
h1 { font-size: 2rem; line-height: 1.15; }
h2 { font-size: 1.35rem; }
p { margin: 0; max-width: 68ch; }
.eyebrow { font: 600 0.75rem/1 var(--mono); text-transform: uppercase; letter-spacing: 0.08em; color: var(--accent); }
header { display: grid; gap: 12px; }
.lede { color: var(--ink-2); font-size: 1.05rem; }
.answers { display: grid; grid-template-columns: repeat(auto-fit, minmax(165px, 1fr)); gap: 12px; }
.answer { background: var(--panel); border: 1px solid var(--rule); border-radius: 6px; padding: 14px 16px; display: grid; gap: 4px; align-content: start; }
.answer .k { font: 600 0.72rem/1.2 var(--mono); text-transform: uppercase; letter-spacing: 0.06em; color: var(--ink-3); }
.answer .v { font: 700 1.5rem/1.15 var(--display); font-variant-numeric: tabular-nums; }
.answer .d { color: var(--ink-2); font-size: 0.92rem; }
section { display: grid; gap: 14px; min-width: 0; }
.chart { background: var(--panel); border: 1px solid var(--rule); border-radius: 6px; padding: 12px 8px 4px; position: relative; min-width: 0; }
.chart svg { display: block; width: 100%; height: auto; }
.legend { display: flex; flex-wrap: wrap; gap: 6px 18px; padding: 0 10px 8px; font-size: 0.88rem; color: var(--ink-2); }
.legend span { display: inline-flex; align-items: center; gap: 6px; }
.legend i { width: 14px; height: 3px; border-radius: 2px; display: inline-block; }
.tip { position: absolute; pointer-events: none; background: var(--ink); color: var(--bg); font: 400 0.8rem/1.35 var(--mono);
  padding: 6px 8px; border-radius: 4px; white-space: nowrap; transform: translate(-50%, calc(-100% - 10px)); }
.tablewrap { overflow-x: auto; border: 1px solid var(--rule); border-radius: 6px; background: var(--panel); }
table { border-collapse: collapse; width: 100%; font-size: 0.9rem; }
th, td { padding: 7px 10px; text-align: right; border-bottom: 1px solid var(--grid); white-space: nowrap; }
th:first-child, td:first-child { text-align: left; }
th { font: 600 0.72rem/1.2 var(--mono); text-transform: uppercase; letter-spacing: 0.05em; color: var(--ink-3); background: var(--bg); }
td { font-variant-numeric: tabular-nums; }
tr:last-child td { border-bottom: 0; }
.pill { font: 600 0.75rem/1 var(--mono); padding: 3px 7px; border-radius: 999px; display: inline-block; }
.pill.ok { background: var(--good-bg); color: var(--good); }
.pill.no { background: var(--bad-bg); color: var(--bad); }
.best td { font-weight: 600; }
.note { color: var(--ink-2); font-size: 0.92rem; }
ul.assume { margin: 0; padding-left: 20px; color: var(--ink-2); display: grid; gap: 4px; max-width: 72ch; }
a { color: var(--accent); }
a:focus-visible, button:focus-visible { outline: 2px solid var(--accent); outline-offset: 2px; }
</style>

<main>
  <header>
    <div class="eyebrow">SubReels towfish · ArduSub 4.7.2 SITL · Fathom 7.5 mm tether · 1 m/s</div>
    <h1>How deep can the towfish fly, and how wide must the lines be?</h1>
    <p class="lede">A longer tether buys depth on straight lines but makes the hairpin harder: in the turn the boat heads back toward the fish and the line goes slack. These are the simulated trade-offs, with the auto-righting recipe that keeps the most depth when a roll-over does happen.</p>
    <div class="answers" id="answers"></div>
  </header>

  <section>
    <div class="eyebrow">Finding 1</div>
    <h2>Depth keeps growing with tether length, with diminishing returns</h2>
    <p>Straight tow at a constant 1 m/s, the fish in STABILIZE with the vertical stick held still. Dots are full simulations; lines are the steady-tether model fitted to them. At full down the fish loses roll control and rolls over on every tether, so 75 % down is the deepest usable setting.</p>
    <div class="chart" id="depthChart"></div>
    <div class="tablewrap"><table id="depthTable"></table></div>
  </section>

  <section>
    <div class="eyebrow">Finding 2</div>
    <h2>How long each hairpin leaves the fish off level</h2>
    <p>Synthetic hairpin with the boat's logged speed profile and no speed boost; SURF TRACK holds the working depth over a flat bottom, with the corner righting recipe from Finding 3 armed. Every spacing was flown on the 8, 16 and 25 m tethers. Times count from the first corner to 90 s after the second; recovery is followed to the end of the run.</p>
    <p class="note">On 8 m of tether the fish flies most spacings clean, including Friday's 7.6 m, but 9 m and 20 m upset it for about 20 s. On 16 m it is clean from 25 m lines, on 25 m from 35 m lines. Below those spacings every turn recovered on its own, but the cost grows quickly as the lines close in: on 25 m of tether at 7.6 m lines the fish spent 31 s upside down at the surface and needed 98 s to get back to depth. At 8 m depth the 25 m tether never flew a clean turn up to 50 m lines, and wider lines made it worse: the fish repeatedly upset while descending back to 8 m, where the wings have little collective left for roll. Asked for 9.4 m (that tether's steady maximum at 75 % down), the fish never got there: depth hold drove the wings to their stops on the straight approach, it rolled and was rescued before the turn, and it settled near 7.2 m, then upset 4–10 times in every turn. The steady maximum leaves depth hold no headroom. <a href="" id="spacingViewerLink">Open the 3D replays</a>.</p>
    <div class="chart" id="gridChart"></div>
    <div id="gridTables" style="display:grid; gap:14px"></div>
  </section>

  <section>
    <div class="eyebrow">Finding 3</div>
    <h2>When it does roll over, act in the corner with the stick at neutral (now the extension's default)</h2>
    <p>Waiting for the boat to straighten up cost the fish its depth: depth hold keeps the wings at full down, which flies an inverted fish to the surface. Acting at 60° with the vertical stick at neutral leaves the wings room to snap the fish upright. Each row is the mean of four runs (two tracks, two repeats). <a href="" id="viewerLink">Open the 3D comparison</a>.</p>
    <div class="tablewrap"><table id="cornerTable"></table></div>
  </section>

  <section>
    <div class="eyebrow">Assumptions</div>
    <h2>What the numbers rest on</h2>
    <ul class="assume">
      <li>Tether: Blue Robotics Fathom, 7.5 mm, neutrally buoyant, normal drag coefficient 1.2 (strumming could raise it toward 1.8, which costs depth).</li>
      <li>Salt water, no current, boat at 1 m/s on straight lines; the boat is not slowed by the tether.</li>
      <li>Wing travel ±45° at ±500 µs, tow-rod position and lateral CG are estimates; roll stabilisation ATC_ANG_RLL_P = 1.5.</li>
      <li>Synthetic hairpins use a 3.5 m boat turn radius; spacings tested: <span id="spacingList"></span> m.</li>
      <li>Results compare options reliably; treat the absolute depths and spacings as estimates to confirm in the field.</li>
    </ul>
  </section>
</main>

<script>
const DATA = __DATA__;
const STICK = {1200: ["75 % down", "--s1"], 1300: ["50 % down", "--s2"], 1400: ["25 % down", "--s3"]};
const DEPTHS = {4: ["4 m depth", "--s1"], 8: ["8 m depth", "--s2"], 12: ["12 m depth", "--s3"]};
const css = v => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const NS = "http://www.w3.org/2000/svg";
const el = (tag, attrs, parent) => { const e = document.createElementNS(NS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]); if (parent) parent.appendChild(e); return e; };
const fmt = (v, d = 1) => v == null ? "—" : Number(v).toFixed(d);
const depthTxt = v => v == null ? "—" : v <= 0.05 ? "surface" : `${Number(v).toFixed(1)} m`;

function chart(host, {xMax, yMax, xLabel, yLabel, yTicks, xTicks, invertY}) {
  const W = 900, H = 420, m = {l: 58, r: 96, t: 14, b: 48};
  const svg = el("svg", {viewBox: `0 0 ${W} ${H}`, role: "img"}, null);
  host.appendChild(svg);
  const x = v => m.l + v / xMax * (W - m.l - m.r);
  const y = v => invertY ? m.t + v / yMax * (H - m.t - m.b) : H - m.b - v / yMax * (H - m.t - m.b);
  const g = el("g", {}, svg);
  for (const t of yTicks) {
    el("line", {x1: m.l, x2: W - m.r, y1: y(t), y2: y(t), stroke: "var(--grid)", "stroke-width": 1}, g);
    el("text", {x: m.l - 8, y: y(t) + 4, "text-anchor": "end", fill: "var(--ink-3)", "font-size": 12, "font-family": "var(--mono)"}, g).textContent = t;
  }
  for (const t of xTicks) {
    el("text", {x: x(t), y: H - m.b + 18, "text-anchor": "middle", fill: "var(--ink-3)", "font-size": 12, "font-family": "var(--mono)"}, g).textContent = t;
  }
  el("line", {x1: m.l, x2: W - m.r, y1: invertY ? m.t : H - m.b, y2: invertY ? m.t : H - m.b, stroke: "var(--ink-3)", "stroke-width": 1}, g);
  el("text", {x: (m.l + W - m.r) / 2, y: H - 8, "text-anchor": "middle", fill: "var(--ink-2)", "font-size": 13}, g).textContent = xLabel;
  const yl = el("text", {x: 16, y: (m.t + H - m.b) / 2, "text-anchor": "middle", fill: "var(--ink-2)", "font-size": 13,
    transform: `rotate(-90 16 ${(m.t + H - m.b) / 2})`}, g); yl.textContent = yLabel;
  const tip = document.createElement("div"); tip.className = "tip"; tip.hidden = true; host.appendChild(tip);
  const hover = (mark, text) => {
    const show = () => { const r = mark.getBoundingClientRect(), h = host.getBoundingClientRect();
      tip.textContent = text; tip.hidden = false;
      tip.style.left = (r.left + r.width / 2 - h.left) + "px"; tip.style.top = (r.top - h.top) + "px"; };
    mark.addEventListener("pointerenter", show); mark.addEventListener("pointerleave", () => tip.hidden = true);
    mark.addEventListener("focus", show); mark.addEventListener("blur", () => tip.hidden = true);
    mark.setAttribute("tabindex", "0");
  };
  return {svg, x, y, W, H, m, hover};
}

function legend(host, items) {
  const d = document.createElement("div"); d.className = "legend";
  for (const [label, color, dashed] of items) {
    const s = document.createElement("span"); const i = document.createElement("i");
    if (color) { i.style.background = dashed ? `repeating-linear-gradient(90deg, var(${color}) 0 5px, transparent 5px 8px)` : `var(${color})`;
      s.append(i); }
    s.append(label); d.appendChild(s);
  }
  host.appendChild(d);
}

// ---------- answers ----------
const stable = DATA.fixed.filter(r => r.stable && r.pwm === 1200).sort((a, b) => a.L - b.L);
const minL = D => (stable.find(r => r.depth >= D + 0.5) || {}).L;
const cell = (L, D) => DATA.spacing.find(c => c.L === L && c.D === D);
const deepest = stable[stable.length - 1];
const best = DATA.corner[0];
const ans = [
  ["Deepest simulated", deepest ? `${fmt(deepest.depth)} m` : "—", deepest ? `on ${deepest.L} m of tether at 75 % down; still gaining ~0.1 m per extra metre` : ""],
];
for (const [key, label] of [["8_4", "8 m tether, 4 m deep"], ["16_4", "16 m tether, 4 m deep"],
                            ["25_4", "25 m tether, 4 m deep"], ["25_8", "25 m tether, 8 m deep"],
                            ["25_9.4", "25 m tether, 9.4 m target"]]) {
  const rows = DATA.grid[key] || [];
  if (!rows.length) continue;
  const fails = rows.filter(r => !r.pass);
  const cleanFrom = rows.find(r => r.pass && rows.filter(q => q.S > r.S).every(q => q.pass));
  const worst = fails.reduce((a, r) => (!a || r.depth_after_s > a.depth_after_s) ? r : a, null);
  const cleanBelow = cleanFrom ? rows.filter(r => r.pass && r.S < cleanFrom.S).map(r => r.S) : [];
  const cleanTxt = cleanFrom ? (cleanBelow.length ? `clean at ${cleanBelow.join(", ")} and ${cleanFrom.S}+ m` : `clean from ${cleanFrom.S} m lines`)
                             : `no clean turn ≤ ${rows[rows.length - 1].S} m`;
  ans.push([label, cleanTxt,
    worst ? `worst: ${worst.S} m lines, back at depth after ${worst.depth_after_s == null ? "never" : fmt(worst.depth_after_s, 0) + " s"}` : "every spacing flew clean"]);
}
if (best) ans.push(["Best righting recipe", `${best.thresh}° · stick ${best.pwm === 1500 ? "neutral" : best.pwm}`,
  `${fmt(best.inverted_s)} s upside down on average, upright in ${fmt(best.upright_s)} s`]);
const A = document.getElementById("answers");
for (const [k, v, d] of ans) {
  const div = document.createElement("div"); div.className = "answer";
  div.innerHTML = `<span class="k"></span><span class="v"></span><span class="d"></span>`;
  div.children[0].textContent = k; div.children[1].textContent = v; div.children[2].textContent = d;
  A.appendChild(div);
}

// ---------- depth chart ----------
{
  const host = document.getElementById("depthChart");
  legend(host, [...Object.entries(STICK).map(([p, [l, c]]) => [l, c]), ["model beyond 60 m", "--ink-3", true]]);
  const c = chart(host, {xMax: 100, yMax: 20, xLabel: "Tether length (m)", yLabel: "Steady depth (m)",
    yTicks: [0, 4, 8, 12, 16, 20], xTicks: [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100], invertY: true});
  for (const D of [4, 8, 12]) {
    el("line", {x1: c.x(0), x2: c.x(100), y1: c.y(D), y2: c.y(D), stroke: "var(--ink-3)", "stroke-width": 1, "stroke-dasharray": "2 4"}, c.svg);
    el("text", {x: c.x(100) + 6, y: c.y(D) + 4, fill: "var(--ink-3)", "font-size": 12, "font-family": "var(--mono)"}, c.svg).textContent = `${D} m target`;
  }
  for (const [p, [label, color]] of Object.entries(STICK)) {
    const pts = DATA.statics[p] || [];
    const seg = (a, b, dash) => { const s = pts.filter(q => q[0] >= a && q[0] <= b);
      if (s.length) el("polyline", {points: s.map(q => `${c.x(q[0])},${c.y(q[1])}`).join(" "), fill: "none",
        stroke: `var(${color})`, "stroke-width": 2, "stroke-dasharray": dash || "none", "stroke-linecap": "round"}, c.svg); };
    seg(4, 60); seg(60, 100, "5 4");
    const rows = DATA.fixed.filter(r => String(r.pwm) === p && r.stable).sort((a, b) => a.L - b.L);
    for (const r of rows) {
      const dot = el("circle", {cx: c.x(r.L), cy: c.y(r.depth), r: 5, fill: `var(${color})`, stroke: "var(--panel)", "stroke-width": 2}, c.svg);
      c.hover(dot, `${r.L} m tether · ${label}: ${fmt(r.depth, 2)} m`);
    }
    const end = pts[pts.length - 1];
    if (end) el("text", {x: c.x(end[0]) + 6, y: c.y(end[1]) + 4, fill: "var(--ink-2)", "font-size": 12}, c.svg).textContent = label;
  }
  const t = document.getElementById("depthTable");
  const Ls = [...new Set(DATA.fixed.map(r => r.L))].sort((a, b) => a - b);
  let h = "<tr><th>Stick</th>" + Ls.map(L => `<th>${L} m</th>`).join("") + "</tr>";
  for (const p of [1200, 1300, 1400, 1100]) {
    h += `<tr><td>${p === 1100 ? "100 % down" : STICK[p][0]} <span class="note">(${p} µs)</span></td>`;
    for (const L of Ls) { const r = DATA.fixed.find(q => q.L === L && q.pwm === p);
      h += `<td>${!r ? "—" : r.stable ? fmt(r.depth) : '<span class="pill no">rolled over</span>'}</td>`; }
    h += "</tr>";
  }
  t.innerHTML = h;
}

// ---------- spacing grid ----------
{
  const CELLS = [["8_4", "8 m tether, 4 m deep", "--s1"], ["16_4", "16 m tether, 4 m deep", "--s2"],
                 ["25_4", "25 m tether, 4 m deep", "--s3"], ["25_8", "25 m tether, 8 m deep", "--s4"],
                 ["25_9.4", "25 m tether, 9.4 m target", "--s5"]];
  const host = document.getElementById("gridChart");
  legend(host, CELLS.map(([, l, c]) => [l, c]).concat([["○ clean turn  ● upset", null]]));
  const allS = Object.values(DATA.grid).flat();
  const yTop = Math.max(60, Math.ceil(Math.max(...allS.map(r => r.not_level_s)) / 10) * 10);
  const c = chart(host, {xMax: 52, yMax: yTop, xLabel: "Line spacing (m)", yLabel: "Time not level, roll > 20° (s)",
    yTicks: Array.from({length: yTop / 10 + 1}, (_, i) => i * 10), xTicks: [0, 10, 20, 30, 40, 50]});
  for (const [key, label, color] of CELLS) {
    const rows = DATA.grid[key] || [];
    if (!rows.length) continue;
    el("polyline", {points: rows.map(r => `${c.x(r.S)},${c.y(r.not_level_s)}`).join(" "), fill: "none",
      stroke: `var(${color})`, "stroke-width": 2, "stroke-linejoin": "round"}, c.svg);
    for (const r of rows) {
      const dot = el("circle", {cx: c.x(r.S), cy: c.y(r.not_level_s), r: r.pass ? 4 : 5.5,
        fill: r.pass ? "var(--panel)" : `var(${color})`, stroke: r.pass ? `var(${color})` : "var(--panel)", "stroke-width": 2}, c.svg);
      c.hover(dot, `${label}, ${r.S} m lines: ` + (r.pass ? "clean" :
        `${fmt(r.not_level_s)} s off level, ${fmt(r.inverted_s)} s inverted, level after ${r.level_after_s == null ? "never" : fmt(r.level_after_s) + " s"}`));
    }
  }
  const T = document.getElementById("gridTables");
  for (const [key, label] of CELLS) {
    const rows = DATA.grid[key] || [];
    if (!rows.length) continue;
    const wrap = document.createElement("div"); wrap.className = "tablewrap";
    const cols = [["Line spacing", r => `${r.S} m`],
      ["Result", r => r.pass ? '<span class="pill ok">clean</span>' : r.recovered === false ? '<span class="pill no">no recovery</span>' : '<span class="pill no">upset</span>'],
      ["Peak roll", r => `${fmt(r.max_roll, 0)}°`],
      ["Upsets", r => r.upsets],
      ["Not level (>20°)", r => `${fmt(r.not_level_s)} s`],
      ["Past 45°", r => `${fmt(r.upset_s)} s`],
      ["Inverted", r => `${fmt(r.inverted_s)} s`],
      ["Level again after", r => r.pass ? "—" : r.level_after_s == null ? "never" : `${fmt(r.level_after_s)} s`],
      ["Back at depth after", r => r.pass ? "—" : r.depth_after_s == null ? "never" : `${fmt(r.depth_after_s)} s`],
      ["Shallowest", r => depthTxt(r.min_depth)]];
    let h = `<tr><th colspan="${cols.length}" style="text-align:left; color:var(--ink); font-size:0.8rem">${label} (holds ${fmt(rows[0].hold_depth)} m)</th></tr>`;
    h += "<tr>" + cols.map(([n]) => `<th>${n}</th>`).join("") + "</tr>";
    for (const r of rows) h += "<tr>" + cols.map(([, f]) => `<td>${f(r)}</td>`).join("") + "</tr>";
    const t = document.createElement("table"); t.innerHTML = h; wrap.appendChild(t); T.appendChild(wrap);
  }
  document.getElementById("spacingViewerLink").href = DATA.spacing_viewer;
}

// ---------- corner table ----------
{
  const t = document.getElementById("cornerTable");
  const word = p => ({1900: "full up", 1700: "half up", 1500: "neutral", 1300: "half down"})[p] || p;
  let h = "<tr><th>Recipe</th><th>Upside down (mean / worst)</th><th>Back upright</th><th>Shallowest depth</th><th>Rescues per run</th><th>Max twist</th></tr>";
  DATA.corner.forEach((r, i) => {
    h += `<tr class="${i === 0 ? "best" : ""}"><td>Act at ${r.thresh}°, stick ${word(r.pwm)} (${r.pwm} µs)${i === 0 ? ' <span class="pill ok">best</span>' : ""}</td>` +
      `<td>${fmt(r.inverted_s)} / ${fmt(r.worst_inverted_s)} s</td><td>${fmt(r.upright_s)} s</td><td>${depthTxt(r.min_depth)}</td><td>${fmt(r.rescues)}</td><td>${fmt(r.twist, 2)} turns</td></tr>`;
  });
  const o = DATA.old_rule;
  if (o) h += `<tr><td>Old rule: wait for the boat + 3 s (all 16 stick recipes)</td><td>${fmt(o.inverted_s)} / ${fmt(o.worst_inverted_s)} s</td><td>${fmt(o.upright_s)} s</td><td>${depthTxt(o.min_depth)}</td><td>${fmt(o.rescues)}</td><td>${fmt(o.twist, 2)} turns</td></tr>`;
  t.innerHTML = h;
  document.getElementById("viewerLink").href = DATA.viewer;
  document.getElementById("spacingList").textContent = DATA.spacings.join(", ");
}
</script>
"""


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=ROOT / "runs" / "report" / "tether_report.html")
    args = ap.parse_args(argv)
    data = gather()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(PAGE.replace("__DATA__", json.dumps(data)))
    print(f"wrote {args.out} ({len(data['spacing'])} spacing cells, {len(data['fixed'])} depth runs)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
