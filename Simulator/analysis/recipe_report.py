#!/usr/bin/env python3
"""Build the towfish setup-recipe page: component positions and minimum
transect spacing for the 8 m and 25 m tethers.

Reads the "recipe" study (sweep.py recipe / recipe-spacing) and reuses the
look of sweep_report.py.

    python3 Simulator/analysis/recipe_report.py --out Simulator/runs/report/setup_recipe.html
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "analysis"))
import sweep_report  # noqa: E402
from sweep import RECIPES, load_results  # noqa: E402

RECIPE_NAMES = {"today": "As flown today", "collar75": "Collar 75 mm aft on the rail",
                "under": "200 g weight under the front of the tube",
                "collar75_under": "Collar 75 mm aft + weight under the tube",
                "tube90": "Tube 90 mm aft in the collar",
                "tube90_under": "Tube 90 mm aft + weight under the tube"}
CASE_NAMES = {"sw_RCP_fri": "Friday's hairpin (8 m tether, 7.6 m lines)",
              "sw_B3_L8_D4_S9": "8 m tether, 9 m lines, 4 m deep",
              "sw_B3_L8_D4_S12": "8 m tether, 12 m lines, 4 m deep",
              "sw_B3_L25_D8_S25": "25 m tether, 25 m lines, 7.5 m deep",
              "sw_B3_L25_D8_S35": "25 m tether, 35 m lines, 7.5 m deep"}
VIEWER = "https://claude.ai/artifact/T9yxFPWrduzvcynspWuY3G"


def gather() -> dict:
    res = load_results()
    comp = {}
    for e in res.values():
        if e.get("study") != "recipe" or "score" not in e or e["case"] not in CASE_NAMES:
            continue
        comp.setdefault(e["recipe"], {}).setdefault(e["case"], []).append(
            {k: e["score"][k] for k in ("max_roll", "rescue", "not_level_s", "min_depth")})
    spacing = {}
    for e in res.values():
        if (e.get("study") != "recipe" or e.get("recipe") != "under" or "S" not in e
                or "score" not in e or e.get("rep", 1) != 1 or not e["case"].startswith("sw_B3")):
            continue
        s = e["score"]
        spacing.setdefault(f'{e["L"]}_{e["D"]:g}', []).append(
            {"S": e["S"], "max_roll": s["max_roll"], "rescue": s["rescue"], "upsets": s["upsets"],
             "not_level_s": s["not_level_s"], "min_depth": s["min_depth"],
             "hold_depth": s["hold_depth"], "depth_after_s": s["depth_after_s"]})
    for v in spacing.values():
        v.sort(key=lambda r: r["S"])
    return {"comp": comp, "spacing": spacing, "names": RECIPE_NAMES, "cases": CASE_NAMES,
            "order": [n for n in RECIPE_NAMES if n in comp], "viewer": VIEWER,
            "recipes": {k: list(v) for k, v in RECIPES.items()}}


def page_css() -> str:
    m = re.search(r"<style>(.*?)</style>", sweep_report.PAGE, re.S)
    return m.group(1)


BODY = r"""<title>Towfish Setup Recipe</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wght@500;700&family=Source+Sans+3:wght@400;600&family=JetBrains+Mono:wght@400;600&display=swap">
<style>__CSS__
.recipe { display: grid; grid-template-columns: max-content 1fr; gap: 8px 18px; background: var(--panel);
  border: 1px solid var(--rule); border-radius: 6px; padding: 16px 18px; }
.recipe dt { font: 600 0.75rem/1.5 var(--mono); text-transform: uppercase; letter-spacing: 0.06em; color: var(--ink-3); }
.recipe dd { margin: 0; }
.recipe dd b { font-weight: 600; }
@media (max-width: 560px) { .recipe { grid-template-columns: 1fr; } }
td.l { text-align: left; white-space: normal; min-width: 14ch; }
</style>

<main>
  <header>
    <div class="eyebrow">SubReels towfish · ArduSub 4.7.2 SITL · Fathom tether · 1 m/s tow</div>
    <h1>Setup recipe and transect spacing for the 8 m and 25 m tethers</h1>
    <p class="lede">One hardware change, the same for both tethers, and a minimum line spacing for each. Every turn below was flown by the shipped extension: SURF TRACK holding depth and auto-righting acting at 60° with the stick at neutral.</p>
    <div class="answers" id="answers"></div>
  </header>

  <section>
    <div class="eyebrow">The recipe</div>
    <h2>Move the 200 g nose weight under the front of the tube</h2>
    <dl class="recipe">
      <dt>Nose weight</dt><dd><b>From the bar's front tip to the lowest point under the tube, at its front (dome) end.</b> Lowers the CG about 9 mm below the tube axis and moves it about 25 mm aft. Re-balance in air afterwards; the sim expects the balance point near 131 mm aft of today's 105.8 mm (model frame).</dd>
      <dt>Collar on rail</dt><dd>Leave where it is (2/3 of the tube length from its back end).</dd>
      <dt>Tube in collar</dt><dd>Leave where it is.</dd>
      <dt>Tow point</dt><dd>Leave at 6 in back from the bar's front tip. Never aft of the dome tip: there the fish flipped in every turn.</dd>
      <dt>Extension</dt><dd>Auto-righting at 60° / 0.5 s with neutral vertical stick (shipped), THR_DZ 100, jog down 1300, ATC_ANG_RLL_P 1.5.</dd>
    </dl>
    <p class="note">Why it works: the weight sits low, so the fish rights itself when the tether goes slack in a turn, and it stays near the front, so the nose trim and weathervaning barely change. Moving it under the CG station shifted too much weight aft and did worse.</p>
  </section>

  <section>
    <div class="eyebrow">How the candidates compared</div>
    <h2>Peak roll in five turns, two runs each</h2>
    <p>"R" marks a turn where auto-righting had to step in. The recipe is the only candidate that never needed it. <a href="" id="viewerLink">Open the 3D replays</a> of the recipe against today's setup.</p>
    <div class="tablewrap"><table id="compTable"></table></div>
  </section>

  <section>
    <div class="eyebrow">Transect spacing</div>
    <h2>How much depth each spacing costs in the turn</h2>
    <p>With the recipe fitted, the fish stays upright through almost every hairpin; what a tight spacing costs on a long tether is depth, as the boat turning back drags the fish up. The chart shows the shallowest depth reached in each turn.</p>
    <div class="chart" id="depthChart"></div>
    <div id="spaceTables" style="display:grid; gap:14px"></div>
  </section>

  <section>
    <div class="eyebrow">Caveats</div>
    <h2>What to check on the water</h2>
    <ul class="assume">
      <li>CG height is the biggest sensitivity: on the tube centreline (owner) the fish behaved far better than with an assumed CG 4 mm higher. Balance it in air and do a roll test (tip it 30° hanging from the tow point) after moving the weight.</li>
      <li>Tow-rod height (100 mm above the bar) and a rigid rod are assumed; servo travel (±45° at ±500 µs) is assumed.</li>
      <li>Salt water, no current, the boat's logged speed through turns (no boost), synthetic hairpins with a 3.5 m turn radius.</li>
      <li>Results repeat closely run to run (within a few degrees), but treat spacings as starting points for the first survey.</li>
    </ul>
  </section>
</main>

<script>
const DATA = __DATA__;
const NS = "http://www.w3.org/2000/svg";
const el = (t, a, p) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const fmt = (v, d = 0) => v == null ? "—" : Number(v).toFixed(d);
const depthTxt = v => v == null ? "—" : v <= 0.15 ? "surface" : `${Number(v).toFixed(1)} m`;
const CELLS = [["8_4", "8 m tether, 4 m deep", "--s1"], ["25_8", "25 m tether, 7.5 m deep", "--s2"],
               ["25_4", "25 m tether, 4 m deep", "--s3"]];

// Recommendations, from the data: narrowest spacing from which every wider one
// needed no rescue and lost no more than `loss` m of depth in the turn.
function minSpacing(rows, loss) {
  for (let i = 0; i < rows.length; i++) {
    if (rows.slice(i).every(r => !r.rescue && r.hold_depth - r.min_depth <= loss)) return rows[i].S;
  }
  return null;
}
const ans = [];
const sp = DATA.spacing;
if (sp["8_4"]) ans.push(["8 m tether, 4 m deep", `${minSpacing(sp["8_4"], 0.35)} m lines`,
  `level and within 0.35 m of depth through the turn; ${sp["8_4"][0].S} m still works with one ~6 s wobble`]);
if (sp["25_8"]) ans.push(["25 m tether, 7.5 m deep", `${minSpacing(sp["25_8"], 1.0)} m lines`,
  `loses ≤ 1 m in the turn; at ${minSpacing(sp["25_8"], 3.0)} m lines it rises about 3 m`]);
if (sp["25_4"]) ans.push(["25 m tether, 4 m deep", `${minSpacing(sp["25_4"], 0.6)} m lines`,
  `tighter and the turn drags it toward the surface`]);
ans.push(["Hardware change", "Weight under the tube", "200 g from the bar's nose tip to under the front of the tube"]);
const A = document.getElementById("answers");
for (const [k, v, d] of ans) {
  const div = document.createElement("div"); div.className = "answer";
  div.innerHTML = '<span class="k"></span><span class="v"></span><span class="d"></span>';
  div.children[0].textContent = k; div.children[1].textContent = v; div.children[2].textContent = d;
  A.appendChild(div);
}

document.getElementById("viewerLink").href = DATA.viewer;

// Comparison table.
{
  const cases = Object.keys(DATA.cases);
  let h = "<tr><th>Setup</th>" + cases.map(c => `<th>${DATA.cases[c]}</th>`).join("") + "<th>Rescues</th></tr>";
  for (const r of DATA.order) {
    const d = DATA.comp[r]; let resc = 0, n = 0;
    h += `<tr class="${r === "under" ? "best" : ""}"><td class="l">${DATA.names[r]}${r === "under" ? ' <span class="pill ok">recipe</span>' : ""}</td>`;
    for (const c of cases) {
      const runs = d[c] || []; n += runs.length; resc += runs.filter(x => x.rescue).length;
      h += `<td>${runs.map(x => `${fmt(x.max_roll)}°${x.rescue ? " R" : ""}`).join(" / ") || "—"}</td>`;
    }
    h += `<td>${resc} of ${n}</td></tr>`;
  }
  document.getElementById("compTable").innerHTML = h;
}

// Depth chart: shallowest depth vs spacing, with each cell's held depth.
{
  const host = document.getElementById("depthChart");
  const lg = document.createElement("div"); lg.className = "legend";
  for (const [, l, c] of CELLS) { const s = document.createElement("span"); const i = document.createElement("i");
    i.style.background = `var(${c})`; s.append(i, l); lg.appendChild(s); }
  const s2 = document.createElement("span"); s2.textContent = "dashed: depth held before the turn"; lg.appendChild(s2);
  host.appendChild(lg);
  const W = 900, H = 380, m = {l: 58, r: 20, t: 14, b: 48}, xMax = 52, yMax = 8;
  const x = v => m.l + v / xMax * (W - m.l - m.r), y = v => m.t + v / yMax * (H - m.t - m.b);
  const svg = el("svg", {viewBox: `0 0 ${W} ${H}`, role: "img"}, host);
  for (let d = 0; d <= 8; d += 2) {
    el("line", {x1: m.l, x2: W - m.r, y1: y(d), y2: y(d), stroke: "var(--grid)"}, svg);
    el("text", {x: m.l - 8, y: y(d) + 4, "text-anchor": "end", fill: "var(--ink-3)", "font-size": 12, "font-family": "var(--mono)"}, svg).textContent = d;
  }
  for (const s of [0, 10, 20, 30, 40, 50])
    el("text", {x: x(s), y: H - m.b + 18, "text-anchor": "middle", fill: "var(--ink-3)", "font-size": 12, "font-family": "var(--mono)"}, svg).textContent = s;
  el("text", {x: (m.l + W - m.r) / 2, y: H - 8, "text-anchor": "middle", fill: "var(--ink-2)", "font-size": 13}, svg).textContent = "Line spacing (m)";
  el("text", {x: 16, y: (m.t + H - m.b) / 2, "text-anchor": "middle", fill: "var(--ink-2)", "font-size": 13,
    transform: `rotate(-90 16 ${(m.t + H - m.b) / 2})`}, svg).textContent = "Shallowest depth in the turn (m)";
  const tip = document.createElement("div"); tip.className = "tip"; tip.hidden = true; host.appendChild(tip);
  for (const [key, label, color] of CELLS) {
    const rows = sp[key] || []; if (!rows.length) continue;
    el("line", {x1: x(rows[0].S), x2: x(rows[rows.length - 1].S), y1: y(rows[0].hold_depth), y2: y(rows[0].hold_depth),
      stroke: `var(${color})`, "stroke-width": 1.5, "stroke-dasharray": "5 4"}, svg);
    el("polyline", {points: rows.map(r => `${x(r.S)},${y(Math.max(0, r.min_depth))}`).join(" "), fill: "none",
      stroke: `var(${color})`, "stroke-width": 2}, svg);
    for (const r of rows) {
      const dot = el("circle", {cx: x(r.S), cy: y(Math.max(0, r.min_depth)), r: 5, fill: `var(${color})`,
        stroke: "var(--panel)", "stroke-width": 2, tabindex: 0}, svg);
      const show = () => { const b = dot.getBoundingClientRect(), hb = host.getBoundingClientRect();
        tip.textContent = `${label}, ${r.S} m lines: shallowest ${depthTxt(r.min_depth)}, peak roll ${fmt(r.max_roll)}°${r.rescue ? ", rescued" : ""}`;
        tip.hidden = false; tip.style.left = (b.left + b.width / 2 - hb.left) + "px"; tip.style.top = (b.top - hb.top) + "px"; };
      dot.addEventListener("pointerenter", show); dot.addEventListener("focus", show);
      dot.addEventListener("pointerleave", () => tip.hidden = true); dot.addEventListener("blur", () => tip.hidden = true);
    }
  }
}

// Spacing tables.
{
  const T = document.getElementById("spaceTables");
  for (const [key, label] of CELLS) {
    const rows = sp[key] || []; if (!rows.length) continue;
    const w = document.createElement("div"); w.className = "tablewrap";
    let h = `<tr><th colspan="7" style="text-align:left;color:var(--ink);font-size:0.8rem">${label} (holds ${fmt(rows[0].hold_depth, 1)} m)</th></tr>`;
    h += "<tr><th>Lines</th><th>Result</th><th>Peak roll</th><th>Upsets past 45°</th><th>Off level (&gt;20°)</th><th>Shallowest</th><th>Depth lost</th></tr>";
    for (const r of rows) {
      const res = r.rescue ? '<span class="pill no">rescued</span>' : r.upsets ? '<span class="pill no">wobble</span>' : '<span class="pill ok">upright</span>';
      h += `<tr><td>${r.S} m</td><td>${res}</td><td>${fmt(r.max_roll)}°</td><td>${r.upsets}</td><td>${fmt(r.not_level_s)} s</td>` +
           `<td>${depthTxt(r.min_depth)}</td><td>${fmt(r.hold_depth - Math.max(0, r.min_depth), 1)} m</td></tr>`;
    }
    const t = document.createElement("table"); t.innerHTML = h; w.appendChild(t); T.appendChild(w);
  }
}
</script>
"""


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=ROOT / "runs" / "report" / "setup_recipe.html")
    args = ap.parse_args(argv)
    data = gather()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(BODY.replace("__CSS__", page_css()).replace("__DATA__", json.dumps(data)))
    print(f"wrote {args.out}: {sum(len(v) for v in data['spacing'].values())} spacing runs, "
          f"{len(data['order'])} recipes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
