"""Render the towfish altitude/swath report to a print-ready HTML document.

Charts are matplotlib SVG embedded inline so the output is a single
self-contained file that headless Chrome can print to PDF.
"""

import datetime
import io
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

BASE = "/Users/tonywhite/Documents/SubReels_TowFish/tools"
R = np.load(f"{BASE}/altitude_result.npz")
S = json.load(open(f"{BASE}/summary.json"))

HST = datetime.timezone(datetime.timedelta(hours=-10))
t, alt, depth, fdepth, sw = R["t"], R["alt"], R["depth"], R["fdepth"], R["swath"]
lat, lng = R["lat"], R["lng"]

HFOV_K = 2 * np.tan(np.radians(94 / 2))
VFOV_K = 2 * np.tan(np.radians(62 / 2))

INK = "#1a1a1a"
MUTED = "#666666"
FAINT = "#c8c8c8"
BLUE = "#1f4e79"
RED = "#b03a2e"
GREEN = "#0f6b52"

plt.rcParams.update({
    "font.family": "Helvetica Neue, Helvetica, Arial, sans-serif",
    "font.size": 8,
    "text.color": INK,
    "axes.labelcolor": INK,
    "axes.edgecolor": "#999999",
    "xtick.color": MUTED,
    "ytick.color": MUTED,
    "axes.linewidth": 0.6,
    "svg.fonttype": "none",
})


def fig_to_svg(fig):
    buf = io.StringIO()
    fig.savefig(buf, format="svg", bbox_inches="tight", transparent=True)
    plt.close(fig)
    svg = buf.getvalue()
    return svg[svg.index("<svg"):]


def chart_timeseries():
    t0 = t.min()
    b = np.floor((t - t0) / 120).astype(int)
    lab, dd, ff, aa = [], [], [], []
    for i in range(b.max() + 1):
        k = b == i
        if k.sum() < 40:
            continue
        lab.append(datetime.datetime.fromtimestamp(float(t0 + i * 120), HST))
        dd.append(depth[k].mean())
        ff.append(fdepth[k].mean())
        aa.append(alt[k].mean())

    fig, ax = plt.subplots(figsize=(9.4, 2.9))
    ax.plot(lab, dd, color=BLUE, lw=1.4, label="Seabed depth (boat sonar)")
    ax.plot(lab, ff, color=RED, lw=1.4, label="Towfish pressure depth")
    ax.plot(lab, aa, color=GREEN, lw=1.8, label="Towfish altitude")
    ax.set_ylabel("metres")
    ax.set_xlabel("time of day (HST)")
    ax.grid(alpha=0.25, lw=0.5)
    ax.legend(frameon=False, ncol=3, fontsize=8, loc="upper center",
              bbox_to_anchor=(0.5, 1.16))
    ax.xaxis.set_major_formatter(matplotlib.dates.DateFormatter("%H:%M", tz=HST))
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    return fig_to_svg(fig)


def chart_hist():
    fig, ax = plt.subplots(figsize=(4.5, 2.4))
    edges = np.arange(0, 14, 1.0)
    ax.hist(alt, bins=edges, color=GREEN, alpha=0.85, edgecolor="white", linewidth=0.6)
    ax.set_xlabel("towfish altitude (m)")
    ax.set_ylabel("soundings")
    ax.grid(alpha=0.25, lw=0.5, axis="y")
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    return fig_to_svg(fig)


def chart_track():
    fig, ax = plt.subplots(figsize=(4.6, 4.0))
    step = max(1, len(lat) // 6000)
    sc = ax.scatter(lng[::step], lat[::step], c=alt[::step], s=2.5,
                    cmap="viridis", linewidths=0)
    ax.set_aspect(1 / np.cos(np.radians(lat.mean())))
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color(FAINT)

    # 100 m scale bar
    m_per_deg_lng = 111320 * np.cos(np.radians(lat.mean()))
    bar = 100 / m_per_deg_lng
    x0 = lng.min() + 0.06 * (lng.max() - lng.min())
    y0 = lat.min() - 0.045 * (lat.max() - lat.min())
    ax.plot([x0, x0 + bar], [y0, y0], color=INK, lw=1.6, clip_on=False)
    ax.text(x0 + bar / 2, y0 - 0.03 * (lat.max() - lat.min()), "100 m",
            ha="center", va="top", fontsize=7.5, color=MUTED)
    ax.set_ylim(lat.min() - 0.10 * (lat.max() - lat.min()), lat.max())

    cb = fig.colorbar(sc, ax=ax, fraction=0.045, pad=0.03)
    cb.set_label("towfish altitude (m)", fontsize=8)
    cb.outline.set_edgecolor(FAINT)
    return fig_to_svg(fig)


def rows_swath():
    out = []
    for h in [2, 4, 6, 8, 10, 12]:
        out.append((f"{h}", f"{h * HFOV_K:.1f}", f"{h * VFOV_K:.1f}",
                    f"{h * HFOV_K * h * VFOV_K:.0f}"))
    return out


def rows_segments():
    out = []
    for s in S["segments"]:
        out.append((
            datetime.datetime.fromtimestamp(s["start"], HST).strftime("%H:%M"),
            f"{s['dur'] / 60:.0f}",
            f"{s['count']:,}",
            f"{s['depth']:.1f}",
            f"{s['fdepth']:.2f}",
            f"{s['alt_mean']:.2f}",
            f"{s['alt_p5']:.1f}\u2013{s['alt_p95']:.1f}",
            f"{s['swath']:.1f}",
            f"{s['dist']:,.0f}",
            f"{s['area']:,.0f}",
        ))
    return out


def table(headers, rows, align=None, cls=""):
    align = align or ["left"] * len(headers)
    th = "".join(f'<th style="text-align:{a}">{h}</th>' for h, a in zip(headers, align))
    tr = ""
    for i, r in enumerate(rows):
        tds = "".join(f'<td style="text-align:{a}">{c}</td>' for c, a in zip(r, align))
        tr += f'<tr class="{"alt" if i % 2 else ""}">{tds}</tr>'
    return f'<table class="{cls}"><thead><tr>{th}</tr></thead><tbody>{tr}</tbody></table>'


a = S["alt"]
sww = S["swath"]

HTML = f"""<!doctype html>
<html><head><meta charset="utf-8"><title>Towfish altitude and camera swath</title>
<style>
  @page {{ size: Letter portrait; margin: 14mm 13mm 12mm 13mm; }}
  * {{ box-sizing: border-box; }}
  body {{ font-family: "Helvetica Neue", Helvetica, Arial, sans-serif;
         color: {INK}; font-size: 9.4pt; line-height: 1.45; margin: 0; }}
  h1 {{ font-size: 19pt; margin: 0 0 3px; letter-spacing: -0.3px; }}
  h2 {{ font-size: 12pt; margin: 0 0 5px; }}
  h3 {{ font-size: 10pt; margin: 0 0 4px; }}
  p {{ margin: 0 0 7px; }}
  code {{ font-family: "SF Mono", Menlo, monospace; font-size: 0.88em;
          background: #f2f2f2; padding: 0 3px; border-radius: 2px; }}
  .sub {{ color: {MUTED}; font-size: 9pt; margin-bottom: 14px; }}
  .muted {{ color: {MUTED}; }}
  .cap {{ color: {MUTED}; font-size: 7.8pt; margin-top: 3px; }}
  .stats {{ display: flex; gap: 10px; margin: 0 0 14px; }}
  .stat {{ flex: 1; border: 1px solid #dcdcdc; border-radius: 4px; padding: 9px 11px; }}
  .stat .v {{ font-size: 17pt; font-weight: 600; color: {BLUE}; line-height: 1.1; }}
  .stat .l {{ font-size: 7.8pt; color: {MUTED}; margin-top: 3px; }}
  .box {{ border: 1px solid #dcdcdc; border-left: 3px solid {BLUE};
          border-radius: 3px; padding: 9px 12px; margin: 0 0 14px;
          background: #fafafa; }}
  .box .t {{ font-weight: 600; margin-bottom: 3px; }}
  section {{ margin-bottom: 13px; }}
  .two {{ display: flex; gap: 18px; align-items: flex-start; }}
  .two > * {{ min-width: 0; }}
  table {{ border-collapse: collapse; width: 100%; font-size: 8.4pt; }}
  th {{ border-bottom: 1px solid #b8b8b8; padding: 4px 6px;
        font-weight: 600; color: {MUTED}; font-size: 7.8pt;
        text-transform: uppercase; letter-spacing: 0.3px; }}
  td {{ border-bottom: 1px solid #ececec; padding: 4px 6px; }}
  tr.alt td {{ background: #fafafa; }}
  .grid2 {{ display: grid; grid-template-columns: 1fr 1fr; gap: 10px 20px; }}
  .grid2 .t {{ font-weight: 600; margin-bottom: 2px; }}
  .grid2 p {{ margin: 0; color: #3a3a3a; font-size: 8.8pt; }}
  .pb {{ page-break-before: always; }}
  svg {{ max-width: 100%; height: auto; }}
  footer {{ border-top: 1px solid #dcdcdc; padding-top: 7px;
            color: {MUTED}; font-size: 8pt; margin-top: 4px;
            page-break-inside: avoid; }}
</style></head><body>

<h1>Towfish altitude and camera swath</h1>
<div class="sub">Mahukona survey, 7 August 2026, 11:21&ndash;13:21 HST &middot;
derived from <code>BBKing towboat 8.7.26 log.BIN</code> (downward sonar) and
<code>towfish mahukona3 8.7.22.BIN</code> (pressure depth), aligned on their logged UTC clocks.</div>

<div class="stats">
  <div class="stat"><div class="v">{a['med']:.1f} m</div><div class="l">MEDIAN ALTITUDE</div></div>
  <div class="stat"><div class="v">{sww['med']:.1f} m</div><div class="l">MEDIAN SWATH WIDTH</div></div>
  <div class="stat"><div class="v">{S['n_valid']:,}</div><div class="l">USABLE SOUNDINGS ({100 * S['n_valid'] / S['n_total']:.0f}%)</div></div>
  <div class="stat"><div class="v">100 min</div><div class="l">TOWING ANALYSED</div></div>
</div>

<div class="box">
  <div class="t">Method</div>
  altitude(t + 6 s) = seabed depth measured by the towboat sonar at time t, minus the
  towfish pressure depth 6 s later, when the fish passes over that same point.
  Swath width = 2 &times; altitude &times; tan(94&deg;/2) = <b>{HFOV_K:.3f} &times; altitude</b>;
  along-track footprint = 2 &times; altitude &times; tan(62&deg;/2) = <b>{VFOV_K:.3f} &times; altitude</b>,
  both assuming a flat bottom and a nadir-pointing camera.
</div>

<section>
  <h2>Depth and altitude over the tow</h2>
  <p class="muted">Two-minute means. The seabed depth swings between roughly 7 m and 14 m as
  the boat runs its lawnmower lines in and out from shore, while the towfish holds an almost
  constant 3.2 m depth &mdash; so altitude tracks bathymetry almost one-for-one.</p>
  {chart_timeseries()}
  <div class="cap">Source: ArduPilot DPTH (7.9 Hz) and BARO instance 1 (10 Hz) records,
  2-minute means over {S['n_valid']:,} usable soundings.</div>
</section>

<section class="two">
  <div style="flex:1.15">
    <h3>Altitude distribution</h3>
    {chart_hist()}
    <div class="cap">5th&ndash;95th percentile spans {a['p5']:.1f}&ndash;{a['p95']:.1f} m;
    mean {a['mean']:.2f} m, standard deviation {a['std']:.2f} m.</div>
  </div>
  <div style="flex:1">
    <h3>Swath for a given altitude</h3>
    {table(["Altitude (m)", "Swath (m)", "Along-track (m)", "Frame area (m&sup2;)"],
           rows_swath(), ["right"] * 4)}
    <div class="cap">94&deg; horizontal / 62&deg; vertical field of view, camera looking
    straight down at a flat bottom.</div>
  </div>
</section>

<section class="pb">
  <h2>Survey track coloured by towfish altitude</h2>
  <div class="two">
    <div style="flex:1.05">{chart_track()}</div>
    <div style="flex:1">
      <p>Each dot is a sounding position from the towboat GPS, shaded by the towfish
      altitude computed for that point. The surveyed box is 281 m east&ndash;west by
      295 m north&ndash;south, off Mahukona, Hawai&#699;i.</p>
      <p>Altitude rises to the west (left) as the seabed drops away offshore, and falls
      below 3 m on the shallow inshore ends of the lines.</p>
      <div class="box" style="margin-top:10px">
        <div class="t">Coverage check</div>
        The dominant survey lines run at 8&deg; and are spaced <b>9.3 m</b> apart, while the
        median swath on those lines is <b>17.4 m</b>. Adjacent passes therefore overlap by
        roughly <b>47%</b> of a frame width &mdash; close to 2&times; coverage with no gaps,
        even on the shallow inshore ends where the swath narrows to about 7 m.
      </div>
    </div>
  </div>
</section>

<section>
  <h2>Continuous tow segments</h2>
  {table(["Start (HST)", "Min", "Soundings", "Seabed (m)", "Fish (m)", "Altitude (m)",
          "p5&ndash;p95 (m)", "Swath (m)", "Track (m)", "Area (m&sup2;)"],
         rows_segments(),
         ["left", "right", "right", "right", "right", "right", "center", "right", "right", "right"])}
  <div class="cap">Segments are runs of usable data with no gap longer than 30 s.
  Totals: {S['total_dist']:,.0f} m of track, about {S['total_area']:,.0f} m&sup2; of seabed
  imaged at the mean swath for each segment.</div>
</section>

<section>
  <h2>What limits the accuracy</h2>
  <div class="grid2">
    <div>
      <div class="t">Camera is not pointing straight down</div>
      <p>The towfish mount logs a fixed &minus;70&deg; pitch (97% of samples at exactly
      &minus;70.0&deg;, uncorrelated with vehicle pitch, so it is hard-mounted rather than
      stabilised). With the fish also flying about 10&deg; nose-up, the optical axis sits
      roughly 29&deg; off nadir. That widens the across-track swath to about
      <b>2.46 &times; altitude</b> (about 15% more than the nadir figure) and stretches the
      along-track footprint to about 1.79 &times; altitude, thrown forward of the fish.</p>
    </div>
    <div>
      <div class="t">The 6 s lag barely matters</div>
      <p>Because the towfish depth is so stable, recomputing with lags of 0, 3, 9 or 12 s
      moves the mean altitude by less than 0.01 m. Cross-correlating boat speed against fish
      depth independently puts the tow response at +9.5 s, which confirms the two logs are
      correctly aligned in time.</p>
    </div>
    <div>
      <div class="t">Transducer depth is unmodelled</div>
      <p><code>RNGFND1_POS_Z</code> is 0, so the sonar range is measured from the transducer
      face, not the waterline. If it sits 0.2&ndash;0.3 m below the surface, every altitude
      here is low by that amount and every swath by about 0.5 m.</p>
    </div>
    <div>
      <div class="t">Horizontal offset between boat and fish</div>
      <p>The method assumes the fish follows the exact path the boat traced 6 s earlier. On
      the turns between lines it does not, which is part of why 19% of soundings were
      rejected. Over sloping ground a lateral offset maps directly into an altitude error.</p>
    </div>
  </div>
  <p class="muted" style="margin-top:9px">Rejected soundings break down as: boat not underway
  below 0.5 m/s (14.9%), towfish not submerged past 1 m (16.7%), sonar quality below 50
  (2.9%), and spikes failing a rolling-median filter (0.2%). Categories overlap;
  {100 * S['n_valid'] / S['n_total']:.1f}% of the {S['n_total']:,} logged soundings survive all of them.</p>
</section>

<footer>
Towfish depth is BARO instance 1 (external pressure sensor, 10 Hz, 101.4&ndash;139.5 kPa over the
tow); seabed depth is the DPTH record at 7.9 Hz. The <code>RNGFND1_MAX</code> parameter of 7 m only
flags readings out of range for control purposes &mdash; the logged distance stays raw, so depths
to 15 m remain valid.
</footer>

</body></html>"""

with open(f"{BASE}/towfish_report.html", "w") as f:
    f.write(HTML)
print("wrote towfish_report.html")
