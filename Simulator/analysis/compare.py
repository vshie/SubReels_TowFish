#!/usr/bin/env python3
"""Score a simulation run against the real 4 Nov tow.

    python3 analysis/compare.py runs/<id>

Two separate score blocks, by design (README "Not cheating"):

* STRAIGHT LEGS -- the only block any estimate/ASSUMED constant may be
  checked against: median pitch, depth, wing collective, layback/lag,
  away from every turn.
* TURNS (held out) -- per boat turn: seconds after the turn starts until
  |roll| passes 20/45/90 deg, whether the fish surfaced, when the wings
  saturated. This block judges the model; nothing may be tuned to it.

Physical state comes from truth.csv; the autopilot's view (ATT, wing PWM,
mode) from the SITL BIN when present. The real fish's attitude and depth are
read here and nowhere else.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from towsim.logs import fish_to_boat_offset, load_boat, load_fish  # noqa: E402

TURN_PAD_S = 60.0
WING_SAT_US = 400.0


def boat_turns(boat, t0, t1):
    """Start/end boat times of heading changes > 60 deg within 15 s."""
    t = np.asarray(boat["ATT"]["t"])
    yaw = np.degrees(np.unwrap(np.radians(np.asarray(boat["ATT"]["Yaw"]))))
    gt = np.asarray(boat["GPS"]["t"])
    spd = np.asarray(boat["GPS"]["Spd"])
    turns = []
    tt = t0
    while tt < t1 - 15:
        d = np.interp(tt + 15, t, yaw) - np.interp(tt, t, yaw)
        if abs(d) > 60 and np.interp(tt, gt, spd) > 0.4:
            ts = tt
            while ts < tt + 15 and abs(np.interp(ts + 1, t, yaw) - np.interp(ts, t, yaw)) < 5:
                ts += 0.2
            te = ts
            while te < ts + 60 and abs(np.interp(te + 3, t, yaw) - np.interp(te, t, yaw)) > 6:
                te += 0.5
            # Net heading change over the whole manoeuvre (a hairpin is two
            # 90s joined by a short crosslink, so look 40 s ahead).
            turns.append((ts, te, np.interp(ts + 40, t, yaw) - np.interp(ts, t, yaw)))
            tt = te + 20
        else:
            tt += 1
    return turns


def turn_metrics(t, roll, depth, coll, ts, window=70.0):
    m = (t >= ts) & (t <= ts + window)
    if not m.any():
        return None
    tw, ar, dp = t[m], np.abs(roll[m]), depth[m]

    def first(mask):
        k = np.where(mask)[0]
        return round(float(tw[k[0]] - ts), 1) if len(k) else None

    out = {"roll20": first(ar > 20), "roll45": first(ar > 45), "roll90": first(ar > 90),
           "max_roll": round(float(ar.max()), 0), "depth0": round(float(dp[0]), 2),
           "surfaced": bool((dp < 0.3).any() and dp[0] > 1.0)}
    if coll is not None:
        out["wing_sat"] = first(coll[m] >= WING_SAT_US)
    return out


def straight_metrics(t, pitch, depth, coll, turns, t0, t1):
    m = (t >= t0) & (t <= t1)
    for ts, te, _ in turns:
        m &= ~((t > ts - 10) & (t < te + TURN_PAD_S))
    m &= depth > 1.0
    if m.sum() < 50:
        return None
    out = {"samples": int(m.sum()),
           "pitch_median": round(float(np.median(pitch[m])), 1),
           "depth_median": round(float(np.median(depth[m])), 2)}
    if coll is not None:
        out["collective_median_us"] = round(float(np.median(coll[m])), 0)
    return out


def load_sim(run: Path, meta):
    import csv
    rows = list(csv.DictReader((run / "truth.csv").open()))
    f = lambda k: np.array([float(r[k]) for r in rows])  # noqa: E731
    t = f("t_boat")
    s5, s6 = f("servo5"), f("servo6")
    coll = np.where((s5 > 0) & (s6 > 0), ((s5 - 1500) + (1500 - s6)) / 2, np.nan)
    jig = f("jig") > 0
    out = {"t": t[~jig], "roll": f("roll")[~jig], "pitch": f("pitch")[~jig],
           "depth": f("depth")[~jig], "coll": coll[~jig], "layback": f("layback_m")[~jig],
           "attitude_source": "physics truth"}
    # Prefer the autopilot's own ATT for attitude: it carries the same
    # AHRS_TRIM the real fish applied, so pitch compares like for like.
    # SITL time is the physics clock (JSON lockstep), so boat time = t0 + TimeUS.
    bin_path = run / "sitl.BIN"
    if bin_path.exists():
        from pymavlink import mavutil
        m = mavutil.mavlink_connection(str(bin_path))
        at, ar, ap = [], [], []
        while True:
            r = m.recv_match(type="ATT")
            if r is None:
                break
            at.append(meta["t0_boat"] + r.TimeUS / 1e6)
            ar.append(r.Roll)
            ap.append(r.Pitch)
        if at:
            out["roll"] = np.interp(out["t"], at, ar)
            out["pitch"] = np.interp(out["t"], at, ap)
            out["attitude_source"] = "SITL ATT (autopilot estimate, AHRS_TRIM applied)"
    return out


def load_real(t0, t1):
    boat, fish = load_boat(), load_fish()
    off = fish_to_boat_offset(boat, fish)
    at = np.asarray(fish["ATT"]["t"]) + off
    ct = np.asarray(fish["CTUN"]["t"]) + off
    ot = np.asarray(fish["RCOU"]["t"]) + off
    m = (at >= t0 - 5) & (at <= t1 + 75)
    t = at[m]
    s5 = np.interp(t, ot, fish["RCOU"]["C5"])
    s6 = np.interp(t, ot, fish["RCOU"]["C6"])
    return boat, {"t": t, "roll": np.asarray(fish["ATT"]["Roll"])[m],
                  "pitch": np.asarray(fish["ATT"]["Pitch"])[m],
                  "depth": -np.interp(t, ct, fish["CTUN"]["Alt"]),
                  "coll": ((s5 - 1500) + (1500 - s6)) / 2}


def fmt(v):
    return "-" if v is None else (f"{v:g}" if isinstance(v, (int, float)) else str(v))


def main(argv) -> int:
    run = Path(argv[1]).resolve()
    meta = json.loads((run / "meta.json").read_text())
    t0, t1 = meta["t_start_boat"], meta["t_end_boat"]
    sim = load_sim(run, meta)
    t1 = min(t1, float(sim["t"][-1]))
    boat, real = load_real(t0, t1)
    turns = boat_turns(boat, t0, t1 - 30)

    lines = [f"# Compare {meta['run_id']}", "",
             f"Window {meta['hst_start']}-{meta['hst_end']} HST, pilot `{meta['args']['pilot']}`, "
             f"config `{meta['config_sha']}`, git `{meta['git_sha']}`.", ""]
    overrides = [r for r in meta["config"] if r[2] == "OVERRIDE"]
    if overrides:
        lines += ["Config overrides for this run: "
                  + ", ".join(f"`{r[0]}={r[1]}`" for r in overrides), ""]

    lines += ["## Straight legs (calibration may look here)", "",
              "| | samples | median pitch (deg) | median depth (m) | median wing collective (us) |",
              "|---|---|---|---|---|"]
    for name, d in (("real", real), ("sim", sim)):
        s = straight_metrics(d["t"], d["pitch"], d["depth"], d["coll"], turns, t0, t1)
        if s:
            lines.append(f"| {name} | {s['samples']} | {s['pitch_median']} | {s['depth_median']} | "
                         f"{fmt(s.get('collective_median_us'))} |")
        else:
            lines.append(f"| {name} | too few straight samples deeper than 1 m | | | |")
    lay = sim["layback"]
    lines += ["", f"Sim attitude from {sim['attitude_source']}; real from the fish's ATT.",
              f"Sim horizontal layback (median): {np.median(lay):.1f} m "
              "(real: ~15-20 m inferred from the ~18 s turn lag).", ""]

    lines += ["## Turns (held out -- judge only, never tune)", "",
              "| boat turn | dHdg | who | depth0 | roll>20 | roll>45 | roll>90 | max roll | surfaced | wings sat |",
              "|---|---|---|---|---|---|---|---|---|---|"]
    for ts, te, dh in turns:
        for name, d in (("real", real), ("sim", sim)):
            mt = turn_metrics(d["t"], d["roll"], d["depth"], d["coll"], ts)
            if mt is None:
                continue
            lines.append(f"| {ts:.0f} | {dh:+.0f} | {name} | {mt['depth0']} | {fmt(mt['roll20'])} | "
                         f"{fmt(mt['roll45'])} | {fmt(mt['roll90'])} | {fmt(mt['max_roll'])} | "
                         f"{'yes' if mt['surfaced'] else 'no'} | {fmt(mt.get('wing_sat'))} |")
    if not turns:
        lines.append("| (no boat turns in this window) | | | | | | | | | |")
    lines += ["", "Times are seconds after the boat starts turning."]
    (run / "compare.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return 0
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for name, d, col in (("real", real, "#2a78d6"), ("sim", sim, "#eb6834")):
        m = (d["t"] >= t0) & (d["t"] <= t1)
        x = d["t"][m] - t0
        ax[0].plot(x, d["roll"][m], color=col, lw=1.4, label=name)
        ax[1].plot(x, d["depth"][m], color=col, lw=1.4)
        ax[2].plot(x, d["coll"][m], color=col, lw=1.4)
    for ts, te, _ in turns:
        for a in ax:
            a.axvspan(ts - t0, te - t0, color="#ecebe7", zorder=0)
    ax[0].set_ylabel("roll (deg)")
    ax[0].legend(frameon=False)
    ax[1].set_ylabel("depth (m)")
    ax[1].invert_yaxis()
    ax[2].set_ylabel("wing collective (us)\n+ = down")
    ax[2].set_xlabel(f"seconds after {meta['hst_start']} HST (grey = boat turning)")
    fig.tight_layout()
    fig.savefig(run / "compare.png", dpi=130)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
