#!/usr/bin/env python3
"""Vertical rates the towfish actually achieved on 4 Nov, from its own log.

Splits the fish log into segments where the mode and the vertical stick
(RCIN C3) stay in one bucket for at least MIN_SEG_S, and fits a depth rate
to each (CTUN depth, skipping the first SETTLE_S as transient). The stick
buckets are full up (>= 1700), part up (1600-1699), neutral (1400-1599),
part down (1301-1399) and full down (<= 1300). The fish log cannot tell the
operator's stick from SURF TRACK's override; both arrive as RCIN.

Per segment it also records the boat's GPS speed, the start depth, the
median wing collective (+ = down, as analysis/compare.py) and the peak roll,
so the table says whether a climb was speed-, wing- or roll-limited.

Writes runs/rates/log_rates.json and prints a summary.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from towsim.logs import (FISH_BIN, fish_to_boat_offset, load_boat,  # noqa: E402
                         read_bin)

MODES = {0: "STABILIZE", 2: "ALT_HOLD", 19: "MANUAL"}
MIN_SEG_S = 3.0
SETTLE_S = 1.0
SURFACE_M = 0.3
OUT = Path(__file__).resolve().parent.parent / "runs" / "rates" / "log_rates.json"


def stick_bucket(pwm: float) -> str:
    if pwm >= 1700:
        return "full_up"
    if pwm >= 1600:
        return "part_up"
    if pwm > 1400:
        return "neutral"
    if pwm > 1300:
        return "part_down"
    return "full_down"


def load():
    fish = read_bin(FISH_BIN, {"CTUN": ["Alt"], "MODE": ["Mode"], "RCIN": ["C3"],
                               "RTC": ["Epoch"], "RCOU": ["C5", "C6"],
                               "ATT": ["Roll"], "PARM": ["Name", "Value"]})
    boat = load_boat()
    off = fish_to_boat_offset(boat, fish)
    t = np.asarray(fish["CTUN"]["t"])
    depth = -np.asarray(fish["CTUN"]["Alt"], dtype=float)
    mt, mm = np.asarray(fish["MODE"]["t"]), np.asarray(fish["MODE"]["Mode"])
    idx = np.searchsorted(mt, t, side="right") - 1
    mode = np.where(idx >= 0, mm[np.clip(idx, 0, None)], -1)
    rc3 = np.interp(t, fish["RCIN"]["t"], fish["RCIN"]["C3"])
    s5 = np.interp(t, fish["RCOU"]["t"], fish["RCOU"]["C5"])
    s6 = np.interp(t, fish["RCOU"]["t"], fish["RCOU"]["C6"])
    coll = ((s5 - 1500) + (1500 - s6)) / 2
    roll = np.interp(t, fish["ATT"]["t"], fish["ATT"]["Roll"])
    g = boat["GPS"]
    spd = np.interp(t + off, g["t"], g["Spd"])
    parm = dict(zip(fish["PARM"]["Name"], fish["PARM"]["Value"]))
    return dict(t=t, depth=depth, mode=mode, rc3=rc3, coll=coll, roll=roll,
                spd=spd), parm


def segments(d):
    key = [(int(m), stick_bucket(p)) for m, p in zip(d["mode"], d["rc3"])]
    out, start = [], 0
    for i in range(1, len(key) + 1):
        if i == len(key) or key[i] != key[start]:
            t0, t1 = d["t"][start], d["t"][i - 1]
            if t1 - t0 >= MIN_SEG_S:
                m = slice(start, i)
                tt = d["t"][m]
                keep = tt >= t0 + SETTLE_S
                z = d["depth"][m][keep]
                if keep.sum() >= 10:
                    rate = float(np.polyfit(tt[keep], z, 1)[0])  # + = deeper
                    out.append({
                        "mode": MODES.get(key[start][0], str(key[start][0])),
                        "stick": key[start][1],
                        "t0": round(float(t0), 1), "dur_s": round(float(t1 - t0), 1),
                        "rate_mps": round(rate, 3),
                        "depth_start": round(float(d["depth"][m][0]), 2),
                        "depth_end": round(float(d["depth"][m][-1]), 2),
                        "boat_mps": round(float(np.median(d["spd"][m])), 2),
                        "coll_us": round(float(np.median(d["coll"][m])), 0),
                        "coll_p95_us": round(float(np.percentile(np.abs(d["coll"][m]), 95)), 0),
                        "roll_peak_deg": round(float(np.max(np.abs(d["roll"][m]))), 1),
                        "hit_surface": bool(np.min(d["depth"][m]) < SURFACE_M),
                    })
            start = i
    return out


def peak_rate(d, sign, window_s=MIN_SEG_S):
    """Fastest rate held over window_s anywhere underway (boat > 0.5 m/s)."""
    t, z = d["t"], d["depth"]
    best = None
    j = 0
    for i in range(len(t)):
        while t[i] - t[j] > window_s:
            j += 1
        if t[i] - t[j] < window_s * 0.9 or d["spd"][i] < 0.5:
            continue
        r = (z[i] - z[j]) / (t[i] - t[j])
        if sign * r > (sign * best["rate_mps"] if best else 0):
            best = {"rate_mps": round(float(r), 3), "t": round(float(t[i]), 1),
                    "depth": round(float(z[i]), 2), "mode": MODES.get(int(d["mode"][i])),
                    "stick": stick_bucket(d["rc3"][i]), "boat_mps": round(float(d["spd"][i]), 2)}
    return best


def summarise(segs):
    groups = {}
    for s in segs:
        if s["boat_mps"] < 0.5:
            continue
        groups.setdefault((s["mode"], s["stick"]), []).append(s)
    rows = []
    for (mode, stick), ss in sorted(groups.items()):
        r = np.array([s["rate_mps"] for s in ss])
        w = np.array([s["dur_s"] for s in ss])
        rows.append({"mode": mode, "stick": stick, "segments": len(ss),
                     "seconds": round(float(w.sum()), 0),
                     "rate_median": round(float(np.median(r)), 3),
                     "rate_fastest_up": round(float(r.min()), 3),
                     "rate_fastest_down": round(float(r.max()), 3),
                     "boat_mps_median": round(float(np.median([s["boat_mps"] for s in ss])), 2),
                     "coll_p95_us_median": round(float(np.median([s["coll_p95_us"] for s in ss])), 0)})
    return rows


def main() -> int:
    d, parm = load()
    segs = segments(d)
    rows = summarise(segs)
    out = {
        "source": FISH_BIN.name,
        "params": {k: parm.get(k) for k in ("PILOT_SPEED_UP", "PILOT_SPEED_DN",
                                            "PILOT_ACCEL_Z", "PSC_D_POS_P", "THR_DZ",
                                            "MOT_THST_EXPO")},
        "rate_sign": "+ = deeper (dive), - = shallower (climb)",
        "summary": rows,
        "peak_climb_3s": peak_rate(d, -1),
        "peak_dive_3s": peak_rate(d, +1),
        "segments": segs,
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(out, indent=1))
    print("params as flown:", out["params"])
    print(f"{'mode':10} {'stick':10} {'n':>3} {'sec':>5} {'median':>7} {'up max':>7} "
          f"{'dn max':>7} {'boat':>5} {'coll p95':>8}")
    for r in rows:
        print(f"{r['mode']:10} {r['stick']:10} {r['segments']:3d} {r['seconds']:5.0f} "
              f"{r['rate_median']:7.3f} {r['rate_fastest_up']:7.3f} {r['rate_fastest_down']:7.3f} "
              f"{r['boat_mps_median']:5.2f} {r['coll_p95_us_median']:8.0f}")
    print("peak 3 s climb:", out["peak_climb_3s"])
    print("peak 3 s dive: ", out["peak_dive_3s"])
    print("wrote", OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
