#!/usr/bin/env python3
"""Analyse bbking 241 + towfish 91. Prints a JSON summary for the canvas."""

from datetime import datetime, timedelta, timezone
import json

import numpy as np

IN = "/Users/tonywhite/Documents/SubReels_TowFish/tools/mission_241.npz"
OUT = "/Users/tonywhite/Documents/SubReels_TowFish/tools/mission_241_summary.json"

HST = timezone(timedelta(hours=-10))
LAG = 7.0  # s, configured layback 7 m at ~1 m/s
MIN_SPD = 0.4
RFND_GOOD = 4
QMIN = 50
MAX_REASONABLE_DEPTH = 40.0
TARGET_ALT = 5.0

ROVER = {0: "MANUAL", 1: "ACRO", 3: "STEERING", 4: "HOLD", 5: "LOITER",
         10: "AUTO", 11: "RTL", 15: "GUIDED"}
SUB = {0: "STABILIZE", 2: "ALT_HOLD", 19: "MANUAL"}


def circ_corr(a, b):
    """Mean cosine of heading difference (both degrees)."""
    d = np.deg2rad(a - b)
    return float(np.mean(np.cos(d)))


def rolling_median(x, win):
    if len(x) < win:
        return x.copy()
    half = win // 2
    pad = np.pad(x, (half, half), mode="edge")
    return np.median(np.lib.stride_tricks.sliding_window_view(pad, win), axis=1)


def downsample(t, y, step):
    if len(t) == 0:
        return t, y
    idx = np.arange(0, len(t), max(1, int(round(step / np.median(np.diff(t))))))
    return t[idx], y[idx]


def pct(x, ps):
    if len(x) == 0:
        return [None] * len(ps)
    return [float(v) for v in np.percentile(x, ps)]


def main():
    D = np.load(IN, allow_pickle=True)

    boat_off = float(D["boat_rtc_epoch"][0] - D["boat_rtc_tus"][0])
    boat_utc = lambda tus: tus + boat_off

    bt = D["boat_gps_t"]
    bspd = D["boat_gps_spd"]
    bcrs = D["boat_gps_crs"]
    blat = D["boat_gps_lat"]
    blng = D["boat_gps_lng"]
    byaw = np.interp(bt, D["boat_att_t"], D["boat_att_yaw"]) if len(D["boat_att_t"]) else bcrs

    ft = D["fish_att_t"]
    fyaw = D["fish_att_yaw"]
    froll = D["fish_att_roll"]
    fpitch = D["fish_att_pitch"]
    fdesroll = D["fish_att_desroll"]
    fbaro_t = D["fish_baro_t"]
    fdepth = -D["fish_baro_alt"]  # baro Alt is negative depth
    fmode_t = D["fish_mode_t"]
    fmode = D["fish_mode"].astype(int)
    rcin_t = D["fish_rcin_t"]
    rcin_c3 = D["fish_rcin_c3"]
    rcou_c5 = D["fish_rcou_c5"]
    rcou_c6 = D["fish_rcou_c6"]
    rcou_t = D["fish_rcou_t"]

    # --- time alignment: heading cross-correlation at 1 Hz ---
    t0 = max(bt[0], 0)
    t1 = bt[-1]
    grid = np.arange(t0, t1, 1.0)
    boat_hdg = np.interp(grid, bt, byaw)
    boat_spd_g = np.interp(grid, bt, bspd)
    underway = boat_spd_g >= MIN_SPD

    # candidate offsets: fish_t = boat_t + offset
    # fish starts ~50s, boat ~4004s; search offset in [-5000, 2000]
    offsets = np.arange(-4800, 800, 2.0)
    scores = []
    for off in offsets:
        fish_hdg = np.interp(grid + off, ft, fyaw, left=np.nan, right=np.nan)
        ok = underway & np.isfinite(fish_hdg)
        if ok.sum() < 600:
            scores.append(-9)
            continue
        scores.append(circ_corr(boat_hdg[ok], fish_hdg[ok]))
    scores = np.array(scores)
    best_i = int(np.argmax(scores))
    offset = float(offsets[best_i])
    # refine ±2 s at 0.2 s
    fine = np.arange(offset - 2, offset + 2.01, 0.2)
    fine_s = []
    for off in fine:
        fish_hdg = np.interp(grid + off, ft, fyaw, left=np.nan, right=np.nan)
        ok = underway & np.isfinite(fish_hdg)
        fine_s.append(circ_corr(boat_hdg[ok], fish_hdg[ok]) if ok.sum() > 600 else -9)
    offset = float(fine[int(np.argmax(fine_s))])
    align_score = float(max(fine_s))
    print("ALIGN offset fish-boat = %.2f s   circ-corr = %.3f" % (offset, align_score))
    print("  meaning: fish_TimeUS = boat_TimeUS + %.1f" % offset)
    print("  boat AUTO 4048 -> fish %.1f" % (4048.7 + offset))

    def fish_at_boat(boat_tus, fish_t, fish_y):
        return np.interp(boat_tus + offset, fish_t, fish_y, left=np.nan, right=np.nan)

    # --- boat AUTO window ---
    bmode_t = D["boat_mode_t"]
    bmode = D["boat_mode"].astype(int)
    auto_start = None
    auto_end = None
    for i, m in enumerate(bmode):
        if m == 10 and auto_start is None:
            auto_start = float(bmode_t[i])
        if auto_start is not None and m != 10:
            auto_end = float(bmode_t[i])
            break
    if auto_end is None:
        auto_end = float(bt[-1])
    print("BOAT AUTO %.1f .. %.1f  (%.1f min)" % (
        auto_start, auto_end, (auto_end - auto_start) / 60))

    # waypoint reaches from MSG
    msg_t = D["boat_msg_t"]
    msg_txt = D["boat_msg_txt"]
    reaches = []
    for t, txt in zip(msg_t, msg_txt):
        s = str(txt)
        if s.startswith("Reached waypoint"):
            try:
                n = int(s.split("#")[1].split()[0])
                reaches.append((float(t), n))
            except Exception:
                pass
        elif s.startswith("Mission:") and "WP" in s:
            pass
    print("waypoint reaches:", len(reaches),
          "first", reaches[0] if reaches else None,
          "last", reaches[-1] if reaches else None)

    # jump in mission: look for skipped WP numbers
    jumps = []
    for i in range(1, len(reaches)):
        if reaches[i][1] - reaches[i - 1][1] > 2:
            jumps.append((reaches[i - 1], reaches[i]))
    print("WP jumps:", jumps)

    # --- RFND quality ---
    rt, rd, rs, rq = D["boat_rfnd_t"], D["boat_rfnd_d"], D["boat_rfnd_stat"], D["boat_rfnd_q"]
    in_auto = (rt >= auto_start) & (rt <= auto_end)
    print("\nRFND during AUTO:")
    print("  n", int(in_auto.sum()))
    for st in range(5):
        print("  stat", st, int(((rs == st) & in_auto).sum()))
    good = in_auto & (rs == RFND_GOOD) & (rq >= QMIN) & (rd > 0.3) & (rd < MAX_REASONABLE_DEPTH)
    over7 = in_auto & (rd > 7) & (rd < MAX_REASONABLE_DEPTH)
    print("  good (stat4 & q>=50 & 0.3-40m)", int(good.sum()),
          "pct", 100 * good.mean())
    print("  dist>7m (any, <40m)", int(over7.sum()))
    print("  dist>7m AND good", int((good & (rd > 7)).sum()))
    print("  dist percentiles all-auto", pct(rd[in_auto], [0, 5, 25, 50, 75, 95, 100]))
    print("  dist percentiles GOOD", pct(rd[good], [0, 5, 25, 50, 75, 95, 100]))
    print("  quality percentiles auto", pct(rq[in_auto], [0, 25, 50, 75, 100]))

    # DPTH vs RFND
    dt, dd = D["boat_dpth_t"], D["boat_dpth_d"]
    d_auto = (dt >= auto_start) & (dt <= auto_end)
    print("DPTH auto percentiles", pct(dd[d_auto], [0, 5, 25, 50, 75, 95, 100]))

    # --- aligned fish during AUTO ---
    # sample on DPTH timeline ( ~8 Hz )
    t = dt
    auto = (t >= auto_start) & (t <= auto_end)
    depth_raw = dd
    # despike DPTH with RFND quality via nearest
    ri = np.searchsorted(rt, t).clip(0, len(rt) - 1)
    rfnd_here = rd[ri]
    q_here = rq[ri]
    st_here = rs[ri]
    spd = np.interp(t, bt, bspd)
    lat = np.interp(t, bt, blat)
    lng = np.interp(t, bt, blng)

    fish_d = fish_at_boat(t + LAG, fbaro_t, fdepth)
    fish_roll = fish_at_boat(t, ft, froll)
    fish_pitch = fish_at_boat(t, ft, fpitch)
    fish_yaw = fish_at_boat(t, ft, fyaw)
    fish_desroll = fish_at_boat(t, ft, fdesroll)
    c3 = fish_at_boat(t, rcin_t, rcin_c3)
    v5 = fish_at_boat(t, rcou_t, rcou_c5)
    v6 = fish_at_boat(t, rcou_t, rcou_c6)

    # fish mode at boat time
    fmode_on_boat = np.full(len(t), np.nan)
    # map fish mode timeline to boat
    fm_boat_t = fmode_t - offset
    for i in range(len(fm_boat_t)):
        t0m = fm_boat_t[i]
        t1m = fm_boat_t[i + 1] if i + 1 < len(fm_boat_t) else t[-1] + 1
        fmode_on_boat[(t >= t0m) & (t < t1m)] = fmode[i]

    # sounding quality
    med = rolling_median(depth_raw, 9)
    tol = np.maximum(1.5, 0.15 * med)
    clean = np.abs(depth_raw - med) <= tol
    good_s = (st_here == RFND_GOOD) & (q_here >= QMIN) & (rfnd_here > 0.3) & (rfnd_here < MAX_REASONABLE_DEPTH)
    underway_s = spd >= MIN_SPD
    fish_ok = np.isfinite(fish_d) & (fish_d >= 0.4)
    valid = auto & clean & good_s & underway_s & fish_ok
    alt = depth_raw - fish_d  # no offset

    print("\nVALID survey samples", int(valid.sum()), "of", int(auto.sum()),
          "(%.0f%%)" % (100 * valid.sum() / max(1, auto.sum())))
    print("seabed", pct(depth_raw[valid], [5, 25, 50, 75, 95]))
    print("fish depth", pct(fish_d[valid], [5, 25, 50, 75, 95]))
    print("altitude", pct(alt[valid], [5, 25, 50, 75, 95]))
    print(" |alt-5m| mean", float(np.mean(np.abs(alt[valid] - TARGET_ALT))) if valid.any() else None)
    print(" alt in 4-6m pct", float(100 * np.mean((alt[valid] >= 4) & (alt[valid] <= 6))) if valid.any() else None)
    print(" alt < 3m pct", float(100 * np.mean(alt[valid] < 3)) if valid.any() else None)
    print(" alt > 8m pct", float(100 * np.mean(alt[valid] > 8)) if valid.any() else None)

    # mode mix during valid
    if valid.any():
        modes_v = fmode_on_boat[valid]
        for m, name in SUB.items():
            print("  fish %s: %.1f%%" % (name, 100 * np.mean(modes_v == m)))
        print("  fish mode nan: %.1f%%" % (100 * np.mean(~np.isfinite(modes_v))))

    # --- roll events ---
    roll_abs = np.abs(fish_roll)
    # wrap roll to [-180,180] already
    roll_event = valid & (roll_abs >= 25)
    print("\nROLL |r|>=25 deg during valid: %.2f%%" % (100 * roll_event.mean() if valid.any() else 0))
    print("ROLL percentiles valid", pct(roll_abs[valid], [50, 75, 90, 95, 99, 100]))
    print("PITCH percentiles valid", pct(np.abs(fish_pitch[valid]), [50, 90, 95, 99]))

    # climb rate of fish (positive = descending, m/s)
    # use baro CRt if we stored it: fish_baro_crt is climb rate of Alt (negative depth), so -CRt is descent
    if "fish_baro_crt" in D.files:
        fcrt = fish_at_boat(t, fbaro_t, D["fish_baro_crt"])
        descent = -fcrt  # +ve = going deeper
    else:
        descent = np.gradient(fish_d, t)
    print("descent rate percentiles valid", pct(descent[valid], [5, 25, 50, 75, 95, 99]))

    # correlation: descent vs |roll|, descent vs boat speed
    if valid.sum() > 100:
        print("corr descent vs |roll|", float(np.corrcoef(descent[valid], roll_abs[valid])[0, 1]))
        print("corr descent vs boat spd", float(np.corrcoef(descent[valid], spd[valid])[0, 1]))
        print("corr fish depth vs boat spd", float(np.corrcoef(fish_d[valid], spd[valid])[0, 1]))

    # bins of descent rate vs roll and speed
    desc_bins = [(-99, -0.2), (-0.2, 0.2), (0.2, 0.5), (0.5, 1.0), (1.0, 99)]
    print("\ndescent bins (valid):")
    desc_table = []
    for lo, hi in desc_bins:
        m = valid & (descent >= lo) & (descent < hi)
        row = {
            "lo": lo, "hi": hi, "n": int(m.sum()),
            "roll_p90": float(np.percentile(roll_abs[m], 90)) if m.any() else None,
            "roll_p95": float(np.percentile(roll_abs[m], 95)) if m.any() else None,
            "spd_mean": float(np.mean(spd[m])) if m.any() else None,
            "pct_roll25": float(100 * np.mean(roll_abs[m] >= 25)) if m.any() else None,
        }
        desc_table.append(row)
        print("  %.1f..%.1f n=%d  spd=%.2f  |roll|p90=%.1f  |r|>=25: %.1f%%" % (
            lo, hi, row["n"], row["spd_mean"] or 0, row["roll_p90"] or 0, row["pct_roll25"] or 0))

    # RC3 activity
    c3_dev = np.abs(c3 - 1500)
    print("\nRC3 during valid:")
    print("  mean", float(np.nanmean(c3[valid])), "p5/p50/p95", pct(c3[valid], [5, 50, 95]))
    print("  |c3-1500|>=40 pct", float(100 * np.mean(c3_dev[valid] >= 40)))
    print("  |c3-1500|>=80 pct", float(100 * np.mean(c3_dev[valid] >= 80)))
    # unique-ish PWM values (coarse control)
    c3_r = np.round(c3[valid] / 5) * 5
    vals, cnts = np.unique(c3_r, return_counts=True)
    top = sorted(zip(cnts, vals), reverse=True)[:12]
    print("  top rounded PWM:", [(int(v), int(c), round(100 * c / valid.sum(), 1)) for c, v in top])

    # vertical thruster PWM
    print("RCOU C5/C6 (vertical) valid:")
    print("  C5", pct(v5[valid], [5, 50, 95]), "C6", pct(v6[valid], [5, 50, 95]))

    # --- identify tow-point adjustment: WP jump + speed dip ---
    # also a period of low speed in AUTO
    slow = auto & (spd < 0.25)
    # contiguous slow stretches > 30s
    slow_segs = []
    if slow.any():
        ds = np.diff(slow.astype(int), prepend=0, append=0)
        starts = np.where(ds == 1)[0]
        ends = np.where(ds == -1)[0]
        for s, e in zip(starts, ends):
            dur = t[min(e, len(t) - 1)] - t[s]
            if dur >= 20:
                slow_segs.append((float(t[s]), float(t[min(e, len(t) - 1)]), float(dur)))
    print("\nslow (<0.25 m/s) stretches in AUTO >=20s:", slow_segs[:15], "n", len(slow_segs))

    # split survey at the big WP jump if present
    split_t = jumps[0][1][0] if jumps else None
    print("split_t (mission jump)", split_t)

    def stats_for(mask, label):
        if mask.sum() < 50:
            print(label, "too few")
            return None
        a = alt[mask]
        fd = fish_d[mask]
        dp = depth_raw[mask]
        out = {
            "label": label,
            "n": int(mask.sum()),
            "dur_min": float((t[mask][-1] - t[mask][0]) / 60),
            "seabed_med": float(np.median(dp)),
            "seabed_p5": float(np.percentile(dp, 5)),
            "seabed_p95": float(np.percentile(dp, 95)),
            "fish_med": float(np.median(fd)),
            "fish_p5": float(np.percentile(fd, 5)),
            "fish_p95": float(np.percentile(fd, 95)),
            "alt_med": float(np.median(a)),
            "alt_p5": float(np.percentile(a, 5)),
            "alt_p95": float(np.percentile(a, 95)),
            "alt_mean": float(np.mean(a)),
            "err5_mae": float(np.mean(np.abs(a - TARGET_ALT))),
            "in_4_6": float(100 * np.mean((a >= 4) & (a <= 6))),
            "below3": float(100 * np.mean(a < 3)),
            "above8": float(100 * np.mean(a > 8)),
            "spd_mean": float(np.mean(spd[mask])),
            "roll_p90": float(np.percentile(roll_abs[mask], 90)),
            "roll_p95": float(np.percentile(roll_abs[mask], 95)),
            "pct_roll25": float(100 * np.mean(roll_abs[mask] >= 25)),
            "stabilize_pct": float(100 * np.nanmean(fmode_on_boat[mask] == 0)),
            "althold_pct": float(100 * np.nanmean(fmode_on_boat[mask] == 2)),
        }
        print(label, {k: (round(v, 2) if isinstance(v, float) else v) for k, v in out.items()})
        return out

    overall = stats_for(valid, "ALL valid AUTO")
    pre = post = None
    if split_t:
        pre = stats_for(valid & (t < split_t - 30), "PRE jump (tests)")
        post = stats_for(valid & (t > split_t + 30), "POST jump (survey)")

    # --- 1-minute series for charts ---
    t_auto = t[auto]
    utc0 = boat_utc(auto_start)
    def hhmm(tus):
        dt = datetime.fromtimestamp(boat_utc(tus), tz=HST)
        return dt.strftime("%H:%M")

    # 2-minute bins across AUTO
    bin_s = 120.0
    edges = np.arange(auto_start, auto_end + bin_s, bin_s)
    series = []
    for i in range(len(edges) - 1):
        m = valid & (t >= edges[i]) & (t < edges[i + 1])
        if m.sum() < 20:
            continue
        series.append({
            "t": hhmm(edges[i]),
            "tus": float(edges[i]),
            "seabed": round(float(np.median(depth_raw[m])), 2),
            "fish": round(float(np.median(fish_d[m])), 2),
            "alt": round(float(np.median(alt[m])), 2),
            "spd": round(float(np.median(spd[m])), 2),
            "roll": round(float(np.percentile(roll_abs[m], 90)), 1),
            "c3": round(float(np.median(c3[m])), 0),
        })

    # finer 30s for a representative "dive + roll" window: pick worst roll minute
    if valid.any():
        win = 30.0
        worst = None
        for tus in np.arange(auto_start, auto_end, 10):
            m = valid & (t >= tus) & (t < tus + win)
            if m.sum() < 10:
                continue
            score = float(np.percentile(roll_abs[m], 95)) + 5 * max(0, float(np.median(descent[m])))
            if worst is None or score > worst[0]:
                worst = (score, tus)
        dive_win = worst[1] if worst else auto_start + 600
    else:
        dive_win = auto_start

    # 2s series around worst window ±3 min
    w0, w1 = dive_win - 180, dive_win + 180
    step = 2.0
    event = []
    for tus in np.arange(w0, w1, step):
        m = (t >= tus) & (t < tus + step) & np.isfinite(fish_d)
        if m.sum() < 1:
            continue
        event.append({
            "s": round(float(tus - w0), 1),
            "seabed": round(float(np.median(depth_raw[m])), 2),
            "fish": round(float(np.nanmedian(fish_d[m])), 2),
            "alt": round(float(np.nanmedian((depth_raw - fish_d)[m])), 2) if np.isfinite(fish_d[m]).any() else None,
            "spd": round(float(np.median(spd[m])), 2),
            "roll": round(float(np.nanmedian(fish_roll[m])), 1),
            "c3": round(float(np.nanmedian(c3[m])), 0),
            "desc": round(float(np.nanmedian(descent[m])), 2),
        })

    # track downsample ~400 pts
    track = []
    if valid.any():
        idx = np.where(valid)[0]
        take = idx[::max(1, len(idx) // 400)]
        lat0, lng0 = lat[take[0]], lng[take[0]]
        m_per_deg = 111320.0
        xs, ys = [], []
        for i in take:
            xs.append((lng[i] - lng0) * m_per_deg * np.cos(np.radians(lat0)))
            ys.append((lat[i] - lat0) * m_per_deg)
        xs, ys = np.array(xs), np.array(ys)
        xmin, xmax = xs.min(), xs.max()
        ymin, ymax = ys.min(), ys.max()
        span = max(xmax - xmin, ymax - ymin, 1.0)
        for i, xi, yi in zip(take, xs, ys):
            track.append([
                int(round(999 * (xi - xmin) / span)),
                int(round(999 * (yi - ymin) / span)),
                round(float(alt[i]), 1),
            ])
        track_extent = {
            "east_m": round(float(xmax - xmin), 1),
            "north_m": round(float(ymax - ymin), 1),
            "lat0": float(lat0), "lng0": float(lng0),
        }
    else:
        track_extent = {}

    # altitude histogram
    hist = []
    if valid.any():
        bins = np.arange(0, 16, 1)
        counts, _ = np.histogram(alt[valid], bins=bins)
        for i, c in enumerate(counts):
            hist.append([f"{bins[i]}-{bins[i+1]}", int(c)])

    # RC3 histogram
    c3_hist = []
    if valid.any():
        edges_c = np.arange(1100, 1901, 50)
        counts, _ = np.histogram(c3[valid], bins=edges_c)
        for i, c in enumerate(counts):
            if c:
                c3_hist.append([f"{int(edges_c[i])}", int(c)])

    # fish mode occupancy over AUTO (time-weighted on DPTH samples)
    mode_occ = []
    auto_n = max(1, int(auto.sum()))
    for m, name in ((0, "STABILIZE"), (2, "ALT_HOLD"), (19, "MANUAL")):
        mode_occ.append({"mode": name, "pct": round(float(100 * np.mean(fmode_on_boat[auto] == m)), 1)})

    # RFND quality vs range to show the 7m clipping change
    q_vs_d = []
    # 1 m bins of good vs all during AUTO
    for lo in range(0, 31):
        m = in_auto & (rd >= lo) & (rd < lo + 1)
        if m.sum() < 20:
            continue
        q_vs_d.append({
            "d": lo,
            "n": int(m.sum()),
            "good": int((m & (rs == RFND_GOOD) & (rq >= QMIN)).sum()),
            "qmed": round(float(np.median(rq[m])), 0),
        })

    start_dt = datetime.fromtimestamp(boat_utc(auto_start), tz=HST)
    end_dt = datetime.fromtimestamp(boat_utc(auto_end), tz=HST)

    # GCS failsafe count
    n_gcs = sum(1 for s in msg_txt if str(s).startswith("GCS Failsafe") and "Cleared" not in str(s))

    summary = {
        "align": {"offset_s": offset, "circ_corr": round(align_score, 3),
                  "note": "fish_TimeUS = boat_TimeUS + offset"},
        "when": {
            "start_hst": start_dt.strftime("%Y-%m-%d %H:%M"),
            "end_hst": end_dt.strftime("%H:%M"),
            "auto_min": round((auto_end - auto_start) / 60, 1),
            "auto_start_tus": auto_start,
            "auto_end_tus": auto_end,
        },
        "params": {
            "RNGFND1_MAX": 100.0,
            "WP_SPEED": 1.0,
            "TURN_RADIUS": 3.5,
            "ATC_ANG_RLL_P": 3.0,
            "PILOT_SPEED_UP_mps": 1.0,
            "PILOT_SPEED_DN_mps": 2.0,
            "surf_target_m": 5.0,
            "surf_deadband_m": 0.1,
            "surf_full_scale_m": 0.7,
            "jog_pwm": [1400, 1600],
            "SURF_MIN_FRAC": 0.35,
            "SURF_TARGET_SLEW_MPS": 0.3,
        },
        "rfnd": {
            "auto_n": int(in_auto.sum()),
            "good_n": int(good.sum()),
            "good_pct": round(float(100 * good.sum() / max(1, in_auto.sum())), 1),
            "over7_good": int((good & (rd > 7)).sum()),
            "over7_any": int(over7.sum()),
            "dist_good": pct(rd[good], [5, 50, 95]),
            "dist_all": pct(rd[in_auto], [5, 50, 95]),
        },
        "overall": overall,
        "pre": pre,
        "post": post,
        "wp_jumps": [[a[0], a[1], b[0], b[1]] for a, b in jumps],
        "slow_segs": slow_segs[:12],
        "n_gcs_failsafe": n_gcs,
        "n_wp_reached": len(reaches),
        "last_wp": reaches[-1][1] if reaches else None,
        "mode_occ": mode_occ,
        "desc_table": desc_table,
        "series": series,
        "event": event,
        "event_start_hst": datetime.fromtimestamp(boat_utc(w0), tz=HST).strftime("%H:%M:%S"),
        "hist": hist,
        "c3_hist": c3_hist,
        "q_vs_d": q_vs_d,
        "track": track,
        "track_extent": track_extent,
        "roll_p": pct(roll_abs[valid], [50, 90, 95, 99]) if valid.any() else [],
        "c3_p": pct(c3[valid], [5, 50, 95]) if valid.any() else [],
        "c3_active40": round(float(100 * np.mean(c3_dev[valid] >= 40)), 1) if valid.any() else None,
        "c3_top": [(int(v), round(100 * c / valid.sum(), 1)) for c, v in top] if valid.any() else [],
    }
    with open(OUT, "w") as f:
        json.dump(summary, f)
    print("\nwrote", OUT)
    print("series points", len(series), "event", len(event), "track", len(track))


if __name__ == "__main__":
    main()
