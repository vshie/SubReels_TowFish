"""Compute towfish altitude above the seabed and the resulting camera swath.

altitude(t+LAG) = boat_sounding(t) - towfish_pressure_depth(t+LAG)

The boat's downward sonar measures water depth at the moment it passes a point;
the towfish arrives over that same point LAG seconds later, so its height above
the bottom is that depth minus its own pressure depth at the later time.
"""

import json

import numpy as np

CACHE = "/Users/tonywhite/Documents/SubReels_TowFish/tools/aligned.npz"

LAG = 6.0            # s, towfish trails the sounding point
HFOV = 94.0          # deg
VFOV = 62.0          # deg
MIN_FISH_DEPTH = 1.0  # m, below this the fish is on deck / at the surface
MIN_SPEED = 0.5      # m/s, boat underway
MAX_ALT = 60.0       # m, reject absurd altitudes
DESPIKE_WIN = 9      # samples (~1.1 s at 7.9 Hz)
DESPIKE_TOL = 1.5    # m absolute
DESPIKE_FRAC = 0.15  # or 15 % of the local median


def rolling_median(x, win):
    n = len(x)
    half = win // 2
    pad = np.pad(x, (half, half), mode="edge")
    out = np.empty(n)
    strided = np.lib.stride_tricks.sliding_window_view(pad, win)
    out[:] = np.median(strided, axis=1)
    return out


def load():
    D = np.load(CACHE)
    return {k: D[k] for k in D.files}


def compute(lag=LAG):
    D = load()

    t = D["boat_dpth_t"]
    depth = D["boat_dpth_d"]
    lat = D["boat_dpth_lat"]
    lng = D["boat_dpth_lng"]

    # --- sounding quality -------------------------------------------------
    med = rolling_median(depth, DESPIKE_WIN)
    tol = np.maximum(DESPIKE_TOL, DESPIKE_FRAC * med)
    clean = np.abs(depth - med) <= tol

    # nearest rangefinder sample gives the driver-reported quality
    ri = np.searchsorted(D["boat_rfnd_t"], t).clip(0, len(D["boat_rfnd_t"]) - 1)
    qual = D["boat_rfnd_q"][ri]
    good_q = qual >= 50

    # --- boat speed at the sounding --------------------------------------
    spd = np.interp(t, D["boat_gps_t"], D["boat_gps_spd"])
    underway = spd >= MIN_SPEED

    # --- towfish depth LAG seconds later ----------------------------------
    ft = D["fish_baro_t"]
    fd = D["fish_baro_depth"]
    tq = t + lag
    in_span = (tq >= ft[0]) & (tq <= ft[-1])
    fish_depth = np.interp(tq, ft, fd)

    # reject where the fish sample is stale (a log gap near tq)
    j = np.searchsorted(ft, tq).clip(1, len(ft) - 1)
    gap = ft[j] - ft[j - 1]
    fresh = gap < 1.0

    submerged = fish_depth >= MIN_FISH_DEPTH

    alt = depth - fish_depth

    valid = (clean & good_q & underway & in_span & fresh & submerged
             & (alt > 0) & (alt < MAX_ALT))

    return dict(t=t, depth=depth, lat=lat, lng=lng, spd=spd, qual=qual,
                fish_depth=fish_depth, alt=alt, valid=valid,
                clean=clean, good_q=good_q, underway=underway,
                submerged=submerged, in_span=in_span, D=D)


def swaths(alt):
    h = 2.0 * alt * np.tan(np.radians(HFOV / 2))
    v = 2.0 * alt * np.tan(np.radians(VFOV / 2))
    return h, v


def segments(t, valid, max_gap=30.0, min_dur=60.0):
    idx = np.where(valid)[0]
    if not len(idx):
        return []
    segs = []
    start = idx[0]
    prev = idx[0]
    for i in idx[1:]:
        if t[i] - t[prev] > max_gap:
            if t[prev] - t[start] >= min_dur:
                segs.append((start, prev))
            start = i
        prev = i
    if t[prev] - t[start] >= min_dur:
        segs.append((start, prev))
    return segs


def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = np.radians(lat1), np.radians(lat2)
    dp = p2 - p1
    dl = np.radians(lon2 - lon1)
    a = np.sin(dp / 2) ** 2 + np.cos(p1) * np.cos(p2) * np.sin(dl / 2) ** 2
    return 2 * R * np.arcsin(np.sqrt(a))


def main():
    r = compute()
    t, valid = r["t"], r["valid"]
    n = len(t)

    print("=" * 72)
    print("SOUNDING RECORD")
    print("  total DPTH soundings          : %d over %.0f s" % (n, t[-1] - t[0]))
    print("  passed despike                : %d (%.1f%%)" % (r["clean"].sum(), 100 * r["clean"].mean()))
    print("  sonar quality >= 50           : %d (%.1f%%)" % (r["good_q"].sum(), 100 * r["good_q"].mean()))
    print("  boat underway (>= %.1f m/s)    : %d (%.1f%%)" % (MIN_SPEED, r["underway"].sum(), 100 * r["underway"].mean()))
    print("  towfish submerged (>= %.1f m)  : %d (%.1f%%)" % (MIN_FISH_DEPTH, r["submerged"].sum(), 100 * r["submerged"].mean()))
    print("  ALL criteria (usable)         : %d (%.1f%%)" % (valid.sum(), 100 * valid.mean()))

    a = r["alt"][valid]
    d = r["depth"][valid]
    fd = r["fish_depth"][valid]
    h, v = swaths(a)

    def pct(x, ps=(5, 25, 50, 75, 95)):
        return np.percentile(x, ps)

    print()
    print("=" * 72)
    print("RESULTS OVER %d USABLE SOUNDINGS (%.1f min of towing)" % (valid.sum(), valid.sum() / 7.85 / 60))
    print()
    print("                       mean   std     p5    p25    med    p75    p95    min    max")
    for name, x, u in [("water depth", d, "m"), ("towfish depth", fd, "m"),
                       ("towfish ALTITUDE", a, "m"),
                       ("swath width (H)", h, "m"), ("footprint (V)", v, "m")]:
        p = pct(x)
        print("  %-19s %6.2f %5.2f  %5.2f  %5.2f  %5.2f  %5.2f  %5.2f  %5.2f  %5.2f"
              % (name, x.mean(), x.std(), p[0], p[1], p[2], p[3], p[4], x.min(), x.max()))

    print()
    print("  swath = %.4f x altitude   (2*tan(%.0f/2))" % (2 * np.tan(np.radians(HFOV / 2)), HFOV))
    print("  vert  = %.4f x altitude   (2*tan(%.0f/2))" % (2 * np.tan(np.radians(VFOV / 2)), VFOV))

    # altitude distribution buckets
    print()
    print("  altitude distribution:")
    edges = [0, 2, 4, 6, 8, 10, 15, 20, 60]
    for lo, hi in zip(edges[:-1], edges[1:]):
        k = ((a >= lo) & (a < hi)).sum()
        if k:
            print("    %5.0f - %-5.0f m : %6d (%5.1f%%)  swath %5.1f - %-5.1f m"
                  % (lo, hi, k, 100 * k / len(a), 2.1447 * lo, 2.1447 * hi))

    # per-segment
    segs = segments(t, valid)
    print()
    print("=" * 72)
    print("CONTINUOUS TOW SEGMENTS (gaps < 30 s, duration > 60 s)")
    print("  #   start(UTC-10)   dur     n    depth   f.depth   ALT    swath   dist   area")
    tot_area = 0.0
    tot_dist = 0.0
    seg_rows = []
    for i, (s, e) in enumerate(segs, 1):
        sel = valid.copy()
        sel[:s] = False
        sel[e + 1:] = False
        aa = r["alt"][sel]
        if not len(aa):
            continue
        hh = 2 * aa * np.tan(np.radians(HFOV / 2))
        la, lo = r["lat"][sel], r["lng"][sel]
        dist = haversine(la[:-1], lo[:-1], la[1:], lo[1:]).sum()
        area = np.mean(hh) * dist
        tot_area += area
        tot_dist += dist
        import datetime
        st = datetime.datetime.fromtimestamp(t[s], datetime.timezone(datetime.timedelta(hours=-10)))
        print("  %-3d %s  %5.0fs %5d  %6.2f  %6.2f  %6.2f  %6.2f  %6.0f %7.0f"
              % (i, st.strftime("%H:%M:%S"), t[e] - t[s], len(aa),
                 np.mean(r["depth"][sel]), np.mean(r["fish_depth"][sel]),
                 np.mean(aa), np.mean(hh), dist, area))
        seg_rows.append(dict(n=i, start=float(t[s]), dur=float(t[e] - t[s]),
                             count=int(len(aa)), depth=float(np.mean(r["depth"][sel])),
                             fdepth=float(np.mean(r["fish_depth"][sel])),
                             alt_mean=float(np.mean(aa)), alt_p5=float(np.percentile(aa, 5)),
                             alt_p95=float(np.percentile(aa, 95)),
                             swath=float(np.mean(hh)), dist=float(dist), area=float(area)))
    print("  %-3s %14s %6s %5s  %6s  %6s  %6s  %6s  %6.0f %7.0f"
          % ("TOT", "", "", "", "", "", "", "", tot_dist, tot_area))

    # lag sensitivity
    print()
    print("=" * 72)
    print("SENSITIVITY TO THE ASSUMED 6 s LAG")
    print("   lag    n      mean alt   median alt   mean swath")
    for lg in [0, 3, 6, 9, 12]:
        rr = compute(lag=lg)
        aa = rr["alt"][rr["valid"]]
        print("  %3d s  %6d   %7.2f m   %7.2f m    %7.2f m"
              % (lg, len(aa), aa.mean(), np.median(aa), 2.1447 * aa.mean()))

    # save for plotting / canvas
    out = dict(
        t=t[valid].tolist(), alt=a.tolist(), depth=d.tolist(),
        fdepth=fd.tolist(), lat=r["lat"][valid].tolist(),
        lng=r["lng"][valid].tolist(), swath=h.tolist(),
    )
    np.savez_compressed(
        "/Users/tonywhite/Documents/SubReels_TowFish/tools/altitude_result.npz",
        **{k: np.array(v) for k, v in out.items()})
    summary = dict(
        n_total=int(n), n_valid=int(valid.sum()),
        alt=dict(mean=float(a.mean()), std=float(a.std()), min=float(a.min()),
                 max=float(a.max()), p5=float(np.percentile(a, 5)),
                 p25=float(np.percentile(a, 25)), med=float(np.median(a)),
                 p75=float(np.percentile(a, 75)), p95=float(np.percentile(a, 95))),
        depth=dict(mean=float(d.mean()), med=float(np.median(d))),
        fdepth=dict(mean=float(fd.mean()), med=float(np.median(fd))),
        swath=dict(mean=float(h.mean()), med=float(np.median(h)),
                   p5=float(np.percentile(h, 5)), p95=float(np.percentile(h, 95))),
        segments=seg_rows, total_dist=tot_dist, total_area=tot_area,
    )
    with open("/Users/tonywhite/Documents/SubReels_TowFish/tools/summary.json", "w") as f:
        json.dump(summary, f, indent=2)


if __name__ == "__main__":
    main()
