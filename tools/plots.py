import datetime
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

R = np.load("/Users/tonywhite/Documents/SubReels_TowFish/tools/altitude_result.npz")
OUTDIR = "/Users/tonywhite/Documents/SubReels_TowFish/tools"

t = R["t"]
alt = R["alt"]
depth = R["depth"]
fdepth = R["fdepth"]
swath = R["swath"]
lat = R["lat"]
lng = R["lng"]

HST = datetime.timezone(datetime.timedelta(hours=-10))
tt = np.array([datetime.datetime.fromtimestamp(x, HST) for x in t])

fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

ax = axes[0]
ax.plot(tt, depth, lw=0.4, color="#1f4e79", label="seabed depth (boat sonar)")
ax.plot(tt, fdepth, lw=0.6, color="#c0392b", label="towfish pressure depth")
ax.invert_yaxis()
ax.set_ylabel("depth below surface (m)")
ax.legend(loc="upper right", fontsize=9)
ax.grid(alpha=0.3)
ax.set_title("Towfish altitude from towboat sonar + towfish pressure depth (6 s lag)")

ax = axes[1]
ax.plot(tt, alt, lw=0.4, color="#117a65")
ax.axhline(np.median(alt), color="k", ls="--", lw=1,
           label=f"median {np.median(alt):.2f} m")
ax.set_ylabel("towfish altitude (m)")
ax.legend(loc="upper right", fontsize=9)
ax.grid(alpha=0.3)

ax = axes[2]
ax.plot(tt, swath, lw=0.4, color="#7d3c98")
ax.axhline(np.median(swath), color="k", ls="--", lw=1,
           label=f"median {np.median(swath):.2f} m")
ax.set_ylabel("image swath width (m)")
ax.set_xlabel("time (HST)")
ax.legend(loc="upper right", fontsize=9)
ax.grid(alpha=0.3)

fig.tight_layout()
fig.savefig(f"{OUTDIR}/timeseries.png", dpi=110)
print("wrote timeseries.png")

fig, axes = plt.subplots(1, 3, figsize=(16, 4.6))

ax = axes[0]
ax.hist(alt, bins=60, color="#117a65", alpha=0.85)
ax.set_xlabel("towfish altitude (m)")
ax.set_ylabel("soundings")
ax.set_title("Altitude distribution")
ax.grid(alpha=0.3)

ax = axes[1]
ax.hist(swath, bins=60, color="#7d3c98", alpha=0.85)
ax.set_xlabel("swath width (m)")
ax.set_title("Swath distribution (94 deg HFOV, nadir)")
ax.grid(alpha=0.3)

ax = axes[2]
sc = ax.scatter(lng, lat, c=alt, s=1.5, cmap="viridis")
ax.set_xlabel("longitude")
ax.set_ylabel("latitude")
ax.set_title("Track coloured by towfish altitude")
ax.set_aspect(1 / np.cos(np.radians(lat.mean())))
plt.colorbar(sc, ax=ax, label="altitude (m)")

fig.tight_layout()
fig.savefig(f"{OUTDIR}/distributions.png", dpi=110)
print("wrote distributions.png")

# downsampled series for the canvas
step = max(1, len(t) // 900)
ds = [
    dict(t=round(float(x - t[0]), 1),
         hhmm=datetime.datetime.fromtimestamp(float(x), HST).strftime("%H:%M"),
         d=round(float(a), 2), f=round(float(b), 2),
         alt=round(float(c), 2), sw=round(float(d2), 2))
    for x, a, b, c, d2 in zip(t[::step], depth[::step], fdepth[::step],
                              alt[::step], swath[::step])
]
hist_a, edges_a = np.histogram(alt, bins=40)
hist_s, edges_s = np.histogram(swath, bins=40)
trk_step = max(1, len(t) // 3000)
with open(f"{OUTDIR}/canvas_data.json", "w") as f:
    json.dump(dict(
        series=ds,
        hist_alt=[dict(x=round(float(e), 2), n=int(n))
                  for e, n in zip(edges_a[:-1], hist_a)],
        hist_swath=[dict(x=round(float(e), 2), n=int(n))
                    for e, n in zip(edges_s[:-1], hist_s)],
        track=[dict(lat=round(float(a), 6), lng=round(float(b), 6), alt=round(float(c), 2))
               for a, b, c in zip(lat[::trk_step], lng[::trk_step], alt[::trk_step])],
    ), f)
print("wrote canvas_data.json", len(ds), "series pts")
