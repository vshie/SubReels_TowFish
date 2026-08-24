"""Validate the RTC-based clock alignment between the two logs.

The towfish is a towed body: when the boat speeds up the fish planes deeper,
when the boat slows the fish rises. Cross-correlating boat speed against fish
depth therefore has a peak at the physical tow-response lag. If the RTC
alignment were badly wrong, the peak would land somewhere implausible.
"""

import numpy as np

D = np.load("/Users/tonywhite/Documents/SubReels_TowFish/tools/aligned.npz")

t0 = max(D["boat_gps_t"][0], D["fish_baro_t"][0])
t1 = min(D["boat_gps_t"][-1], D["fish_baro_t"][-1])
grid = np.arange(t0, t1, 0.5)

spd = np.interp(grid, D["boat_gps_t"], D["boat_gps_spd"])
dep = np.interp(grid, D["fish_baro_t"], D["fish_baro_depth"])

# only use the towing portion, where the fish is actually in the water
ok = dep > 1.0
print("usable samples: %d of %d" % (ok.sum(), len(grid)))


def detrend(x):
    x = x - x.mean()
    return x / x.std()


s = detrend(spd[ok])
d = detrend(dep[ok])

lags = np.arange(-60, 121)  # in 0.5 s steps -> -30..+60 s
cors = []
for L in lags:
    if L >= 0:
        a, b = s[: len(s) - L], d[L:]
    else:
        a, b = s[-L:], d[: len(d) + L]
    cors.append(np.corrcoef(a, b)[0, 1])
cors = np.array(cors)
best = lags[np.argmax(cors)] * 0.5
print("peak correlation %.3f at lag %+.1f s (fish depth lags boat speed)" % (cors.max(), best))
print("correlation at 0 s: %.3f, at +6 s: %.3f" % (
    cors[lags == 0][0], cors[lags == 12][0]))

for L in range(-20, 41, 4):
    i = np.where(lags == L * 2)[0]
    if len(i):
        print("  lag %+4d s : r = %+.3f" % (L, cors[i[0]]))
