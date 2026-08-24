"""Is the camera looking straight down? Characterise mount pitch + fish attitude."""

import numpy as np

D = np.load("/Users/tonywhite/Documents/SubReels_TowFish/tools/aligned.npz")

mp = D["fish_mnt_pitch"]
mt = D["fish_mnt_t"]
ap = D["fish_att_pitch"]
at = D["fish_att_t"]

print("MNT pitch unique-ish values:")
vals, cnt = np.unique(np.round(mp, 1), return_counts=True)
order = np.argsort(-cnt)
for i in order[:10]:
    print("   %8.1f deg : %6d (%5.1f%%)" % (vals[i], cnt[i], 100 * cnt[i] / len(mp)))

api = np.interp(mt, at, ap)
print("\ncorrelation MNT pitch vs vehicle pitch: %.3f" % np.corrcoef(mp, api)[0, 1])
print("(near zero => mount angle is fixed, not stabilised)")

# restrict to the towing window used in the altitude analysis
R = np.load("/Users/tonywhite/Documents/SubReels_TowFish/tools/altitude_result.npz")
t0, t1 = R["t"].min() + 6, R["t"].max() + 6
sel = (mt >= t0) & (mt <= t1)
sela = (at >= t0) & (at <= t1)
print("\nduring towing:")
print("  MNT pitch  mean %.2f  std %.2f  median %.2f" % (mp[sel].mean(), mp[sel].std(), np.median(mp[sel])))
print("  fish pitch mean %.2f  std %.2f  median %.2f" % (ap[sela].mean(), ap[sela].std(), np.median(ap[sela])))
print("  fish roll  mean %.2f  std %.2f" % (D["fish_att_roll"][sela].mean(), D["fish_att_roll"][sela].std()))

mnt_med = np.median(mp[sel])
fish_med = np.median(ap[sela])

for label, dep in [("mount angle as body-fixed + vehicle pitch", -(mnt_med + fish_med)),
                   ("mount angle as earth-frame (stabilised)", -mnt_med)]:
    off = 90.0 - dep  # deg of optical axis away from straight down
    print("\n%s -> depression %.1f deg below horizontal, %.1f deg off nadir" % (label, dep, off))
    o = np.radians(off)
    hh = np.radians(94 / 2)
    vv = np.radians(62 / 2)
    slant = 1 / np.cos(o)
    print("   slant range to image centre = %.3f x altitude" % slant)
    print("   across-track width at centre = %.3f x altitude (nadir: 2.145)" % (2 * slant * np.tan(hh)))
    near = np.tan(o - vv)
    far = np.tan(o + vv)
    print("   along-track ground extent    = %.3f x altitude (nadir: 1.202)" % (far - near))
    print("   near edge %.2f x alt, far edge %.2f x alt ahead of nadir" % (near, far))
