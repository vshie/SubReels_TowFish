"""Relate the computed swath to the survey line spacing.

The track is a lawnmower grid, so whether adjacent passes overlap depends on
line spacing versus the swath width computed from altitude.
"""

import numpy as np

R = np.load("/Users/tonywhite/Documents/SubReels_TowFish/tools/altitude_result.npz")
lat, lng, alt, sw, t = R["lat"], R["lng"], R["alt"], R["swath"], R["t"]

lat0, lng0 = lat.mean(), lng.mean()
mperdeg_lat = 111320.0
mperdeg_lng = 111320.0 * np.cos(np.radians(lat0))
x = (lng - lng0) * mperdeg_lng  # east, m
y = (lat - lat0) * mperdeg_lat  # north, m

# heading from successive positions, smoothed
dx = np.gradient(x)
dy = np.gradient(y)
hdg = np.degrees(np.arctan2(dx, dy)) % 180.0  # line orientation mod 180

hist, edges = np.histogram(hdg, bins=36, range=(0, 180))
print("track orientation histogram (mod 180 deg):")
for h, e in zip(hist, edges[:-1]):
    if h > len(hdg) * 0.02:
        print("   %5.0f-%-5.0f deg : %6d (%4.1f%%)" % (e, e + 5, h, 100 * h / len(hdg)))

dom = edges[np.argmax(hist)] + 2.5
print("\ndominant line orientation: %.0f deg" % dom)

# project onto the axis perpendicular to the dominant line direction
th = np.radians(dom)
perp = x * np.cos(th) - y * np.sin(th)

on_line = np.abs(((hdg - dom + 90) % 180) - 90) < 15
print("samples on dominant-orientation lines: %d (%.0f%%)" % (on_line.sum(), 100 * on_line.mean()))

p = perp[on_line]
hist2, edges2 = np.histogram(p, bins=200)
centres = 0.5 * (edges2[:-1] + edges2[1:])
peaks = []
for i in range(1, len(hist2) - 1):
    if hist2[i] > hist2[i - 1] and hist2[i] >= hist2[i + 1] and hist2[i] > len(p) * 0.005:
        peaks.append(centres[i])
peaks = np.array(peaks)
if len(peaks) > 1:
    merged = [peaks[0]]
    for q in peaks[1:]:
        if q - merged[-1] > 8:
            merged.append(q)
    merged = np.array(merged)
    spacing = np.diff(merged)
    print("detected %d survey lines, spacing (m):" % len(merged), spacing.round(1))
    print("median line spacing: %.1f m" % np.median(spacing))
    med_sw = np.median(sw[on_line])
    print("median swath on those lines: %.1f m" % med_sw)
    ov = 100 * (med_sw - np.median(spacing)) / med_sw
    print("=> overlap between adjacent passes: %.0f%% of swath" % ov)
    print("   (negative means gaps between passes)")

# fraction of soundings where swath is narrower than a nominal spacing
for sp in [10, 15, 20, 25]:
    print("   swath >= %2d m for %5.1f%% of soundings" % (sp, 100 * (sw >= sp).mean()))
