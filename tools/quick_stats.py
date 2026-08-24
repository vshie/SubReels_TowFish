"""Quick distribution stats for the towboat sonar and towfish depth."""

from collections import Counter

import numpy as np
from pymavlink import mavutil

BOAT = "/Users/tonywhite/Downloads/BBKing towboat 8.7.26 log.BIN"
FISH = "/Users/tonywhite/Downloads/towfish mahukona3 8.7.22.BIN"

m = mavutil.mavlink_connection(BOAT)
stat = Counter()
qual = Counter()
dists = []
dpth = []
while True:
    r = m.recv_match(type=["RFND", "DPTH"])
    if r is None:
        break
    if r.get_type() == "RFND":
        stat[r.Stat] += 1
        qual[r.Quality] += 1
        dists.append(r.Dist)
    else:
        dpth.append(r.Depth)
dists = np.array(dists)
dpth = np.array(dpth)
print("RFND Stat counts:", dict(stat))
print("RFND Quality counts:", dict(sorted(qual.items())))
print("RFND Dist percentiles:", np.percentile(dists, [0, 1, 25, 50, 75, 99, 100]).round(2))
print("DPTH Depth percentiles:", np.percentile(dpth, [0, 1, 25, 50, 75, 99, 100]).round(2))
print("DPTH == 0 count:", int((dpth == 0).sum()), "of", len(dpth))

m = mavutil.mavlink_connection(FISH)
b0, b1 = [], []
mnt = []
while True:
    r = m.recv_match(type=["BARO", "MNT"])
    if r is None:
        break
    if r.get_type() == "BARO":
        (b0 if r.I == 0 else b1).append(r.Alt)
    else:
        mnt.append((r.Pitch if hasattr(r, "Pitch") else None, r))
b1 = np.array(b1)
print("\nFISH BARO[1] Alt percentiles:", np.percentile(b1, [0, 1, 25, 50, 75, 99, 100]).round(2))
print("FISH depth (=-Alt) max:", round(-b1.min(), 2))
if mnt:
    print("MNT sample:", mnt[0][1])
    ps = np.array([x[0] for x in mnt if x[0] is not None])
    if len(ps):
        print("MNT Pitch percentiles:", np.percentile(ps, [0, 25, 50, 75, 100]).round(2))
