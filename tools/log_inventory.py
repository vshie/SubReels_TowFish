import sys
from collections import Counter

from pymavlink import mavutil


def inventory(path):
    mlog = mavutil.mavlink_connection(path)
    counts = Counter()
    tmin = None
    tmax = None
    while True:
        m = mlog.recv_match()
        if m is None:
            break
        t = m.get_type()
        counts[t] += 1
        ts = getattr(m, "_timestamp", None)
        if ts:
            tmin = ts if tmin is None else min(tmin, ts)
            tmax = ts if tmax is None else max(tmax, ts)
    return counts, tmin, tmax


if __name__ == "__main__":
    for path in sys.argv[1:]:
        counts, tmin, tmax = inventory(path)
        print("=" * 70)
        print(path)
        print(f"time range: {tmin} .. {tmax}  ({(tmax - tmin) if tmin else 0:.1f} s)")
        for name, n in sorted(counts.items()):
            print(f"  {name:20s} {n}")
