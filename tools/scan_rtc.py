#!/usr/bin/env python3
"""Scan fish/boat logs for ALL RTC samples and other UTC sources."""
from pymavlink import mavutil

FISH = "/Users/tonywhite/Downloads/towfish_00000091.BIN"
BOAT = "/Users/tonywhite/Downloads/bbking_00000241.BIN"


def scan_rtc(path, label, extra_types=None):
    types = ["RTC"] + (extra_types or [])
    m = mavutil.mavlink_connection(path)
    n = 0
    n_rtc = 0
    nonzero = []
    first = last = None
    gpsn = 0
    unix = []
    orgs = []
    while True:
        r = m.recv_match(type=types)
        if r is None:
            break
        ty = r.get_type()
        t = r.TimeUS / 1e6
        if ty == "RTC":
            n_rtc += 1
            rec = (t, r.Epoch, r.SourceType)
            if first is None:
                first = rec
            last = rec
            if r.Epoch:
                if len(nonzero) < 8 or n_rtc % 50 == 0:
                    nonzero.append(rec)
        elif ty == "GPS" and gpsn < 3:
            print(label, "GPS sample", r)
            gpsn += 1
        elif ty == "UNIX":
            unix.append((t, getattr(r, "Usec", None) or getattr(r, "TimeUS", None)))
        elif ty == "ORGN":
            orgs.append((t, r))
        n += 1
    print("=" * 60)
    print(label, "RTC count", n_rtc, "first", first, "last", last)
    print("  nonzero epoch samples", len(nonzero))
    for rec in nonzero[:12]:
        print("   ", rec)
    if unix:
        print("  UNIX", unix[:5], "n", len(unix))
    if orgs:
        print("  ORGN", orgs[0])


if __name__ == "__main__":
    scan_rtc(FISH, "FISH", extra_types=["GPS", "ORGN"])
    scan_rtc(BOAT, "BOAT")
