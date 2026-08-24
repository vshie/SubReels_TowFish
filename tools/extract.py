"""Extract the towboat sounding track and towfish depth into a single npz cache.

Both logs are aligned to UTC through their RTC message, which records an epoch
timestamp alongside the flight controller's TimeUS clock.
"""

import numpy as np
from pymavlink import mavutil

BOAT = "/Users/tonywhite/Downloads/BBKing towboat 8.7.26 log.BIN"
FISH = "/Users/tonywhite/Downloads/towfish mahukona3 8.7.22.BIN"
OUT = "/Users/tonywhite/Documents/SubReels_TowFish/tools/aligned.npz"


def rtc_offset(path):
    """Return utc_seconds = TimeUS/1e6 + offset."""
    m = mavutil.mavlink_connection(path)
    r = m.recv_match(type=["RTC"])
    return r.Epoch / 1e6 - r.TimeUS / 1e6, r


def read_boat():
    m = mavutil.mavlink_connection(BOAT)
    dt, dlat, dlng, ddep = [], [], [], []
    rt, rdist, rstat, rqual = [], [], [], []
    gt, gspd, glat, glng, gcrs = [], [], [], [], []
    while True:
        r = m.recv_match(type=["DPTH", "RFND", "GPS"])
        if r is None:
            break
        ty = r.get_type()
        if ty == "DPTH":
            dt.append(r.TimeUS / 1e6)
            dlat.append(r.Lat)
            dlng.append(r.Lng)
            ddep.append(r.Depth)
        elif ty == "RFND" and r.Instance == 0:
            rt.append(r.TimeUS / 1e6)
            rdist.append(r.Dist)
            rstat.append(r.Stat)
            rqual.append(r.Quality)
        elif ty == "GPS" and r.I == 0:
            gt.append(r.TimeUS / 1e6)
            gspd.append(r.Spd)
            glat.append(r.Lat)
            glng.append(r.Lng)
            gcrs.append(r.GCrs)
    return dict(
        dpth_t=np.array(dt), dpth_lat=np.array(dlat), dpth_lng=np.array(dlng),
        dpth_d=np.array(ddep),
        rfnd_t=np.array(rt), rfnd_d=np.array(rdist),
        rfnd_stat=np.array(rstat), rfnd_q=np.array(rqual),
        gps_t=np.array(gt), gps_spd=np.array(gspd),
        gps_lat=np.array(glat), gps_lng=np.array(glng), gps_crs=np.array(gcrs),
    )


def read_fish():
    m = mavutil.mavlink_connection(FISH)
    bt, bd = [], []
    at, ap, ar, ay = [], [], [], []
    mt, mp = [], []
    while True:
        r = m.recv_match(type=["BARO", "ATT", "MNT"])
        if r is None:
            break
        ty = r.get_type()
        if ty == "BARO" and r.I == 1:
            bt.append(r.TimeUS / 1e6)
            bd.append(-r.Alt)
        elif ty == "ATT":
            at.append(r.TimeUS / 1e6)
            ap.append(r.Pitch)
            ar.append(r.Roll)
            ay.append(r.Yaw)
        elif ty == "MNT":
            mt.append(r.TimeUS / 1e6)
            mp.append(r.Pitch)
    return dict(
        baro_t=np.array(bt), baro_depth=np.array(bd),
        att_t=np.array(at), att_pitch=np.array(ap),
        att_roll=np.array(ar), att_yaw=np.array(ay),
        mnt_t=np.array(mt), mnt_pitch=np.array(mp),
    )


if __name__ == "__main__":
    boff, brtc = rtc_offset(BOAT)
    foff, frtc = rtc_offset(FISH)
    print("boat RTC:", brtc)
    print("fish RTC:", frtc)
    print("boat offset %.6f  fish offset %.6f" % (boff, foff))
    print("fish_TimeUS - boat_TimeUS at same UTC = %.6f s" % (boff - foff))

    b = read_boat()
    f = read_fish()
    for k in list(b):
        if k.endswith("_t"):
            b[k] = b[k] + boff
    for k in list(f):
        if k.endswith("_t"):
            f[k] = f[k] + foff

    data = {f"boat_{k}": v for k, v in b.items()}
    data.update({f"fish_{k}": v for k, v in f.items()})
    data["boat_rtc_offset"] = np.array([boff])
    data["fish_rtc_offset"] = np.array([foff])
    np.savez_compressed(OUT, **data)
    print("wrote", OUT)
    print("boat DPTH utc %.1f .. %.1f" % (b["dpth_t"][0], b["dpth_t"][-1]))
    print("fish BARO utc %.1f .. %.1f" % (f["baro_t"][0], f["baro_t"][-1]))
