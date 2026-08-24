#!/usr/bin/env python3
"""Extract bbking 241 + towfish 91 into a compact npz for the mission review."""

from collections import defaultdict

import numpy as np
from pymavlink import mavutil

BOAT = "/Users/tonywhite/Downloads/bbking_00000241.BIN"
FISH = "/Users/tonywhite/Downloads/towfish_00000091.BIN"
OUT = "/Users/tonywhite/Documents/SubReels_TowFish/tools/mission_241.npz"

BOAT_TYPES = ["RTC", "GPS", "DPTH", "RFND", "MODE", "CMD", "MSG", "PARM",
              "ATT", "NTUN", "THR", "RCOU", "EV"]
FISH_TYPES = ["RTC", "GPS", "BARO", "ATT", "MODE", "RCIN", "RCOU", "CTUN",
              "MSG", "PARM", "EV", "MAG"]


def as_arr(xs, dtype=float):
    return np.asarray(xs, dtype=dtype) if xs else np.asarray([], dtype=dtype)


def extract_boat():
    m = mavutil.mavlink_connection(BOAT)
    d = defaultdict(list)
    parms = {}
    parm_chg = []
    msgs = []
    modes = []
    cmds = []
    n = 0
    while True:
        r = m.recv_match(type=BOAT_TYPES)
        if r is None:
            break
        n += 1
        ty = r.get_type()
        t = r.TimeUS / 1e6
        if ty == "RTC" and not d["rtc_epoch"]:
            d["rtc_epoch"].append(r.Epoch / 1e6)
            d["rtc_tus"].append(t)
        elif ty == "GPS" and getattr(r, "I", 0) == 0:
            d["gps_t"].append(t)
            d["gps_lat"].append(r.Lat)
            d["gps_lng"].append(r.Lng)
            d["gps_spd"].append(r.Spd)
            d["gps_crs"].append(r.GCrs)
            d["gps_alt"].append(r.Alt)
            d["gps_nsats"].append(r.NSats)
        elif ty == "DPTH":
            d["dpth_t"].append(t)
            d["dpth_d"].append(r.Depth)
            d["dpth_lat"].append(r.Lat)
            d["dpth_lng"].append(r.Lng)
        elif ty == "RFND" and r.Instance == 0:
            d["rfnd_t"].append(t)
            d["rfnd_d"].append(r.Dist)
            d["rfnd_stat"].append(r.Stat)
            d["rfnd_q"].append(r.Quality)
        elif ty == "MODE":
            modes.append((t, int(r.Mode), int(r.Rsn)))
        elif ty == "CMD":
            cmds.append((t, int(r.CTot), int(r.CNum), int(r.CId),
                         r.Lat, r.Lng, r.Alt, r.Prm1, r.Prm2))
        elif ty == "ATT":
            d["att_t"].append(t)
            d["att_roll"].append(r.Roll)
            d["att_pitch"].append(r.Pitch)
            d["att_yaw"].append(r.Yaw)
        elif ty == "NTUN":
            d["ntun_t"].append(t)
            # field names vary; grab whatever exists
            for name, key in (("ThD", "ntun_thd"), ("ThA", "ntun_tha"),
                              ("Spd", "ntun_spd"), ("SpdE", "ntun_spde"),
                              ("TSpd", "ntun_tspd"), ("ThrOut", "ntun_thro")):
                if hasattr(r, name):
                    d[key].append(getattr(r, name))
        elif ty == "THR":
            d["thr_t"].append(t)
            if hasattr(r, "ThrOut"):
                d["thr_out"].append(r.ThrOut)
            elif hasattr(r, "ThO"):
                d["thr_out"].append(r.ThO)
        elif ty == "RCOU":
            d["rcou_t"].append(t)
            d["rcou_c3"].append(r.C3)
        elif ty == "MSG":
            msgs.append((t, r.Message))
        elif ty == "PARM":
            name = r.Name
            val = float(r.Value)
            if name in parms and parms[name] != val:
                parm_chg.append((t, name, parms[name], val))
            parms[name] = val
        elif ty == "EV":
            d["ev_t"].append(t)
            d["ev_id"].append(r.Id)
        if n % 200000 == 0:
            print("  boat messages", n, "t=%.0f" % t, flush=True)
    print("boat done, messages", n, flush=True)
    out = {k: as_arr(v) for k, v in d.items()}
    if modes:
        a = np.array(modes)
        out["mode_t"] = a[:, 0]
        out["mode"] = a[:, 1]
        out["mode_rsn"] = a[:, 2]
    if cmds:
        a = np.array(cmds)
        out["cmd_t"] = a[:, 0]
        out["cmd_tot"] = a[:, 1]
        out["cmd_num"] = a[:, 2]
        out["cmd_id"] = a[:, 3]
        out["cmd_lat"] = a[:, 4]
        out["cmd_lng"] = a[:, 5]
        out["cmd_alt"] = a[:, 6]
    out["parm_names"] = np.array(list(parms.keys()))
    out["parm_vals"] = np.array(list(parms.values()))
    if parm_chg:
        out["parm_chg_t"] = np.array([p[0] for p in parm_chg])
        out["parm_chg_name"] = np.array([p[1] for p in parm_chg])
        out["parm_chg_old"] = np.array([p[2] for p in parm_chg])
        out["parm_chg_new"] = np.array([p[3] for p in parm_chg])
    out["msg_t"] = np.array([p[0] for p in msgs]) if msgs else as_arr([])
    out["msg_txt"] = np.array([p[1] for p in msgs], dtype=object) if msgs else np.array([], dtype=object)
    return out, parms, msgs, modes, parm_chg


def extract_fish():
    m = mavutil.mavlink_connection(FISH)
    d = defaultdict(list)
    parms = {}
    parm_chg = []
    msgs = []
    modes = []
    n = 0
    while True:
        r = m.recv_match(type=FISH_TYPES)
        if r is None:
            break
        n += 1
        ty = r.get_type()
        t = r.TimeUS / 1e6
        if ty == "RTC" and not d["rtc_epoch"]:
            d["rtc_epoch"].append(r.Epoch / 1e6)
            d["rtc_tus"].append(t)
            d["rtc_src"].append(r.SourceType)
        elif ty == "GPS" and getattr(r, "I", 0) == 0:
            d["gps_t"].append(t)
            d["gps_lat"].append(r.Lat)
            d["gps_lng"].append(r.Lng)
            d["gps_spd"].append(r.Spd)
            d["gps_crs"].append(r.GCrs)
            if hasattr(r, "GWk"):
                d["gps_gwk"].append(r.GWk)
                d["gps_gms"].append(r.GMS)
        elif ty == "BARO" and r.I == 1:
            d["baro_t"].append(t)
            d["baro_alt"].append(r.Alt)          # negative depth in ArduSub
            d["baro_press"].append(r.Press)
            d["baro_temp"].append(r.Temp)
            d["baro_crt"].append(r.CRt)
        elif ty == "ATT":
            d["att_t"].append(t)
            d["att_roll"].append(r.Roll)
            d["att_pitch"].append(r.Pitch)
            d["att_yaw"].append(r.Yaw)
            d["att_desroll"].append(r.DesRoll)
            d["att_despitch"].append(r.DesPitch)
        elif ty == "MODE":
            modes.append((t, int(r.Mode), int(r.Rsn)))
        elif ty == "RCIN":
            d["rcin_t"].append(t)
            d["rcin_c3"].append(r.C3)
            d["rcin_c1"].append(r.C1)
            d["rcin_c2"].append(r.C2)
            d["rcin_c4"].append(r.C4)
        elif ty == "RCOU":
            d["rcou_t"].append(t)
            d["rcou_c5"].append(r.C5)
            d["rcou_c6"].append(r.C6)
            d["rcou_c1"].append(r.C1)
            d["rcou_c2"].append(r.C2)
            d["rcou_c3"].append(r.C3)
            d["rcou_c4"].append(r.C4)
        elif ty == "CTUN":
            d["ctun_t"].append(t)
            d["ctun_alt"].append(r.Alt)
            d["ctun_dalt"].append(r.DAlt)
            d["ctun_thi"].append(r.ThI)
            d["ctun_tho"].append(r.ThO)
            d["ctun_crt"].append(r.CRt)
            d["ctun_dcrt"].append(r.DCRt)
            d["ctun_balt"].append(r.BAlt)
        elif ty == "MSG":
            msgs.append((t, r.Message))
        elif ty == "PARM":
            name = r.Name
            val = float(r.Value)
            if name in parms and parms[name] != val:
                parm_chg.append((t, name, parms[name], val))
            parms[name] = val
        elif ty == "EV":
            d["ev_t"].append(t)
            d["ev_id"].append(r.Id)
        if n % 200000 == 0:
            print("  fish messages", n, "t=%.0f" % t, flush=True)
    print("fish done, messages", n, flush=True)
    out = {k: as_arr(v) for k, v in d.items()}
    if modes:
        a = np.array(modes)
        out["mode_t"] = a[:, 0]
        out["mode"] = a[:, 1]
        out["mode_rsn"] = a[:, 2]
    out["parm_names"] = np.array(list(parms.keys()))
    out["parm_vals"] = np.array(list(parms.values()))
    if parm_chg:
        out["parm_chg_t"] = np.array([p[0] for p in parm_chg])
        out["parm_chg_name"] = np.array([p[1] for p in parm_chg])
        out["parm_chg_old"] = np.array([p[2] for p in parm_chg])
        out["parm_chg_new"] = np.array([p[3] for p in parm_chg])
    out["msg_t"] = np.array([p[0] for p in msgs]) if msgs else as_arr([])
    out["msg_txt"] = np.array([p[1] for p in msgs], dtype=object) if msgs else np.array([], dtype=object)
    return out, parms, msgs, modes, parm_chg


def main():
    print("=== boat ===", flush=True)
    b, bparm, bmsg, bmode, bchg = extract_boat()
    print("=== fish ===", flush=True)
    f, fparm, fmsg, fmode, fchg = extract_fish()

    data = {f"boat_{k}": v for k, v in b.items()}
    data.update({f"fish_{k}": v for k, v in f.items()})
    np.savez_compressed(OUT, **data)
    print("wrote", OUT)

    def span(name, t):
        if t is None or len(t) == 0:
            print(name, "EMPTY")
            return
        print("%s n=%d  TimeUS %.1f .. %.1f  (%.1f min)" % (
            name, len(t), t[0], t[-1], (t[-1] - t[0]) / 60))

    print("\n--- spans ---")
    span("boat GPS ", b.get("gps_t"))
    span("boat DPTH", b.get("dpth_t"))
    span("boat RFND", b.get("rfnd_t"))
    span("fish BARO", f.get("baro_t"))
    span("fish ATT ", f.get("att_t"))
    span("fish RCIN", f.get("rcin_t"))
    print("boat RTC epoch", b.get("rtc_epoch"), "tus", b.get("rtc_tus"))
    print("fish RTC epoch", f.get("rtc_epoch"), "src", f.get("rtc_src"), "tus", f.get("rtc_tus"))
    print("fish GPS n", len(f.get("gps_t", [])))
    print("\nboat modes:")
    for t, mode, rsn in bmode:
        print("  t=%8.1f  mode=%s  rsn=%s" % (t, mode, rsn))
    print("\nfish modes:")
    for t, mode, rsn in fmode:
        print("  t=%8.1f  mode=%s  rsn=%s" % (t, mode, rsn))
    print("\nboat msgs:")
    for t, txt in bmsg:
        print("  t=%8.1f  %s" % (t, txt))
    print("\nfish msgs:")
    for t, txt in fmsg:
        print("  t=%8.1f  %s" % (t, txt))
    print("\nboat parm changes:", len(bchg))
    for p in bchg[:40]:
        print(" ", p)
    print("\nfish parm changes:", len(fchg))
    for p in fchg[:40]:
        print(" ", p)
    interesting = ("RNGFND1_MAX", "RNGFND1_MIN", "WP_SPEED", "CRUISE_SPEED",
                   "TURN_RADIUS", "ATC_ANG_RLL_P", "PILOT_SPEED", "PILOT_SPEED_DN",
                   "PILOT_SPEED_UP", "MOT_THST_HOVER")
    print("\nkey parms boat:")
    for k in interesting:
        if k in bparm:
            print("  %s = %s" % (k, bparm[k]))
    print("key parms fish:")
    for k in interesting:
        if k in fparm:
            print("  %s = %s" % (k, fparm[k]))


if __name__ == "__main__":
    main()
