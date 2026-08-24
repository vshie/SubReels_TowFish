import numpy as np
d=np.load("mission_241.npz",allow_pickle=True)
P=dict(zip(d["fish_parm_names"],d["fish_parm_vals"]))
print("=== ALT_HOLD / depth controller limits ===")
for k in sorted(P):
    if any(s in k for s in ("PILOT_SPEED","PSC_","SURFACE_DEPTH","RNGFND",
                            "ATC_ACCEL_Z","ATC_RAT")):
        print(f"  {k:22s} = {P[k]}")

print()
print("=== RC3 -> collective gain, per mode ===")
rt=d["fish_rcou_t"]; c5=d["fish_rcou_c5"]; c6=d["fish_rcou_c6"]
it=d["fish_rcin_t"]; rc3=d["fish_rcin_c3"]
mt=d["fish_mode_t"]; mm=d["fish_mode"]
coll=((c5-1425.0)-(c6-1500.0))/2.0
rc3_i=np.interp(rt,it,rc3)
mode_i=mm[np.clip(np.searchsorted(mt,rt,side="right")-1,0,len(mm)-1)].astype(int)
for m,nm in ((0,"STABILIZE"),(2,"ALT_HOLD")):
    k=mode_i==m
    if k.sum()<500: continue
    g=np.polyfit(rc3_i[k]-1500,coll[k],1)
    print(f"  {nm:10s} coll = {g[0]:+.3f}*(RC3-1500) {g[1]:+.1f}   corr={np.corrcoef(rc3_i[k],coll[k])[0,1]:+.3f}")
    for pwm in (1400,1450,1550,1600):
        print(f"      RC3={pwm} -> coll {g[0]*(pwm-1500)+g[1]:+7.1f} us")

print()
print("=== how much ALT_HOLD time is past the +190us stall knee? ===")
k=mode_i==2
for thr in (160,190,230,300):
    print(f"  coll > +{thr}: {100*np.mean(coll[k]>thr):5.2f}% of ALT_HOLD  "
          f"({np.sum(coll[k]>thr)*np.median(np.diff(rt))/60:5.1f} min)")
print(f"  coll < -190 (up side): {100*np.mean(coll[k]<-190):5.2f}%")

print()
print("=== CTUN: what descent rate was the controller asking for? ===")
if "fish_ctun_dcrt" in d.files:
    ct=d["fish_ctun_t"]; dcrt=d["fish_ctun_dcrt"]; crt=d["fish_ctun_crt"]
    cm=mm[np.clip(np.searchsorted(mt,ct,side="right")-1,0,len(mm)-1)].astype(int)
    k=cm==2
    print(f"  desired climb rate (cm/s): p1 {np.percentile(dcrt[k],1):+8.1f} "
          f"p50 {np.percentile(dcrt[k],50):+8.1f} p99 {np.percentile(dcrt[k],99):+8.1f}")
    print(f"  actual  climb rate (cm/s): p1 {np.percentile(crt[k],1):+8.1f} "
          f"p50 {np.percentile(crt[k],50):+8.1f} p99 {np.percentile(crt[k],99):+8.1f}")
    for lim in (30,50,100,150):
        print(f"  |desired| > {lim:3d} cm/s : {100*np.mean(np.abs(dcrt[k])>lim):5.2f}% of ALT_HOLD")
