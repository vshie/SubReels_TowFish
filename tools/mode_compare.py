import numpy as np
d=np.load("mission_241.npz",allow_pickle=True)
OFF=-2004.8
rt=d["fish_rcou_t"]; c5=d["fish_rcou_c5"]; c6=d["fish_rcou_c6"]
bt=d["fish_baro_t"]; depth=-d["fish_baro_alt"]
at=d["fish_att_t"]; roll=d["fish_att_roll"]
it=d["fish_rcin_t"]; rc3=d["fish_rcin_c3"]
mt=d["fish_mode_t"]; mm=d["fish_mode"]
gt=d["boat_gps_t"]+OFF; gspd=d["boat_gps_spd"]
coll=((c5-1425.0)-(c6-1500.0))/2.0
depth_i=np.interp(rt,bt,depth); roll_i=np.interp(rt,at,roll)
rc3_i=np.interp(rt,it,rc3); spd_i=np.interp(rt,gt,gspd,left=np.nan,right=np.nan)
mode_i=mm[np.clip(np.searchsorted(mt,rt,side="right")-1,0,len(mm)-1)].astype(int)
dt=np.median(np.diff(rt))

print("=== mode comparison, boat underway (0.6-1.1 m/s), fish below 1 m ===")
base=(spd_i>0.6)&(spd_i<1.1)&(depth_i>1.0)
hdr=f"{'metric':38s} {'STABILIZE':>12} {'ALT_HOLD':>12}"
print(hdr); print("-"*len(hdr))
rows={}
for m,nm in ((0,"STABILIZE"),(2,"ALT_HOLD")):
    k=base&(mode_i==m); rows[nm]={}
    r=rows[nm]
    r["minutes underway"]=k.sum()*dt/60
    r["depth median (m)"]=np.median(depth_i[k])
    r["depth IQR (m)"]=np.percentile(depth_i[k],75)-np.percentile(depth_i[k],25)
    r["|roll| median (deg)"]=np.median(np.abs(roll_i[k]))
    r["|roll| p90 (deg)"]=np.percentile(np.abs(roll_i[k]),90)
    r["|roll| p99 (deg)"]=np.percentile(np.abs(roll_i[k]),99)
    r["% time |roll|>25 deg"]=100*np.mean(np.abs(roll_i[k])>25)
    r["% time |roll|>45 deg"]=100*np.mean(np.abs(roll_i[k])>45)
    r["% time |roll|>90 deg"]=100*np.mean(np.abs(roll_i[k])>90)
    r["collective median (us)"]=np.median(coll[k])
    r["collective p1 (us)"]=np.percentile(coll[k],1)
    r["collective p99 (us)"]=np.percentile(coll[k],99)
    r["% time past +190us stall knee"]=100*np.mean(coll[k]>190)
for key in rows["STABILIZE"]:
    a=rows["STABILIZE"][key]; b=rows["ALT_HOLD"][key]
    print(f"{key:38s} {a:12.2f} {b:12.2f}")

print()
print("=== roll excursions >45 deg: what mode and what collective? ===")
k=base&(np.abs(roll_i)>45)
print(f"  total samples |roll|>45 while underway: {k.sum()} ({k.sum()*dt/60:.1f} min)")
for m,nm in ((0,"STABILIZE"),(2,"ALT_HOLD")):
    kk=k&(mode_i==m)
    print(f"    {nm:10s} {kk.sum():6d} ({100*kk.sum()/max(k.sum(),1):5.1f}% of excursions)"
          f"  median coll {np.median(coll[kk]) if kk.sum() else float('nan'):+7.1f} us")
print(f"  fraction of all >45deg excursions with coll > +190: "
      f"{100*np.mean(coll[k]>190):.1f}%")
print(f"  fraction of all >45deg excursions with coll < -190: "
      f"{100*np.mean(coll[k]<-190):.1f}%")

print()
print("=== is the extension's RC3 jog itself ever past the knee? ===")
for m,nm in ((0,"STABILIZE"),(2,"ALT_HOLD")):
    k=(mode_i==m)&(np.abs(rc3_i-1400)<12)   # jog_down_pwm
    if k.sum()<50: continue
    print(f"  {nm:10s} RC3~1400 n={k.sum():6d}  coll median {np.median(coll[k]):+7.1f} us"
          f"  p99 {np.percentile(coll[k],99):+7.1f}  % past knee {100*np.mean(coll[k]>190):5.1f}%")
