import numpy as np
d=np.load("mission_241.npz",allow_pickle=True)
MODES={0:"STABILIZE",2:"ALT_HOLD",19:"MANUAL"}

bt=d["fish_baro_t"]; depth=-d["fish_baro_alt"]          # positive down, m
rt=d["fish_rcou_t"]; c5=d["fish_rcou_c5"]; c6=d["fish_rcou_c6"]
it=d["fish_rcin_t"]; rc3=d["fish_rcin_c3"]
at=d["fish_att_t"];  roll=d["fish_att_roll"]; pitch=d["fish_att_pitch"]
mt=d["fish_mode_t"]; mm=d["fish_mode"]

print("=== C5 / C6 relationship ===")
n=min(len(c5),len(c6))
print(f"  corr(C5,C6) = {np.corrcoef(c5[:n],c6[:n])[0,1]:+.4f}")
print(f"  C5+C6: mean {np.mean(c5+c6):.1f} std {np.std(c5+c6):.1f}")
print(f"  C5-C6: mean {np.mean(c5-c6):.1f} std {np.std(c5-c6):.1f}")
# collective = deflection from each channel's own trim, sign-corrected
coll = ((c5-1425.0) - (c6-1500.0))/2.0
print(f"  collective ((C5-1425)-(C6-1500))/2 : mean {coll.mean():+.1f} std {coll.std():.1f} "
      f"min {coll.min():+.0f} max {coll.max():+.0f}")

# resample everything onto the RCOU clock
depth_i = np.interp(rt, bt, depth)
rc3_i   = np.interp(rt, it, rc3)
roll_i  = np.interp(rt, at, roll)
pitch_i = np.interp(rt, at, pitch)
# mode at each sample
midx = np.searchsorted(mt, rt, side="right")-1
midx = np.clip(midx,0,len(mm)-1)
mode_i = mm[midx].astype(int)

# depth rate: central difference over a ~2 s window
dt = np.median(np.diff(rt)); w=max(1,int(round(1.0/dt)))
dr = np.full_like(depth_i, np.nan)
dr[w:-w] = (depth_i[2*w:]-depth_i[:-2*w])/(rt[2*w:]-rt[:-2*w])

print()
print("=== time in mode (RCOU samples) ===")
for m in (0,2,19):
    k=mode_i==m
    if k.sum(): print(f"  {MODES[m]:10s} n={k.sum():6d}  {k.sum()*dt/60:6.1f} min")

print()
print("=== RC3 input vs wing collective, per mode ===")
print(f"{'mode':11s} {'RC3 mean':>9} {'RC3 std':>8} {'coll mean':>10} {'coll std':>9} {'corr':>7}")
for m in (0,2):
    k=(mode_i==m)
    if k.sum()<100: continue
    c=np.corrcoef(rc3_i[k],coll[k])[0,1]
    print(f"{MODES[m]:11s} {rc3_i[k].mean():9.1f} {rc3_i[k].std():8.1f} "
          f"{coll[k].mean():+10.1f} {coll[k].std():9.1f} {c:+7.3f}")

print()
print("=== passive trim: what does the fish do with RC3 at neutral? ===")
for m in (0,2):
    k=(mode_i==m)&(np.abs(rc3_i-1500)<15)&np.isfinite(dr)
    if k.sum()<100: continue
    print(f"  {MODES[m]:10s} RC3~neutral n={k.sum():6d} ({k.sum()*dt/60:5.1f} min)")
    print(f"      depth      mean {depth_i[k].mean():5.2f} m  median {np.median(depth_i[k]):5.2f}  p10 {np.percentile(depth_i[k],10):5.2f}  p90 {np.percentile(depth_i[k],90):5.2f}")
    print(f"      depth rate mean {dr[k].mean():+5.3f} m/s median {np.median(dr[k]):+5.3f}")
    print(f"      wing coll  mean {coll[k].mean():+5.1f} us  median {np.median(coll[k]):+5.1f}")

print()
print("=== wing deflection -> depth rate transfer, ALT_HOLD ===")
k=(mode_i==2)&np.isfinite(dr)
bins=[(-260,-200),(-200,-150),(-150,-100),(-100,-50),(-50,-15),(-15,15),
      (15,50),(50,100),(100,150),(150,200),(200,260)]
print(f"{'coll us':>14} {'n':>7} {'%time':>6} {'depth rate m/s':>16} {'depth m':>9} {'|roll| p90':>11}")
for lo,hi in bins:
    kk=k&(coll>=lo)&(coll<hi)
    if kk.sum()<50: continue
    print(f"{lo:+5.0f}..{hi:+5.0f} {kk.sum():7d} {100*kk.sum()/k.sum():5.1f}% "
          f"{dr[kk].mean():+8.3f} +-{dr[kk].std():5.3f} {depth_i[kk].mean():9.2f} "
          f"{np.percentile(np.abs(roll_i[kk]),90):11.1f}")

print()
print("=== saturation: how often is the wing at the stop? ===")
lim=np.percentile(np.abs(coll),99.9)
for m in (0,2):
    k=(mode_i==m)
    if k.sum()<100: continue
    for thr in (200,230,250):
        f=100*np.mean(np.abs(coll[k])>thr)
        print(f"  {MODES[m]:10s} |coll| > {thr:3d} us : {f:5.2f}% of samples")
print(f"  observed |coll| p99.9 = {lim:.0f} us ; hard max = {np.abs(coll).max():.0f} us")
