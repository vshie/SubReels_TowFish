import numpy as np
d=np.load("mission_241.npz",allow_pickle=True)
OFF=-2004.8   # fish_TimeUS = boat_TimeUS + OFF

bt=d["fish_baro_t"]; depth=-d["fish_baro_alt"]
rt=d["fish_rcou_t"]; c5=d["fish_rcou_c5"]; c6=d["fish_rcou_c6"]
it=d["fish_rcin_t"]; rc3=d["fish_rcin_c3"]
at=d["fish_att_t"];  roll=d["fish_att_roll"]; pitch=d["fish_att_pitch"]
mt=d["fish_mode_t"]; mm=d["fish_mode"]
gt=d["boat_gps_t"]+OFF; gspd=d["boat_gps_spd"]   # boat clock -> fish clock

# mixer decomposition. ArduSub vectored: motors 5/6 are the vertical pair,
# carrying collective heave plus differential roll.
t5,t6=1425.0,1500.0
coll = ((c5-t5)-(c6-t6))/2.0     # + = dive  (verified vs RC3 corr = -0.985)
diff = ((c5-t5)+(c6-t6))/2.0

depth_i=np.interp(rt,bt,depth); rc3_i=np.interp(rt,it,rc3)
roll_i =np.interp(rt,at,roll);  pitch_i=np.interp(rt,at,pitch)
spd_i  =np.interp(rt,gt,gspd,left=np.nan,right=np.nan)
mode_i = mm[np.clip(np.searchsorted(mt,rt,side="right")-1,0,len(mm)-1)].astype(int)
dt=np.median(np.diff(rt)); w=max(1,int(round(1.0/dt)))
dr=np.full_like(depth_i,np.nan); dr[w:-w]=(depth_i[2*w:]-depth_i[:-2*w])/(rt[2*w:]-rt[:-2*w])

print("=== mixer sanity ===")
print(f"  corr(coll, RC3)  = {np.corrcoef(coll,rc3_i)[0,1]:+.3f}   (expect strong -ve)")
print(f"  corr(diff, roll) = {np.corrcoef(diff[np.isfinite(roll_i)],roll_i[np.isfinite(roll_i)])[0,1]:+.3f}   (expect roll term)")
print(f"  coll range {coll.min():+.0f} .. {coll.max():+.0f} us   diff range {diff.min():+.0f} .. {diff.max():+.0f} us")

print()
print("=== full-range collective -> depth rate (ALT_HOLD, speed 0.7-1.1 m/s) ===")
k=(mode_i==2)&np.isfinite(dr)&(spd_i>0.7)&(spd_i<1.1)
edges=list(range(-350,401,50))
print(f"{'coll us':>14} {'n':>6} {'%':>5} {'dz/dt m/s':>18} {'|roll|p90':>10} {'|roll|p99':>10} {'depth m':>8}")
for lo,hi in zip(edges[:-1],edges[1:]):
    kk=k&(coll>=lo)&(coll<hi)
    if kk.sum()<40: continue
    print(f"{lo:+5.0f}..{hi:+5.0f} {kk.sum():6d} {100*kk.sum()/k.sum():4.1f}% "
          f"{dr[kk].mean():+8.3f} +-{dr[kk].std():5.3f} "
          f"{np.percentile(np.abs(roll_i[kk]),90):10.1f} {np.percentile(np.abs(roll_i[kk]),99):10.1f} "
          f"{depth_i[kk].mean():8.2f}")

print()
print("=== linear fit of the unstalled band, and the trim point ===")
kk=k&(coll>-300)&(coll<130)
A=np.polyfit(coll[kk],dr[kk],1)
print(f"  fit over coll in [-300,+130]: dz/dt = {A[0]*1000:+.4f} (m/s per 1000us) * coll + {A[1]:+.4f}")
print(f"  zero-rate trim point: coll = {-A[1]/A[0]:+.1f} us   (negative = airframe wants to dive)")
print(f"  authority per 100 us: {A[0]*100:+.4f} m/s")

print()
print("=== stall onset: |roll| p90 vs collective, dive side only ===")
for lo,hi in [(50,100),(100,130),(130,160),(160,190),(190,230),(230,300),(300,400)]:
    kk=k&(coll>=lo)&(coll<hi)
    if kk.sum()<30: continue
    print(f"  coll {lo:+4.0f}..{hi:+4.0f}  n={kk.sum():5d}  |roll|p90={np.percentile(np.abs(roll_i[kk]),90):6.1f}  "
          f"dz/dt={dr[kk].mean():+.3f}  roll>45deg in {100*np.mean(np.abs(roll_i[kk])>45):5.1f}% of samples")

print()
print("=== V^2 test: does authority scale with boat speed squared? ===")
print(f"{'speed band':>14} {'n':>6} {'slope m/s per 1000us':>22} {'ratio to 0.9-1.1':>17}")
ref=None
for lo,hi in [(0.4,0.6),(0.6,0.75),(0.75,0.9),(0.9,1.1)]:
    kk=(mode_i==2)&np.isfinite(dr)&(spd_i>=lo)&(spd_i<hi)&(coll>-300)&(coll<130)
    if kk.sum()<300: continue
    s=np.polyfit(coll[kk],dr[kk],1)[0]*1000
    if hi==1.1: ref=s
    print(f"  {lo:.2f}-{hi:.2f} {kk.sum():8d} {s:22.4f}", end="")
    print(f" {s/ref:17.2f}" if ref else "")
vmid={ (0.4,0.6):0.5,(0.6,0.75):0.68,(0.75,0.9):0.82,(0.9,1.1):1.0 }
print("  expected V^2 ratios vs 1.0 m/s:", {f"{v}":round((v/1.0)**2,2) for v in (0.5,0.68,0.82,1.0)})

print()
print("=== STABILIZE passive equilibrium vs boat speed (RC3 within 15us of neutral) ===")
k=(mode_i==0)&(np.abs(rc3_i-1500)<15)&np.isfinite(spd_i)&np.isfinite(dr)
print(f"{'speed band':>14} {'n':>6} {'depth median':>13} {'depth p90':>10} {'dz/dt':>9} {'coll':>8}")
for lo,hi in [(0.0,0.2),(0.2,0.4),(0.4,0.6),(0.6,0.8),(0.8,1.0),(1.0,1.3)]:
    kk=k&(spd_i>=lo)&(spd_i<hi)
    if kk.sum()<60: continue
    print(f"  {lo:.1f}-{hi:.1f} {kk.sum():9d} {np.median(depth_i[kk]):13.2f} "
          f"{np.percentile(depth_i[kk],90):10.2f} {np.median(dr[kk]):+9.3f} {np.median(coll[kk]):+8.1f}")
