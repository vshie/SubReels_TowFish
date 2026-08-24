import numpy as np
d=np.load("mission_241.npz",allow_pickle=True)
P=dict(zip([str(x) for x in d["fish_parm_names"]],[float(v) for v in d["fish_parm_vals"]]))

t=d["fish_rcou_t"]; C=np.stack([d[f"fish_rcou_c{i}"] for i in range(1,7)])
bt,bd=d["fish_baro_t"],d["fish_baro_alt"]
depth=-bd
dep=np.interp(t,bt,depth)
mt,mv=d["fish_mode_t"],d["fish_mode"]
mode=np.array([mv[max(0,np.searchsorted(mt,x,'right')-1)] for x in t])
at,ar=d["fish_att_t"],d["fish_att_roll"]
roll=np.interp(t,at,ar)

# depth rate, smoothed over ~1s
dt=np.median(np.diff(t)); w=max(3,int(round(1.0/dt)))
k=np.ones(w)/w
ds=np.convolve(dep,k,'same')
rate=np.gradient(ds,t)          # m/s, +ve = descending

coll=(C[4]-1425)-(1500-C[5])    # SERVO5 trim 1425, SERVO6 reversed trim 1500
coll=((C[4]-1425)+(1500-C[5]))/2.0
under=dep>1.0

print("="*74)
print("1. ACHIEVED VERTICAL RATE ENVELOPE  (fish deeper than 1 m)")
print("="*74)
for nm,sel in (("STABILIZE",mode==0),("ALT_HOLD",mode==2),("MANUAL",mode==19)):
    s=sel&under
    if s.sum()<50: continue
    r=rate[s]
    print(f"{nm:10s} n={s.sum():6d}  descend p95={np.percentile(r,95)*100:6.1f} "
          f"p99={np.percentile(r,99)*100:6.1f}   ascend p5={np.percentile(r,5)*100:6.1f} "
          f"p1={np.percentile(r,1)*100:6.1f} cm/s")

# sustained rates: require the rate to hold for >=3 s
print()
print("sustained (rate held same sign for >= 3 s), ALT_HOLD+STABILIZE, below 1 m:")
s=under&((mode==0)|(mode==2))
n3=max(3,int(round(3.0/dt)))
for lbl,sgn in (("descend",+1),("ascend",-1)):
    v=sgn*rate.copy(); v[~s]=np.nan
    # rolling min over n3 samples -> the rate sustained across the window
    ok=[]
    for i in range(0,len(v)-n3):
        win=v[i:i+n3]
        if np.all(np.isfinite(win)) and np.all(win>0): ok.append(win.min())
    ok=np.array(ok)
    if len(ok): print(f"  {lbl:8s} p90={np.percentile(ok,90)*100:5.1f} "
                      f"p99={np.percentile(ok,99)*100:5.1f} max={ok.max()*100:5.1f} cm/s  (n={len(ok)})")

print()
print("="*74)
print("2. WING SERVO TRAVEL vs CONFIGURED LIMITS")
print("="*74)
print(f"MOT_PWM_MIN/MAX  = {P['MOT_PWM_MIN']:.0f} / {P['MOT_PWM_MAX']:.0f}   <- what actually drives the servos")
print(f"SERVO5_MIN/MAX   = {P['SERVO5_MIN']:.0f} / {P['SERVO5_MAX']:.0f}   (trim {P['SERVO5_TRIM']:.0f})")
print(f"SERVO6_MIN/MAX   = {P['SERVO6_MIN']:.0f} / {P['SERVO6_MAX']:.0f}   (trim {P['SERVO6_TRIM']:.0f}, reversed)")
for i in (5,6):
    c=C[i-1]
    lo,hi=P[f'SERVO{i}_MIN'],P[f'SERVO{i}_MAX']
    print(f"  C{i} observed {c.min():.0f}..{c.max():.0f}   "
          f"beyond servo limits: {100*np.mean((c<lo)|(c>hi)):5.2f}% of samples "
          f"({int(np.sum((c<lo)|(c>hi)))} of {len(c)})")

print()
print("="*74)
print("3. DO THRUSTER OUTPUTS 1-4 SATURATE?  (they share the mixer budget)")
print("="*74)
for i in range(1,5):
    c=C[i-1]
    lo,hi=P[f'SERVO{i}_MIN'],P[f'SERVO{i}_MAX']
    sat=np.mean((c<=lo+2)|(c>=hi-2))
    print(f"  C{i} range {c.min():.0f}..{c.max():.0f} mean {c.mean():7.1f} sd {c.std():6.1f}  "
          f"at-rail {100*sat:5.2f}%")
print(f"  corr(C1,C2)={np.corrcoef(C[0],C[1])[0,1]:+.3f}  corr(C1,C3)={np.corrcoef(C[0],C[2])[0,1]:+.3f}")
print(f"  C1+C2 spread: {np.ptp(C[0]+C[1]):.0f} us  (0 => pure mirror, one common signal)")
