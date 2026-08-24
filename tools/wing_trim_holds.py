import numpy as np
d=np.load("mission_241.npz",allow_pickle=True)
P=dict(zip([str(x) for x in d["fish_parm_names"]],[float(v) for v in d["fish_parm_vals"]]))
t=d["fish_rcou_t"]; C5,C6=d["fish_rcou_c5"].astype(float),d["fish_rcou_c6"].astype(float)
at,ar=d["fish_att_t"],d["fish_att_roll"]; roll=np.interp(t,at,ar)
bt,bd=d["fish_baro_t"],d["fish_baro_alt"]; dep=np.interp(t,bt,-bd)
mt,mv=d["fish_mode_t"],d["fish_mode"]
mode=np.array([mv[max(0,np.searchsorted(mt,x,'right')-1)] for x in t])
dt=np.median(np.diff(t)); w=max(3,int(round(1.0/dt)))
rate=np.gradient(np.convolve(dep,np.ones(w)/w,'same'),t)

NEUT=(P['MOT_PWM_MIN']+P['MOT_PWM_MAX'])/2.0
print(f"mixer neutral = (MOT_PWM_MIN+MAX)/2 = {NEUT:.1f} us   "
      f"(MOT_5_DIRECTION={P['MOT_5_DIRECTION']:.0f}, MOT_6_DIRECTION={P['MOT_6_DIRECTION']:.0f})")
sym =(C5+C6)/2.0-NEUT      # both wings same way
anti=(C5-C6)/2.0           # wings opposite

print("\nwhich term is which? correlate against depth rate and roll")
und=dep>1.0
for nm,v in (("symmetric (C5+C6)/2",sym),("antisymmetric (C5-C6)/2",anti)):
    print(f"  {nm:24s} vs depth-rate r={np.corrcoef(v[und],rate[und])[0,1]:+.3f}"
          f"   vs roll r={np.corrcoef(v[und],roll[und])[0,1]:+.3f}")

print("\n=== does the loop hold a steady offset to level unequal wings? ===")
print("(SERVO5_TRIM is bypassed for motor outputs, so any standing offset is the controller)")
lvl=[]
for nm,sel in (("STABILIZE",mode==0),("ALT_HOLD",mode==2),("MANUAL",mode==19),
               ("DISARMED-ish",np.abs(C5-NEUT)+np.abs(C6-NEUT)<4)):
    s=sel&und
    if s.sum()<300: continue
    r=roll[s]
    print(f"  {nm:12s} n={s.sum():6d}  roll median {np.median(r):+6.2f} deg   "
          f"roll-producing term median {np.median(sym[s]):+7.1f} us  "
          f"(IQR {np.percentile(sym[s],25):+.0f}..{np.percentile(sym[s],75):+.0f})")
    lvl.append((nm,np.median(sym[s])))

# the standing offset while actually holding level
near_lvl=und&(np.abs(roll)<5)&((mode==0)|(mode==2))
print(f"\n  while holding within +-5 deg of level (n={near_lvl.sum()}):")
print(f"    roll-producing term = {np.median(sym[near_lvl]):+.1f} us median, "
      f"mean {sym[near_lvl].mean():+.1f}, sd {sym[near_lvl].std():.1f}")
print(f"    SERVO5_TRIM offset from 1500 was {P['SERVO5_TRIM']-1500:+.0f} us (no effect on a motor output)")
print("\n  raw servo values while level:")
print(f"    C5 median {np.median(C5[near_lvl]):7.1f}   C6 median {np.median(C6[near_lvl]):7.1f}"
      f"   difference {np.median(C5[near_lvl])-np.median(C6[near_lvl]):+.1f} us")

print("\n=== how much of the range does roll control actually need? ===")
s=und&((mode==0)|(mode==2))
for q in (50,75,90,99):
    print(f"  |roll-producing term| p{q}: {np.percentile(np.abs(sym[s]),q):6.1f} us")
print(f"  |collective| p50/p90/p99: {np.percentile(np.abs(anti[s]),50):.0f} / "
      f"{np.percentile(np.abs(anti[s]),90):.0f} / {np.percentile(np.abs(anti[s]),99):.0f} us")
print(f"  worst single-servo excursion from neutral: "
      f"{max(np.abs(C5-NEUT).max(),np.abs(C6-NEUT).max()):.0f} us")
