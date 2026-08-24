import numpy as np
d=np.load("mission_241.npz",allow_pickle=True)
P=dict(zip([str(x) for x in d["fish_parm_names"]],[float(v) for v in d["fish_parm_vals"]]))
t=d["fish_rcou_t"]; C5,C6=d["fish_rcou_c5"],d["fish_rcou_c6"]
at,ar=d["fish_att_t"],d["fish_att_roll"]
roll=np.interp(t,at,ar)
bt,bd=d["fish_baro_t"],d["fish_baro_alt"]; dep=np.interp(t,bt,-bd)
mt,mv=d["fish_mode_t"],d["fish_mode"]
mode=np.array([mv[max(0,np.searchsorted(mt,x,'right')-1)] for x in t])

print(f"SERVO5_TRIM = {P['SERVO5_TRIM']:.0f}   SERVO6_TRIM = {P['SERVO6_TRIM']:.0f} (REVERSED={P['SERVO6_REVERSED']:.0f})")
print(f"  -> the two wings sit {abs(P['SERVO5_TRIM']-P['SERVO6_TRIM']):.0f} us apart at mixer neutral\n")

# differential = roll-producing term. At mixer neutral both should be at trim.
diff=(C5-P['SERVO5_TRIM'])-(1500-C6-(1500-P['SERVO6_TRIM']))
near=np.abs((C5-P['SERVO5_TRIM'])+(1500-C6-(1500-P['SERVO6_TRIM'])))<20  # collective ~0
under=dep>1.0
for nm,sel in (("STABILIZE",mode==0),("ALT_HOLD",mode==2),("MANUAL",mode==19)):
    s=sel&under
    if s.sum()<200: continue
    print(f"{nm:10s} roll: median {np.median(roll[s]):+6.2f} deg  mean {roll[s].mean():+6.2f}  "
          f"sd {roll[s].std():5.2f}  n={s.sum()}")
s=under&near&((mode==0)|(mode==2))
if s.sum()>100:
    print(f"\nnear-zero collective, below 1 m: roll median {np.median(roll[s]):+.2f} deg (n={s.sum()})")
print(f"\nwhole-mission roll median below 1 m: {np.median(roll[under]):+.2f} deg")
print("a persistent non-zero roll median points at the trim split, not the controller.")
