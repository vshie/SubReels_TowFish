import numpy as np
d=np.load("mission_241.npz",allow_pickle=True)
names=d["fish_parm_names"]; vals=d["fish_parm_vals"]
P=dict(zip(names,vals))
print("=== SERVO function / trim / range params ===")
for i in range(1,9):
    row=[]
    for suf in ("FUNCTION","MIN","MAX","TRIM","REVERSED"):
        k=f"SERVO{i}_{suf}"
        if k in P: row.append(f"{suf}={P[k]:.0f}")
    if row: print(f"  SERVO{i}: "+"  ".join(row))
print()
print("=== frame / motor / buoyancy related ===")
for k in sorted(P):
    if any(s in k for s in ("FRAME","MOT_","PILOT_SPEED","SURFACE_DEPTH",
                            "ATC_ANG","THR_","RC3_","RC_FEEL","JS_")):
        print(f"  {k} = {P[k]}")
