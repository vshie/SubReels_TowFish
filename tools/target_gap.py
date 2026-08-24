import numpy as np
d=np.load("mission_241.npz",allow_pickle=True)
P=dict(zip([str(x) for x in d["fish_parm_names"]],[float(v) for v in d["fish_parm_vals"]]))
# targets the extension already ships for the towfish
tgt={"ATC_ANG_RLL_P":0.00,"ATC_RAT_RLL_D":0.0072,
     "ATC_RAT_RLL_FLTE":3.0,"ATC_RAT_RLL_FLTD":4.0}
print("existing extension towfish targets vs what mission 241 actually flew")
print(f"{'param':20s} {'target':>9s} {'on vehicle':>11s}  {'':s}")
for k,v in tgt.items():
    cur=P.get(k)
    ok = abs(cur-v)<=max(1e-4,abs(v)*1e-3)
    ratio = "" if v==0 or cur is None else f"  ({cur/v:.2g}x target)" if v else ""
    print(f"{k:20s} {v:9.4f} {cur:11.4f}  {'MATCH' if ok else 'UNSET'}{ratio}")
