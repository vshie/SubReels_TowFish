import numpy as np
d=np.load("mission_241.npz",allow_pickle=True)

# What the extension manages, and what it now targets.
TARGETS={"ATC_ANG_RLL_P":0.0,"ATC_RAT_RLL_D":0.0072,"ATC_RAT_RLL_FLTE":3.0,
         "ATC_RAT_RLL_FLTD":4.0}
BOAT={"TURN_RADIUS":2.5,"WP_PIVOT_ANGLE":0.0,"WP_SPEED":1.0,"CRUISE_SPEED":1.0}

def report(tag, targets):
    names=[str(x) for x in d[f"{tag}_parm_names"]]
    vals=[float(v) for v in d[f"{tag}_parm_vals"]]
    P=dict(zip(names,vals))
    ct=d.get(f"{tag}_parm_chg_t"); cn=d.get(f"{tag}_parm_chg_name")
    co=d.get(f"{tag}_parm_chg_old"); cnew=d.get(f"{tag}_parm_chg_new")
    chg={}
    if ct is not None:
        for t,n,o,v in zip(ct,[str(x) for x in cn],co,cnew):
            chg.setdefault(n,[]).append((float(t),float(o),float(v)))
    t0=float(ct.min()) if ct is not None and len(ct) else 0.0
    print(f"\n{'='*76}\n{tag.upper()} -- parameters the extension manages\n{'='*76}")
    print(f"{'param':20s} {'end of log':>11s} {'ext target':>11s}  {'':s}")
    for k,tv in targets.items():
        cur=P.get(k)
        if cur is None:
            print(f"{k:20s} {'not in log':>11s} {tv:11.4g}"); continue
        ok=abs(cur-tv)<=max(1e-4,abs(tv)*1e-3)
        mark="MATCH" if ok else "DIFFERS"
        note=""
        if k in chg:
            note="  changed in flight: "+" then ".join(
                f"{o:g}->{v:g}" for _,o,v in chg[k])
        print(f"{k:20s} {cur:11.4g} {tv:11.4g}  {mark}{note}")
    print(f"\n  total params changed mid-log on {tag}: "
          f"{len(chg)}"+(f" -> {', '.join(sorted(chg))}" if chg and len(chg)<=25 else ""))
    return chg

fc=report("fish",TARGETS)
bc=report("boat",BOAT)

# any change at all to the managed names?
print(f"\n{'='*76}\nwere the managed parameters touched during the survey at all?\n{'='*76}")
for tag,chg,targets in (("fish",fc,TARGETS),("boat",bc,BOAT)):
    hit=[k for k in targets if k in chg]
    print(f"  {tag}: {'yes -> '+', '.join(hit) if hit else 'no -- constant for the whole log'}")
