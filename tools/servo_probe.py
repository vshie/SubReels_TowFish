import numpy as np
d = np.load("mission_241.npz", allow_pickle=True)
keys = sorted(k for k in d.files if k.startswith("fish_"))
print("fish keys:", keys)
print()
MODES={0:"STABILIZE",1:"ACRO",2:"ALT_HOLD",3:"AUTO",4:"GUIDED",7:"CIRCLE",
       9:"SURFACE",16:"POSHOLD",19:"MANUAL",20:"MOTOR_DETECT"}
mt,mm = d["fish_mode_t"], d["fish_mode"]
print("fish mode timeline:")
for i,(t,m) in enumerate(zip(mt,mm)):
    end = mt[i+1] if i+1<len(mt) else d["fish_baro_t"][-1]
    print(f"  t={t:8.1f} -> {end:8.1f}  ({(end-t)/60:6.2f} min)  {MODES.get(int(m),int(m))}")
print()
for ch in ("c1","c2","c3","c4","c5","c6"):
    k=f"fish_rcou_{ch}"
    if k in d.files:
        v=d[k]
        uniq=np.unique(np.round(v/10)*10)
        print(f"RCOU {ch.upper()}: n={len(v)} min={v.min():.0f} max={v.max():.0f} "
              f"mean={v.mean():.1f} std={v.std():.1f} distinct(10us)={len(uniq)}")
print()
for ch in ("c1","c2","c3","c4"):
    k=f"fish_rcin_{ch}"
    if k in d.files:
        v=d[k]
        print(f"RCIN {ch.upper()}: n={len(v)} min={v.min():.0f} max={v.max():.0f} "
              f"mean={v.mean():.1f} std={v.std():.1f}")
print()
print("baro depth: n=%d  alt range %.2f .. %.2f" % (len(d["fish_baro_alt"]),
      d["fish_baro_alt"].min(), d["fish_baro_alt"].max()))
print("ctun keys present:", [k for k in d.files if "ctun" in k])
