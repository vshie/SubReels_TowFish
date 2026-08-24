import re
import sys

from pymavlink import mavutil

path = sys.argv[1]
pat = re.compile(sys.argv[2])
mlog = mavutil.mavlink_connection(path)
out = {}
while True:
    m = mlog.recv_match(type=["PARM"])
    if m is None:
        break
    if pat.search(m.Name):
        out[m.Name] = m.Value
for k in sorted(out):
    print(f"{k:24s} {out[k]}")
