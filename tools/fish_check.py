"""Sanity-check towfish pressure/attitude and the boat sounding continuity."""

import numpy as np
from pymavlink import mavutil

FISH = "/Users/tonywhite/Downloads/towfish mahukona3 8.7.22.BIN"

m = mavutil.mavlink_connection(FISH)
press, alt, t = [], [], []
pitch, roll = [], []
ctun_t, ctun_alt = [], []
while True:
    r = m.recv_match(type=["BARO", "ATT", "CTUN"])
    if r is None:
        break
    ty = r.get_type()
    if ty == "BARO" and r.I == 1:
        press.append(r.Press)
        alt.append(r.Alt)
        t.append(r.TimeUS / 1e6)
    elif ty == "ATT":
        pitch.append(r.Pitch)
        roll.append(r.Roll)
    elif ty == "CTUN":
        ctun_t.append(r.TimeUS / 1e6)
        ctun_alt.append(r.Alt)

press = np.array(press)
alt = np.array(alt)
t = np.array(t)
pitch = np.array(pitch)
roll = np.array(roll)
ctun_alt = np.array(ctun_alt)

print("BARO[1] Press range: %.0f .. %.0f Pa" % (press.min(), press.max()))
print("implied depth from press at max: %.2f m" % ((press.max() - 101520) / (1025 * 9.80665)))
print("BARO[1] rate: %.1f Hz" % (len(t) / (t[-1] - t[0])))
print("ATT Pitch percentiles:", np.percentile(pitch, [0, 1, 25, 50, 75, 99, 100]).round(2))
print("ATT Roll  percentiles:", np.percentile(roll, [0, 1, 25, 50, 75, 99, 100]).round(2))
print("CTUN Alt percentiles:", np.percentile(ctun_alt, [0, 1, 50, 99, 100]).round(2))

# how long was the fish deeper than 1 m?
sub = -alt > 1.0
print("fraction of log with depth > 1 m: %.3f" % sub.mean())
print("fraction of log with depth > 2 m: %.3f" % (-alt > 2.0).mean())
# contiguous submerged window
idx = np.where(sub)[0]
if len(idx):
    print("first/last submerged log-time: %.1f .. %.1f s" % (t[idx[0]], t[idx[-1]]))
