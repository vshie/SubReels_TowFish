#!/usr/bin/env python3
"""Check the wing wiring the physics assumes, using only the 4 Nov log.

Two facts fix it, neither involving how the fish behaved in a turn:

1. Collective: in MANUAL with full-up RC3 (1900 us) the mixer drives the
   wings to their "climb" extreme. Which way each servo goes then tells
   which PWM direction is "more lift" for that wing.
2. Roll: the rate controller's ROut against the equal-PWM (differential)
   component shows which servo's PWM moves which way for a roll-right
   command. A roll-right command must add lift on the port wing, so the
   servo whose "more lift" PWM direction matches the roll-right PWM
   direction is the port wing.

Prints the inferred mapping next to config/towfish.yaml's and exits non-zero
if they disagree.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from towsim.frames import Config  # noqa: E402
from towsim.logs import load_fish  # noqa: E402


def mode_series(fish, t):
    mt = np.asarray(fish["MODE"]["t"])
    mm = np.asarray(fish["MODE"]["Mode"])
    idx = np.searchsorted(mt, t, side="right") - 1
    return np.where(idx >= 0, mm[np.clip(idx, 0, None)], -1)


def main() -> int:
    fish = load_fish()
    cfg = Config()
    t = np.asarray(fish["RCOU"]["t"])
    s5 = np.asarray(fish["RCOU"]["C5"], dtype=float)
    s6 = np.asarray(fish["RCOU"]["C6"], dtype=float)
    rc3 = np.interp(t, fish["RCIN"]["t"], fish["RCIN"]["C3"])
    mode = mode_series(fish, t)

    full_up = (mode == 19) & (rc3 > 1850)
    print(f"MANUAL full-up samples: {full_up.sum()}  SERVO5 median {np.median(s5[full_up]):.0f}  "
          f"SERVO6 median {np.median(s6[full_up]):.0f}")
    s5_up = np.sign(np.median(s5[full_up]) - 1500)   # -1: SERVO5 PWM falls for more lift
    s6_up = np.sign(np.median(s6[full_up]) - 1500)   # +1: SERVO6 PWM rises for more lift

    rt = np.asarray(fish["RATE"]["t"])
    rout = np.interp(t, rt, fish["RATE"]["ROut"])
    stab = np.isin(mode, [0, 2])
    diff = (s5 + s6 - 3000) / 2
    corr = np.corrcoef(rout[stab], diff[stab])[0, 1]
    print(f"corr(ROut, equal-PWM component) in STABILIZE/ALT_HOLD: {corr:+.2f} "
          f"({'roll-right lowers both PWMs' if corr < 0 else 'roll-right raises both PWMs'})")
    # Roll right (ROut > 0) needs the port wing to gain lift.
    roll_right_pwm_dir = -1 if corr < 0 else 1
    port = 5 if s5_up == roll_right_pwm_dir else 6
    stbd = 6 if port == 5 else 5
    inferred = {"port_servo": port, "port_sign": float(s5_up if port == 5 else s6_up),
                "stbd_servo": stbd, "stbd_sign": float(s5_up if stbd == 5 else s6_up)}
    current = {k: cfg[f"wings.{k}"] for k in inferred}
    print("inferred:", inferred)
    print("config:  ", current)
    ok = all(float(inferred[k]) == float(current[k]) for k in inferred)
    print("MATCH" if ok else "MISMATCH -- update config/towfish.yaml wings.* (status log-inferred)")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
