#!/usr/bin/env python3
"""Physics building blocks: frames, foils, cable, rigid body, JSON bridge.

Run with: python3 Simulator/tests/test_physics.py
"""
from __future__ import annotations

import math

import numpy as np

from _check import check, finish
from towsim import hydro
from towsim.cable import Cable
from towsim.fish import TowfishPhysics
from towsim.frames import (Config, euler_from_quat, quat_from_euler, quat_integrate,
                           quat_to_dcm, step_to_body)
from towsim.sitl_json import (JsonBridge, build_servo_packet, parse_servo_packet,
                              state_json)

cfg = Config()

print("frames")
cg = np.array([0.0, 100.0, 0.0])
p = step_to_body([10.0, 0.0, 50.0], cg)   # 10 mm up, 100 mm forward, 50 mm stbd
check("STEP -Y forward maps to body +x", math.isclose(p[0], 0.1))
check("STEP +Z (stbd) maps to body +y", math.isclose(p[1], 0.05))
check("STEP +X (up) maps to body -z", math.isclose(p[2], -0.01))
q = quat_from_euler(0.3, -0.2, 1.1)
r, pi, y = euler_from_quat(q)
check("euler round trip", np.allclose([r, pi, y], [0.3, -0.2, 1.1]))
q2 = quat_integrate(quat_from_euler(0, 0, 0), np.array([0.0, 0.0, 0.5]), 2.0)
check("yaw integration", math.isclose(euler_from_quat(q2)[2], 1.0, abs_tol=1e-9))
R = quat_to_dcm(quat_from_euler(0, 0, math.pi / 2))
check("dcm: body x at yaw 90 points east", np.allclose(R @ [1, 0, 0], [0, 1, 0]))

print("\nfoil and strips")
foil = hydro.Foil(cfg)
a = np.radians(np.array([0.0, 5.0, 12.0, 30.0, 90.0, -5.0]))
slope = np.full(6, hydro.lift_slope(3.8))
cl, cd = foil.coeffs(a, slope, np.full(6, 3.8))
check("CL ~ slope*alpha at 5 deg", math.isclose(cl[1], slope[1] * a[1], rel_tol=0.02))
check("CL odd in alpha", math.isclose(cl[1], -cl[5], rel_tol=1e-9))
check("lift drops past stall (30 deg < 12 deg extrapolated)", cl[3] < slope[3] * a[3] * 0.6)
check("flat plate drag ~cd90 at 90 deg", math.isclose(cd[4], 1.98 + 0.02, rel_tol=0.05))

phys = TowfishPhysics(cfg)
s = phys.strips
check("12 wing strips + 12 tail strips", len(s.c) == 24, str(len(s.c)))
wing = s.group != hydro.GROUP_FIXED
area = float(s.area[wing].sum())
check("wing planform area ~666 cm^2 (report 4.1)", abs(area * 1e4 - 666) < 40, f"{area * 1e4:.0f}")
v = np.array([1.0, 0.0, 0.0])
f0, m0, _ = hydro.strip_forces(s, foil, 1025.0, v, np.zeros(3), np.zeros(3),
                               np.zeros(len(s.c)), np.ones(len(s.c)))
defl = np.where(wing, math.radians(5.0), 0.0)
f1, m1, _ = hydro.strip_forces(s, foil, 1025.0, v, np.zeros(3), np.zeros(3),
                               defl, np.ones(len(s.c)))
check("nose-up wing deflection lifts (force -z)", f1[2] < f0[2] - 1.0, f"{f0[2]:.2f} -> {f1[2]:.2f}")
expect = 0.5 * 1025.0 * area * hydro.lift_slope(3.8) * math.radians(5.0)
check("5 deg of wing at 1 m/s matches 0.5*rho*V^2*S*a*alpha (~12 N)",
      math.isclose(f0[2] - f1[2], expect, rel_tol=0.15), f"{f0[2] - f1[2]:.2f} vs {expect:.2f}")
fr, mr, _ = hydro.strip_forces(s, foil, 1025.0, v, np.array([1.0, 0.0, 0.0]), np.zeros(3),
                               np.zeros(len(s.c)), np.ones(len(s.c)))
check("rolling produces an opposing (damping) roll moment", mr[0] < m0[0] - 0.01,
      f"{m0[0]:.3f} -> {mr[0]:.3f}")
fd, _, _ = hydro.strip_forces(s, foil, 1025.0, v, np.zeros(3), np.zeros(3),
                              defl, np.zeros(len(s.c)))
check("strips in air produce nothing", np.allclose(fd, 0.0))

print("\nwing wiring")
phys.set_servos([0, 0, 0, 0, 1000, 2000])
check("MANUAL full-up PWMs command nose-up (more lift) on both wings",
      phys.defl_cmd[0] > 0 and phys.defl_cmd[1] > 0, str(phys.defl_cmd))
phys.set_servos([0, 0, 0, 0, 1400, 1400])
check("equal PWM drop (roll-right command) lifts port, drops stbd",
      phys.defl_cmd[0] > 0 > phys.defl_cmd[1], str(phys.defl_cmd))
phys.set_servos([0] * 16)
check("undriven outputs leave wings neutral", np.allclose(phys.defl_cmd, 0.0))

print("\ncable")
c = Cable(Config(overrides={"cable.wet_weight": 1.0, "cable.segments": 10,
                            "cable.length": 10.0}), 1025.0)
top = np.array([0.0, 0.0, 0.0])
bottom = np.array([6.0, 0.0, 0.0])
c.lay_straight(top, bottom, np.zeros(3))
for _ in range(60000):
    c.step(0.001, top, np.zeros(3), bottom, np.zeros(3), np.zeros(3))
sag = c.x[5, 2]
# Catenary through (0,0),(6,0) with arc length 10 m: analytic midpoint sag ~3.6 m.
check("hanging cable sags like a catenary (~3.6 m)", 3.2 < sag < 4.0, f"{sag:.2f}")
check("cable never pushes", (c.tension >= 0).all())

# Steady tow: the lumped cable held between the boat and the shape the
# statics solver predicts should pull on the fish with the force the
# solver started from.
import sys as _sys  # noqa: E402
_sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent.parent))
from analysis.tether_statics import tether_shape  # noqa: E402
tc = Config(overrides={"cable.length": 10.0, "cable.segments": 10})
D, N = 7.0, 8.0
s_, x_, z_, F_ = tether_shape(D, N, U=1.0, d=float(tc["cable.diameter"]) / 1000,
                              cn=float(tc["cable.cd_normal"]), ct=float(tc["cable.cd_tangent"]),
                              w=float(tc["cable.wet_weight"]), rho=1000.0, Lmax=10.0,
                              return_force=True)
c = Cable(tc, 1000.0)
top = np.array([0.0, 0.0, 0.0])
fish = np.array([-x_[-1], 0.0, z_[-1]])
c.lay_straight(top, fish, np.zeros(3))
flow = np.array([-1.0, 0.0, 0.0])   # water moving aft past a boat-fixed frame
for _ in range(30000):
    c.step(0.001, top, np.zeros(3), fish, np.zeros(3), flow)
# The lumped model resolves the cable to one segment (1 m), so its tension
# in the last segment should lie on the statics curve somewhere within the
# first metre up from the fish.
u_end = (c.x[-2] - c.x[-1]) / np.linalg.norm(c.x[-2] - c.x[-1])
pull = c.tension[-1] * u_end
k1 = int(np.searchsorted(s_, 1.0))


def _within(v, a, b, tol=0.03):
    lo, hi = min(a, b), max(a, b)
    return lo * (1 - tol) <= v <= hi * (1 + tol)


check("lumped cable tension within one segment of the statics curve (forward)",
      _within(pull[0], F_[0, 0], F_[k1, 0]),
      f"{pull[0]:.2f} vs {F_[0, 0]:.2f}..{F_[k1, 0]:.2f}")
check("lumped cable tension within one segment of the statics curve (up)",
      _within(-pull[2], F_[0, 1], F_[k1, 1]),
      f"{-pull[2]:.2f} vs {F_[0, 1]:.2f}..{F_[k1, 1]:.2f}")

seg_len = np.linalg.norm(np.diff(c.x, axis=0), axis=1)
check("inextensible: no link stretches more than 0.05 % under tow load",
      float(np.max(np.abs(seg_len - c.l0)) / c.l0) < 5e-4,
      f"max {float(np.max(np.abs(seg_len - c.l0)) / c.l0) * 100:.3f} %")

print("\ninextensible tether")
c = Cable(Config(overrides={"cable.wet_weight": 0.0, "cable.segments": 10,
                            "cable.length": 10.0}), 1025.0)
c.lay_straight(np.zeros(3), np.array([3.0, 0.0, 0.0]), np.zeros(3))
for _ in range(200):
    c.step(0.001, np.zeros(3), np.zeros(3), np.array([3.0, 0.0, 0.0]), np.zeros(3), np.zeros(3))
check("slack links (ends 3 m apart on 10 m of tether) carry exactly zero",
      np.all(c.tension == 0.0), f"max {c.tension.max():.3g}")


def _snap_peak(model):
    sc = Config(overrides={"cable.length": 5.0, "cable.segments": 5, "cable.model": model})
    f = TowfishPhysics(sc)
    f.place([-4.0, 0.0, 2.0], quat_from_euler(0, 0, 0), np.array([-1.0, 0.0, 0.0]))
    f.cable.lay_straight(np.array([0.0, 0.0, -0.1]), f.tow_point_ned(), np.zeros(3))
    f.set_servos([0, 0, 0, 0, 1500, 1500])
    peak, stretch = 0.0, 0.0
    for _ in range(4000):
        f.step(0.001, np.array([0.0, 0.0, -0.1]), np.zeros(3))
        peak = max(peak, f.cable.fish_end_tension)
        sl = np.linalg.norm(np.diff(f.cable.x, axis=0), axis=1)
        stretch = max(stretch, float(np.max(sl - f.cable.l0)) / f.cable.l0)
    return peak, stretch


pk_i, st_i = _snap_peak("inextensible")
pk_s, st_s = _snap_peak("spring")
check("slack-to-taut snap: a sharper tension spike than the old spring",
      pk_i > 1.5 * pk_s, f"inextensible {pk_i:.0f} N vs spring {pk_s:.0f} N")
check("the snap does not stretch the tether (< 0.5 %)", st_i < 5e-3, f"{st_i * 100:.2f} %")


def _straight_tow(segments, dt, seconds=40.0):
    sc = Config(overrides={"cable.length": 8.0, "cable.segments": segments})
    f = TowfishPhysics(sc)
    top = np.array([0.0, 0.0, -0.1])
    f.place([-7.0, 0.0, 3.5], quat_from_euler(0, 0, 0), np.array([1.0, 0.0, 0.0]))
    f.cable.lay_straight(top, f.tow_point_ned(), np.array([1.0, 0.0, 0.0]))
    f.set_servos([0, 0, 0, 0, 1650, 1350])          # wings part-way down
    for _ in range(int(seconds / dt)):
        top = top + np.array([1.0, 0.0, 0.0]) * dt
        f.step(dt, top, np.array([1.0, 0.0, 0.0]))
    lay = float(np.hypot(*(top[:2] - f.pos[:2])))
    return lay, float(f.pos[2]), bool(np.all(np.isfinite(f.pos)))


lay1, z1, ok1 = _straight_tow(8, 0.001)
lay2, z2, ok2 = _straight_tow(16, 0.001)
check("straight tow stays bounded", ok1 and ok2 and 1.0 < z1 < 7.9, f"depth {z1:.2f}")
check("1 m and 0.5 m links give the same layback (within 0.1 m)", abs(lay1 - lay2) < 0.1,
      f"{lay1:.2f} vs {lay2:.2f}")
lay3, z3, ok3 = _straight_tow(8, 0.0025)
check("2.5 ms physics step matches 1 ms (layback within 0.1 m)", ok3 and abs(lay3 - lay1) < 0.1,
      f"{lay3:.2f} vs {lay1:.2f}")

print("\nrigid body")
check("total mass matches owner weighing", math.isclose(phys.mass, 3.2205))
check("CG height computed (contents on axis) ~3.9 mm", abs(phys.cg_mm[0] - 3.92) < 0.1)
check("added mass dominates roll inertia", phys.M[3, 3] > 5 * phys.J[0, 0])
f = TowfishPhysics(cfg)
f.place([0, 0, 5.0], quat_from_euler(0, 0, 0), np.zeros(3))
f.cable.lay_straight(f.tow_point_ned() + np.array([0, 0, -0.01]), f.tow_point_ned(), np.zeros(3))
f.set_servos([0, 0, 0, 0, 1500, 1500])
for _ in range(3000):
    f.step(0.001, f.cable.x[0], np.zeros(3))
# +85.6 g net in sea water (report 2.3) against ~18 kg heave mass incl. added
# mass: about 0.2 m of rise in 3 s before drag matters.
check("still water: slightly positive in sea water, rises ~0.2 m in 3 s",
      -0.4 < f.pos[2] - 5.0 < -0.05 and f.vel_ned[2] < 0, f"dz={f.pos[2] - 5.0:.3f}")

print("\nJSON bridge")
pkt = parse_servo_packet(build_servo_packet(400, 7, [1500] * 6))
check("servo packet round trip", pkt.frame_rate == 400 and pkt.frame_count == 7
      and pkt.pwm[:6] == (1500,) * 6)
js = state_json(1.25, [0, 0, 0], [0, 0, -9.8], [1, 2, 3], [1, 0, 0, 0], [0, 0, 0])
check("reply framed by newlines", js.startswith(b"\n") and js.endswith(b"\n"))
import json as _json  # noqa: E402
import socket  # noqa: E402
msg = _json.loads(js.decode())
check("reply carries the fields SIM_JSON needs",
      all(k in msg for k in ("timestamp", "imu", "position", "quaternion", "velocity")))
b = JsonBridge(port=19002, timeout_s=2.0)
sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sk.settimeout(2.0)
sk.sendto(build_servo_packet(400, 1, [1500] * 6), ("127.0.0.1", 19002))
got, new = b.receive()
b.reply(js)
check("bridge replies to the sender", sk.recvfrom(4096)[0] == js and new)
sk.sendto(build_servo_packet(400, 1, [1500] * 6), ("127.0.0.1", 19002))
_, new2 = b.receive()
check("repeated frame_count is flagged as a resend", new2 is False)
b.close()
sk.close()

finish("physics")
