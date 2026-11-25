"""6-DOF towfish rigid body coupled to the lumped-mass cable.

State: NED position of the CG, body-to-NED quaternion, body velocity v and
rate omega, plus the two wing servo angles. Forces: gravity at the CG,
buoyancy at the CB (reduced as the hull breaks the surface), strip-theory
foils, element cross-flow drag, lumped axial drag, and the cable at the tow
point. The mass matrix includes the strip/element added mass.
"""
from __future__ import annotations

import math

import numpy as np

from . import hydro
from .cable import Cable
from .frames import G, Config, quat_integrate, quat_to_dcm, step_to_body

HULL_RADIUS = 0.056


def _cg_and_inertia(cfg: Config):
    comps = cfg["mass.components"]
    total = float(cfg["mass.total"])
    m = np.array([c["mass"] for c in comps])
    p = np.array([c["pos"] for c in comps], dtype=float)
    cg = np.array([
        cfg["cg.x_mm"] if cfg["cg.x_mm"] is not None else float((m * p[:, 0]).sum() / m.sum()),
        float(cfg["cg.y_mm"]),
        float(cfg["cg.z_mm"]),
    ])
    J = np.zeros((3, 3))
    for c in comps:
        r = step_to_body(np.array(c["pos"], dtype=float), cg)
        mc = c["mass"]
        J += mc * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
        if c["shape"] == "rod_y":
            L = c["length"] / 1000
            J += np.diag([0.0, mc * L * L / 12, mc * L * L / 12])
        elif c["shape"] == "cylinder_y":
            R, L = c["radius"] / 1000, c["length"] / 1000
            J += np.diag([0.5 * mc * R * R, mc * (3 * R * R + L * L) / 12,
                          mc * (3 * R * R + L * L) / 12])
    # Component masses come from the report's split; scale to the weighed total.
    J *= total / m.sum()
    return cg, J, total


class Bridle:
    """Two-leg bridle tied to a fixed point on the line, legs attached on the
    rail's centreline.

    The legs and the rail make a triangle that swings freely about the line
    through the two attachments (a hinge), so it always turns to contain
    the tether. While the tether's angle from the rail, in that plane, lies
    between the rear leg's angle and 180 deg minus the front leg's, both
    legs are taut and the tow point is the triangle's apex. Pulled flatter
    than that, the front leg goes slack and the fish hangs from the rear
    attachment with the rear leg in line with the tether; pulled further
    aft, the reverse. The tow point is continuous across both changes.
    """

    def __init__(self, cfg: Config, cg_mm):
        b = cfg["tow_point.bridle"]
        self.p1 = step_to_body(np.array(b["front"], dtype=float), cg_mm)   # front attachment
        self.p2 = step_to_body(np.array(b["rear"], dtype=float), cg_mm)    # rear attachment
        self.a = float(b["front_leg"]) / 1000.0
        self.b = float(b["rear_leg"]) / 1000.0
        c = float(np.linalg.norm(self.p1 - self.p2))
        self.u = (self.p1 - self.p2) / c                                    # hinge axis, forward
        self.xa = (self.a ** 2 - self.b ** 2 + c ** 2) / (2 * c)            # apex aft of p1
        self.h = math.sqrt(max(0.0, self.a ** 2 - self.xa ** 2))
        self.foot = self.p1 - self.xa * self.u
        self.psi_rear = math.atan2(self.h, c - self.xa)                     # rear leg's angle
        self.psi_front = math.pi - math.atan2(self.h, self.xa)              # 180 - front leg's
        up = np.array([0.0, 0.0, -1.0])
        self.n = up - (up @ self.u) * self.u
        self.n /= np.linalg.norm(self.n)
        self.state = "both"

    def apex_body(self, d):
        """Tow point (body FRD, m) for a tether direction ``d`` (body unit
        vector from the tow point toward the boat); None = at rest, up."""
        if d is None:
            return self.foot + self.h * self.n
        along = float(d @ self.u)
        perp = d - along * self.u
        pn = float(np.linalg.norm(perp))
        if pn > 1e-6:
            self.n = perp / pn
        psi = math.atan2(pn, along)
        if psi < self.psi_rear:
            self.state = "rear"
            return self.p2 + self.b * d
        if psi > self.psi_front:
            self.state = "front"
            return self.p1 + self.a * d
        self.state = "both"
        return self.foot + self.h * self.n


class TowfishPhysics:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.rho = float(cfg["water.density"])
        self.current = np.array(cfg["water.current_ned"], dtype=float)
        self.cg_mm, self.J, self.mass = _cg_and_inertia(cfg)
        self.r_cb = step_to_body(np.array(cfg["buoyancy.cb"], dtype=float), self.cg_mm)
        self.r_tow = step_to_body(np.array(cfg["tow_point.pos"], dtype=float), self.cg_mm)
        try:
            self.tow_mode = str(cfg["tow_point.mode"])
        except KeyError:
            self.tow_mode = "rigid"
        if self.tow_mode == "bridle":
            self.bridle = Bridle(cfg, self.cg_mm)
            self.r_tow = self.bridle.apex_body(None)
        self.volume = (float(cfg["buoyancy.volume"]) + float(cfg["buoyancy.extra_volume"])) * 1e-6
        self.r_axial = step_to_body(np.array([0.0, float(cfg["body.axial_center_y"]), 0.0]), self.cg_mm)
        self.axial_cda = float(cfg["body.axial_cda"])

        self.strips, self.elements = hydro.build_geometry(cfg, self.cg_mm)
        self.foil = hydro.Foil(cfg)
        housing_v = 2411.78e-6
        A = hydro.added_mass_matrix(self.strips, self.elements, self.rho, housing_v,
                                    float(cfg["added_mass.scale"]))
        M = np.zeros((6, 6))
        M[:3, :3] = self.mass * np.eye(3)
        M[3:, 3:] = self.J
        self.M = M + A
        self.Minv = np.linalg.inv(self.M)
        self._set_tow_point(self.r_tow)

        self.k_defl = math.radians(float(cfg["wings.deflection_per_us"]))
        self.defl_lim = math.radians(float(cfg["wings.deflection_limit"]))
        self.servo_rate = math.radians(float(cfg["wings.servo_rate"]))
        self.port_servo = int(cfg["wings.port_servo"])
        self.port_sign = float(cfg["wings.port_sign"])
        self.stbd_servo = int(cfg["wings.stbd_servo"])
        self.stbd_sign = float(cfg["wings.stbd_sign"])

        self.cable = Cable(cfg, self.rho)

        # State.
        self.pos = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.v = np.zeros(3)
        self.w = np.zeros(3)
        self.defl = np.zeros(2)  # port, stbd (rad, nose-up)
        self.defl_cmd = np.zeros(2)
        self.accel_body = np.array([0.0, 0.0, -G])
        self.last_tow_force = np.zeros(3)
        self.last_alpha = np.zeros(len(self.strips.c))
        self.jig = False

    def _set_tow_point(self, r):
        """Tow point (body FRD, m) and its effective inverse mass: how the
        tow point's velocity answers an impulse there, rotation included."""
        self.r_tow = r
        skew = np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]])
        Jt = np.hstack([np.eye(3), -skew])
        self.K_tow_body = Jt @ self.Minv @ Jt.T

    def _update_bridle(self, R):
        """Re-seat the bridle apex on the tether's current direction."""
        if self.tow_mode != "bridle":
            return
        d = self.cable.x[-2] - self.cable.x[-1]          # NED, apex toward the boat
        n = np.linalg.norm(d)
        if n < 1e-9:
            return
        self._set_tow_point(self.bridle.apex_body(R.T @ (d / n)))

    # -- inputs ---------------------------------------------------------
    def set_servos(self, pwm: list[int] | np.ndarray):
        """PWM per servo channel (index 0 = SERVO1). 0 means 'not driven'."""
        def target(ch, sign):
            p = pwm[ch - 1] if len(pwm) >= ch else 0
            if not p:
                return 0.0
            return float(np.clip(sign * (p - 1500) * self.k_defl, -self.defl_lim, self.defl_lim))
        self.defl_cmd = np.array([target(self.port_servo, self.port_sign),
                                  target(self.stbd_servo, self.stbd_sign)])

    def wing_collective_deg(self) -> float:
        return math.degrees(float(self.defl.mean()))

    # -- dynamics ---------------------------------------------------------
    def _wet_fraction(self, z_ned: np.ndarray) -> np.ndarray:
        return np.clip(z_ned / 0.02 + 0.5, 0.0, 1.0)

    def forces(self, R: np.ndarray, tow_force_ned: np.ndarray):
        Rt = R.T
        water_body = Rt @ self.current
        # Strip deflections.
        defl = np.zeros(len(self.strips.c))
        defl[self.strips.group == hydro.GROUP_PORT] = self.defl[0]
        defl[self.strips.group == hydro.GROUP_STBD] = self.defl[1]
        z_strips = self.pos[2] + (self.strips.pos @ R.T)[:, 2]
        fs, ms, alpha = hydro.strip_forces(self.strips, self.foil, self.rho, self.v, self.w,
                                           water_body, defl, self._wet_fraction(z_strips))
        self.last_alpha = alpha
        z_el = self.pos[2] + (self.elements.pos @ R.T)[:, 2]
        fe, me = hydro.element_forces(self.elements, self.rho, self.v, self.w, water_body,
                                      self._wet_fraction(z_el))
        # Axial drag.
        wa = (water_body - (self.v + np.cross(self.w, self.r_axial)))[0]
        z_ax = self.pos[2]
        fa = np.array([0.5 * self.rho * self.axial_cda * abs(wa) * wa, 0.0, 0.0]) \
            * float(self._wet_fraction(np.array([z_ax]))[0])
        ma = np.cross(self.r_axial, fa)
        # Gravity and buoyancy (buoyancy fades as the hull breaks surface).
        fg = Rt @ np.array([0.0, 0.0, self.mass * G])
        z_cb = self.pos[2] + (R @ self.r_cb)[2]
        sub = float(np.clip((z_cb + HULL_RADIUS) / (2 * HULL_RADIUS), 0.0, 1.0))
        fb = Rt @ np.array([0.0, 0.0, -self.rho * self.volume * G * sub])
        mb = np.cross(self.r_cb, fb)
        ft = Rt @ tow_force_ned
        mt = np.cross(self.r_tow, ft)
        F = fs + fe + fa + fg + fb + ft
        Mo = ms + me + ma + mb + mt
        return F, Mo

    def step(self, dt: float, top_pos: np.ndarray, top_vel: np.ndarray):
        if self.cable.model != "spring":
            return self._step_inextensible(dt, top_pos, top_vel)
        R = quat_to_dcm(self.q)
        self._update_bridle(R)
        tow_pos = self.pos + R @ self.r_tow
        tow_vel = R @ (self.v + np.cross(self.w, self.r_tow))
        tow_force = self.cable.step(dt, top_pos, top_vel, tow_pos, tow_vel, self.current)
        self.last_tow_force = tow_force

        # Servo slew toward the commanded deflection.
        dmax = self.servo_rate * dt
        self.defl += np.clip(self.defl_cmd - self.defl, -dmax, dmax)

        if self.jig:
            return
        F, Mo = self.forces(R, tow_force)
        # Rigid-body Coriolis (added-mass Coriolis not modelled).
        rhs = np.concatenate([F - self.mass * np.cross(self.w, self.v),
                              Mo - np.cross(self.w, self.J @ self.w)])
        acc = self.Minv @ rhs
        vdot, wdot = acc[:3], acc[3:]
        self.v = self.v + vdot * dt
        self.w = self.w + wdot * dt
        self.q = quat_integrate(self.q, self.w, dt)
        R = quat_to_dcm(self.q)
        self.pos = self.pos + R @ self.v * dt
        # Specific force at the CG for the IMU: inertial accel minus gravity.
        inertial = vdot + np.cross(self.w, self.v)
        self.accel_body = inertial - R.T @ np.array([0.0, 0.0, G])

    def _step_inextensible(self, dt, top_pos, top_vel):
        """Predict the fish without the tether, solve the tether's link
        tensions against the fish's effective mass, apply the impulse."""
        R = quat_to_dcm(self.q)
        self._update_bridle(R)
        tow_pos = self.pos + R @ self.r_tow
        tow_vel = R @ (self.v + np.cross(self.w, self.r_tow))
        drag_end = self.cable.predict(dt, top_pos, top_vel, tow_pos, tow_vel, self.current)

        dmax = self.servo_rate * dt
        self.defl += np.clip(self.defl_cmd - self.defl, -dmax, dmax)

        if self.jig:
            imp = self.cable.solve(dt, tow_vel, None)
            self.cable.advance(dt, tow_pos, tow_vel)
            self.last_tow_force = imp / dt + drag_end
            return
        F, Mo = self.forces(R, drag_end)
        rhs = np.concatenate([F - self.mass * np.cross(self.w, self.v),
                              Mo - np.cross(self.w, self.J @ self.w)])
        acc = self.Minv @ rhs
        v_pred = self.v + acc[:3] * dt
        w_pred = self.w + acc[3:] * dt
        tow_vel_pred = R @ (v_pred + np.cross(w_pred, self.r_tow))
        K_ned = R @ self.K_tow_body @ R.T
        imp = self.cable.solve(dt, tow_vel_pred, K_ned)       # NED impulse on the fish
        ib = R.T @ imp
        dgen = self.Minv @ np.concatenate([ib, np.cross(self.r_tow, ib)])
        v_new = v_pred + dgen[:3]
        w_new = w_pred + dgen[3:]
        vdot = (v_new - self.v) / dt
        self.v, self.w = v_new, w_new
        self.q = quat_integrate(self.q, self.w, dt)
        R = quat_to_dcm(self.q)
        self.pos = self.pos + R @ self.v * dt
        inertial = vdot + np.cross(self.w, self.v)
        self.accel_body = inertial - R.T @ np.array([0.0, 0.0, G])
        self.last_tow_force = imp / dt + drag_end
        self.cable.advance(dt, self.pos + R @ self.r_tow,
                           R @ (self.v + np.cross(self.w, self.r_tow)))

    # -- jig (initial settle) ------------------------------------------
    def place(self, pos, q, vel_ned):
        """Hold the fish kinematically (used while the cable settles)."""
        self.pos = np.array(pos, dtype=float)
        self.q = np.array(q, dtype=float)
        R = quat_to_dcm(self.q)
        self.v = R.T @ np.asarray(vel_ned, dtype=float)
        self.w = np.zeros(3)
        self.accel_body = R.T @ np.array([0.0, 0.0, -G])

    def jig_advance(self, dt: float, vel_ned):
        self.pos = self.pos + np.asarray(vel_ned) * dt

    def tow_point_ned(self) -> np.ndarray:
        return self.pos + quat_to_dcm(self.q) @ self.r_tow

    def water_speed(self) -> float:
        """Speed of the CG through the water (m/s)."""
        return float(np.linalg.norm(self.vel_ned - self.current))

    @property
    def vel_ned(self) -> np.ndarray:
        return quat_to_dcm(self.q) @ self.v
