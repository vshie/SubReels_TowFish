"""Where the towfish is, from the tow boat's fix.

The towfish has no GPS. Its position is estimated from what the extension
already reads: the boat's GPS fix and speed, the fish's pressure depth and
compass heading, and the operator's tether length.

Straight-leg geometry (fit to the simulator, 8-60 m Fathom tether):

    layback = SAG * sqrt(L^2 - depth^2) + TOW_POINT_TO_BODY_M

The tether runs almost straight under tow (SAG ~0.985), and the fish's
body sits TOW_POINT_TO_BODY_M behind its tow point.

In a turn the fish does not stay astern of the boat. It is dragged after
the boat's tow point like a trailer: the tether pulls the fish's tow point
straight toward the boat (it cuts inside the boat's track), and when the
boat turns back toward the fish the tether goes slack and the fish coasts
on along its heading. ``method="pursuit"`` models exactly that and is what
the extension ships; the others are kept for comparison
(``Simulator/analysis/position_eval.py`` scores all of them against the
simulator's truth).

Everything here works in a local north/east plane in metres; the
lat/lon wrapper converts with an equirectangular projection around the
first fix, which is far more accurate than GPS over a survey area.
"""
from __future__ import annotations

import collections
import math

SAG = 0.985
TOW_POINT_TO_BODY_M = 0.43
# Geometry for the reach limit: the boat's tow point sits this far above the
# water, the fish's tow point this far above its depth sensor (the CG).
BOAT_TOW_POINT_HEIGHT_M = 0.1
FISH_TOW_POINT_ABOVE_CG_M = 0.17
EARTH_R = 6371000.0
METHODS = ("fixed", "geometric", "path", "pursuit", "chain", "drag")

# Drag model constants (Blue Robotics Fathom tether, towfish geometry report).
RHO_WATER = 1025.0
TETHER_D_M = 0.0075
TETHER_CN = 1.2
TETHER_CT = 0.02
TETHER_NODE_KG_PER_M = 0.044 + RHO_WATER * math.pi * TETHER_D_M ** 2 / 4.0   # + added mass
FISH_CDA_M2 = 0.0115          # ~5.9 N at 1 m/s (geometry report 2.5)
FISH_MASS_KG = 3.6            # fish + surge added mass


def layback_m(tether_length_m, depth_m):
    """Horizontal boat-to-fish distance on a steady straight leg."""
    L = float(tether_length_m)
    d = max(0.0, min(float(depth_m or 0.0), 0.99 * L))
    return SAG * math.sqrt(L * L - d * d) + TOW_POINT_TO_BODY_M


# 1-sigma horizontal error, from Simulator/analysis/position_eval.py with
# 1 m drifting boat GPS and a 5 deg noisy, 3 deg biased fish compass:
# straight legs ~1.5-2 m on any tether; turns (and slack) grow with tether.
#
# The drag model was scored on the inextensible-tether runs (8 and 25 m,
# 4 m and max depth, 8 m and wide lines): its turn error barely grows with
# tether length (p95 ~2.7-2.8 m on both), so its band is nearly flat.
def uncertainty_m(tether_length_m, turning, method="drag"):
    L = float(tether_length_m or 0.0)
    if method == "drag":
        return round((1.8 + 0.02 * L) if turning else (1.5 + 0.01 * L), 2)
    return round((1.8 + 0.08 * L) if turning else (1.5 + 0.02 * L), 2)


def recommended_min_spacing_m(tether_length_m):
    """Narrowest transect spacing the simulator flew without auto-righting
    stepping in, for this tether (fish with the weight under the tube):
    12 m on the 8 m tether, 35 m on the 25 m tether."""
    L = float(tether_length_m or 0.0)
    return float(max(12, round(1.4 * L)))


def _bearing_deg(lat1, lon1, lat2, lon2):
    dn = math.radians(lat2 - lat1) * EARTH_R
    de = math.radians(lon2 - lon1) * EARTH_R * math.cos(math.radians(lat1))
    return math.degrees(math.atan2(de, dn)) % 360, math.hypot(dn, de), (dn, de)


def spacing_from_waypoints(waypoints):
    """Transect line spacing of a lawnmower mission, or None.

    ``waypoints`` is the ordered list of (lat, lon) navigation waypoints.
    Transect lines are the long legs (at least half the longest); the
    spacing is the perpendicular distance between each pair of consecutive
    lines that run in roughly opposite directions, taken as the median.
    """
    pts = [(float(a), float(b)) for a, b in waypoints if a is not None and b is not None]
    if len(pts) < 4:
        return None
    lat0, lon0 = pts[0]
    ne = []
    for lat, lon in pts:
        n = math.radians(lat - lat0) * EARTH_R
        e = math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
        ne.append((n, e))
    legs = []
    for (n1, e1), (n2, e2) in zip(ne, ne[1:]):
        legs.append(((n1, e1), (n2, e2), math.hypot(n2 - n1, e2 - e1)))
    longest = max(l for _, _, l in legs)
    lines = [g for g in legs if g[2] >= 0.5 * longest and g[2] > 1.0]
    out = []
    for a, b in zip(lines, lines[1:]):
        (an1, ae1), (an2, ae2), al = a
        (bn1, be1), (bn2, be2), _ = b
        un, ue = (an2 - an1) / al, (ae2 - ae1) / al
        vn, ve = bn2 - bn1, be2 - be1
        if (un * vn + ue * ve) > -0.5 * math.hypot(vn, ve):
            continue                                   # not an opposite-running line
        mn, me = (bn1 + bn2) / 2 - an1, (be1 + be2) / 2 - ae1
        out.append(abs(mn * ue - me * un))             # perpendicular distance
    if not out:
        return None
    out.sort()
    return round(out[len(out) // 2], 2)


def max_reach_m(tether_length_m, depth_m):
    """Furthest the fish's tow point can be, horizontally, from the boat's
    tow point: the tether can never be longer than it is, and the vertical
    drop uses some of it."""
    L = float(tether_length_m)
    drop = max(0.0, float(depth_m or 0.0) - FISH_TOW_POINT_ABOVE_CG_M + BOAT_TOW_POINT_HEIGHT_M)
    return math.sqrt(max(0.0, L * L - drop * drop))


def _unit(deg):
    r = math.radians(deg)
    return math.cos(r), math.sin(r)   # (north, east)


class TowedPositionEstimator:
    """Fish position from boat position, in a local north/east frame (m).

    ``update_ne`` takes the boat fix, course and speed and the fish's depth
    and heading, and returns the estimate. ``method`` picks the model:

    * ``fixed``     -- ``fixed_offset_m`` astern along the fish heading
                       (what the extension did before this model);
    * ``geometric`` -- ``layback_m(L, depth)`` astern along the fish heading;
    * ``path``      -- on the boat's own past track, ``layback`` behind it;
    * ``pursuit``   -- the tether drags the fish's tow point toward the boat
                       (corner-cutting); slack lets it coast on its heading.
    """

    # Slack coasting: the fish keeps its speed through the water, decaying
    # with this time constant, along its own heading.
    COAST_TAU_S = 4.0
    # No part of the tether outruns the boat by more than this factor, so a
    # GPS jump can never fling the chain; node speeds are smoothed with
    # this time constant. The boat fix itself is smoothed over BOAT_TAU_S.
    COAST_MAX_SPEED_RATIO = 1.5
    NODE_V_TAU_S = 1.0
    BOAT_TAU_S = 0.6
    # A fix that implies the boat jumped further than this many times its
    # speed (plus JUMP_SLACK_M) since the last one is a GPS glitch and is
    # skipped -- unless JUMP_ACCEPT_AFTER arrive in a row (it really moved).
    JUMP_SPEED_RATIO = 4.0
    JUMP_SLACK_M = 3.0
    JUMP_ACCEPT_AFTER = 3
    PATH_HISTORY_S = 180.0
    # One link, plus one per this many metres of tether: 2 links on 8 m,
    # 3 on 16 m, 4 on 25 m (fit to the simulator's hairpins). Set from the
    # tether length only, so depth changes never re-seed the chain.
    TETHER_M_PER_LINK = 8.0

    def __init__(self, tether_length_m, method="pursuit", fixed_offset_m=7.0, links=None,
                 link_len_m=None):
        if method not in METHODS:
            raise ValueError(f"method must be one of {METHODS}")
        self.L = float(tether_length_m or 0.0)
        self.method = method
        self.fixed_offset_m = float(fixed_offset_m)
        # pursuit = one straight link; chain = several, each dragging the next
        self.links_fixed = 1 if method == "pursuit" else (int(links) if links else None)
        self.m_per_link = float(link_len_m or self.TETHER_M_PER_LINK)
        self.nodes = None            # tether nodes below the boat, last = fish tow point
        self.node_v = None           # their coasting velocities (n, e)
        self.towpt = None            # fish tow point (n, e)
        self.fish = None             # fish body (n, e)
        self.coast_speed = 0.0
        self.slack = False
        self.t = None
        self.trail = collections.deque()   # (t, n, e, cumulative distance)
        self.boat = None                     # smoothed boat fix (n, e)
        self.raw = None                      # last accepted raw fix (t, n, e)
        self.rejected = 0
        self.courses = collections.deque()   # (t, boat course) for turn detection

    def reset(self):
        self.towpt = self.fish = self.t = self.boat = self.raw = None
        self.rejected = 0
        self.courses.clear()
        self.nodes = self.node_v = None
        self.trail.clear()

    # -- core -------------------------------------------------------------
    def update_ne(self, t, boat_n, boat_e, boat_course, boat_speed,
                  fish_depth, fish_heading):
        heading = fish_heading if fish_heading is not None else boat_course
        if heading is None:
            heading = 0.0
        h = (layback_m(self.L, fish_depth) if self.L > 0 else self.fixed_offset_m)
        hn, he = _unit(heading)
        raw_n, raw_e = boat_n, boat_e
        if self._glitch(t, boat_n, boat_e, boat_speed):
            if self.fish is not None:
                return self._result(t, boat_course, h)
        dt = 0.0 if self.t is None else max(0.0, t - self.t)
        self.t = t
        # Light smoothing of the boat fix: GPS jitter is not boat motion.
        if self.boat is None or dt <= 0 or dt > 5.0:
            self.boat = (boat_n, boat_e)
        else:
            a = dt / (self.BOAT_TAU_S + dt)
            self.boat = (self.boat[0] + a * (boat_n - self.boat[0]),
                         self.boat[1] + a * (boat_e - self.boat[1]))
        boat_n, boat_e = self.boat
        if boat_course is not None and boat_speed:
            # The low-pass lags a moving boat by speed * tau; lead it back.
            cn, ce = _unit(boat_course)
            lead = float(boat_speed) * self.BOAT_TAU_S
            boat_n, boat_e = boat_n + lead * cn, boat_e + lead * ce
        self._remember(t, boat_n, boat_e)

        if self.method == "fixed":
            self.fish = (boat_n - self.fixed_offset_m * hn, boat_e - self.fixed_offset_m * he)
        elif self.method == "geometric":
            self.fish = (boat_n - h * hn, boat_e - h * he)
        elif self.method == "path":
            self.fish = self._along_trail(h, boat_n, boat_e, hn, he)
        elif self.method == "drag":
            self._drag_model(dt, boat_n, boat_e, boat_course, boat_speed, fish_depth, hn, he)
        else:
            self._chain(dt, boat_n, boat_e, boat_speed, h, hn, he)
        self._clamp_to_reach(raw_n, raw_e, fish_depth, hn, he)
        return self._result(t, boat_course, h)

    def _clamp_to_reach(self, bn, be, depth, hn, he):
        """Hard limit: never further from the boat's actual fix than the
        tether can reach. Smoothing and lead on the boat fix (and any model
        error) must not put the fish where the tether cannot."""
        self.clamped = False
        if self.L <= 0 or self.fish is None:
            return
        tn, te = self.towpt if self.towpt is not None else (
            self.fish[0] + TOW_POINT_TO_BODY_M * hn, self.fish[1] + TOW_POINT_TO_BODY_M * he)
        reach = max_reach_m(self.L, depth)
        d = math.hypot(tn - bn, te - be)
        if d <= reach or d < 1e-9:
            return
        k = reach / d
        sn, se = bn + (tn - bn) * k - tn, be + (te - be) * k - te
        self.clamped = True
        self.towpt = (tn + sn, te + se)
        self.fish = (self.fish[0] + sn, self.fish[1] + se)
        # Keep the model's own tether end inside the limit too, so the next
        # update starts from a reachable state.
        if self.nodes:
            last = self.nodes[-1]
            if len(last) == 3:
                last[0] += sn
                last[1] += se
            else:
                self.nodes[-1] = (last[0] + sn, last[1] + se)

    def _glitch(self, t, n, e, speed):
        if self.raw is None:
            self.raw = (t, n, e)
            return False
        t0, n0, e0 = self.raw
        jump = math.hypot(n - n0, e - e0)
        limit = self.JUMP_SLACK_M + self.JUMP_SPEED_RATIO * max(0.5, float(speed or 0.0)) * max(0.0, t - t0)
        if jump > limit and self.rejected < self.JUMP_ACCEPT_AFTER - 1:
            self.rejected += 1
            return True
        if jump > limit:                     # several in a row: re-seat on the new fix
            self.boat = None
        self.rejected = 0
        self.raw = (t, n, e)
        return False

    def _result(self, t, boat_course, h):
        turning = self._turning(t, boat_course) or self.slack
        return {"north": self.fish[0], "east": self.fish[1], "layback_m": h,
                "clamped": getattr(self, "clamped", False),
                "along_path_m": self._path_length_to_fish(),
                "slack": self.slack, "turning": turning,
                "uncertainty_m": uncertainty_m(self.L or self.fixed_offset_m, turning, self.method)}

    # Same turn definition as the evaluation: the boat's course changed by
    # more than TURN_DEG within the last TURN_WINDOW_S (turn + aftermath).
    TURN_DEG = 15.0
    TURN_WINDOW_S = 40.0

    def _turning(self, t, course):
        if course is not None:
            self.courses.append((t, float(course)))
        while self.courses and self.courses[0][0] < t - self.TURN_WINDOW_S:
            self.courses.popleft()
        if not self.courses:
            return False
        last = self.courses[-1][1]
        return max(abs((c - last + 180.0) % 360.0 - 180.0) for _, c in self.courses) > self.TURN_DEG

    def _chain(self, dt, bn, be, speed, h, hn, he):
        """The tether as ``links`` straight links, each dragging the next.

        Taut link: its lower node is pulled straight toward the node above
        to the link length (a trailer behind a car). Slack link (the node
        above came back toward it): the lower node coasts on, slowing; the
        fish's tow point coasts along the fish's own heading. One link is
        pure pursuit (corner-cutting); many approach following the boat's
        path. Normal drag on a cable is much larger than tangential, so a
        real tether sits in between.
        """
        cable = h - TOW_POINT_TO_BODY_M          # boat tow point -> fish tow point
        N = self.links_fixed or max(1, int(round(1 + self.L / self.m_per_link)))
        seg = cable / N
        if self.nodes is not None and len(self.nodes) != N:
            self.nodes = None                    # tether length changed
        if self.nodes is None:
            self.nodes = [(bn - seg * (k + 1) * hn, be - seg * (k + 1) * he) for k in range(N)]
            v0 = float(speed or 0.0)
            self.node_v = [(v0 * hn, v0 * he)] * N
        decay = math.exp(-dt / self.COAST_TAU_S) if dt > 0 else 1.0
        vmax = self.COAST_MAX_SPEED_RATIO * max(0.3, float(speed or 0.0))
        av = dt / (self.NODE_V_TAU_S + dt) if dt > 0 else 0.0
        un, ue = bn, be
        slack_any = False
        for k in range(N):
            tn, te = self.nodes[k]
            dn, de = un - tn, ue - te
            dist = math.hypot(dn, de)
            if dist >= seg:
                f = (dist - seg) / dist if dist > 1e-9 else 0.0
                nn, ne = tn + dn * f, te + de * f
                if dt > 0:
                    vn0, ve0 = self.node_v[k]
                    self.node_v[k] = (vn0 + av * ((nn - tn) / dt - vn0),
                                      ve0 + av * ((ne - te) / dt - ve0))
            else:
                slack_any = True
                vn, ve = self.node_v[k]
                sp = min(vmax, math.hypot(vn, ve) * decay)
                if k == N - 1:
                    vn, ve = sp * hn, sp * he
                else:
                    norm = math.hypot(vn, ve) or 1.0
                    vn, ve = vn / norm * sp, ve / norm * sp
                self.node_v[k] = (vn, ve)
                nn, ne = tn + vn * dt, te + ve * dt
            self.nodes[k] = (nn, ne)
            un, ue = nn, ne
        self.slack = slack_any
        self.towpt = self.nodes[-1]
        tn, te = self.towpt
        self.fish = (tn - TOW_POINT_TO_BODY_M * hn, te - TOW_POINT_TO_BODY_M * he)

    # -- drag model ---------------------------------------------------------
    DRAG_STEP_S = 0.1
    BOAT_ATTACH_M = 0.1          # tow point height above the water

    def _drag_model(self, dt, bn, be, course, speed, depth, hn, he):
        """The tether as 1 m inextensible links with water drag, in still water.

        Node 0 follows the boat; the last node is the fish's tow point, held
        at the measured depth (so wing lift never needs modelling) and pulled
        back by the fish's drag. Between updates the tether is stepped at
        DRAG_STEP_S: drag on every node (implicit, so it is stable at this
        step), then the links are projected back to their length -- tension
        only, so a link shorter than its length is simply slack.
        """
        L = self.L if self.L > 0 else self.fixed_offset_m
        N = max(1, int(round(L)))
        seg = L / N
        z_fish = max(0.0, float(depth or 0.0))
        if self.nodes is None or len(self.nodes) != N + 1:
            reach = math.sqrt(max(0.0, L * L - (z_fish + self.BOAT_ATTACH_M) ** 2))
            v0 = float(speed or 0.0)
            vn0, ve0 = (v0 * math.cos(math.radians(course)), v0 * math.sin(math.radians(course))) \
                if course is not None else (v0 * hn, v0 * he)
            self.nodes = []
            self.node_v = []
            for k in range(N + 1):
                f = k / N
                self.nodes.append([bn - reach * f * hn, be - reach * f * he,
                                   -self.BOAT_ATTACH_M + (z_fish + self.BOAT_ATTACH_M) * f])
                self.node_v.append([vn0, ve0, 0.0])
            self._drag_boat = (bn, be)
        x, v = self.nodes, self.node_v
        if dt <= 0:
            self._drag_finish(seg, hn, he)
            return
        steps = max(1, int(math.ceil(dt / self.DRAG_STEP_S)))
        h = dt / steps
        b0n, b0e = self._drag_boat
        m_node = TETHER_NODE_KG_PER_M * seg
        kn = 0.5 * RHO_WATER * TETHER_D_M * TETHER_CN * seg / m_node
        kt = 0.5 * RHO_WATER * math.pi * TETHER_D_M * TETHER_CT * seg / m_node
        kf = 0.5 * RHO_WATER * FISH_CDA_M2 / FISH_MASS_KG
        w_node, w_fish = 1.0 / m_node, 1.0 / FISH_MASS_KG
        z_prev_fish = x[N][2]
        bvn, bve = (bn - b0n) / dt, (be - b0e) / dt
        for s_ in range(1, steps + 1):
            f0, f = (s_ - 1) / steps, s_ / steps
            # Boat node at the start of the step, moving with the boat; it is
            # advanced to the end of the step after the solve.
            x[0] = [b0n + (bn - b0n) * f0, b0e + (be - b0e) * f0, -self.BOAT_ATTACH_M]
            v[0] = [bvn, bve, 0.0]
            # Drag, implicit per node (still water: drag opposes node velocity).
            for k in range(1, N + 1):
                vx, vy, vz = v[k]
                if k < N:
                    tx = x[k - 1][0] - x[k + 1][0]
                    ty = x[k - 1][1] - x[k + 1][1]
                    tz = x[k - 1][2] - x[k + 1][2]
                    tl = math.sqrt(tx * tx + ty * ty + tz * tz) or 1.0
                    tx, ty, tz = tx / tl, ty / tl, tz / tl
                    vt = vx * tx + vy * ty + vz * tz
                    nx, ny, nz = vx - vt * tx, vy - vt * ty, vz - vt * tz
                    vnm = math.sqrt(nx * nx + ny * ny + nz * nz)
                    nfac = 1.0 / (1.0 + h * kn * vnm)
                    tfac = 1.0 / (1.0 + h * kt * abs(vt))
                    v[k] = [nx * nfac + vt * tfac * tx, ny * nfac + vt * tfac * ty,
                            nz * nfac + vt * tfac * tz]
                else:
                    sp = math.hypot(vx, vy)
                    fac = 1.0 / (1.0 + h * kf * sp)
                    v[k] = [vx * fac, vy * fac, 0.0]
            # Link tensions: one exact tridiagonal solve per step (as in the
            # simulator), tension only -- links that would push are slack.
            self._drag_tension(x, v, seg, h, w_node, w_fish)
            for k in range(1, N + 1):
                x[k][0] += v[k][0] * h
                x[k][1] += v[k][1] * h
                x[k][2] += v[k][2] * h
            x[N][2] = z_prev_fish + (z_fish - z_prev_fish) * f     # measured depth
            v[N][2] = 0.0
            x[0] = [b0n + (bn - b0n) * f, b0e + (be - b0e) * f, -self.BOAT_ATTACH_M]
        self._drag_boat = (bn, be)
        self._drag_finish(seg, hn, he)

    def _drag_tension(self, x, v, seg, h, w_node, w_fish):
        N = len(x) - 1
        u, vs, vt = [], [], []
        for k in range(N):
            a, b = x[k], x[k + 1]
            d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]]
            ln = math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) or 1e-9
            uk = [d[0] / ln, d[1] / ln, d[2] / ln]
            u.append(uk)
            vs.append(sum((v[k + 1][i] - v[k][i]) * uk[i] for i in range(3)))
            C = ln - seg
            vt.append(-0.2 * C / h if C > 0 else -C / h)
        # Inverse masses along each link. The fish only moves level (its
        # depth is measured), so it answers only the level part of a pull.
        uz = u[-1][2]
        wf = w_fish * (1.0 - uz * uz)
        winv = [0.0] + [w_node] * (N - 1) + [wf]
        cosn = [sum(u[k][i] * u[k + 1][i] for i in range(3)) for k in range(N - 1)]
        # Full coefficients: how each link's separation answers its own and
        # its neighbours' tensions (used to re-check links left inactive).
        Af = [0.0] + [winv[k] * cosn[k - 1] for k in range(1, N)]
        Bf = [-(winv[k] + winv[k + 1]) for k in range(N)]
        Cf = [winv[k + 1] * cosn[k] for k in range(N - 1)] + [0.0]
        active = [vs[k] > vt[k] for k in range(N)]
        lam = [0.0] * N
        for _ in range(12):
            A = [0.0] * N; B = [0.0] * N; Cc = [0.0] * N; R = [0.0] * N
            for k in range(N):
                if not active[k]:
                    B[k] = 1.0
                    continue
                B[k] = -(winv[k] + winv[k + 1])
                if k > 0 and active[k - 1]:
                    A[k] = winv[k] * cosn[k - 1]
                if k < N - 1 and active[k + 1]:
                    Cc[k] = winv[k + 1] * cosn[k]
                R[k] = vt[k] - vs[k]
            lam = _thomas(A, B, Cc, R)
            neg = [active[k] and lam[k] < 0.0 for k in range(N)]
            if any(neg):
                active = [active[k] and not neg[k] for k in range(N)]
                continue
            newly = []
            for k in range(N):
                if active[k]:
                    continue
                dvs = (Af[k] * lam[k - 1] if k > 0 else 0.0) + (Cf[k] * lam[k + 1] if k < N - 1 else 0.0)
                if vs[k] + dvs > vt[k] + 1e-9:
                    newly.append(k)
            if not newly:
                break
            for k in newly:
                active[k] = True
        for k in range(N):
            if not active[k] or lam[k] <= 0.0:
                continue
            ix, iy, iz = lam[k] * u[k][0], lam[k] * u[k][1], lam[k] * u[k][2]
            if k > 0:
                v[k][0] += ix * w_node; v[k][1] += iy * w_node; v[k][2] += iz * w_node
            if k + 1 == N:
                v[N][0] -= ix * w_fish; v[N][1] -= iy * w_fish
            else:
                v[k + 1][0] -= ix * w_node; v[k + 1][1] -= iy * w_node; v[k + 1][2] -= iz * w_node

    def _drag_finish(self, seg, hn, he):
        x = self.nodes
        N = len(x) - 1
        self.slack = any(
            math.dist(x[k], x[k + 1]) < 0.98 * seg for k in range(N))
        tn, te = x[N][0], x[N][1]
        self.towpt = (tn, te)
        self.fish = (tn - TOW_POINT_TO_BODY_M * hn, te - TOW_POINT_TO_BODY_M * he)

    # -- boat trail (path method, along-path distance) ---------------------
    def _remember(self, t, n, e):
        if self.trail:
            _, pn, pe, s = self.trail[-1]
            step = math.hypot(n - pn, e - pe)
            if step < 0.05:
                return
            self.trail.append((t, n, e, s + step))
        else:
            self.trail.append((t, n, e, 0.0))
        while self.trail and self.trail[0][0] < t - self.PATH_HISTORY_S:
            self.trail.popleft()

    def _along_trail(self, h, bn, be, hn, he):
        if len(self.trail) < 2:
            return (bn - h * hn, be - h * he)
        s_end = self.trail[-1][3]
        target = s_end - h
        if target <= self.trail[0][3]:
            return (bn - h * hn, be - h * he)
        for i in range(len(self.trail) - 1, 0, -1):
            s0, s1 = self.trail[i - 1][3], self.trail[i][3]
            if s0 <= target <= s1:
                f = (target - s0) / max(1e-9, s1 - s0)
                n = self.trail[i - 1][1] + f * (self.trail[i][1] - self.trail[i - 1][1])
                e = self.trail[i - 1][2] + f * (self.trail[i][2] - self.trail[i - 1][2])
                return (n, e)
        return (bn - h * hn, be - h * he)

    def _path_length_to_fish(self):
        """Distance along the boat's own track back to where the fish is
        now -- what the boat's sounding needs to travel to reach the fish
        (SURF TRACK's look-ahead)."""
        if self.fish is None or len(self.trail) < 2:
            return None
        fn, fe = self.fish
        best, best_s = None, None
        for _, n, e, s in self.trail:
            d = (n - fn) ** 2 + (e - fe) ** 2
            if best is None or d < best:
                best, best_s = d, s
        return self.trail[-1][3] - best_s


def _thomas(a, b, c, r):
    """Tridiagonal solve (a: sub, b: main, c: super diagonals); plain lists."""
    n = len(b)
    cp = [0.0] * n; rp = [0.0] * n
    cp[0] = c[0] / b[0]; rp[0] = r[0] / b[0]
    for i in range(1, n):
        m = b[i] - a[i] * cp[i - 1]
        cp[i] = c[i] / m if i < n - 1 else 0.0
        rp[i] = (r[i] - a[i] * rp[i - 1]) / m
    out = [0.0] * n
    out[-1] = rp[-1]
    for i in range(n - 2, -1, -1):
        out[i] = rp[i] - cp[i] * out[i + 1]
    return out


class TowedPositionTracker:
    """Lat/lon front end: projects fixes to a local plane around the first."""

    def __init__(self, tether_length_m, method="pursuit", fixed_offset_m=7.0):
        self.est = TowedPositionEstimator(tether_length_m, method, fixed_offset_m)
        self.origin = None

    def set_tether_length(self, tether_length_m):
        if float(tether_length_m or 0.0) != self.est.L:
            self.est.L = float(tether_length_m or 0.0)

    def _to_ne(self, lat, lon):
        lat0, lon0 = self.origin
        n = math.radians(lat - lat0) * EARTH_R
        e = math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
        return n, e

    def _to_latlon(self, n, e):
        lat0, lon0 = self.origin
        lat = lat0 + math.degrees(n / EARTH_R)
        lon = lon0 + math.degrees(e / (EARTH_R * math.cos(math.radians(lat0))))
        return lat, lon

    def update(self, t, boat_lat, boat_lon, boat_course, boat_speed,
               fish_depth, fish_heading):
        if self.origin is None:
            self.origin = (boat_lat, boat_lon)
        n, e = self._to_ne(boat_lat, boat_lon)
        if math.hypot(n, e) > 20000.0:          # moved to a new site: re-centre
            self.origin = (boat_lat, boat_lon)
            self.est.reset()
            n, e = 0.0, 0.0
        out = self.est.update_ne(t, n, e, boat_course, boat_speed, fish_depth, fish_heading)
        out["lat"], out["lon"] = self._to_latlon(out["north"], out["east"])
        return out
