"""The boat's logged GPS track as a prescribed tow point (NED, metres).

The track is resampled to 10 Hz, converted to a local tangent plane about the
run's home, and smoothed over 1 s so 5-10 Hz GPS steps do not hit the cable
as impulses. Velocity is the derivative of the smoothed track.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone

import numpy as np

from .frames import ll_to_ne
from .logs import load_boat, utc_offset

HST = timezone(timedelta(hours=-10))


@dataclass
class BoatTrack:
    t: np.ndarray          # boat boot seconds
    n: np.ndarray
    e: np.ndarray
    vn: np.ndarray
    ve: np.ndarray
    speed: np.ndarray
    lat0: float
    lon0: float
    heading0_deg: float
    utc_off: float
    attach_height: float
    # Boat times at which ArduRover reached a mission waypoint (its
    # MISSION_CURRENT advanced). Drives the boat-navigation stub the
    # extension's auto-righting reads.
    wp_times: list = field(default_factory=list)

    def state(self, t_boat: float):
        """(pos NED, vel NED) of the boat tow point at a boat boot time."""
        pos = np.array([np.interp(t_boat, self.t, self.n),
                        np.interp(t_boat, self.t, self.e),
                        -self.attach_height])
        vel = np.array([np.interp(t_boat, self.t, self.vn),
                        np.interp(t_boat, self.t, self.ve), 0.0])
        return pos, vel

    def hst(self, t_boat: float) -> str:
        return datetime.fromtimestamp(t_boat + self.utc_off, HST).strftime("%H:%M:%S")


def parse_time(s: str, utc_off: float) -> float:
    """'HH:MM[:SS]' (HST, 4 Nov 2026) or plain boat boot seconds -> boot seconds."""
    if ":" not in s:
        return float(s)
    parts = [int(p) for p in s.split(":")]
    hh, mm = parts[0], parts[1]
    ss = parts[2] if len(parts) > 2 else 0
    dt = datetime(2026, 11, 4, hh, mm, ss, tzinfo=HST)
    return dt.timestamp() - utc_off


def load_track(t_home: float, attach_height: float) -> BoatTrack:
    boat = load_boat()
    g = boat["GPS"]
    t = np.asarray(g["t"])
    ok = (np.asarray(g["Status"]) >= 3) & (np.asarray(g["Lat"]) != 0)
    t, lat, lon = t[ok], np.asarray(g["Lat"])[ok], np.asarray(g["Lng"])[ok]
    lat0 = float(np.interp(t_home, t, lat))
    lon0 = float(np.interp(t_home, t, lon))
    tr = np.arange(t[0], t[-1], 0.1)
    n, e = ll_to_ne(np.interp(tr, t, lat), np.interp(tr, t, lon), lat0, lon0)
    k = np.ones(10) / 10.0
    n_s = np.convolve(n, k, mode="same")
    e_s = np.convolve(e, k, mode="same")
    vn = np.gradient(n_s, 0.1)
    ve = np.gradient(e_s, 0.1)
    i = int(np.searchsorted(tr, t_home))
    heading = math.degrees(math.atan2(ve[i], vn[i])) % 360
    msg = boat.get("MSG", {"t": [], "Message": []})
    wps = [float(t) for t, m in zip(msg["t"], msg["Message"])
           if str(m).startswith("Reached waypoint")]
    return BoatTrack(t=tr, n=n_s, e=e_s, vn=vn, ve=ve, speed=np.hypot(vn, ve),
                     lat0=lat0, lon0=lon0, heading0_deg=heading,
                     utc_off=utc_offset(boat), attach_height=attach_height,
                     wp_times=wps)


# --- synthetic turns ---------------------------------------------------------
# The boat's TURN_RADIUS on 4 Nov (towboat_00000110.BIN PARM).
BOAT_TURN_RADIUS_M = 3.5


# The boat's own speed-change limits on 4 Nov (ATC_ACCEL_MAX / ATC_DECEL_MAX).
BOAT_ACCEL_MPS2 = 0.7
BOAT_DECEL_MPS2 = 1.0


def synthetic_turn_track(real: BoatTrack, t_from: float, t_turn: float, turn_sign: float,
                         spacing_m: float, radius_m: float = BOAT_TURN_RADIUS_M,
                         turn_speed: float | None = None,
                         turn_speed_hold_s: float = 0.0) -> BoatTrack:
    """The logged hairpin rebuilt with a different lane spacing.

    Geometry: a straight approach on the logged heading, a 90-degree arc of
    the boat's TURN_RADIUS, a crosslink of (spacing - 2R), a second 90-degree
    arc, and a long return leg -- the shape ArduRover flew between the 7.6 m
    lines. Timing: the boat moves along this path at the logged speed at each
    instant, so its slowdowns in the turn are kept. Only the spacing differs
    between runs, which is the point.

    `turn_speed` holds the boat at that speed from the start of the first
    arc to the end of the second, or for `turn_speed_hold_s` from the start
    of the first arc if that is longer (the fish turns ~layback/speed after
    the boat does). Speed changes stay within the boat's ATC_ACCEL_MAX /
    ATC_DECEL_MAX; elsewhere the logged speed.
    """
    i0 = int(np.searchsorted(real.t, t_from))
    it = int(np.searchsorted(real.t, t_turn))
    # Approach heading: the logged course over the 20 s before the turn.
    j = max(i0, it - 200)
    hdg = math.atan2(real.e[it] - real.e[j], real.n[it] - real.n[j])
    r = min(radius_m, spacing_m / 2.0)
    cross = max(0.0, spacing_m - 2 * r)

    # Arc-length parametrised path, sampled every 5 cm.
    pts = []
    pos = np.array([real.n[i0], real.e[i0]])
    h = hdg
    step = 0.05

    def go_straight(length, pos, h):
        for _ in range(int(length / step)):
            pos = pos + step * np.array([math.cos(h), math.sin(h)])
            pts.append(pos.copy())
        return pos

    def go_arc(angle, pos, h):
        n = int(abs(angle) * r / step)
        dh = angle / max(1, n)
        for _ in range(n):
            h += dh
            pos = pos + step * np.array([math.cos(h), math.sin(h)])
            pts.append(pos.copy())
        return pos, h

    pts.append(pos.copy())
    approach = float(np.sum(real.speed[i0:it]) * 0.1)
    pos = go_straight(approach, pos, h)
    s_turn0 = (len(pts) - 1) * step
    pos, h = go_arc(turn_sign * math.pi / 2, pos, h)
    pos = go_straight(cross, pos, h)
    pos, h = go_arc(turn_sign * math.pi / 2, pos, h)
    s_turn1 = (len(pts) - 1) * step
    go_straight(400.0, pos, h)
    path = np.array(pts)
    s_path = np.arange(len(path)) * step

    t = real.t[i0:].copy()
    logged_v = real.speed[i0:]
    if turn_speed is None:
        s = np.concatenate([[0.0], np.cumsum(logged_v[:-1] * 0.1)])
    else:
        s = np.zeros(len(t))
        v = float(logged_v[0])
        t_boost = None
        for k in range(1, len(t)):
            in_turn = s_turn0 <= s[k - 1] <= s_turn1
            if in_turn and t_boost is None:
                t_boost = t[k - 1]
            holding = t_boost is not None and t[k - 1] - t_boost < turn_speed_hold_s
            target = turn_speed if (in_turn or holding) else float(logged_v[k - 1])
            v += float(np.clip(target - v, -BOAT_DECEL_MPS2 * 0.1, BOAT_ACCEL_MPS2 * 0.1))
            s[k] = s[k - 1] + v * 0.1
    s = np.minimum(s, s_path[-1])
    n = np.interp(s, s_path, path[:, 0])
    e = np.interp(s, s_path, path[:, 1])
    vn = np.gradient(n, 0.1)
    ve = np.gradient(e, 0.1)
    # ArduRover reaches each corner waypoint as it starts that corner.
    wps = [float(np.interp(sw, s, t)) for sw in (s_turn0, s_turn0 + abs(math.pi / 2) * r + cross)]
    return BoatTrack(t=t, n=n, e=e, vn=vn, ve=ve, speed=np.hypot(vn, ve),
                     lat0=real.lat0, lon0=real.lon0,
                     heading0_deg=math.degrees(hdg) % 360,
                     utc_off=real.utc_off, attach_height=real.attach_height,
                     wp_times=wps)


def synthetic_straight_track(real: BoatTrack, t_from: float, speed: float) -> BoatTrack:
    """A straight tow at a constant speed on the logged heading at `t_from`.

    For steady-state checks (e.g. the deepest the fish can fly on a given
    tether): no turns, no speed changes.
    """
    i0 = int(np.searchsorted(real.t, t_from))
    j = min(len(real.t) - 1, i0 + 200)
    hdg = math.atan2(real.e[j] - real.e[i0], real.n[j] - real.n[i0])
    t = real.t[i0:].copy()
    s = speed * (t - t[0])
    n = real.n[i0] + s * math.cos(hdg)
    e = real.e[i0] + s * math.sin(hdg)
    vn = np.full(len(t), speed * math.cos(hdg))
    ve = np.full(len(t), speed * math.sin(hdg))
    return BoatTrack(t=t, n=n, e=e, vn=vn, ve=ve, speed=np.hypot(vn, ve),
                     lat0=real.lat0, lon0=real.lon0,
                     heading0_deg=math.degrees(hdg) % 360,
                     utc_off=real.utc_off, attach_height=real.attach_height)
