"""Run the SubReels extension's controllers against the simulator.

SurfTrackController and AutoRightController are imported unmodified from
app/main.py (the same way the extension's own tests import them). Only
their I/O is redirected:

  fish depth      <- SITL VFR_HUD (what the extension reads via mavlink2rest)
  fish attitude   <- SITL ATTITUDE
  armed / mode    <- SITL HEARTBEAT
  wing outputs    <- the servo PWMs SITL sends the physics (SERVO_OUTPUT_RAW)
  boat sonar      <- the extension's own SonarHistory, sampled at its 4 Hz
                     from the boat's logged RFND packets (quality gate and
                     all) or a terrain/flat bottom seen by a Ping model;
                     boat speed from the track
  boat navigation <- mission seq, heading and bearing to the next waypoint,
                     from the boat track (see BoatNav)
  RC2/RC3         -> RC_CHANNELS_OVERRIDE on SITL
  mode changes    -> DO_SET_MODE on SITL
  NAMED_VALUE     -> SITL, so ST_* lands in the sim BIN as in the field

Time is the sim clock: the controller's `time.monotonic()` is pointed at
it and `_tick()` is called every 0.1 s of sim time, as its thread would.
"""
from __future__ import annotations

import math
import queue
import os
import sys
import types
from pathlib import Path
from unittest import mock

import numpy as np

from .logs import load_boat

APP_DIR = Path(os.environ.get("TOWSIM_APP_DIR",
                              Path(__file__).resolve().parent.parent.parent / "app"))
MODE_LABELS = {0: "STABILIZE", 2: "ALT_HOLD", 19: "MANUAL"}


def _stub(name, **attrs):
    mod = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(mod, k, v)
    sys.modules[name] = mod
    return mod


def import_extension():
    """Import app/main.py with its hardware/web dependencies stubbed out."""
    if "main" in sys.modules and hasattr(sys.modules["main"], "SurfTrackController"):
        return sys.modules["main"]
    _stub("flask", Flask=mock.MagicMock(), jsonify=mock.MagicMock(),
          request=mock.MagicMock(), send_file=mock.MagicMock())
    _stub("requests", get=mock.MagicMock(), post=mock.MagicMock(),
          exceptions=mock.MagicMock(), Session=mock.MagicMock(), adapters=mock.MagicMock())
    gi = _stub("gi", require_version=lambda *a, **k: None)
    _stub("gi.repository", Gst=mock.MagicMock())
    gi.repository = sys.modules["gi.repository"]
    _stub("websockets", serve=mock.MagicMock())
    _stub("websockets.exceptions", ConnectionClosed=Exception)
    sys.modules["websockets"].exceptions = sys.modules["websockets.exceptions"]
    _stub("usb_storage", get_base_dir=lambda: None, start_probe=lambda: None)
    _stub("piexif", dump=mock.MagicMock(), insert=mock.MagicMock(), ImageIFD=mock.MagicMock(),
          ExifIFD=mock.MagicMock(), GPSIFD=mock.MagicMock())
    sys.path.insert(0, str(APP_DIR))
    import main  # noqa: E402
    return main


class _Clock:
    """Stands in for the `time` module inside main: monotonic() is sim time."""

    def __init__(self, real):
        self._real = real
        self.now = 0.0

    def monotonic(self):
        return self.now

    def __getattr__(self, name):
        return getattr(self._real, name)


class BoatPing:
    """What the boat's Ping publishes, as the extension's sampler sees it.

    Three sources: the 4 Nov boat log's RFND (replayed packet by packet:
    each new value is a new packet, with its logged quality, so the
    extension's own gate drops what it dropped in the field), or a
    terrain model (towsim.terrain) seen through the Ping model, or a flat
    bottom (the same Ping model over flat terrain).
    """

    def __init__(self, track, terrain=None, seed=0):
        self.track = track
        self.ping = None
        if terrain is not None:
            from .terrain import PingModel
            self.ping = PingModel(terrain, seed=seed)
        else:
            from .logs import BOAT_BIN, read_bin
            r = read_bin(BOAT_BIN, {"RFND": ["Dist", "Quality", "Stat"]})["RFND"]
            t = np.asarray(r["t"])
            d = np.asarray(r["Dist"], dtype=float)
            new = np.r_[True, np.diff(d) != 0]
            self.t = t[new]
            self.d = d[new]
            self.q = np.asarray(r["Quality"], dtype=float)[new]
            self.stat = np.asarray(r["Stat"])[new]

    def snapshot(self, t_boat):
        if self.ping is not None:
            (n, e, _), _ = self.track.state(t_boat)
            return self.ping.update(t_boat, n, e)
        i = int(np.searchsorted(self.t, t_boat, side="right")) - 1
        if i < 0:
            return None
        return {"distance_m": float(self.d[i]), "signal_quality": float(self.q[i]),
                "in_range": int(self.stat[i]) == 4, "last_update": i}

    def speed(self, t_boat):
        _, v = self.track.state(t_boat)
        return float(math.hypot(v[0], v[1]))


class BoatNav:
    """The boat's MISSION_CURRENT.seq.

    Waypoint "reached" times come from the track (the boat log's MSG
    lines, or the corners of a synthetic hairpin).
    """

    def __init__(self, track):
        self.track = track
        self.wps = sorted(track.wp_times)

    def seq(self, t_boat):
        return 1 + int(np.searchsorted(self.wps, t_boat, side="right"))


class ExtensionBridge:
    def __init__(self, mav, track, *, jog_up_pwm=1800, jog_down_pwm=1300,
                 target_altitude_m=None, max_depth_m=None, tow_offset_m=7.0,
                 tether_length_m=0.0, terrain=None, ext_set=None,
                 surftrack=True, ping_seed=0):
        self.main = main = import_extension()
        self.mav = mav
        self.clock = _Clock(main.time)
        main.time = self.clock
        self.rc3 = 1500
        self.rc2: int | None = None
        self._rc_owner: str | None = None
        self.depth = None
        self.attitude: dict = {}
        self.vehicle: dict = {}
        self.servo: dict = {}
        self.track = track
        self.ping = BoatPing(track, terrain, seed=ping_seed)
        self.sonar = main.SonarHistory()
        self.next_sonar = None
        self.packet = None
        self.nav = BoatNav(track)
        self.t_boat = 0.0
        main._get_towfish_depth_m = lambda: self.depth
        main.get_towfish_attitude = lambda: dict(self.attitude)
        main.get_vehicle_status_snapshot = lambda *a, **k: dict(self.vehicle)
        main._cached_heartbeat_snapshot = lambda *a, **k: {
            "armed": self.vehicle.get("armed"), "custom_mode": self.vehicle.get("custom_mode")}
        main._servo_output_message = lambda: dict(self.servo)
        main._sonar_history = self.sonar
        # The model-free tow delay (tow_position_mode "fixed") reads this.
        main.tow_offset_m = tow_offset_m
        main._set_thrust_command = self._thrust
        main.get_default_writer = lambda: self
        main._persist_config = lambda: None
        main._tow_mission_seq = lambda: self.nav.seq(self.t_boat)
        # The towfish position tracker, ticked on the sim clock from the
        # boat track (as a GPS fix) and the fish's depth and heading.
        main.get_blueboat_fix = self._boat_fix
        main._towfish_position = main.TowfishPositionSampler()
        main._transect_spacing.observe_fix = lambda *a, **k: None
        main.tether_twist_turns = 0.0
        # The values the fish flew on 4 Nov where the log shows them (jog
        # PWMs from RCIN); everything else at the extension's defaults.
        main.jog_up_pwm = jog_up_pwm
        main.jog_down_pwm = jog_down_pwm
        main.altitude_offset_m = 0.0
        main.tether_length_m = tether_length_m
        if target_altitude_m is not None:
            main.surf_target_altitude_m = target_altitude_m
        if max_depth_m is not None:
            main.surf_max_depth_m = max_depth_m
        for name, value in (ext_set or {}).items():
            if not hasattr(main, name):
                raise SystemExit(f"--ext-set: app/main.py has no {name}")
            setattr(main, name, value)
        self.ctrl = main.SurfTrackController()
        self.ar = main.AutoRightController()
        # The controllers reach each other through the module singletons.
        main._surftrack, main._autoright = self.ctrl, self.ar
        # Enabled without threads: we tick them on the sim clock.
        self.ctrl.state = "tracking" if surftrack else "disabled"
        self.ar.state = "idle" if main.autoright_enabled else "disabled"
        self.next_tick = None

    # -- extension-facing I/O --------------------------------------------
    def _thrust(self, direction, pwm, source, roll_pwm=None):
        """Same arbitration as main._set_thrust_command, minus the operator."""
        if direction is None:
            if self._rc_owner == source:
                self._rc_owner = None
                self.rc3 = 1500
                if self.rc2 is not None:
                    self.rc2 = 1500
            return True
        if source == "surftrack" and self._rc_owner in ("autoright", "launch_guard"):
            return False
        if source == "launch_guard":
            if self.ar.is_active():
                return False
            roll_pwm = None
        self._rc_owner = source
        self.rc3 = int(pwm)
        if roll_pwm is not None:
            self.rc2 = int(roll_pwm)
        elif self.rc2 is not None:
            self.rc2 = 1500
        return True

    def _boat_fix(self):
        import towed_position as tp
        (n, e, _), (vn, ve, _) = self.track.state(self.t_boat)
        lat0, lon0 = self.track.lat0, self.track.lon0
        lat = lat0 + math.degrees(n / tp.EARTH_R)
        lon = lon0 + math.degrees(e / (tp.EARTH_R * math.cos(math.radians(lat0))))
        speed = math.hypot(vn, ve)
        course = math.degrees(math.atan2(ve, vn)) % 360 if speed > 0.1 else None
        return {"lat": lat, "lon": lon, "course": course, "speed": speed}

    def position_estimate(self):
        """(north, east, uncertainty) of the extension's towfish estimate in
        the sim frame, or blanks while it has none (truth.csv est_*)."""
        import towed_position as tp
        last = self.main._towfish_position.estimate()
        if last is None:
            return ("", "", "")
        est = last[3]
        lat0, lon0 = self.track.lat0, self.track.lon0
        n = math.radians(est["lat"] - lat0) * tp.EARTH_R
        e = math.radians(est["lon"] - lon0) * tp.EARTH_R * math.cos(math.radians(lat0))
        return (f"{n:.3f}", f"{e:.3f}", f"{est['uncertainty_m']:.2f}")

    def set_mode(self, mode):
        self.mav.set_mode(int(mode))
        return True

    def named_value_float(self, name, value):
        self.mav.named_value(name, value)
        return True

    # -- driven by run.py ----------------------------------------------------
    def step(self, t_sim: float, t_boat: float, pwm):
        if self.next_tick is not None and t_sim < self.next_tick:
            return
        self.next_tick = t_sim + self.main.SURF_TICK_S
        self.clock.now = t_sim
        self.t_boat = t_boat
        # The extension's sampler thread, at its own period.
        if self.next_sonar is None or t_sim >= self.next_sonar:
            self.next_sonar = t_sim + self.main._SONAR_SAMPLE_PERIOD_S
            self.packet = self.ping.snapshot(t_boat)
            self.sonar.sample_once(self.packet, self.ping.speed(t_boat), self._boat_fix())
        m = self.mav
        if m.vfr_alt is not None:
            self.depth = round(-m.vfr_alt, 2) if m.vfr_alt < 0 else 0.0
        if m.attitude is not None:
            r, p, y = m.attitude
            self.attitude = {"roll": math.degrees(r), "pitch": math.degrees(p),
                             "yaw": math.degrees(y) % 360}
        self.vehicle = {"armed": m.armed, "custom_mode": m.mode,
                        "mode_label": MODE_LABELS.get(m.mode, str(m.mode))}
        self.servo = {f"servo{i + 1}_raw": int(v) for i, v in enumerate(pwm[:16])}
        if self.ar.state != "disabled":
            self.ar._tick()
        if self.ctrl.state != "disabled":
            self.ctrl._tick()
        # The shore-launch guard ships running: full up in any armed MANUAL
        # that is not an auto-righting rescue.
        self.main._launch_lift_guard._tick()
        self.main._towfish_position.tick(t_sim)
        # The extension's own NVAL worker may drain the same queue, so an
        # empty() check can go stale before get_nowait().
        q = self.ctrl._bin_queue
        while True:
            try:
                name, value = q.get_nowait()
            except queue.Empty:
                break
            self.named_value_float(name, value)

    def altitudes(self):
        """(position-based, time-delay) altitude the extension would report
        now, each as (altitude or None, source)."""
        m = self.main
        out = []
        saved = m.altitude_lookup
        try:
            for mode in ("position", "delay"):
                m.altitude_lookup = mode
                alt, det = m.compute_towfish_altitude(self.depth)
                out.append((alt, det["source"]))
        finally:
            m.altitude_lookup = saved
        return out

    def label(self) -> str:
        """truth.csv `surf` column: 'rescue/<phase>[/gate]' while the
        auto-righting drill flies the fish, else the SURF TRACK state."""
        a = self.ar.snapshot()
        if a.get("active"):
            parts = ["rescue", a["state"]]
            if a.get("down_gate"):
                parts.append(a["down_gate"])
            return "/".join(parts)
        s = self.ctrl.snapshot()
        parts = [s["state"]]
        if s.get("down_gate"):
            parts.append(s["down_gate"])
        elif s.get("hold_reason"):
            parts.append(s["hold_reason"])
        if a.get("state") == "waiting":
            parts.append("ar_waiting")
        return "/".join(parts)

    def twist(self) -> float:
        return float(self.main.tether_twist_turns)
