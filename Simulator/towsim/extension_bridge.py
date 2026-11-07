"""Run the SubReels extension's controllers against the simulator.

SurfTrackController and AutoRightController are imported unmodified from
app/main.py (the same way the extension's own tests import them). Only
their I/O is redirected:

  fish depth      <- SITL VFR_HUD (what the extension reads via mavlink2rest)
  fish attitude   <- SITL ATTITUDE
  armed / mode    <- SITL HEARTBEAT
  wing outputs    <- the servo PWMs SITL sends the physics (SERVO_OUTPUT_RAW)
  boat sonar      <- the boat's logged DPTH and speed, on the sim clock
                     (or a flat bottom at --bottom-depth)
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


class BoatSonar:
    """The extension's SonarHistory interface over the boat's logged sounding."""

    def __init__(self, track, tow_offset_m: float, bottom_depth_m: float | None = None):
        boat = load_boat()
        d = boat["DPTH"]
        self.t = np.asarray(d["t"])
        self.depth = np.asarray(d["Depth"], dtype=float)
        self.track = track
        self.tow_offset = tow_offset_m
        self.bottom = bottom_depth_m
        self.t_boat = 0.0

    def _valid(self, t):
        if self.bottom is not None:
            return float(self.bottom)
        v = float(np.interp(t, self.t, self.depth))
        return v if 0.3 < v < 100 else None

    def latest(self, max_age_s):
        v = self._valid(self.t_boat)
        return None if v is None else (v, 0.2)

    def latest_speed_ms(self):
        i = int(np.searchsorted(self.track.t, self.t_boat))
        i = min(max(i, 0), len(self.track.t) - 1)
        return float(self.track.speed[i])

    def depth_at_tow_delay(self):
        spd = max(0.15, self.latest_speed_ms())
        delay = self.tow_offset / spd
        v = self._valid(self.t_boat - delay)
        return None if v is None else (v, delay, 0.0)

    def last_update_id(self):
        return int(self.t_boat * 2.8)


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
                 tether_length_m=0.0, bottom_depth_m=None, ext_set=None,
                 surftrack=True):
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
        self.sonar = BoatSonar(track, tow_offset_m, bottom_depth_m)
        self.nav = BoatNav(track)
        self.t_boat = 0.0
        main._get_towfish_depth_m = lambda: self.depth
        main.get_towfish_attitude = lambda: dict(self.attitude)
        main.get_vehicle_status_snapshot = lambda *a, **k: dict(self.vehicle)
        main._servo_output_message = lambda: dict(self.servo)
        main._sonar_history = self.sonar
        main._set_thrust_command = self._thrust
        main.get_default_writer = lambda: self
        main._persist_config = lambda: None
        main._tow_mission_seq = lambda: self.nav.seq(self.t_boat)
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
        if source == "surftrack" and self._rc_owner == "autoright":
            return False
        self._rc_owner = source
        self.rc3 = int(pwm)
        if roll_pwm is not None:
            self.rc2 = int(roll_pwm)
        elif self.rc2 is not None:
            self.rc2 = 1500
        return True

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
        self.sonar.t_boat = t_boat
        self.t_boat = t_boat
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
        # The extension's own NVAL worker may drain the same queue, so an
        # empty() check can go stale before get_nowait().
        q = self.ctrl._bin_queue
        while True:
            try:
                name, value = q.get_nowait()
            except queue.Empty:
                break
            self.named_value_float(name, value)

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
