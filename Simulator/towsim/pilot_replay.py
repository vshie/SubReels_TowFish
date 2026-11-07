"""Operator actions from the 4 Nov fish log, re-timed onto the sim clock.

Only what the operator (via Cockpit / the extension) sent the fish is
replayed: DO_SET_MODE commands from GCS 255/240 and the RC3 channel. These
are inputs to the autopilot, not the fish's response, so replaying them does
not leak the outcome into the run.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .logs import fish_to_boat_offset, load_boat, load_fish

MODE_STABILIZE, MODE_ALT_HOLD, MODE_MANUAL = 0, 2, 19
MODE_NAMES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 19: "MANUAL"}


@dataclass
class PilotSchedule:
    """Times are boat boot seconds."""
    modes: list[tuple[float, int]] = field(default_factory=list)
    rc3_t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    rc3_v: np.ndarray = field(default_factory=lambda: np.zeros(0))
    logged_modes: list[tuple[float, int]] = field(default_factory=list)

    def mode_at(self, t: float) -> int | None:
        m = None
        for tt, mm in self.logged_modes:
            if tt <= t:
                m = mm
            else:
                break
        return m

    def rc3_at(self, t: float) -> int:
        if not len(self.rc3_t):
            return 1500
        i = int(np.searchsorted(self.rc3_t, t, side="right")) - 1
        return int(self.rc3_v[i]) if i >= 0 else 1500


def load_schedule() -> PilotSchedule:
    boat, fish = load_boat(), load_fish()
    off = fish_to_boat_offset(boat, fish)
    mc = fish["MAVC"]
    modes = []
    for t, ss, cmd, p2 in zip(mc["t"], mc["SS"], mc["Cmd"], mc["P2"]):
        if cmd == 176 and ss == 255:
            modes.append((t + off, int(p2)))
    logged = [(t + off, int(m)) for t, m in zip(fish["MODE"]["t"], fish["MODE"]["Mode"])]
    rc = fish["RCIN"]
    return PilotSchedule(modes=modes,
                         rc3_t=np.asarray(rc["t"]) + off,
                         rc3_v=np.asarray(rc["C3"], dtype=float),
                         logged_modes=logged)


def no_pilot(start_mode: int = MODE_ALT_HOLD) -> PilotSchedule:
    """ALT_HOLD with neutral RC3 throughout: what the autopilot does alone."""
    return PilotSchedule(modes=[], rc3_t=np.zeros(0), rc3_v=np.zeros(0),
                         logged_modes=[(-1e9, start_mode)])
