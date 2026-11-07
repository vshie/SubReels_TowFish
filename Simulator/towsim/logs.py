"""Read the 4 Nov dataflash logs once and cache the few fields the sim uses.

Times are kept as each log's own boot seconds plus the UTC offset from its
RTC record, so the boat and fish logs line up the way the log review did
(fish = boat + ~19.6 s of boot time). The fish log supplies ONLY operator
actions (mode commands, RC3) and the as-flown parameters; its attitude and
depth are read solely by analysis/compare.py, never fed into a run.
"""
from __future__ import annotations

import hashlib
import os
import pickle
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
# The repo's "Log review" folder; the container mounts it elsewhere.
LOG_DIR = Path(os.environ.get("TOWSIM_LOG_DIR", ROOT.parent / "Log review"))
CACHE_DIR = ROOT / "runs" / "cache"
BOAT_BIN = LOG_DIR / "towboat_00000110.BIN"
FISH_BIN = LOG_DIR / "towfish_00000103.BIN"

BOAT_FIELDS = {
    "GPS": ["Lat", "Lng", "Spd", "GCrs", "Status"],
    "ATT": ["Roll", "Pitch", "Yaw"],
    "MODE": ["Mode"],
    "MSG": ["Message"],
    "RTC": ["Epoch"],
    "DPTH": ["Depth"],
}
FISH_FIELDS = {
    "MODE": ["Mode"],
    "MAVC": ["SS", "SC", "Cmd", "P1", "P2"],
    "RCIN": ["C3"],
    "RTC": ["Epoch"],
    "PARM": ["Name", "Value"],
    "ATT": ["Roll", "Pitch", "Yaw"],
    "CTUN": ["Alt"],
    "RCOU": ["C5", "C6"],
    "RATE": ["R", "RDes", "ROut"],
}


def read_bin(path: Path, fields: dict[str, list[str]]) -> dict[str, dict[str, list]]:
    """{type: {"t": [boot s], field: [...]}} for the requested types, cached."""
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    sig = repr(sorted((k, tuple(v)) for k, v in fields.items())).encode()
    key = f"{path.stem}_{hashlib.sha1(sig).hexdigest()[:10]}.pkl"
    cache = CACHE_DIR / key
    if cache.exists() and cache.stat().st_mtime > path.stat().st_mtime:
        return pickle.loads(cache.read_bytes())
    from pymavlink import mavutil

    out: dict[str, dict[str, list]] = defaultdict(lambda: defaultdict(list))
    mlog = mavutil.mavlink_connection(str(path))
    while True:
        m = mlog.recv_match(type=list(fields))
        if m is None:
            break
        ty = m.get_type()
        rec = out[ty]
        rec["t"].append(m.TimeUS / 1e6)
        for f in fields[ty]:
            rec[f].append(getattr(m, f, None))
    data = {k: dict(v) for k, v in out.items()}
    cache.write_bytes(pickle.dumps(data))
    return data


def utc_offset(log: dict) -> float:
    """UTC epoch seconds minus boot seconds, from the first RTC with an epoch."""
    rtc = log["RTC"]
    for t, ep in zip(rtc["t"], rtc["Epoch"]):
        if ep:
            return ep / 1e6 - t
    raise ValueError("log has no RTC epoch")


def load_boat():
    return read_bin(BOAT_BIN, BOAT_FIELDS)


def load_fish():
    return read_bin(FISH_BIN, FISH_FIELDS)


def fish_to_boat_offset(boat: dict, fish: dict) -> float:
    """Add to a fish boot time to get the boat boot time."""
    return utc_offset(fish) - utc_offset(boat)
