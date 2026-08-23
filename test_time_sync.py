#!/usr/bin/env python3
"""Exercise main.TimeSyncMonitor without touching the network.

The monitor is small and its interesting behaviour lives in a handful
of pure-ish methods -- ``_pick_source``, ``_post_time``,
``_read_boat_time_usec``, ``_read_fish_time_usec`` -- plus the state
transitions inside one iteration of ``_run``. All of them can be
driven with stubbed mavlink2rest and a stubbed writer, so this test
runs synchronously without spinning up a real thread.

Run with: python3 test_time_sync.py
"""
from __future__ import annotations

import sys
import types
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "app"))

FAILURES: list[str] = []


def check(label, condition, detail=""):
    print(f"  {'ok  ' if condition else 'FAIL'} {label}"
          + (f" {detail}" if detail and not condition else ""))
    if not condition:
        FAILURES.append(label)


def stub(name, **attrs):
    mod = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(mod, key, value)
    sys.modules[name] = mod
    return mod


# Same stub shape as test_surftrack.py so both suites can import ``main``.
stub("flask", Flask=mock.MagicMock(), jsonify=mock.MagicMock(),
     request=mock.MagicMock(), send_file=mock.MagicMock())
stub("requests", get=mock.MagicMock(), post=mock.MagicMock(),
     exceptions=mock.MagicMock(),
     Session=mock.MagicMock(), adapters=mock.MagicMock())
gi = stub("gi", require_version=lambda *a, **k: None)
stub("gi.repository", Gst=mock.MagicMock())
gi.repository = sys.modules["gi.repository"]
stub("websockets", serve=mock.MagicMock())
stub("websockets.exceptions", ConnectionClosed=Exception)
sys.modules["websockets"].exceptions = sys.modules["websockets.exceptions"]
stub("usb_storage", get_base_dir=lambda: None, start_probe=lambda: None)

import main  # noqa: E402


def fresh_monitor():
    return main.TimeSyncMonitor()


# ---------------------------------------------------------------------------
# _read_boat_time_usec: parses a mavlink2rest SYSTEM_TIME reading
# ---------------------------------------------------------------------------
print("read boat SYSTEM_TIME")

mon = fresh_monitor()
with mock.patch.object(main, "_mav_get_message",
                       return_value={"time_unix_usec": 1_700_000_000_000_000}):
    got = mon._read_boat_time_usec()
check("valid usec returned as int",
      got == 1_700_000_000_000_000, f"got {got}")

with mock.patch.object(main, "_mav_get_message", return_value=None):
    got = mon._read_boat_time_usec()
check("None message -> None", got is None, f"got {got}")

with mock.patch.object(main, "_mav_get_message",
                       return_value={"time_unix_usec": 0}):
    got = mon._read_boat_time_usec()
check("zero usec -> None (no fix on the boat)", got is None, f"got {got}")

with mock.patch.object(main, "_mav_get_message",
                       return_value={"other": 42}):
    got = mon._read_boat_time_usec()
check("missing time_unix_usec -> None", got is None, f"got {got}")


# ---------------------------------------------------------------------------
# _read_fish_time_usec: parses local mavlink2rest 200/other
# ---------------------------------------------------------------------------
print("\nread fish SYSTEM_TIME")

class FakeResp:
    def __init__(self, status_code=200, payload=None):
        self.status_code = status_code
        self._payload = payload or {}

    def json(self):
        return self._payload


mon = fresh_monitor()
with mock.patch.object(main._HTTP, "get",
                       return_value=FakeResp(200, {"message": {
                           "time_unix_usec": 1_700_000_123_456_789}})):
    got = mon._read_fish_time_usec()
check("200 + valid payload -> usec",
      got == 1_700_000_123_456_789, f"got {got}")

with mock.patch.object(main._HTTP, "get",
                       return_value=FakeResp(404, {})):
    got = mon._read_fish_time_usec()
check("non-200 -> None", got is None, f"got {got}")

with mock.patch.object(main._HTTP, "get",
                       return_value=FakeResp(200, {"message": {
                           "time_unix_usec": "nope"}})):
    got = mon._read_fish_time_usec()
check("non-numeric usec -> None", got is None, f"got {got}")

with mock.patch.object(main._HTTP, "get",
                       side_effect=RuntimeError("boom")):
    got = mon._read_fish_time_usec()
check("exception swallowed -> None", got is None, f"got {got}")


# ---------------------------------------------------------------------------
# _pick_source: prefers boat GPS, falls back to container clock
# ---------------------------------------------------------------------------
print("\npick source")

mon = fresh_monitor()
with mock.patch.object(mon, "_read_boat_time_usec",
                       return_value=1_700_000_000_000_000):
    unix_usec, source = mon._pick_source()
check("boat GPS wins", source == "boat_gps"
      and unix_usec == 1_700_000_000_000_000,
      f"got ({unix_usec}, {source})")

with mock.patch.object(mon, "_read_boat_time_usec", return_value=None), \
     mock.patch.object(main.time, "time", return_value=1_700_000_555.0):
    unix_usec, source = mon._pick_source()
check("no boat -> container_clock",
      source == "container_clock" and unix_usec == 1_700_000_555_000_000,
      f"got ({unix_usec}, {source})")


# ---------------------------------------------------------------------------
# _post_time: dispatches to the writer's system_time, propagates the bool
# ---------------------------------------------------------------------------
print("\npost time")

mon = fresh_monitor()
writer = mock.MagicMock()
writer.system_time.return_value = True
with mock.patch.object(main, "get_default_writer", return_value=writer):
    ok = mon._post_time(1_700_000_000_000_000)
check("writer returned True -> _post_time True", ok is True)
call = writer.system_time.call_args
check("unix_usec forwarded verbatim",
      call.args[0] == 1_700_000_000_000_000,
      f"got {call.args}")
check("boot_ms is a 32-bit non-negative int",
      isinstance(call.args[1], int) and 0 <= call.args[1] < (1 << 32),
      f"got {call.args[1]}")

writer.system_time.return_value = False
with mock.patch.object(main, "get_default_writer", return_value=writer):
    ok = mon._post_time(1_700_000_000_000_000)
check("writer returned False -> _post_time False", ok is False)

writer.system_time.side_effect = RuntimeError("bad")
with mock.patch.object(main, "get_default_writer", return_value=writer):
    ok = mon._post_time(1_700_000_000_000_000)
check("writer raise -> _post_time False, no propagation",
      ok is False)
writer.system_time.side_effect = None


# ---------------------------------------------------------------------------
# _boot_ms: monotonic, 32-bit bounded
# ---------------------------------------------------------------------------
print("\nboot ms")

mon = fresh_monitor()
mon._start_monotonic = 100.0
with mock.patch.object(main.time, "monotonic", return_value=100.5):
    a = mon._boot_ms()
with mock.patch.object(main.time, "monotonic", return_value=101.75):
    b = mon._boot_ms()
check("boot_ms increases with wall clock", b > a, f"got {a} -> {b}")
check("boot_ms fits in 32 bits",
      0 <= a < (1 << 32) and 0 <= b < (1 << 32),
      f"got {a}, {b}")


# ---------------------------------------------------------------------------
# One synthetic iteration of _run: fully happy path -> state 'synced'
# ---------------------------------------------------------------------------
print("\nfull loop: boat GPS ok, autopilot accepts -> synced")

def run_once(mon):
    """Execute exactly one iteration of the monitor's _run loop.

    We hijack ``self._stop.wait`` to set the stop flag right after
    the first sleep so ``_run`` returns without looping. The initial
    ``is_set()`` check has already passed by then.
    """
    original_wait = mon._stop.wait

    def one_shot(*a, **k):
        mon._stop.set()
        return True

    mon._stop.wait = one_shot  # type: ignore[assignment]
    try:
        mon._run()
    finally:
        mon._stop.wait = original_wait  # type: ignore[assignment]
        mon._stop.clear()


mon = fresh_monitor()
writer = mock.MagicMock()
writer.system_time.return_value = True
with mock.patch.object(mon, "_read_boat_time_usec",
                       return_value=1_700_000_000_000_000), \
     mock.patch.object(mon, "_read_fish_time_usec",
                       return_value=1_700_000_000_100_000), \
     mock.patch.object(main, "get_default_writer", return_value=writer):
    run_once(mon)
snap = mon.snapshot()
check("state == 'synced' after happy iteration",
      snap["state"] == "synced", f"got {snap}")
check("source recorded as 'boat_gps'",
      snap["source"] == "boat_gps", f"got {snap['source']}")
check("last_write_at populated", snap["last_write_at"] is not None)
check("last_success_at populated", snap["last_success_at"] is not None)
check("write_count == 1", snap["write_count"] == 1,
      f"got {snap['write_count']}")
check("boat_unix_usec surfaced",
      snap["boat_unix_usec"] == 1_700_000_000_000_000,
      f"got {snap['boat_unix_usec']}")
check("fish_unix_usec surfaced",
      snap["fish_unix_usec"] == 1_700_000_000_100_000,
      f"got {snap['fish_unix_usec']}")
check("no error recorded", snap["last_error"] is None,
      f"got {snap['last_error']}")


# ---------------------------------------------------------------------------
# One iteration: boat GPS ok, but fish autopilot still reads 0 -> starting
# ---------------------------------------------------------------------------
print("\nfull loop: boat ok, fish drops the write -> 'starting'")

mon = fresh_monitor()
writer = mock.MagicMock()
writer.system_time.return_value = True
with mock.patch.object(mon, "_read_boat_time_usec",
                       return_value=1_700_000_000_000_000), \
     mock.patch.object(mon, "_read_fish_time_usec", return_value=0), \
     mock.patch.object(main, "get_default_writer", return_value=writer):
    run_once(mon)
snap = mon.snapshot()
check("state stays 'starting' when fish still reads 0",
      snap["state"] == "starting", f"got {snap['state']}")
check("last_write_at still populated (we did POST)",
      snap["last_write_at"] is not None)
check("last_success_at NOT populated (never verified)",
      snap["last_success_at"] is None)


# ---------------------------------------------------------------------------
# One iteration: no boat source, fell back to container clock, fish 0
# ---------------------------------------------------------------------------
print("\nfull loop: no boat source, container clock, fish 0 -> 'no_source'")

mon = fresh_monitor()
writer = mock.MagicMock()
writer.system_time.return_value = True
with mock.patch.object(mon, "_read_boat_time_usec", return_value=None), \
     mock.patch.object(mon, "_read_fish_time_usec", return_value=0), \
     mock.patch.object(main.time, "time", return_value=1_700_000_777.0), \
     mock.patch.object(main, "get_default_writer", return_value=writer):
    run_once(mon)
snap = mon.snapshot()
check("state == 'no_source' when only container clock is available",
      snap["state"] == "no_source", f"got {snap['state']}")
check("source == 'container_clock'",
      snap["source"] == "container_clock", f"got {snap['source']}")
check("boat_unix_usec cleared when we did not read it",
      snap["boat_unix_usec"] is None,
      f"got {snap['boat_unix_usec']}")


# ---------------------------------------------------------------------------
# One iteration: post fails -> 'starting' + last_error
# ---------------------------------------------------------------------------
print("\nfull loop: post fails -> 'starting' with error")

mon = fresh_monitor()
writer = mock.MagicMock()
writer.system_time.return_value = False
with mock.patch.object(mon, "_read_boat_time_usec",
                       return_value=1_700_000_000_000_000), \
     mock.patch.object(main, "get_default_writer", return_value=writer):
    run_once(mon)
snap = mon.snapshot()
check("state 'starting' after failed post",
      snap["state"] == "starting", f"got {snap['state']}")
check("last_error surfaced",
      isinstance(snap["last_error"], str) and snap["last_error"],
      f"got {snap['last_error']}")
check("write_count NOT incremented on failure",
      snap["write_count"] == 0, f"got {snap['write_count']}")


# ---------------------------------------------------------------------------
# snapshot() shape: JSON-serialisable keys the status endpoint promises
# ---------------------------------------------------------------------------
print("\nsnapshot shape")

import json
mon = fresh_monitor()
snap = mon.snapshot()
expected_keys = {"state", "source", "write_count",
                 "last_write_at", "last_success_at", "last_error",
                 "boat_unix_usec", "fish_unix_usec"}
check("snapshot exposes the expected keys",
      set(snap.keys()) == expected_keys,
      f"got {set(snap.keys())}")
try:
    json.dumps(snap)
    serialisable = True
except Exception as e:
    serialisable = False
    print(f"  json error: {e}")
check("snapshot is JSON-serialisable", serialisable)


print()
if FAILURES:
    print(f"{len(FAILURES)} FAILED: {FAILURES}")
    raise SystemExit(1)
print("all checks passed")
