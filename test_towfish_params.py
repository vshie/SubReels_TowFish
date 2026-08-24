#!/usr/bin/env python3
"""Exercise the towfish parameter targets and the enforcement pass.

Two things are worth testing here and neither needs a vehicle.

The specs are data, so the tests assert the properties that make the data
safe rather than restating the numbers: every target inside its own
envelope, no duplicates, every towfish spec explaining itself, and the
sanitiser clamping anything a hand-edited config could contain.

The enforcement pass is a state machine over a mocked ParamClient. The
behaviour that matters is what mission 241 lacked: a parameter found off
target gets rewritten, one already on target is left alone (an EEPROM
cycle is not free), an armed fish defers the whole pass rather than
moving the wings mid-tow, and an unreachable autopilot is a failure the
worker will retry rather than a success it reports.

Run with: python3 test_towfish_params.py
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
from mavlink_params import ParamReadError  # noqa: E402


# ---------------------------------------------------------------------------
# Fake ParamClient
# ---------------------------------------------------------------------------


class FakeClient:
    """Mimics ParamClient over an in-memory parameter store.

    ``vehicle_values`` is the autopilot's side of the wire, so a write
    mutates it and the next read sees the new value -- which is what lets
    a test assert that enforcement converged rather than merely issued a
    POST. ``writes`` records the order so "did not rewrite an
    already-correct parameter" is checkable.
    """

    def __init__(self, vehicle_values, reachable=True, fail=()):
        self.vehicle_values = dict(vehicle_values)
        self.reachable = reachable
        self.fail = set(fail)
        self.writes = []
        self.reads = []

    def is_reachable(self, timeout_s=1.5):
        return self.reachable

    def read(self, name, timeout_s=3.0):
        self.reads.append(name)
        if name in self.fail:
            raise ParamReadError(f"no PARAM_VALUE for {name}")
        return {"name": name, "value": self.vehicle_values.get(name),
                "type": "MAV_PARAM_TYPE_REAL32", "index": 0, "count": 1}

    def write(self, name, value, param_type=None, timeout_s=5.0):
        self.writes.append((name, value))
        if name in self.fail:
            raise ParamReadError(f"no PARAM_VALUE ack for {name}")
        self.vehicle_values[name] = float(value)
        return {"name": name, "value": float(value),
                "type": "MAV_PARAM_TYPE_REAL32", "index": 0, "count": 1}


def towfish_names():
    return [s["name"] for s in main.PARAM_SPECS if s["vehicle"] == "towfish"]


def run_worker(kind, names, client, armed=False):
    """Drive _param_worker synchronously with a faked vehicle."""
    with mock.patch.object(main, "_param_client", return_value=client), \
         mock.patch.object(main, "_cached_heartbeat_snapshot",
                           return_value={"armed": armed, "custom_mode": 2,
                                         "mode_label": "ALT_HOLD"}):
        main._param_worker(kind, names)
    return dict(main._param_job)


def reset_param_state():
    with main._param_lock:
        main._param_readings.clear()
        main._param_job.update({
            "running": False, "kind": None, "total": 0, "done": 0,
            "current": None, "started_at": None, "finished_at": None,
            "message": None, "corrections": [], "deferred": False,
            "failures": 0,
        })
        main._param_enforce_last.update({
            "state": "pending", "checked": 0, "corrections": [],
            "failures": 0, "message": None, "finished_at": None,
        })


# ---------------------------------------------------------------------------
print("\nspec envelope")
# ---------------------------------------------------------------------------

names = [s["name"] for s in main.PARAM_SPECS]
check("no duplicate parameter names", len(names) == len(set(names)),
      f"dupes: {[n for n in names if names.count(n) > 1]}")

for spec in main.PARAM_SPECS:
    name = spec["name"]
    check(f"{name}: target within [min, max]",
          spec["min"] <= spec["default"] <= spec["max"],
          f"{spec['min']} <= {spec['default']} <= {spec['max']}")
    check(f"{name}: vehicle is known", spec["vehicle"] in main.PARAM_VEHICLES)
    check(f"{name}: has a description", bool(spec.get("desc", "").strip()))
    presets = spec.get("presets")
    if presets:
        check(f"{name}: presets within envelope",
              all(spec["min"] <= p <= spec["max"] for p in presets),
              f"{presets} vs [{spec['min']}, {spec['max']}]")

check("DEFAULT_PARAM_TARGETS covers every spec",
      set(main.DEFAULT_PARAM_TARGETS) == set(names))

# ---------------------------------------------------------------------------
print("\ncontrol-surface corrections are present and point away from stock")
# ---------------------------------------------------------------------------

# The stock ArduSub value each of these is correcting, straight off the
# mission 241 parameter dump. The test is that the target actually
# differs -- a spec whose target equals the stock value corrects nothing.
for name, stock in main.TOWFISH_STOCK.items():
    spec = main.PARAM_SPECS_BY_NAME.get(name)
    check(f"{name}: spec exists", spec is not None)
    if spec is None:
        continue
    check(f"{name}: spec is on the towfish", spec["vehicle"] == "towfish")
    check(f"{name}: target differs from BlueROV2 stock {stock:g}",
          main._param_matches(spec, stock, spec["default"]) is False,
          f"target {spec['default']:g} == stock")
    check(f"{name}: stock is carried on the spec",
          spec.get("stock") == stock)

check("every towfish spec declares what it converts from",
      all(s.get("stock") is not None
          for s in main.PARAM_SPECS if s["vehicle"] == "towfish"),
      "missing: " + ", ".join(s["name"] for s in main.PARAM_SPECS
                              if s["vehicle"] == "towfish"
                              and s.get("stock") is None))
check("boat specs carry no stock value -- they are preferences, not a conversion",
      all(s.get("stock") is None
          for s in main.PARAM_SPECS if s["vehicle"] == "boat"))

# The two that caused the mission-241 dive behaviour specifically.
check("PILOT_SPEED_DN is set explicitly, not left at 0",
      main.PARAM_SPECS_BY_NAME["PILOT_SPEED_DN"]["default"] > 0)
check("PILOT_SPEED_DN target is no faster than the wing can manage",
      main.PARAM_SPECS_BY_NAME["PILOT_SPEED_DN"]["default"] <= 30.0)
check("MOT_THST_EXPO target linearises the servo",
      main.PARAM_SPECS_BY_NAME["MOT_THST_EXPO"]["default"] == 0.0)

# Retired on evidence: the servos take out-of-range PWM without
# complaint and have no end stops, so capping MOT_PWM would only be a
# blunt gain cut; and throttle_hover measured nearer 0.31 than the 0.5
# mixer neutral would imply, so pinning it is worse than letting the
# autopilot learn it.
for name in ("MOT_PWM_MIN", "MOT_PWM_MAX", "MOT_HOVER_LEARN",
             "MOT_THST_HOVER"):
    check(f"{name} is not enforced", name not in main.PARAM_SPECS_BY_NAME)

# ---------------------------------------------------------------------------
print("\ntarget sanitisation")
# ---------------------------------------------------------------------------

clamped = main._sanitize_param_targets({"PILOT_SPEED_DN": 999.0})
check("over-max target is clamped to the spec max",
      clamped["PILOT_SPEED_DN"] == main.PARAM_SPECS_BY_NAME["PILOT_SPEED_DN"]["max"],
      f"got {clamped['PILOT_SPEED_DN']}")

clamped = main._sanitize_param_targets({"MOT_THST_EXPO": -5.0})
check("under-min target is clamped to the spec min",
      clamped["MOT_THST_EXPO"] == main.PARAM_SPECS_BY_NAME["MOT_THST_EXPO"]["min"],
      f"got {clamped['MOT_THST_EXPO']}")

clamped = main._sanitize_param_targets({"NOT_A_PARAM": 1.0})
check("unknown name is dropped", "NOT_A_PARAM" not in clamped)

clamped = main._sanitize_param_targets({"PSC_JERK_D": "not a number"})
check("non-numeric falls back to the shipped default",
      clamped["PSC_JERK_D"] == main.PARAM_SPECS_BY_NAME["PSC_JERK_D"]["default"])

check("garbage input yields the full default map",
      main._sanitize_param_targets(None) == main.DEFAULT_PARAM_TARGETS)

# ---------------------------------------------------------------------------
print("\nenforcement corrects drift")
# ---------------------------------------------------------------------------

reset_param_state()
tf = towfish_names()
# Vehicle is on target everywhere except the two that mattered.
on_target = {n: main.param_targets[n] for n in tf}
drifted = dict(on_target)
drifted["PILOT_SPEED_DN"] = 0.0
drifted["ATC_RAT_RLL_D"] = 0.0004

client = FakeClient(drifted)
job = run_worker("enforce", tf, client)

corrected = {c["name"] for c in job["corrections"]}
check("both drifted parameters were corrected",
      corrected == {"PILOT_SPEED_DN", "ATC_RAT_RLL_D"},
      f"got {sorted(corrected)}")
check("only the drifted parameters were written",
      {n for n, _ in client.writes} == {"PILOT_SPEED_DN", "ATC_RAT_RLL_D"},
      f"wrote {sorted(n for n, _ in client.writes)}")
check("every towfish parameter was read",
      set(client.reads) == set(tf))
check("vehicle converged on the targets",
      all(main._param_matches(main.PARAM_SPECS_BY_NAME[n],
                              client.vehicle_values[n],
                              main.param_targets[n]) for n in tf))
check("correction records the before value",
      next(c for c in job["corrections"]
           if c["name"] == "ATC_RAT_RLL_D")["was"] == 0.0004)
check("no failures reported", job["failures"] == 0, str(job["failures"]))
check("enforce snapshot state is ok",
      main._param_enforce_snapshot()["state"] == "ok")

# ---------------------------------------------------------------------------
print("\nenforcement leaves a correct vehicle alone")
# ---------------------------------------------------------------------------

reset_param_state()
client = FakeClient(on_target)
job = run_worker("enforce", tf, client)
check("nothing written when already on target", client.writes == [],
      f"wrote {client.writes}")
check("message says so", "already on target" in (job["message"] or ""),
      job["message"] or "")
check("snapshot still ok", main._param_enforce_snapshot()["state"] == "ok")

# ---------------------------------------------------------------------------
print("\nenforcement stands down on an armed fish")
# ---------------------------------------------------------------------------

reset_param_state()
client = FakeClient(drifted)
job = run_worker("enforce", tf, client, armed=True)
check("armed fish is never written to", client.writes == [],
      f"wrote {client.writes}")
check("armed fish is not even read", client.reads == [])
check("job reports deferred", job["deferred"] is True)
check("message names the reason", "armed" in (job["message"] or "").lower(),
      job["message"] or "")
check("snapshot state is deferred",
      main._param_enforce_snapshot()["state"] == "deferred")
check("deferred pass reports failures so the worker retries",
      job["failures"] == len(tf), str(job["failures"]))

# An apply triggered by the operator is a deliberate act, so the armed
# gate must not silently swallow it.
reset_param_state()
client = FakeClient(drifted)
run_worker("apply", tf, client, armed=True)
check("operator apply still writes while armed",
      len(client.writes) == len(tf), f"wrote {len(client.writes)}")

# ---------------------------------------------------------------------------
print("\nenforcement treats an unreachable autopilot as a failure")
# ---------------------------------------------------------------------------

reset_param_state()
client = FakeClient(drifted, reachable=False)
job = run_worker("enforce", tf, client)
check("nothing written", client.writes == [])
check("failures equal the batch size", job["failures"] == len(tf),
      str(job["failures"]))
check("not marked deferred", job["deferred"] is False)
check("snapshot state is failed",
      main._param_enforce_snapshot()["state"] == "failed")
check("per-parameter error explains why",
      "not responding" in (main._param_readings[tf[0]]["error"] or ""),
      main._param_readings[tf[0]]["error"] or "")

# ---------------------------------------------------------------------------
print("\na single unreadable parameter does not sink the batch")
# ---------------------------------------------------------------------------

reset_param_state()
client = FakeClient(drifted, fail={"MOT_THST_EXPO"})
job = run_worker("enforce", tf, client)
check("one failure recorded", job["failures"] == 1, str(job["failures"]))
check("the other drifted parameters were still corrected",
      {c["name"] for c in job["corrections"]}
      == {"PILOT_SPEED_DN", "ATC_RAT_RLL_D"},
      f"got {sorted(c['name'] for c in job['corrections'])}")
check("snapshot state is failed", main._param_enforce_snapshot()["state"] == "failed")

# ---------------------------------------------------------------------------
print("\nsnapshot shape")
# ---------------------------------------------------------------------------

import json  # noqa: E402

snap = main._param_enforce_snapshot()
check("enforce snapshot is JSON-serialisable",
      isinstance(json.dumps(snap), str))
check("enforce snapshot has the expected keys",
      set(snap) == {"state", "checked", "corrections", "failures",
                    "message", "finished_at"},
      str(sorted(snap)))
snap["corrections"].append("mutated")
check("snapshot is a copy, not the live dict",
      main._param_enforce_snapshot()["corrections"] != snap["corrections"])

reset_param_state()
with mock.patch.object(main, "_boat_vehicle_ids", return_value=(1, 1)):
    full = main._param_snapshot()
check("/params still lists every spec",
      len(full["params"]) == len(main.PARAM_SPECS))
check("/params is JSON-serialisable", isinstance(json.dumps(full), str))
check("/params carries stock so the console can show the conversion",
      all(row["stock"] == main.TOWFISH_STOCK[row["name"]]
          for row in full["params"] if row["name"] in main.TOWFISH_STOCK))

# ---------------------------------------------------------------------------
print()
if FAILURES:
    print(f"{len(FAILURES)} FAILED: {', '.join(FAILURES)}")
    sys.exit(1)
print("all towfish parameter checks passed")
