#!/usr/bin/env python3
"""Pack one or more simulation runs into the interactive 3D turn viewer.

    python3 analysis/export_viz.py --from 10:31:55 --to 10:34:10 \
        --case runs/case1_friday --case runs/case2_rollp --case runs/case3_ext \
        --out runs/viz_hairpin1/turn_viewer.html

Each run needs `--log-cable`. With several cases the page gets a switcher;
all cases share the time window and the real fish's roll/pitch/depth from
the 4 Nov log. Story beats (turn start, slack tether, pitch-up, wings at
their limit, roll past 20/90 deg, surfacing, mode changes, SURF TRACK
rescue phases) are found in each run's data, not written by hand.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from analysis.compare import boat_turns  # noqa: E402
from towsim.boat_track import parse_time  # noqa: E402
from towsim.fish import _cg_and_inertia  # noqa: E402
from towsim.frames import Config, step_to_body  # noqa: E402
from towsim.logs import fish_to_boat_offset, load_boat, load_fish, utc_offset  # noqa: E402

WING_SAT_US = 400.0


def rows(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def first_after(t, mask, t_from):
    k = np.where(mask & (t >= t_from))[0]
    return float(t[k[0]]) if len(k) else None


def run_param(run: Path, name: str):
    """A parameter's value as SITL logged it, or None."""
    p = run / "sitl.BIN"
    if not p.exists():
        return None
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(str(p))
    while True:
        r = m.recv_match(type="PARM")
        if r is None:
            return None
        if r.Name == name:
            return float(r.Value)


def run_spacing(meta: dict) -> float | None:
    v = meta["args"].get("spacing")
    return float(v) if v not in (None, "None", "") else None


def describe(meta: dict, roll_p: float | None) -> tuple[str, str]:
    """Short label and one-line description from how the run was made."""
    a = meta["args"]
    ext = a.get("extension") == "True"
    rp = "?" if roll_p is None else f"{roll_p:g}"
    sp = run_spacing(meta)
    ts = a.get("turn_speed")
    ts = float(ts) if ts not in (None, "None", "") else None
    hold = float(a.get("turn_speed_hold") or 0)
    speed_txt = ((f"boat held at {ts:g} m/s for {hold:g} s from the start of its turn" if hold
                  else f"boat held at {ts:g} m/s through its turn") if ts else "boat's logged speed")
    where = (f"Synthetic hairpin with lines {sp:g} m apart ({speed_txt}, 3.5 m turn radius). "
             if sp else "The boat's logged track (lines 7.6 m apart). ")
    pre = (f"{sp:g} m" + (f" @ {ts:g} m/s" + (f" {hold:g} s" if hold else "") if ts else "")
           + " · ") if sp else ""
    L = next((float(r[1]) for r in meta.get("config", []) if r[0] == "cable.length"), None)
    if L is not None and L != 8.0:
        pre = f"{L:g} m tether · " + pre
    ext_set = {}
    for kv in (a.get("ext_set") or "").strip("[]").replace("'", "").split(","):
        if "=" in kv:
            k, v = kv.strip().split("=", 1)
            ext_set[k] = v
    trig = meta.get("autoright_trigger")
    if ext and trig and not trig.get("boat_wait"):
        drill_trigger = (f"acts in the corner: roll past {trig['roll_deg']:g}° "
                         f"for {trig['dwell_s']:g} s, boat ignored")
    elif ext and "SURF_INVERT_ROLL_DEG" in ext_set:
        drill_trigger = (f"acts in the corner: roll past {float(ext_set['SURF_INVERT_ROLL_DEG']):g}° "
                         f"for {float(ext_set.get('SURF_INVERT_DWELL_S', 3)):g} s, boat ignored")
    elif ext and any(k.startswith("AUTORIGHT_BOAT") for k in ext_set):
        drill_trigger = "old trigger (5 s past 90°, boat ignored)"
    else:
        drill_trigger = "waits for the boat's new leg + 3 s"
    drill = []
    if "SURF_INVERT_ROLL_DEG" in ext_set:
        drill.append(f"corner drill at {float(ext_set['SURF_INVERT_ROLL_DEG']):g}°")
    elif ext and "autoright_vertical_pwm" in ext_set and not any(
            k.startswith("AUTORIGHT_BOAT") for k in ext_set):
        drill.append("waits for boat + 3 s")
    if "autoright_vertical_pwm" in ext_set:
        v = int(float(ext_set["autoright_vertical_pwm"]))
        word = {1900: "full up", 1700: "half up", 1500: "neutral", 1300: "half down",
                1100: "full down"}.get(v)
        drill.append(f"vertical {v} µs" + (f" ({word})" if word else ""))
    if float(ext_set.get("autoright_roll_min_depth_m", 0) or 0) > 0:
        drill.append(f"roll once {float(ext_set['autoright_roll_min_depth_m']):g} m deep")
    if float(ext_set.get("autoright_roll_frac", 1) or 0) == 0:
        drill.append("no roll stick")
    if float(ext_set.get("autoright_descent_start_frac", 0) or 0) > 0:
        drill.append(f"descent starts at {float(ext_set['autoright_descent_start_frac']) * 100:.0f}%")
    drill_txt = (" · " + ", ".join(drill)) if drill else ""
    if ext and a.get("autoright_only") == "True":
        return (f"{pre}Roll P {rp}, ALT_HOLD + auto-righting{drill_txt}",
                where + f"ATC_ANG_RLL_P = {rp}; SURF TRACK off, the extension's auto-righting "
                f"watching ({drill_trigger}); no operator.")
    if ext:
        return (f"{pre}Roll P {rp} + SURF TRACK + auto-righting{drill_txt}",
                where + f"ATC_ANG_RLL_P = {rp}; the extension's SurfTrackController and "
                f"AutoRightController fly (dive gates; righting {drill_trigger}); no operator.")
    if a.get("pilot") == "replay":
        return (f"{pre}Friday as flown (roll P {rp})",
                where + f"ATC_ANG_RLL_P = {rp}; ALT_HOLD with the operator's logged mode changes and RC3.")
    return (f"{pre}Roll P {rp}, ALT_HOLD alone",
            where + f"ATC_ANG_RLL_P = {rp}; ALT_HOLD with nobody flying: what the autopilot does alone.")


def build_case(run: Path, t_a: float, t_b: float, turn):
    meta = json.loads((run / "meta.json").read_text())
    truth = rows(run / "truth.csv")
    cable = rows(run / "cable.csv")
    t_boat = np.array([float(r["t_boat"]) for r in truth])
    keep = [i for i, t in enumerate(t_boat) if t_a <= t <= t_b and i < len(cable)]
    tr = [truth[i] for i in keep]
    cb = [cable[i] for i in keep]
    t = t_boat[keep]

    def col(name):
        return np.array([float(r[name]) for r in tr])

    roll, pitch, depth = col("roll"), col("pitch"), col("depth")
    s5, s6 = col("servo5"), col("servo6")
    coll = np.where((s5 > 0) & (s6 > 0), ((s5 - 1500) + (1500 - s6)) / 2, 0.0)
    tension = col("tension_n")
    modes = [r["sitl_mode"] for r in tr]
    surf = [r.get("surf", "") for r in tr]
    n_nodes = (len(cb[0]) - 1) // 3
    roll_p = run_param(run, "ATC_ANG_RLL_P")
    label, desc = describe(meta, roll_p)
    ext = meta["args"].get("extension") == "True"
    replay = meta["args"].get("pilot") == "replay"
    cable_len = next((float(r[1]) for r in meta.get("config", []) if r[0] == "cable.length"),
                     float(Config()["cable.length"]))

    frames = []
    for i, r in enumerate(tr):
        nodes = [round(float(cb[i][f"{a}{k}"]), 2) for k in range(n_nodes) for a in "ned"]
        frames.append([
            round(float(t[i] - t_a), 2),
            round(float(r["north"]), 3), round(float(r["east"]), 3), round(float(r["depth"]), 3),
            round(float(r["roll"]), 1), round(float(r["pitch"]), 1), round(float(r["yaw"]), 1),
            round(float(r["defl_port_deg"]), 1), round(float(r["defl_stbd_deg"]), 1),
            round(float(coll[i]), 0), round(float(r["tension_n"]), 1),
            round(float(r["boat_speed"]), 2), modes[i], nodes,
            round(float(r["water_speed"]), 2), surf[i],
        ])

    ar = np.abs(roll)
    events = []

    def add(key, when, text):
        if when is not None:
            events.append({"key": key, "t": round(when - t_a, 1), "text": text})

    add("start", t[0], "Straight leg. The tether is taut, the fish is towed at about 1 m/s, and "
                       + ("SURF TRACK holds it over the bottom." if ext else
                          "ALT_HOLD holds depth with small wing corrections."))
    note_bits = {}
    if turn is not None:
        sp = run_spacing(meta) or 7.6
        add("turn", turn, f"The boat starts its hairpin onto a line {sp:g} m over: 90 degrees, "
                          f"a {max(0.0, sp - 7.0):.1f} m crosslink, then 90 more onto the return line. "
                          f"The fish, ~{cable_len:.0f} m of tether behind, is still going straight.")
        tslack = first_after(t, tension < 1.0, turn)
        add("slack", tslack, "Partway round, the boat is no longer pulling away from the fish. "
                             "The tether goes slack (under 1 N) and the fish loses its tow: speed "
                             "through the water falls toward 0.2 m/s.")
        tpitch = first_after(t, pitch > 30, turn)
        add("pitch", tpitch, "With no tow and slightly positive buoyancy, the fish pitches "
                             "nose-up and starts to rise.")
        tsat = first_after(t, coll >= WING_SAT_US, turn)
        add("wings", tsat, "ALT_HOLD sees the fish rising and drives the wings to their full-down "
                           "limit. With so little water speed they produce almost no force.")
        if ext:
            gate_t = None
            for i, s in enumerate(surf):
                parts = s.split("/")
                if t[i] >= turn and parts[0] == "tracking" and len(parts) > 1 and parts[1] in (
                        "wings_saturated", "too_slow_to_dive", "roll_dive_block", "down_stall"):
                    gate_t = t[i]
                    break
            add("gate", gate_t, "SURF TRACK's dive gates stop it asking for more depth (boat too "
                                "slow, wings at their limit). They never command a climb.")
        rp_text = "0" if not roll_p else f"{roll_p:g}"
        t20 = first_after(t, ar > 20, turn)
        add("roll20", t20, "Nose-up and barely moving, the fish rolls off. "
                           + (f"With ATC_ANG_RLL_P = {rp_text} the autopilot only damps roll rate; "
                              "nothing pulls it back to level." if not roll_p else
                              f"ATC_ANG_RLL_P = {rp_text} now asks for level, but at ~0.2 m/s the "
                              "wings have no authority to deliver it."))
        t90 = first_after(t, ar > 90, turn)
        add("roll90", t90, "Rolled past 90 degrees, the wings' commanded 'down' force now points "
                           "up. The fish keeps rising toward the surface.")
        tsurf = first_after(t, depth < 0.3, t90 or turn)
        if tsurf is not None:
            k = int(np.searchsorted(t, tsurf))
            if surf[k].startswith("rescue") or modes[k] != "ALT_HOLD":
                add("surface", tsurf, "The fish breaks the surface while it is being rolled "
                                      "back upright.")
            else:
                add("surface", tsurf, "At the surface, inverted, wings still pinned at full down. "
                                      "ALT_HOLD holds it there"
                                      + (" until the operator steps in." if replay else "."))
        note_bits = {"slack": tslack, "pitch": tpitch, "roll20": t20}

    prev_mode, prev_surf = None, None
    for i, m in enumerate(modes):
        s = surf[i]
        rescue = s.startswith("rescue")
        if prev_mode is not None and m != prev_mode and m:
            if rescue and m == "MANUAL":
                text = ("Auto-righting: still past 90 degrees 3 s after the boat settled on its "
                        "new leg. MANUAL, the pilot's drill: vertical stick plus roll stick "
                        "toward upright the short way.")
            elif rescue and m == "STABILIZE":
                text = ("Still inverted after 30 s of the drill: auto-righting tries "
                        "STABILIZE, the roll controller alone with the sticks centred.")
            elif rescue and m == "ALT_HOLD":
                text = ("Upright for 1 s: back to ALT_HOLD (depth hold) and a gradual descent "
                        "to the depth it held before the turn, easing off whenever the boat "
                        "drops below 0.9 m/s.")
            else:
                text = {
                    "MANUAL": "Operator switches to MANUAL with full-up throttle. The wings "
                              "reverse and the fish rolls back upright.",
                    "STABILIZE": "STABILIZE with neutral wings: the fish glides back down.",
                    "ALT_HOLD": "ALT_HOLD again: depth hold resumes.",
                }.get(m, f"Mode {m}.")
            add(f"mode{i}", t[i], text)
        if ext and "ar_waiting" in s and (prev_surf is None or "ar_waiting" not in prev_surf):
            add(f"wait{i}", t[i], "Past 90 degrees, but the boat is still turning: auto-righting "
                                  "waits and lets ALT_HOLD's roll controller try first.")
        if ext and prev_surf is not None and prev_surf.startswith("rescue") \
                and s.startswith("tracking"):
            add(f"resume{i}", t[i], "Back at its pre-turn depth: SURF TRACK resumes tracking the bottom.")
        prev_mode, prev_surf = m, s
    events.sort(key=lambda e: e["t"])

    after = t >= (turn or t[0])
    up_after = None
    if turn is not None:
        t90 = first_after(t, ar > 90, turn)
        if t90 is not None:
            k = np.where((t > t90) & (ar < 20) & (depth > 1.0))[0]
            up_after = round(float(t[k[0]] - t90), 0) if len(k) else None
    outcome = {"max_roll": round(float(ar[after].max()), 0) if after.any() else None,
               "inverted_s": round(float(np.sum(ar[after] > 90) * 0.1), 0),
               "upright_after_s": up_after,
               "end_depth": round(float(depth[-1]), 1),
               "end_mode": modes[-1]}
    return {"label": label, "desc": desc, "run": meta["run_id"],
            "config_sha": meta["config_sha"], "git_sha": meta["git_sha"],
            "frames": frames, "events": events, "outcome": outcome, "note_bits": note_bits}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", action="append", type=Path, required=True,
                    help="a run directory made with --log-cable (repeat for a switcher)")
    ap.add_argument("--from", dest="t_from", required=True)
    ap.add_argument("--to", dest="t_to", required=True)
    ap.add_argument("--out", type=Path, help="default: <first case>/turn_viewer.html")
    ap.add_argument("--title", default="Towfish hairpin turn")
    ap.add_argument("--label", action="append", default=[],
                    help="override the switcher label of the case in the same position")
    args = ap.parse_args()
    boat_log = load_boat()
    off = utc_offset(boat_log)
    t_a = parse_time(args.t_from, off)
    t_b = parse_time(args.t_to, off)

    fish = load_fish()
    foff = fish_to_boat_offset(boat_log, fish)
    at = np.asarray(fish["ATT"]["t"]) + foff
    ct = np.asarray(fish["CTUN"]["t"]) + foff
    grid = np.arange(t_a, t_b, 0.2)
    real = {
        "t": [round(float(x - t_a), 1) for x in grid],
        "roll": [round(float(x), 1) for x in np.interp(grid, at, fish["ATT"]["Roll"])],
        "pitch": [round(float(x), 1) for x in np.interp(grid, at, fish["ATT"]["Pitch"])],
        "depth": [round(float(x), 2) for x in -np.interp(grid, ct, fish["CTUN"]["Alt"])],
    }
    turns = boat_turns(boat_log, t_a, t_b - 15)
    turn = turns[0][0] if turns else None

    cases = [build_case(p.resolve(), t_a, t_b, turn) for p in args.case]
    for c, lab in zip(cases, args.label):
        c["label"] = lab

    # Sequence note vs the real fish, per case, from the data.
    rt = np.asarray(real["t"]) + t_a
    r20 = first_after(rt, np.abs(np.asarray(real["roll"])) > 20, turn) if turn else None
    rp30 = first_after(rt, np.asarray(real["pitch"]) > 30, turn) if turn else None

    def rel(x):
        return "-" if x is None or turn is None else f"+{x - turn:.0f} s"

    for c in cases:
        nb = c.pop("note_bits")
        what = [("the tether went slack", nb.get("slack")),
                ("the fish pitched up past 30 degrees", nb.get("pitch")),
                ("it rolled past 20 degrees", nb.get("roll20"))]
        did = [f"{w} at {rel(x)}" for w, x in what if x is not None]
        never = [w.replace("went", "never went").replace("pitched", "never pitched")
                  .replace("rolled", "never rolled") for w, x in what if x is None]
        c["note"] = (f"After the boat starts turning, the real fish rolled past 20 degrees at "
                     f"{rel(r20)} and pitched nose-up past 30 degrees at {rel(rp30)}. In this run "
                     + "; ".join(did + never) + ".")

    cfg = Config()
    cg, _, _ = _cg_and_inertia(cfg)
    stations = cfg["wings.stations"]
    xw = float(cfg["wings.chord_line_x"])
    wing = [[*step_to_body([xw, s["le"], s["z"]], cg)[:2], *step_to_body([xw, s["te"], s["z"]], cg)[:2]]
            for s in stations]
    r4 = lambda v: round(float(v), 4)  # noqa: E731
    geo = {
        "wing_stations": [[r4(v) for v in w] for w in wing],
        "wing_pivot_x": r4(step_to_body([0, float(cfg["wings.pivot_y"]), 0], cg)[0]),
        "tow": [r4(v) for v in step_to_body(np.array(cfg["tow_point.pos"], dtype=float), cg)],
        "tube": [r4(step_to_body([0, y, 0], cg)[0]) for y in (-30.0, 170.0)],
        "dome_front": r4(step_to_body([0, -85.3, 0], cg)[0]),
        "cap_back": r4(step_to_body([0, 182.0, 0], cg)[0]),
        "axis_z": r4(step_to_body([0, 0, 0], cg)[2]),
        "bar": [r4(step_to_body([0, y, 0], cg)[0]) for y in (-479.6, 739.6)]
               + [r4(step_to_body([x, 0, 0], cg)[2]) for x in (65.3, 78.1)],
        "tail_x": [r4(step_to_body([0, y, 0], cg)[0]) for y in (584.8, 674.8)],
        "tail_h": float(cfg["tail.half_span_h"]) / 1000,
        "tail_v": [r4(step_to_body([x, 0, 0], cg)[2])
                   for x in (float(cfg["tail.v_top"]), float(cfg["tail.v_bottom"]))],
    }
    payload = {
        "hst0": args.t_from, "t0_utc": t_a + off,
        "cable_length": float(cfg["cable.length"]),
        "fields": ["t", "n", "e", "d", "roll", "pitch", "yaw", "defl_port", "defl_stbd",
                   "collective", "tension", "boat_speed", "mode", "nodes", "speed", "surf"],
        "cases": cases, "real": real, "geo": geo,
    }
    template = (ROOT / "viz" / "turn_viewer.html").read_text()
    html = template.replace("/*__DATA__*/null", json.dumps(payload, separators=(",", ":")))
    html = html.replace("<title>Towfish Hairpin Turn</title>", f"<title>{args.title.title()}</title>")
    html = html.replace("<h1>Towfish hairpin turn</h1>", f"<h1>{args.title}</h1>")
    out = (args.out or args.case[0] / "turn_viewer.html").resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(html)
    print(f"wrote {out} ({len(html) / 1e6:.1f} MB, {len(cases)} case(s))")
    for c in cases:
        print(f"- {c['label']}: {c['outcome']}")
        for e in c["events"]:
            print(f"    +{e['t']:6.1f}s {e['key']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
