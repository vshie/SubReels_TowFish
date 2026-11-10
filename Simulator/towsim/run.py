#!/usr/bin/env python3
"""Run one towfish simulation window against ArduSub SITL.

    python3 -m towsim.run --start 10:28 --end 10:31 --pilot none

Clock: the physics is SITL's clock (JSON lockstep). Sim time 0 is
`--warmup` seconds before `--start`; during the warmup the fish is held on a
jig at `--init-depth` while the cable settles, SITL boots, the vehicle is
armed and put in its starting mode. At `--start` the jig releases and
everything after that is physics + ArduSub + the replayed operator.

Outputs land in runs/<id>/: truth.csv (physics state at 10 Hz), meta.json
(arguments, config hash and status table, git SHA), sitl.log, logs/*.BIN.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from towsim.boat_track import load_track, parse_time  # noqa: E402
from towsim.fish import TowfishPhysics  # noqa: E402
from towsim.frames import Config, euler_from_quat, quat_from_euler  # noqa: E402
from towsim.logs import load_boat, utc_offset  # noqa: E402
from towsim.pilot_replay import MODE_ALT_HOLD, MODE_NAMES, MODE_STABILIZE, load_schedule, no_pilot  # noqa: E402
from towsim.sitl_json import JsonBridge, state_json  # noqa: E402

DEFAULT_SITL_BIN = os.environ.get("ARDUSUB_BIN", "/ardupilot/build/sitl/bin/ardusub")
RC_KEEPALIVE_S = 0.25
TRUTH_PERIOD_S = 0.1
# Warmup jig profile: the real fish boots at the surface, where ArduSub zeroes
# its depth from the pressure sensor. Hold the jig there while SITL boots,
# then lower it to --init-depth before arming.
JIG_SURFACE_DEPTH = 0.15
JIG_SURFACE_S = 15.0
JIG_DESCENT_MPS = 0.3


def jig_depth(t_sim: float, init_depth: float) -> float:
    if t_sim < JIG_SURFACE_S:
        return JIG_SURFACE_DEPTH
    return min(init_depth, JIG_SURFACE_DEPTH + JIG_DESCENT_MPS * (t_sim - JIG_SURFACE_S))


def git_sha() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT,
                                       stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return "unknown"


class Mav:
    """Operator link to SITL, sending as GCS 255/240 like Cockpit/the extension."""

    def __init__(self, url: str):
        # MAVLink 2 for the 18-channel RC_CHANNELS_OVERRIDE; must be set
        # before pymavlink loads its dialect.
        os.environ.setdefault("MAVLINK20", "1")
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.url = url
        self.conn = None
        self.mode: int | None = None
        self.armed: bool | None = None
        self.attitude: tuple[float, float, float] | None = None
        self.vfr_alt: float | None = None

    def ensure(self) -> bool:
        if self.conn is not None:
            return True
        try:
            self.conn = self.mavutil.mavlink_connection(self.url, source_system=255,
                                                        source_component=240, autoreconnect=True)
            return True
        except Exception:
            self.conn = None
            return False

    def poll(self):
        if self.conn is None:
            return
        while True:
            m = self.conn.recv_match(type=["HEARTBEAT", "ATTITUDE", "VFR_HUD"], blocking=False)
            if m is None:
                return
            if m.get_srcSystem() != 1:
                continue
            ty = m.get_type()
            if ty == "HEARTBEAT" and m.type != self.mavutil.mavlink.MAV_TYPE_GCS:
                self.mode = m.custom_mode
                self.armed = bool(m.base_mode & self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif ty == "ATTITUDE":
                self.attitude = (m.roll, m.pitch, m.yaw)
            elif ty == "VFR_HUD":
                self.vfr_alt = m.alt

    def named_value(self, name: str, value: float):
        if self.ensure():
            self.conn.mav.named_value_float_send(0, name.encode()[:10], float(value))

    def heartbeat(self):
        if self.ensure():
            self.conn.mav.heartbeat_send(self.mavutil.mavlink.MAV_TYPE_GCS,
                                         self.mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def arm(self):
        if self.ensure():
            self.conn.mav.command_long_send(1, 1, self.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                            0, 1, 21196, 0, 0, 0, 0, 0)

    def set_mode(self, mode: int):
        if self.ensure():
            self.conn.mav.command_long_send(1, 1, self.mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                            0, 1, mode, 0, 0, 0, 0, 0)

    def rc3(self, pwm: int, roll_pwm: int | None = None):
        """RC3 override, plus RC2 (roll stick) when given; 0 leaves a channel alone."""
        if self.ensure():
            ch = [0] * 18
            ch[2] = int(pwm)
            if roll_pwm is not None:
                ch[1] = int(roll_pwm)
            self.conn.mav.rc_channels_override_send(1, 1, *ch)


def spawn_sitl(args, run_dir: Path, track, scenario_parm: Path | None) -> subprocess.Popen:
    params = [ROOT / "params" / "towfish_as_flown.parm", ROOT / "params" / "sitl_overrides.parm"]
    if scenario_parm:
        params.append(scenario_parm)
    home = f"{track.lat0:.8f},{track.lon0:.8f},0,{track.heading0_deg:.1f}"
    # -w wipes eeprom.bin so a reused run id cannot inherit stored params.
    cmd = [args.sitl_bin, "-S", "-w", "-I0", "--model", "JSON:127.0.0.1",
           "--speedup", str(args.speedup), "--home", home,
           "--defaults", ",".join(str(p) for p in params),
           "--serial0", "tcp:0", "--serial1", "tcp:2"]
    log = (run_dir / "sitl.log").open("w")
    print("SITL:", " ".join(cmd))
    return subprocess.Popen(cmd, cwd=run_dir, stdout=log, stderr=subprocess.STDOUT)


def terrain_cols(terrain, pos, top, ext):
    """bottom_fish, alt_true, sonar_depth, sonar_q, surf_target, surf_raw,
    s_fish, s_boat (distance along the terrain), and the altitude the
    extension reports by position lookup (with the source it fell back
    to) and by time delay."""
    out = ["", "", "", "", "", "", "", "", "", "", ""]
    if terrain is not None:
        b = terrain.depth_at(pos[0], pos[1])
        out[0], out[1] = f"{b:.3f}", f"{b - pos[2]:.3f}"
        out[6] = f"{terrain.along_cross(pos[0], pos[1])[0]:.2f}"
        out[7] = f"{terrain.along_cross(top[0], top[1])[0]:.2f}"
    if ext is not None:
        pk = ext.packet or {}
        if pk.get("distance_m") is not None:
            out[2] = f"{pk['distance_m']:.3f}"
            out[3] = f"{pk.get('signal_quality', '')}"
        c = ext.ctrl
        if c._target_depth_m is not None:
            out[4] = f"{c._target_depth_m:.3f}"
        if c._raw_target_depth_m is not None:
            out[5] = f"{c._raw_target_depth_m:.3f}"
        (ap, src), (ad, _) = ext.altitudes()
        out[8] = f"{ap:.3f}" if ap is not None else ""
        out[9] = src or ""
        out[10] = f"{ad:.3f}" if ad is not None else ""
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--start", required=True, help="HH:MM[:SS] HST on 4 Nov, or boat log seconds")
    ap.add_argument("--end", required=True)
    ap.add_argument("--pilot", choices=["replay", "none"], default="replay")
    ap.add_argument("--init-depth", type=float, default=3.0,
                    help="jig depth during warmup (scenario initial condition, m)")
    ap.add_argument("--warmup", type=float, default=45.0,
                    help="seconds before --start: surface hold, descent to --init-depth, arm")
    ap.add_argument("--sitl", choices=["spawn", "external", "none"], default="spawn",
                    help="spawn ardusub here, use an already-running one, or run physics "
                         "open-loop with neutral wings (smoke test only)")
    ap.add_argument("--sitl-bin", default=DEFAULT_SITL_BIN)
    ap.add_argument("--mavlink", default="tcp:127.0.0.1:5760")
    ap.add_argument("--speedup", type=float, default=50.0)
    ap.add_argument("--param-file", type=Path, help="extra .parm layered last (scenario)")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="override a towfish.yaml value for this run (recorded in meta.json)")
    ap.add_argument("--run-id")
    ap.add_argument("--spacing", type=float,
                    help="replace the logged boat track with the same hairpin rebuilt at this "
                         "lane spacing in metres (logged lines were 7.6 m apart)")
    ap.add_argument("--turn-speed", type=float,
                    help="with --spacing: boat speed (m/s) held through the hairpin, from the "
                         "start of the first 90-degree turn to the end of the second")
    ap.add_argument("--turn-speed-hold", type=float, default=0.0,
                    help="with --turn-speed: keep that speed at least this many seconds from "
                         "the start of the boat's turn (covers the fish's own turn)")
    ap.add_argument("--extension", action="store_true",
                    help="fly SURF TRACK: the extension's SurfTrackController from app/main.py, "
                         "unmodified, ticked on the sim clock (implies --pilot none)")
    ap.add_argument("--autoright-only", action="store_true",
                    help="with --extension: auto-righting on, SURF TRACK off (plain ALT_HOLD)")
    ap.add_argument("--ext-set", action="append", default=[], metavar="NAME=VALUE",
                    help="with --extension: set an app/main.py module variable, e.g. "
                         "autoright_vertical_pwm=1700 (recorded in meta.json)")
    ap.add_argument("--straight", type=float, metavar="SPEED",
                    help="replace the boat track with a straight tow at this constant speed "
                         "(m/s) on the logged heading")
    ap.add_argument("--bottom-depth", type=float,
                    help="with --extension: a flat bottom at this depth instead of the logged "
                         "sounding")
    ap.add_argument("--terrain", metavar="SPEC",
                    help="seabed along the tow, seen by a Ping model (towsim/terrain.py): "
                         "slope, coral or flat:<depth>; distance is measured from the boat "
                         "at --start")
    ap.add_argument("--ping-seed", type=int, default=0,
                    help="with --terrain: where in the logged Ping packet schedule to start")
    ap.add_argument("--fixed-stick", type=int, metavar="PWM",
                    help="STABILIZE with RC3 held at PWM after release (e.g. 1300 = half "
                         "down): the wings' collective is set by hand, roll stays stabilised")
    ap.add_argument("--stick-steps", metavar="PWM@S,...",
                    help="scripted vertical stick: RC3 PWM from S seconds after --start, e.g. "
                         "1500@0,1900@30,1500@40 (rate step tests; implies --pilot none)")
    ap.add_argument("--stick-mode", choices=["alt_hold", "stabilize"], default="alt_hold",
                    help="with --stick-steps: the mode the steps are flown in")
    ap.add_argument("--log-cable", action="store_true",
                    help="also write cable.csv (every node's NED position at 10 Hz) for visualisation")
    args = ap.parse_args(argv)

    overrides = {}
    for kv in args.set:
        k, v = kv.split("=", 1)
        overrides[k] = json.loads(v) if v[:1] in "[{" else (float(v) if v.replace(".", "", 1).replace("-", "", 1).isdigit() else v)
    cfg = Config(overrides=overrides)

    boat_off = utc_offset(load_boat())
    t_start = parse_time(args.start, boat_off)
    t_end = parse_time(args.end, boat_off)
    t0 = t_start - args.warmup  # boat time at sim time 0
    track = load_track(t_start, float(cfg["cable.boat_attach_height"]))
    if args.spacing:
        from analysis.compare import boat_turns
        from towsim.boat_track import synthetic_turn_track
        turns = boat_turns(load_boat(), t_start, t_end)
        if not turns:
            raise SystemExit("--spacing needs a boat turn inside the window")
        ts, _, dh = turns[0]
        track = synthetic_turn_track(track, t_start - args.warmup - 5, ts,
                                     1.0 if dh > 0 else -1.0, args.spacing,
                                     turn_speed=args.turn_speed,
                                     turn_speed_hold_s=args.turn_speed_hold)
    if args.straight:
        from towsim.boat_track import synthetic_straight_track
        track = synthetic_straight_track(track, t_start - args.warmup - 5, args.straight)
    if args.autoright_only:
        args.extension = True
    if args.extension:
        args.pilot = "none"
    ext_set = {}
    for kv in args.ext_set:
        k, v = kv.split("=", 1)
        try:
            ext_set[k] = json.loads(v)
        except ValueError:
            ext_set[k] = v      # a bare string, e.g. surf_lookahead=shallowest
    if args.fixed_stick is not None:
        args.pilot = "none"
    if args.stick_steps:
        args.pilot = "none"
    sched = load_schedule() if args.pilot == "replay" else no_pilot()
    if args.fixed_stick is not None:
        sched = no_pilot(MODE_STABILIZE)
        sched.rc3_t = np.array([-1e9])
        sched.rc3_v = np.array([float(args.fixed_stick)])
    if args.stick_steps:
        steps = [(float(s.split("@")[1]), float(s.split("@")[0]))
                 for s in args.stick_steps.split(",")]
        steps.sort()
        sched = no_pilot(MODE_STABILIZE if args.stick_mode == "stabilize" else MODE_ALT_HOLD)
        sched.rc3_t = np.array([t_start + t for t, _ in steps])
        sched.rc3_v = np.array([v for _, v in steps])
    start_mode = (sched.mode_at(t_start)
                  if args.pilot == "replay" or args.fixed_stick is not None or args.stick_steps
                  else MODE_ALT_HOLD)
    if start_mode is None:
        start_mode = MODE_ALT_HOLD

    run_id = args.run_id or datetime.now().strftime("%Y%m%d_%H%M%S") + f"_{args.pilot}"
    run_dir = ROOT / "runs" / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    meta = {
        "run_id": run_id, "args": {k: str(v) for k, v in vars(args).items()},
        "t_start_boat": t_start, "t_end_boat": t_end, "t0_boat": t0,
        "hst_start": track.hst(t_start), "hst_end": track.hst(t_end),
        "home": [track.lat0, track.lon0, track.heading0_deg],
        "start_mode": MODE_NAMES.get(start_mode, start_mode),
        "config_sha": cfg.sha, "git_sha": git_sha(),
        "config": [list(map(str, r)) for r in cfg.status_table()],
    }
    (run_dir / "meta.json").write_text(json.dumps(meta, indent=1))

    phys = TowfishPhysics(cfg)
    dt_phys = float(cfg["sim.physics_dt"])

    # Jig: level, heading along the boat's course, trailing by most of the
    # cable's horizontal reach at the initial depth.
    top, vtop = track.state(t0)
    hdg = math.atan2(vtop[1], vtop[0]) if np.hypot(*vtop[:2]) > 0.2 else math.radians(track.heading0_deg)
    reach = math.sqrt(max(1.0, phys.cable.length ** 2 - args.init_depth ** 2)) * 0.95
    back = np.array([math.cos(hdg), math.sin(hdg), 0.0])
    q0 = quat_from_euler(0.0, 0.0, hdg)
    fish_pos = top - back * reach
    fish_pos[2] = JIG_SURFACE_DEPTH
    phys.place(fish_pos, q0, vtop)
    phys.cable.lay_straight(top, phys.tow_point_ned(), vtop)
    phys.jig = True

    sitl_proc = None
    if args.sitl == "spawn":
        # SITL runs inside run_dir, so hand it an absolute path.
        sitl_proc = spawn_sitl(args, run_dir, track,
                               args.param_file.resolve() if args.param_file else None)
    mav = Mav(args.mavlink) if args.sitl != "none" else None
    terrain = None
    if args.bottom_depth is not None and not args.terrain:
        args.terrain = f"flat:{args.bottom_depth}"
    if args.terrain:
        from towsim.terrain import Terrain
        p_s, v_s = track.state(t_start)
        h_s = (math.atan2(v_s[1], v_s[0]) if np.hypot(*v_s[:2]) > 0.2
               else math.radians(track.heading0_deg))
        terrain = Terrain.from_spec(args.terrain, p_s, h_s)
    ext = None
    if args.extension and mav is not None:
        from towsim.extension_bridge import ExtensionBridge
        # Sonar layback: the tether's horizontal reach at the starting depth.
        L = phys.cable.length
        ext = ExtensionBridge(mav, track, tether_length_m=L,
                              tow_offset_m=math.sqrt(max(1.0, L ** 2 - args.init_depth ** 2)),
                              terrain=terrain, ext_set=ext_set,
                              surftrack=not args.autoright_only, ping_seed=args.ping_seed)
        # The trigger the extension actually flew (after any --ext-set), so
        # a later viewer labels the run by its code, not by today's defaults.
        m = ext.main
        meta["autoright_trigger"] = {"roll_deg": m.SURF_INVERT_ROLL_DEG,
                                     "dwell_s": m.SURF_INVERT_DWELL_S,
                                     "boat_wait": hasattr(m, "AUTORIGHT_BOAT_ALIGNED_DEG"),
                                     "vertical_pwm": m.autoright_vertical_pwm}
        (run_dir / "meta.json").write_text(json.dumps(meta, indent=1))
    bridge = JsonBridge() if args.sitl != "none" else None

    truth_f = (run_dir / "truth.csv").open("w", newline="")
    cable_f = (run_dir / "cable.csv").open("w", newline="") if args.log_cable else None
    cw = csv.writer(cable_f) if cable_f else None
    if cw:
        cw.writerow(["t_sim"] + [f"{a}{i}" for i in range(phys.cable.n + 1) for a in "ned"])
    tw = csv.writer(truth_f)
    tw.writerow(["t_sim", "t_boat", "hst", "north", "east", "depth", "roll", "pitch", "yaw",
                 "u", "v", "w", "p", "q", "r", "defl_port_deg", "defl_stbd_deg",
                 "servo5", "servo6", "tension_n", "boat_speed", "water_speed",
                 "layback_m", "sitl_mode", "armed", "rc3", "jig", "surf", "rc2", "ar_twist",
                 "est_n", "est_e", "est_unc",
                 "bottom_fish", "alt_true", "sonar_depth", "sonar_q", "surf_target", "surf_raw",
                 "s_fish", "s_boat", "alt_pos", "alt_pos_src", "alt_delay"])

    t_sim = 0.0
    next_truth = 0.0
    next_rc = 0.0
    next_hb = 0.0
    events_done = set()
    mode_idx = 0
    modes = [(t, m) for t, m in sched.modes if t_start < t <= t_end]
    rc_now = 1500
    rc2_now = None
    pwm = [0] * 16
    wall0 = time.time()
    last_print = 0.0
    print(f"run {run_id}: {track.hst(t_start)}..{track.hst(t_end)} HST, pilot={args.pilot}, "
          f"start mode {MODE_NAMES.get(start_mode)}, config {cfg.sha}")
    try:
        while True:
            t_boat = t0 + t_sim
            if t_boat >= t_end:
                break
            if bridge is not None:
                got = bridge.receive()
                if got is None:
                    print("no packet from SITL for 30 s; stopping")
                    break
                pkt, new = got
                if not new:
                    bridge.reply()
                    continue
                frame_dt = 1.0 / max(1, pkt.frame_rate)
                pwm = list(pkt.pwm)
            else:
                frame_dt = 0.0025
                pwm = [0] * 4 + [1500, 1500] + [0] * 10
            phys.set_servos(pwm)

            n_sub = max(1, math.ceil(frame_dt / dt_phys - 1e-9))
            h = frame_dt / n_sub
            for _ in range(n_sub):
                top, vtop = track.state(t0 + t_sim)
                if phys.jig:
                    phys.jig_advance(h, vtop)
                    phys.pos[2] = jig_depth(t_sim, args.init_depth)
                phys.step(h, top, vtop)
                t_sim += h
            # Cable node 0 is the boat, node N the fish's tow point.
            t_boat = t0 + t_sim

            if bridge is not None:
                bridge.reply(state_json(t_sim, phys.w, phys.accel_body, phys.pos, phys.q, phys.vel_ned))

            # Operator / scheduler (sim time).
            if mav is not None:
                mav.poll()
                if t_sim >= next_hb:
                    mav.heartbeat()
                    next_hb = t_sim + 1.0
                if "arm" not in events_done and t_sim >= args.warmup - 8:
                    mav.arm()
                    events_done.add("arm")
                if "mode0" not in events_done and t_sim >= args.warmup - 5:
                    mav.set_mode(start_mode)
                    events_done.add("mode0")
                if "rearm" not in events_done and t_sim >= args.warmup - 2:
                    if not mav.armed:
                        mav.arm()
                    if mav.mode != start_mode:
                        mav.set_mode(start_mode)
                    events_done.add("rearm")
                if t_boat > t_start:
                    while mode_idx < len(modes) and modes[mode_idx][0] <= t_boat:
                        mav.set_mode(modes[mode_idx][1])
                        mode_idx += 1
                if ext is not None and t_boat > t_start:
                    ext.step(t_sim, t_boat, pwm)
                    if ext.rc3 != rc_now or ext.rc2 != rc2_now:
                        next_rc = t_sim  # send a changed command at once
                if t_sim >= next_rc:
                    if ext is not None:
                        rc_now = ext.rc3 if t_boat > t_start else 1500
                        rc2_now = ext.rc2 if t_boat > t_start else None
                    else:
                        rc_now = sched.rc3_at(t_boat) if t_boat > t_start else 1500
                    mav.rc3(rc_now, rc2_now)
                    next_rc = t_sim + RC_KEEPALIVE_S

            if phys.jig and t_boat >= t_start:
                phys.jig = False

            if t_sim >= next_truth and cw is not None:
                cw.writerow([f"{t_sim:.3f}"] + [f"{x:.3f}" for x in phys.cable.x.ravel()])
            if t_sim >= next_truth:
                next_truth = t_sim + TRUTH_PERIOD_S
                r, p, y = (math.degrees(a) for a in euler_from_quat(phys.q))
                water = phys.water_speed()
                lay = float(np.hypot(*(top[:2] - phys.pos[:2])))
                tw.writerow([f"{t_sim:.3f}", f"{t_boat:.3f}", track.hst(t_boat),
                             f"{phys.pos[0]:.3f}", f"{phys.pos[1]:.3f}", f"{phys.pos[2]:.3f}",
                             f"{r:.2f}", f"{p:.2f}", f"{y % 360:.2f}",
                             *(f"{x:.3f}" for x in phys.v), *(f"{math.degrees(x):.2f}" for x in phys.w),
                             f"{math.degrees(phys.defl[0]):.2f}", f"{math.degrees(phys.defl[1]):.2f}",
                             pwm[4], pwm[5], f"{phys.cable.fish_end_tension:.2f}",
                             f"{np.hypot(*vtop[:2]):.3f}", f"{water:.3f}", f"{lay:.2f}",
                             MODE_NAMES.get(mav.mode, mav.mode) if mav else "",
                             int(bool(mav.armed)) if mav else 0, rc_now, int(phys.jig),
                             ext.label() if ext is not None else "",
                             rc2_now if rc2_now is not None else "",
                             f"{ext.twist():.3f}" if ext is not None else "",
                             *(ext.position_estimate() if ext is not None else ("", "", "")),
                             *terrain_cols(terrain, phys.pos, top, ext)])
            if time.time() - last_print > 10:
                last_print = time.time()
                r, p, _ = (math.degrees(a) for a in euler_from_quat(phys.q))
                print(f"  {track.hst(t_boat)} depth {phys.pos[2]:5.2f} roll {r:6.1f} pitch {p:6.1f} "
                      f"mode {MODE_NAMES.get(mav.mode, mav.mode) if mav else '-'} "
                      f"x{t_sim / max(1e-6, time.time() - wall0):.2f} realtime")
            if not np.isfinite(phys.pos).all():
                print("physics diverged; stopping")
                break
    finally:
        truth_f.close()
        if cable_f:
            cable_f.close()
        if bridge is not None:
            bridge.close()
        if sitl_proc is not None:
            sitl_proc.terminate()
            try:
                sitl_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                sitl_proc.kill()
    logs = sorted((run_dir / "logs").glob("*.BIN")) if (run_dir / "logs").exists() else []
    if logs:
        shutil.copy(logs[-1], run_dir / "sitl.BIN")
    print(f"done: {run_dir}  ({t_sim:.0f} s simulated in {time.time() - wall0:.0f} s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
