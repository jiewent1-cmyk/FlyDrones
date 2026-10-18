"""S12-G4: MiniFly sees the Gazebo camera and (optionally) flies ArduPilot SITL in Gazebo.

modes
  probe     open loop: the brain watches but its commands are ignored; a script moves the drone (yaw right/left,
            climb, descend, approach a box). Checks that the decoded command opposes each self-motion and that
            looming neurons fire on approach.
  hover     closed loop, cruise 0: MiniFly holds height/heading from optic flow (+ haltere yaw rate)
  approach  closed loop with a forward cruise toward the boxes: looming should brake / trigger escape

needs gazebo/cam_bridge.py running (gz_g4.sh starts everything)
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import multiprocessing as mp
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import sitl_util as su
from envs import ECPS295_DYN, ECPS295_SAFETY_20FT, TAKEOFF_ALT_M
from gz_camera_drone import GazeboCameraMavlinkDrone
from world_layout import room_from_layout

from flydrones.brain import Brain, load_connectome
from flydrones.config import load_config
from flydrones.motor.command import FlightCommand
from flydrones.runtime import Pilot

ap = argparse.ArgumentParser()
ap.add_argument("--mode", choices=["probe", "hover", "approach", "patrol"], required=True)
ap.add_argument("--url", default="tcp:127.0.0.1:5760")
ap.add_argument("--seconds", type=float, default=40)
ap.add_argument("--cruise", type=float, default=0.5, help="approach: decoder.cruise (x safety max_forward x v_max)")
ap.add_argument("--max-forward", type=float, default=0.6, help="approach: safety.max_forward")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--config", default="", help="MiniFly variant YAML (ecps295/minifly/v*.yaml), merged over defaults")
ap.add_argument("--decoder", choices=["upstream", "ecps"], default="upstream", help="ecps = ecps_decoder.EcpsDecoder")
ap.add_argument("--pilot", choices=["upstream", "ecps"], default="upstream", help="ecps = ecps_pilot.EcpsPilot (watchdogs)")
ap.add_argument("--fence-turn", action="store_true", help="EcpsPilot: yaw back toward the centre at the geofence")
ap.add_argument("--retina", choices=["upstream", "ecps"], default="upstream", help="ecps = ecps_retina.EcpsRetina (+blank)")
ap.add_argument("--gif", default="", help="render the brain dashboard to this GIF after the flight")
ap.add_argument("--gif-every", type=int, default=3)
ap.add_argument("--freeze-cam-at", type=float, default=-1, help="fault injection: camera stops updating at t [s]")
ap.add_argument("--record-frames", default="", help="save the colour drone camera as JPEG each tick (pil_replay.py input)")
ap.add_argument("--live", action="store_true", help="start monitor.py (dashboard + Gazebo view) unless it is running")
ap.add_argument("--port", type=int, default=5799, help="tick stream for monitor.py; 0 = off")
ap.add_argument("--layout", default="", help="<world>.layout.json from make_quad.py: draws the obstacles in ROOM")
ap.add_argument("--nav", choices=["gps", "flow"], default="gps", help="flow = 3901-L0X flow + ToF EKF, no GPS (gz_g4.sh NAV=)")
ap.add_argument(
    "--ext-height",
    action="store_true",
    help="companion supplies the FC height (VISION_POSITION_ESTIMATE z; needs sitl/variants/G_extnav_height.parm)",
)
ap.add_argument("--ext-drop-at", type=float, default=-1, help="fault: stop the companion height at t [s] (flight time)")
ap.add_argument("--ext-drop-for", type=float, default=0, help="fault: resume the companion height after this [s]; 0 = never")
ap.add_argument("--mute-at", type=float, default=-1, help="fault: companion hangs at t [s]: nothing is sent any more")
ap.add_argument(
    "--set",
    action="append",
    default=[],
    metavar="KEY.PATH=VALUE",
    help="config override after --config (ablations), e.g. --set vision.ecps_blank.gain=0; VALUE is parsed as YAML",
)
ap.add_argument("--shm", default="/dev/shm/ecps295_cam", help="camera shm written by cam_bridge.py (per instance)")
ap.add_argument("--home", default="33.6430,-117.8420,20", help="SITL --home (Gazebo origin) for SIM_STATE truth")
ap.add_argument("--out", default="g4")
a = ap.parse_args()

cfg = load_config(a.config or None, {"safety": dict(ECPS295_SAFETY_20FT), "brain": {"seed": a.seed}})
for item in a.set:  # ablation overrides, applied last
    import yaml

    path, _, value = item.partition("=")
    node = cfg
    *parents, leaf = path.split(".")
    for k in parents:
        node = node.setdefault(k, {})
    node[leaf] = yaml.safe_load(value)
_src = str(cfg["brain"]["source"])
if a.config and _src.endswith(".npz") and not Path(_src).is_absolute():  # brain file next to the variant YAML
    cfg["brain"]["source"] = str(Path(a.config).resolve().parent / _src)
if a.mode in ("approach", "patrol"):
    cfg["decoder"]["cruise"] = a.cruise
    cfg["safety"]["max_forward"] = a.max_forward
ARM = os.environ.get("ECPS_ARM", "intact")  # rl.arms control arm (as the server copy)
from rl import arms as _arms  # noqa: E402

brain = Brain(_arms.connectome_for(load_connectome(cfg["brain"]["source"]), ARM), cfg)
dyn = {k: ECPS295_DYN[k] for k in ("v_max", "vz_max", "yaw_rate_max_dps")}
dyn.update(cfg.get("ecps_drone", {}) or {})  # e.g. a faster yaw limit for saccades
drone = GazeboCameraMavlinkDrone(
    a.url,
    passive=(a.mode == "probe"),
    takeoff_alt=TAKEOFF_ALT_M,
    nav=a.nav,
    shm=a.shm,
    ext_height=a.ext_height,
    origin=tuple(float(v) for v in a.home.split(",")[:3]),
    **dyn,
)
if a.pilot == "ecps":
    from ecps_pilot import EcpsPilot

    pilot = EcpsPilot(brain, drone, cfg, fence_turn=a.fence_turn, **(cfg.get("ecps_pilot", {}) or {}))
else:
    pilot = Pilot(brain, drone, cfg)
if a.retina == "ecps":
    from ecps_retina import EcpsRetina

    pilot.retina = EcpsRetina.from_config(cfg)
if a.decoder == "ecps":
    from ecps_decoder import EcpsDecoder

    pilot.decoder = EcpsDecoder(cfg)
_arms.install(ARM, brain, pilot, drone, cfg, a.seed)
if getattr(drone, "drift_mps", 0.0) > 0:  # N4 drift stress: paired runs (same brain seed) drift the same way
    import random as _random

    drone.drift_dir = _random.Random(77_000 + a.seed).uniform(0.0, 2 * math.pi)
HZ = cfg["control"]["hz"]

if a.layout:
    drone.room = room_from_layout(a.layout)  # read by the dashboard ROOM panel

PIL = os.environ.get("ECPS_PIL")  # 2026-10-16 processor-in-the-loop: the brain runs on the Orange Pi (pil_rog.py)
pil = None
if PIL:
    from pil_rog import PilRog

    _sh = getattr(getattr(pilot, "struct", None), "shield", None)
    pil = PilRog(PIL, drone, a.shm, list(getattr(_sh, "map_sensors", None) or []), jpeg=os.environ.get("ECPS_PIL_RAW") != "1")

PROP_TIP_R = 0.13  # body centre to prop tip: 0.08 m arm diagonal + 0.0508 m prop radius


def clearance(north: float, east: float, alt: float) -> float:
    """Horizontal gap between the prop disc and the nearest obstacle that reaches the drone's height (NED room)."""
    room = getattr(drone, "room", None)
    best = float("inf")
    for b in room.boxes if room else []:
        if b.hi[2] < alt - 0.03:  # drone is above it
            continue
        dx = max(b.lo[0] - north, 0.0, north - b.hi[0])
        dy = max(b.lo[1] - east, 0.0, east - b.hi[1])
        best = min(best, (dx * dx + dy * dy) ** 0.5 - PROP_TIP_R)
    return best


TOF_TAN, TOF_DZ = math.tan(math.radians(13.5)), 0.015  # 27 deg cone, sensor below the body origin


def box_under(north: float, east: float, alt: float, min_step: float = 0.12) -> bool:
    """A box under the ToF cone whose top is >= min_step below the sensor (what corrupts the flow EKF height)."""
    room = getattr(drone, "room", None)
    h = alt - TOF_DZ
    for b in room.boxes if room else []:
        if (
            b.hi[2] < h - min_step
            and math.hypot(max(b.lo[0] - north, 0.0, north - b.hi[0]), max(b.lo[1] - east, 0.0, east - b.hi[1]))
            <= max(0.0, h) * TOF_TAN
        ):
            return True
    return False


def _publish(q, port: int, hello: dict) -> None:
    """Forked child: serves ticks to monitor.py clients; ticks are dropped while no monitor is connected."""
    import threading
    from multiprocessing.connection import Listener

    try:
        lis = Listener(("127.0.0.1", port), authkey=b"ecps295")
    except OSError as e:
        print(f"tick stream disabled ({e})", flush=True)
        while q.get() is not None:
            pass
        return
    clients, lock = [], threading.Lock()

    def accept():
        while True:
            try:
                c = lis.accept()
                c.send(("hello", hello))
            except OSError:
                return
            with lock:
                clients.append(c)

    threading.Thread(target=accept, daemon=True).start()
    while (item := q.get()) is not None:
        with lock:
            for c in list(clients):
                try:
                    c.send(item)
                except OSError:
                    clients.remove(c)
    with lock:
        for c in clients:
            c.close()
    lis.close()


class TickPublisher:
    def __init__(self, port: int, hello: dict):
        ctx = mp.get_context("fork")
        self.q = ctx.Queue()
        self.proc = ctx.Process(target=_publish, args=(self.q, port, hello), daemon=True)
        self.proc.start()

    def put(self, info) -> None:
        self.q.put(("tick", info))

    def close(self, summary: dict) -> None:
        self.q.put(("end", summary))
        self.q.put(None)
        self.proc.join(timeout=5)


# fork before any pymavlink / brain threads exist
pub = (
    TickPublisher(a.port, {"cfg": cfg, "title": f"MiniFly in Gazebo ({a.mode})", "layout": a.layout, "mode": a.mode})
    if a.port
    else None
)
if a.live and subprocess.run(["pgrep", "-f", "ecps295/monito[r].py"], capture_output=True).returncode != 0:
    here = Path(__file__).resolve().parent
    subprocess.Popen(
        [sys.executable, str(here / "monitor.py"), "--port", str(a.port)],
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=open("monitor.log", "w"),
    )
DT = 1.0 / HZ

rows: list[dict] = []
tick_dt_ms: list[float] = []
infos: list = []
pos = {"x": 0.0, "y": 0.0}  # EKF estimate (what the pilot and fence see)


rec = None
if a.record_frames:
    import cv2
    from shm_frames import ShmFrame

    Path(a.record_frames).mkdir(parents=True, exist_ok=True)
    rec = ShmFrame("/dev/shm/ecps295_cam_rgb")


def tick(t: float, dt: float, phase: str):
    if 0 <= a.ext_drop_at <= t:
        resumed = a.ext_drop_for > 0 and t >= a.ext_drop_at + a.ext_drop_for
        if drone.ext_paused == resumed:
            drone.ext_paused = not resumed
            print(f"fault injection: companion height {'stopped' if drone.ext_paused else 'resumed'} at t={t:.1f} s", flush=True)
    if 0 <= a.mute_at <= t and not drone.mute:
        drone.mute = True
        print(f"fault injection: companion hung at t={t:.1f} s", flush=True)
    if 0 <= a.freeze_cam_at <= t and not drone.frozen:
        drone.frozen = True
        print(f"fault injection: camera frozen at t={t:.1f} s", flush=True)
    c0 = time.perf_counter()
    info = pilot.tick(t, dt) if pil is None else pil.info(drone.telemetry())
    compute_ms = (time.perf_counter() - c0) * 1000
    if rec is not None:
        rec.read()
        if rec.img is not None:  # ELP streams MJPEG: store as JPEG so the replay also pays the decode
            cv2.imwrite(f"{a.record_frames}/{len(rows):05d}.jpg", rec.img[..., ::-1], [cv2.IMWRITE_JPEG_QUALITY, 85])
    tel = info.tel
    lp = drone.m.messages.get("LOCAL_POSITION_NED")
    if lp is not None:
        pos["x"], pos["y"] = lp.x, lp.y
    # metrics use simulator truth: under optical flow the EKF position drifts away from where the drone really is
    tru = drone.truth_ned()
    tn, te, ta = tru if tru is not None else (pos["x"], pos["y"], tel.alt_m or 0.0)
    rows.append(
        {
            "t": round(t, 3),
            "phase": phase,
            "alt": round(ta, 3),
            "north": round(tn, 3),
            "east": round(te, 3),
            "ekf_alt": tel.alt_m,
            "ekf_north": pos["x"],
            "ekf_east": pos["y"],
            "ekf_err_xy": round(math.hypot(pos["x"] - tn, pos["y"] - te), 3),
            "rng": drone.rangefinder_m(),
            "baro": drone.baro_m(),
            "ext_h": None if drone.ext_h_m is None else round(drone.ext_h_m, 3),
            "ext_src": drone.ext_src,
            "mute": int(drone.mute),
            "ventral": round(getattr(pilot.retina, "ventral", None).level, 2) if hasattr(pilot.retina, "ventral") else 0.0,
            "box_under": int(box_under(tn, te, ta)),
            "tof": round(max(getattr(pilot.retina, "last_tof", {"L": 0.0}).values()), 2),  # S5 retina input
            "h_est": None if getattr(pilot, "last_height_est", None) is None else round(pilot.last_height_est, 3),
            "fc_mode": getattr(drone.m.messages.get("HEARTBEAT"), "custom_mode", -1),
            "yaw_rate_dps": tel.yaw_rate_dps,
            "raw_throttle": info.raw.throttle,
            "raw_yaw": info.raw.yaw,
            "raw_forward": info.raw.forward,
            "cmd_throttle": info.cmd.throttle,
            "cmd_yaw": info.cmd.yaw,
            "cmd_forward": info.cmd.forward,
            "cmd_lateral": getattr(info.cmd, "lateral", 0.0),
            **_shield_dbg(pilot),
            "escape": int(info.cmd.escape),
            "wp_idx": getattr(pilot, "wp_idx", -1) if getattr(pilot, "wp_on", False) else -1,
            "frame_age_ms": round(drone.frame_age_s() * 1000, 1),
            "brain_rtf": info.rtf,
            "clearance": round(clearance(tn, te, ta), 3),
            "frozen": int(drone.frozen),
            "compute_ms": round(compute_ms, 2),  # whole Pilot.tick: frame, retina, brain, decoder, safety, send
            "brain_ms": round(dt * 1000 / info.rtf, 2) if info.rtf > 0 else float("nan"),
            **{k: round(v, 2) for k, v in info.rates.items() if k.startswith(("DN", "MDN"))},
            **(pil.row() if pil is not None else {}),
        }
    )
    if a.gif:
        infos.append(info)
    if pub:
        pub.put(info)
    return info


clock = {"t": 0.0}  # flight time across phases


def _shield_dbg(pilot) -> dict:
    """2026-10-15 debug columns: what the range shield saw this tick (nearest constraint reading / body bearing, number of
    constraints the requested command violated); empty without a shield."""
    sh = getattr(getattr(pilot, "struct", None), "shield", None)
    if sh is None:
        return {}
    d, b, n = getattr(sh, "dbg", (None, None, 0))
    return {"sh_near": d, "sh_brg": b, "sh_bind": n}


SIMCLOCK = os.environ.get("ECPS_SIMCLOCK") == "1"  # 2026-10-15: pace the brain by simulated time (see _run_phase_sim)
pace = {"sim_s": 0.0, "wall_s": 0.0}


def _sim_now() -> float | None:
    """Simulated time (s) from the FC's time_boot_ms (SITL runs in lockstep with Gazebo). Reading telemetry refreshes the
    cached MAVLink messages; the pilot reads the same cache, so nothing is lost."""
    drone.telemetry()
    ts = [getattr(drone.m.messages.get(k), "time_boot_ms", None) for k in ("ATTITUDE", "LOCAL_POSITION_NED")] if drone.m is not None else []
    ts = [x for x in ts if x is not None]
    return max(ts) / 1000.0 if ts else None


def _run_phase_sim(phase: str, secs: float, s0: float) -> bool:
    """Tick every DT of simulated time for secs of simulated time; dt passed to the brain is simulated time."""
    last, last_wall, w0 = s0, time.monotonic(), time.monotonic()
    # flydrones SafetyGovernor times max_flight_s on tel.t = wall clock: at RTF ~0.8 it landed the drone after ~90 s simulated
    pilot.safety.max_flight = max(pilot.safety.max_flight, 3 * secs + 60)
    while True:
        now = _sim_now()
        if time.monotonic() - w0 > 3 * secs + 60:
            print(f"simclock: wall-clock guard hit in {phase} (sim {0 if now is None else now - s0:.1f} of {secs} s)", flush=True)
            return True
        if now is None or now - last < DT - 1e-6:
            if now is not None and now - s0 >= secs:
                break
            time.sleep(0.002)
            continue
        if now - s0 >= secs:
            break
        dt = min(0.25, now - last)
        w = time.monotonic()
        tick_dt_ms.append((w - last_wall) * 1000)
        last, last_wall = now, w
        tick(clock["t"], dt, phase)
        clock["t"] += dt
        if pilot.safety.land_requested:
            pace["sim_s"] += now - s0; pace["wall_s"] += time.monotonic() - w0
            return False
        if _wp_finished():
            break
    pace["sim_s"] += last - s0; pace["wall_s"] += time.monotonic() - w0
    return True


def _wp_finished(hold_s: float = 3.0) -> bool:
    """2026-10-17 waypoint flight: end the run hold_s after the last point (the drone hovers meanwhile)."""
    done = getattr(pilot, "wp_done_t", None)
    return done is not None and clock["t"] - done > hold_s


def run_phase(phase: str, secs: float, vx=0.0, vz_up=0.0, yaw_dps=0.0):
    """Tick the brain at HZ for secs; in probe mode also drive the vehicle with a scripted body velocity."""
    if SIMCLOCK and a.mode != "probe":
        s0 = _sim_now()
        if s0 is not None:
            return _run_phase_sim(phase, secs, s0)
    t_end = time.monotonic() + secs
    last = time.monotonic()
    while time.monotonic() < t_end:
        now = time.monotonic()
        dt = min(0.25, max(1e-3, now - last))
        tick_dt_ms.append((now - last) * 1000)
        last = now
        if a.mode == "probe":
            su.send_vel(drone.m, vx, 0, -vz_up, math.radians(yaw_dps))
        tick(clock["t"], dt, phase)
        clock["t"] += dt
        if pilot.safety.land_requested:
            return False
        if _wp_finished():
            break
        sl = DT - (time.monotonic() - now)
        if sl > 0:
            time.sleep(sl)
    return True


drone.connect()
print("warming up the brain on the ground...", flush=True)
pilot.warmup(pilot.decoder.settle_s + 0.1, DT)
if pil is not None:
    pil.start()
drone.takeoff()
print(f"airborne at {drone.telemetry().alt_m:.2f} m, frames so far {drone.frames_new}", flush=True)
try:
    if a.mode == "probe":
        plan = [
            ("hover", 5),
            ("yaw_right", 4, 0, 0, 30),
            ("hover", 3),
            ("yaw_left", 4, 0, 0, -30),
            ("hover", 3),
            ("climb", 2.0, 0, 0.2),
            ("hover", 3),
            ("descend", 2.0, 0, -0.2),
            ("hover", 3),
            ("approach", 3.0, 0.3),
            ("hover", 2),
            ("back", 3.0, -0.3),
            ("hover", 2),
        ]
        for p in plan:
            print("phase", p[0], flush=True)
            if not run_phase(p[0], p[1], *p[2:]):
                break
    else:
        if pil is not None:
            pil.go = 1
        run_phase(a.mode, a.seconds)
finally:
    if pil is not None:
        pil.stop()  # go = 0: the Pi releases control before we land
    drone.mute = False  # the run is over: land normally
    drone.send(FlightCommand.hover("stop"))
    drone.passive = False
    drone.land()
    t0 = time.time()
    while time.time() - t0 < 20 and drone.m.motors_armed():
        drone.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)

with open(f"{a.out}.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)

alts = [r["alt"] for r in rows if r["alt"] is not None]


def flight_metrics() -> dict:
    """Obstacle clearances from the layout (prop tip to surface, m); negative = contact."""
    cl = [r["clearance"] for r in rows]
    if not cl or cl[0] == float("inf"):
        return {}
    cruise_cmd = min(cfg["decoder"]["cruise"], cfg["safety"]["max_forward"])

    def first(cond):
        r = next((r for r in rows if cond(r)), None)
        return None if r is None else round(r["clearance"], 3)

    return {
        "min_clearance_m": round(min(cl), 3),
        "contact": min(cl) < 0,
        "brake_onset_clearance_m": first(lambda r: r["clearance"] < 1.2 and r["cmd_forward"] < 0.5 * cruise_cmd)
        if cruise_cmd > 0
        else None,
        "escape_onset_clearance_m": first(lambda r: r["escape"]),
        "final_clearance_m": round(cl[-1], 3),
        **patrol_metrics(cl),
    }


def patrol_metrics(cl: list) -> dict:
    """Collision episodes, near misses, path and coverage (0.5 m cells inside the geofence)."""
    episodes = sum(1 for p, q in zip(cl, cl[1:]) if p >= 0 > q) + (1 if cl[0] < 0 else 0)
    near = sum(1 for p, q in zip(cl, cl[1:]) if p >= 0.1 > q)
    pts = [(r["north"], r["east"]) for r in rows]
    path = sum(math.hypot(b[0] - a_[0], b[1] - a_[1]) for a_, b in zip(pts, pts[1:]))
    fence, cell = cfg["safety"]["geofence_radius_m"], 0.5
    k = int(math.ceil(fence / cell))
    allowed = {(i, j) for i in range(-k, k) for j in range(-k, k) if math.hypot((i + 0.5) * cell, (j + 0.5) * cell) <= fence}
    visited = {(math.floor(n / cell), math.floor(e / cell)) for n, e in pts} & allowed
    return {
        "contact_episodes": episodes,
        "near_miss_episodes": near,
        "path_m": round(path, 1),
        "coverage": round(len(visited) / max(1, len(allowed)), 3),
        "fence_turn_s": round(getattr(pilot, "fence_turn_ticks", 0) / HZ, 1),
    }


summary = {
    "mode": a.mode,
    "world": (Path(a.layout).name.removesuffix(".layout.json") if a.layout else ""),
    "ticks": len(rows),
    "frames_new": drone.frames_new,
    "frames_repeated": drone.frames_repeated,
    "frame_age_ms_p95": float(np.percentile([r["frame_age_ms"] for r in rows], 95)),
    "brain_rtf_min": float(min(r["brain_rtf"] for r in rows if r["brain_rtf"] == r["brain_rtf"])),
    "alt_min": min(alts),
    "alt_max": max(alts),
    "north_max": max(r["north"] for r in rows),
    "escapes": sum(1 for p, q in zip(rows, rows[1:]) if q["escape"] and not p["escape"]),
    "nav": a.nav,
    # EKF vs truth (flow: integrated drift); radius_true_max vs geofence shows how far the drifting fence lets it go
    "ekf_err_xy_max_m": round(max(r["ekf_err_xy"] for r in rows), 3),
    "ekf_err_xy_final_m": rows[-1]["ekf_err_xy"],
    "ekf_err_alt_max_m": round(max(abs((r["ekf_alt"] or 0.0) - r["alt"]) for r in rows), 3),
    "radius_true_max_m": round(max(math.hypot(r["north"], r["east"]) for r in rows), 2),
    # S7d / v4: time over low obstacles (truth), whether the FC took over (LAND = 9, e.g. after the 1.3 m hard fence)
    "over_box_s": round(sum(r["box_under"] for r in rows) / HZ, 1),
    "alt_true_max_m": round(max(r["alt"] for r in rows), 2),
    "fc_land": any(r["fc_mode"] == 9 for r in rows),
    "height_guard_s": round(getattr(pilot, "hg_ticks", 0) / HZ, 1),
    "ext_height": a.ext_height,
    "overrides": a.set,
    "arm": ARM,
    # companion height vs truth (alt = body origin; the ToF sits 15 mm lower)
    "ext_h_err_max_m": max((abs(r["ext_h"] - (r["alt"] - TOF_DZ)) for r in rows if r["ext_h"] is not None), default=None),
    "ventral_events": getattr(getattr(pilot.retina, "ventral", None), "events", [])[:40],
    **flight_metrics(),
    "safety_events": pilot.safety.events,
    "variant": Path(a.config).stem if a.config else "v0",
    "decoder": a.decoder,
    "escape_log": getattr(pilot.decoder, "escapes", []),
    "watchdog_events": getattr(pilot, "watchdog_events", []),
    "pilot": a.pilot,
    "retina": a.retina,
    "tick_interval_ms_p50_p95_max": [round(float(np.percentile(tick_dt_ms[1:], q)), 1) for q in (50, 95, 100)],
    # compute budget per 50 ms tick (TechRoute §7.1: <= 25 ms on the Orange Pi)
    "compute_ms_p50_p95_max": [round(float(np.percentile([r["compute_ms"] for r in rows], q)), 2) for q in (50, 95, 100)],
    "brain_ms_p50_p95_max": [round(float(np.nanpercentile([r["brain_ms"] for r in rows], q)), 2) for q in (50, 95, 100)],
}
if pub:
    pub.close(summary)

if a.mode == "probe":
    # compare each motion phase with the hover just before it
    def mean(rs, k):
        return float(np.mean([r[k] for r in rs])) if rs else float("nan")

    keys = ["raw_throttle", "raw_yaw", "DNg02_L", "DNg02_R", "DNp03_L", "DNp03_R", "DNp01_L", "DNp01_R"]
    table = {}
    prev_hover: list = []
    i = 0
    while i < len(rows):
        ph = rows[i]["phase"]
        j = i
        while j < len(rows) and rows[j]["phase"] == ph:
            j += 1
        seg = rows[i:j]
        if ph == "hover":
            prev_hover = seg[len(seg) // 3 :]  # skip the transient after the previous motion
        else:
            table[ph] = {k: round(mean(seg[len(seg) // 4 :], k) - mean(prev_hover, k), 3) for k in keys}
            table[ph]["escapes"] = sum(1 for p, q in zip(seg, seg[1:]) if q["escape"] and not p["escape"])
        i = j
    checks = {
        "yaw_right -> yaw cmd < 0 (oppose)": table.get("yaw_right", {}).get("raw_yaw", 0) < 0,
        "yaw_left -> yaw cmd > 0 (oppose)": table.get("yaw_left", {}).get("raw_yaw", 0) > 0,
        "climb -> throttle cmd < 0 (oppose)": table.get("climb", {}).get("raw_throttle", 0) < 0,
        "descend -> throttle cmd > 0 (oppose)": table.get("descend", {}).get("raw_throttle", 0) > 0,
        "approach -> looming DNs up": (table.get("approach", {}).get("DNp03_L", 0) + table.get("approach", {}).get("DNp03_R", 0))
        > 1,
    }
    summary["probe_delta_vs_hover"] = table
    summary["probe_checks"] = checks
if getattr(pilot, "struct", None) is not None:  # 2026-10-13 ecps_struct counters (S2 vetoes, S3 pushes, S4 shield)
    summary.update(pilot.struct.summary())
summary["drift_mps"] = getattr(drone, "drift_mps", 0.0)  # N4 drift stress (ECPS_DRIFT_MPS), 0 = off
summary["drift_dir_deg"] = round(math.degrees(getattr(drone, "drift_dir", 0.0)), 1)
summary["shield_drift_ticks"] = getattr(getattr(getattr(pilot, "struct", None), "shield", None), "drift_ticks", 0)
if pil is not None:
    summary.update(pil.summary())
summary["simclock"] = SIMCLOCK  # 2026-10-15: brain paced by simulated time
summary["sim_s"], summary["wall_s"] = round(pace["sim_s"], 1), round(pace["wall_s"], 1)
summary["tof_cone3d"] = os.environ.get("ECPS_TOF_CONE3D") == "1"  # N1b: Gazebo ToF emulation = full cone (else planar fan)
summary["tof_retina"] = bool(getattr(getattr(pilot, "retina", None), "tof_on", False))  # N1c: S5 active
if getattr(pilot, "wp_on", False):  # 2026-10-17 waypoint flight, scored on truth (the pilot only sees the EKF)
    _pts, _r = pilot.wp_points, pilot.wp_radius
    _fly_alt = float(np.median([r["alt"] for r in rows]))
    _wp = []
    for i, (n, e) in enumerate(_pts):
        act = [r for r in rows if r["wp_idx"] == i]
        ev = next((x for x in pilot.wp_log if x[1] == i), None)
        at = None
        if ev is not None:  # truth distance at the tick the pilot declared the point
            r_ev = min(rows, key=lambda r: abs(r["t"] - ev[0]))
            at = round(math.hypot(r_ev["north"] - n, r_ev["east"] - e), 3)
        _wp.append(
            {
                "point": [n, e],
                "result": ev[2] if ev else "pending",
                "t": ev[0] if ev else None,
                "true_err_at_m": at,
                "true_min_dist_m": round(min(math.hypot(r["north"] - n, r["east"] - e) for r in act), 3) if act else None,
                "blocked": bool(clearance(n, e, _fly_alt) < 0.0),  # a box taller than the flight height covers the point
            }
        )
    summary["waypoints"] = _wp
    summary["wp_total"] = len(_pts)
    summary["wp_reached"] = sum(w["result"] == "reached" for w in _wp)
    summary["wp_skipped"] = sum(w["result"] == "skipped" for w in _wp)
    summary["wp_blocked"] = sum(w["blocked"] for w in _wp)
    summary["wp_done_t"] = pilot.wp_done_t
    summary["wp_true_err_med_m"] = (
        round(float(np.median([w["true_err_at_m"] for w in _wp if w["result"] == "reached"])), 3) if summary["wp_reached"] else None
    )
    summary["wp_bias_s"] = round(pilot.wp_ticks / HZ, 1)
json.dump(summary, open(f"{a.out}.json", "w"), indent=1)
print(json.dumps(summary, indent=1))

if a.gif and infos:
    from flydrones.viz import Dashboard, save_gif

    dash = Dashboard(brain, [pilot], title=f"MiniFly in Gazebo ({a.mode})")
    frames = []
    for k, info in enumerate(infos):
        if k % a.gif_every == 0:
            frames.append(dash.render([info]))
        else:
            dash.push([info])
    save_gif(frames, a.gif, fps=int(HZ / a.gif_every))
    print(f"saved {len(frames)} dashboard frames -> {a.gif}")
