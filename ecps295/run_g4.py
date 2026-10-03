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
ap.add_argument("--mode", choices=["probe", "hover", "approach"], required=True)
ap.add_argument("--url", default="tcp:127.0.0.1:5760")
ap.add_argument("--seconds", type=float, default=40)
ap.add_argument("--cruise", type=float, default=0.5, help="approach: decoder.cruise (x safety max_forward x v_max)")
ap.add_argument("--max-forward", type=float, default=0.6, help="approach: safety.max_forward")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--gif", default="", help="render the brain dashboard to this GIF after the flight")
ap.add_argument("--gif-every", type=int, default=3)
ap.add_argument("--record-frames", default="", help="save the colour drone camera as JPEG each tick (pil_replay.py input)")
ap.add_argument("--live", action="store_true", help="start monitor.py (dashboard + Gazebo view) unless it is running")
ap.add_argument("--port", type=int, default=5799, help="tick stream for monitor.py; 0 = off")
ap.add_argument("--layout", default="", help="<world>.layout.json from make_quad.py: draws the obstacles in ROOM")
ap.add_argument("--out", default="g4")
a = ap.parse_args()

cfg = load_config(None, {"safety": dict(ECPS295_SAFETY_20FT), "brain": {"seed": a.seed}})
if a.mode == "approach":
    cfg["decoder"]["cruise"] = a.cruise
    cfg["safety"]["max_forward"] = a.max_forward
brain = Brain(load_connectome(cfg["brain"]["source"]), cfg)
dyn = {k: ECPS295_DYN[k] for k in ("v_max", "vz_max", "yaw_rate_max_dps")}
drone = GazeboCameraMavlinkDrone(a.url, passive=(a.mode == "probe"), takeoff_alt=TAKEOFF_ALT_M, **dyn)
pilot = Pilot(brain, drone, cfg)
HZ = cfg["control"]["hz"]

if a.layout:
    drone.room = room_from_layout(a.layout)  # read by the dashboard ROOM panel


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
pos = {"x": 0.0, "y": 0.0}


rec = None
if a.record_frames:
    import cv2
    from shm_frames import ShmFrame

    Path(a.record_frames).mkdir(parents=True, exist_ok=True)
    rec = ShmFrame("/dev/shm/ecps295_cam_rgb")


def tick(t: float, dt: float, phase: str):
    c0 = time.perf_counter()
    info = pilot.tick(t, dt)
    compute_ms = (time.perf_counter() - c0) * 1000
    if rec is not None:
        rec.read()
        if rec.img is not None:  # ELP streams MJPEG: store as JPEG so the replay also pays the decode
            cv2.imwrite(f"{a.record_frames}/{len(rows):05d}.jpg", rec.img[..., ::-1], [cv2.IMWRITE_JPEG_QUALITY, 85])
    tel = info.tel
    lp = drone.m.messages.get("LOCAL_POSITION_NED")
    if lp is not None:
        pos["x"], pos["y"] = lp.x, lp.y
    rows.append(
        {
            "t": round(t, 3),
            "phase": phase,
            "alt": tel.alt_m,
            "north": pos["x"],
            "east": pos["y"],
            "yaw_rate_dps": tel.yaw_rate_dps,
            "raw_throttle": info.raw.throttle,
            "raw_yaw": info.raw.yaw,
            "raw_forward": info.raw.forward,
            "cmd_throttle": info.cmd.throttle,
            "cmd_yaw": info.cmd.yaw,
            "cmd_forward": info.cmd.forward,
            "escape": int(info.cmd.escape),
            "frame_age_ms": round(drone.frame_age_s() * 1000, 1),
            "brain_rtf": info.rtf,
            "compute_ms": round(compute_ms, 2),  # whole Pilot.tick: frame, retina, brain, decoder, safety, send
            "brain_ms": round(dt * 1000 / info.rtf, 2) if info.rtf > 0 else float("nan"),
            **{k: round(v, 2) for k, v in info.rates.items() if k.startswith("DN")},
        }
    )
    if a.gif:
        infos.append(info)
    if pub:
        pub.put(info)
    return info


clock = {"t": 0.0}  # flight time across phases


def run_phase(phase: str, secs: float, vx=0.0, vz_up=0.0, yaw_dps=0.0):
    """Tick the brain at HZ for secs; in probe mode also drive the vehicle with a scripted body velocity."""
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
        sl = DT - (time.monotonic() - now)
        if sl > 0:
            time.sleep(sl)
    return True


drone.connect()
print("warming up the brain on the ground...", flush=True)
pilot.warmup(pilot.decoder.settle_s + 0.1, DT)
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
        run_phase(a.mode, a.seconds)
finally:
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
summary = {
    "mode": a.mode,
    "ticks": len(rows),
    "frames_new": drone.frames_new,
    "frames_repeated": drone.frames_repeated,
    "frame_age_ms_p95": float(np.percentile([r["frame_age_ms"] for r in rows], 95)),
    "brain_rtf_min": float(min(r["brain_rtf"] for r in rows if r["brain_rtf"] == r["brain_rtf"])),
    "alt_min": min(alts),
    "alt_max": max(alts),
    "north_max": max(r["north"] for r in rows),
    "escapes": sum(1 for p, q in zip(rows, rows[1:]) if q["escape"] and not p["escape"]),
    "safety_events": pilot.safety.events,
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
