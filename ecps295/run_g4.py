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
import queue
import time
import types

import numpy as np

import sitl_util as su
from envs import ECPS295_DYN, ECPS295_SAFETY_20FT, TAKEOFF_ALT_M
from flydrones.brain import Brain, load_connectome
from flydrones.config import load_config
from flydrones.drones.sim import Box, Room
from flydrones.motor.command import FlightCommand
from flydrones.runtime import Pilot
from gz_camera_drone import GazeboCameraMavlinkDrone

ap = argparse.ArgumentParser()
ap.add_argument("--mode", choices=["probe", "hover", "approach"], required=True)
ap.add_argument("--url", default="tcp:127.0.0.1:5760")
ap.add_argument("--seconds", type=float, default=40)
ap.add_argument("--cruise", type=float, default=0.5, help="approach: decoder.cruise (x safety max_forward x v_max)")
ap.add_argument("--max-forward", type=float, default=0.6, help="approach: safety.max_forward")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--gif", default="", help="render the brain dashboard to this GIF after the flight")
ap.add_argument("--gif-every", type=int, default=3)
ap.add_argument("--live", action="store_true", help="show the brain dashboard in a window while flying")
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


def room_from_layout(path: str) -> Room:
    """Gazebo ENU obstacles -> flydrones Room in the telemetry frame (x = north, y = east, as LOCAL_POSITION_NED)."""
    lay = json.load(open(path))
    boxes = []
    for b in lay["boxes"]:
        (e, n, z), (se, sn, sz) = b["center_enu"], b["size_enu"]
        boxes.append(Box((n - sn / 2, e - se / 2, z - sz / 2), (n + sn / 2, e + se / 2, z + sz / 2), 0.2, b["name"]))
    return Room(size_x=lay["floor_m"], size_y=lay["floor_m"], height=3.0, boxes=boxes)


if a.layout:
    drone.room = room_from_layout(a.layout)   # read by the dashboard ROOM panel


def _viewer(q, cfg_v: dict, title: str, layout: str) -> None:
    """Child process: owns the dashboard and the OpenCV window, so the control loop never draws or holds the GIL."""
    from flydrones.viz import Dashboard, LiveWindow

    brain_v = Brain(load_connectome(cfg_v["brain"]["source"]), cfg_v)   # same seed -> same recorded neurons/layout
    stub = types.SimpleNamespace(name="fly-1", drone=types.SimpleNamespace(room=room_from_layout(layout) if layout else None))
    dash = Dashboard(brain_v, [stub], title=title)
    win = LiveWindow("MiniFly brain (Gazebo)")
    done = False
    while not done:
        items = [q.get()]
        while True:
            try:
                items.append(q.get_nowait())
            except queue.Empty:
                break
        if None in items:
            done = True
            items = [i for i in items if i is not None]
        if not items:
            break
        for it in items[:-1]:
            dash.push([it])
        if not win.show(dash.render([items[-1]])):
            break
    if win.cv2 is not None:
        win.cv2.waitKey(1500)
    win.close()


class LiveDashboard:
    def __init__(self):
        self.q = mp.get_context("fork").Queue()
        self.proc = mp.get_context("fork").Process(
            target=_viewer, args=(self.q, cfg, f"MiniFly in Gazebo ({a.mode})", a.layout), daemon=True)
        self.proc.start()
        self.rendered = -1   # counted in the child

    def put(self, info) -> None:
        self.q.put(info)

    def stop(self) -> None:
        self.q.put(None)
        self.proc.join(timeout=10)


live = LiveDashboard() if a.live else None   # fork before any pymavlink / brain threads exist
DT = 1.0 / HZ

rows: list[dict] = []
tick_dt_ms: list[float] = []
infos: list = []
pos = {"x": 0.0, "y": 0.0}


def tick(t: float, dt: float, phase: str):
    info = pilot.tick(t, dt)
    tel = info.tel
    lp = drone.m.messages.get("LOCAL_POSITION_NED")
    if lp is not None:
        pos["x"], pos["y"] = lp.x, lp.y
    rows.append({"t": round(t, 3), "phase": phase, "alt": tel.alt_m, "north": pos["x"], "east": pos["y"],
                 "yaw_rate_dps": tel.yaw_rate_dps,
                 "raw_throttle": info.raw.throttle, "raw_yaw": info.raw.yaw, "raw_forward": info.raw.forward,
                 "cmd_throttle": info.cmd.throttle, "cmd_yaw": info.cmd.yaw, "cmd_forward": info.cmd.forward,
                 "escape": int(info.cmd.escape), "frame_age_ms": round(drone.frame_age_s() * 1000, 1),
                 "brain_rtf": info.rtf, **{k: round(v, 2) for k, v in info.rates.items() if k.startswith("DN")}})
    if a.gif:
        infos.append(info)
    if live:
        live.put(info)
    return info


def run_phase(phase: str, secs: float, vx=0.0, vz_up=0.0, yaw_dps=0.0, t_base=[0.0]):
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
        tick(t_base[0], dt, phase)
        t_base[0] += dt
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
        plan = [("hover", 5), ("yaw_right", 4, 0, 0, 30), ("hover", 3), ("yaw_left", 4, 0, 0, -30), ("hover", 3),
                ("climb", 2.0, 0, 0.2), ("hover", 3), ("descend", 2.0, 0, -0.2), ("hover", 3),
                ("approach", 3.0, 0.3), ("hover", 2), ("back", 3.0, -0.3), ("hover", 2)]
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
summary = {"mode": a.mode, "ticks": len(rows), "frames_new": drone.frames_new, "frames_repeated": drone.frames_repeated,
           "frame_age_ms_p95": float(np.percentile([r["frame_age_ms"] for r in rows], 95)),
           "brain_rtf_min": float(min(r["brain_rtf"] for r in rows if r["brain_rtf"] == r["brain_rtf"])),
           "alt_min": min(alts), "alt_max": max(alts), "north_max": max(r["north"] for r in rows),
           "escapes": sum(1 for p, q in zip(rows, rows[1:]) if q["escape"] and not p["escape"]),
           "safety_events": pilot.safety.events,
           "tick_interval_ms_p50_p95_max": [round(float(np.percentile(tick_dt_ms[1:], q)), 1) for q in (50, 95, 100)],
           "live_frames_rendered": live.rendered if live else 0}
if live:
    live.stop()

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
            prev_hover = seg[len(seg) // 3:]   # skip the transient after the previous motion
        else:
            table[ph] = {k: round(mean(seg[len(seg) // 4:], k) - mean(prev_hover, k), 3) for k in keys}
            table[ph]["escapes"] = sum(1 for p, q in zip(seg, seg[1:]) if q["escape"] and not p["escape"])
        i = j
    checks = {
        "yaw_right -> yaw cmd < 0 (oppose)": table.get("yaw_right", {}).get("raw_yaw", 0) < 0,
        "yaw_left -> yaw cmd > 0 (oppose)": table.get("yaw_left", {}).get("raw_yaw", 0) > 0,
        "climb -> throttle cmd < 0 (oppose)": table.get("climb", {}).get("raw_throttle", 0) < 0,
        "descend -> throttle cmd > 0 (oppose)": table.get("descend", {}).get("raw_throttle", 0) > 0,
        "approach -> looming DNs up": (table.get("approach", {}).get("DNp03_L", 0) + table.get("approach", {}).get("DNp03_R", 0)) > 1,
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
