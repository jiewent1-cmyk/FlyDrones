"""S5: FlyDrones (MiniFly + scripted gestures) flying ArduPilot SITL in GUIDED, plus the matching L1 run.

modes
  l2-upstream  flydrones.drones.mavlink.MavlinkDrone as-is (reproduces F1 when MAV1_* stream rates are 0)
  l2-fixed     ecps295.mavlink_twin.Ecps295MavlinkDrone (F1/F3/F4/F7/F9)
  l1           ecps295.envs.TwinSimDrone, started airborne at the takeoff height so t=0 matches l2-fixed
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import time

from envs import ECPS295_DYN, ECPS295_SAFETY_20FT, TAKEOFF_ALT_M, TwinSimDrone

from flydrones.brain import Brain, load_connectome
from flydrones.config import load_config
from flydrones.runtime import Pilot, run_realtime, run_sim
from flydrones.senses.gestures import ScriptedGestures, demo_timeline

ap = argparse.ArgumentParser()
ap.add_argument("--mode", choices=["l2-upstream", "l2-fixed", "l1"], required=True)
ap.add_argument("--url", default="tcp:127.0.0.1:5760")
ap.add_argument("--seconds", type=float, default=30)
ap.add_argument(
    "--pre-wait",
    type=float,
    default=0,
    help="l2-upstream: seconds to wait after connect, before takeoff (upstream has no EKF wait, F4)",
)
ap.add_argument("--out", default="s5")
ap.add_argument("--seed", type=int, default=0)
a = ap.parse_args()

cfg = load_config(None, {"safety": dict(ECPS295_SAFETY_20FT), "brain": {"seed": a.seed}})
brain = Brain(load_connectome(cfg["brain"]["source"]), cfg)
dyn = {k: ECPS295_DYN[k] for k in ("v_max", "vz_max", "yaw_rate_max_dps")}
gestures = ScriptedGestures(demo_timeline())

if a.mode == "l1":
    drone = TwinSimDrone(start=(0.0, 0.0, TAKEOFF_ALT_M), seed=a.seed)
    drone.has_camera = False  # gesture-only, same as the SITL runs (MavlinkDrone has no camera)
    c = copy.deepcopy(cfg)
    c["control"]["takeoff"] = False
    pilot = Pilot(brain, drone, c, gestures=gestures)
    drone.start_hovering()
    run_sim([pilot], a.seconds, hz=cfg["control"]["hz"])
else:
    if a.mode == "l2-upstream":
        from flydrones.drones.mavlink import MavlinkDrone

        drone = MavlinkDrone(a.url, takeoff_alt=TAKEOFF_ALT_M, **dyn)
        _connect = drone.connect

        def connect_then_wait():
            _connect()
            print(f"pre-wait {a.pre_wait:.0f}s for EKF (upstream does not check it)", flush=True)
            time.sleep(a.pre_wait)

        drone.connect = connect_then_wait
    else:
        from mavlink_twin import Ecps295MavlinkDrone

        drone = Ecps295MavlinkDrone(a.url, takeoff_alt=TAKEOFF_ALT_M, **dyn)
    pilot = Pilot(brain, drone, cfg, gestures=gestures)
    try:
        run_realtime(pilot, a.seconds, hz=cfg["control"]["hz"])
    finally:
        try:
            drone.land()
            t0 = time.time()
            while time.time() - t0 < 20 and drone.m.motors_armed():
                drone.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        except Exception as e:  # noqa: BLE001
            print("land:", e)

h = pilot.history
with open(f"{a.out}.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(h[0].keys()))
    w.writeheader()
    w.writerows(h)
alts = [r["alt"] for r in h if r["alt"] is not None]
summary = {
    "mode": a.mode,
    "ticks": len(h),
    "ticks_without_alt": sum(r["alt"] is None for r in h),
    "alt_min": min(alts) if alts else None,
    "alt_max": max(alts) if alts else None,
    "escapes": sum(1 for p, q in zip(h, h[1:]) if q["escape"] and not p["escape"]),
    "safety_events": pilot.safety.events,
    "land_requested": pilot.safety.land_requested,
}
json.dump(summary, open(f"{a.out}.json", "w"), indent=1)
print(json.dumps(summary, indent=1))
