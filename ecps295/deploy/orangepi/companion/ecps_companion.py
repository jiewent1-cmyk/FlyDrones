#!/usr/bin/env python3
"""ECPS295 companion runtime for the Orange Pi Zero 3W: the run_g4.py control loop with the real drone backend.

    camera thread --+
    ToF threads x2 -+--> 20 Hz loop: EcpsRetina -> MiniFly v4 -> EcpsDecoder -> SafetyGovernor -> EcpsPilot watchdogs
    MAVLink (router)+                -> RangeShield + LocalMap (w2map) -> MAVLink (velocity, only while in control)
                                     + route-B height (VISION_POSITION_ESTIMATE z) + heartbeat (sysid 254)

Modes
  bench   no flight controller (BenchDrone): sensors + brain + safety layer only; for the bench and for profiling
  shadow  flight controller connected; heartbeat + route-B height are sent, velocity commands are NOT (logged only)
  guided  as shadow, and velocity commands / watchdog LAND are sent while the FC is in GUIDED and armed
          (the pilot takes off and hands over with the mode switch; the companion never arms or takes off)

Every run writes <log-dir>/<stamp>_<mode>/: ticks.csv (one row per tick, with per-stage timing), system.csv (1 Hz:
temperatures, CPU clocks, load, RSS), run.json (config, summary, timing percentiles, watchdog events).

Ground console (2026-10-17, optional): --console PORT (default: hw config console.port, 0 = off) streams the brain,
camera, local 3D map and flight state to the Mac console (deploy/console) and takes guidance and the emergency stop
back; see console_link.py. Without a console connected the control path is unchanged.

Target tracking (2026-10-18, optional): hw config `marker.enabled` starts an ArUco detector thread on the A55 cores
(ecps295/ecps_target.py) on the full camera frame; the console's TRACK mode (or --track, tests) starts the pilot's
target layer: search, fly to standoff_m in front of the marker, hover facing it.
"""

from __future__ import annotations

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import argparse  # noqa: E402
import csv  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import signal  # noqa: E402
import sys  # noqa: E402
import time  # noqa: E402
from pathlib import Path  # noqa: E402

ROOT = Path(os.environ.get("ECPS_ROOT", "/opt/ecps295"))
sys.path[:0] = [str(Path(__file__).resolve().parent), str(ROOT / "ecps295"), str(ROOT / "src")]

import numpy as np  # noqa: E402
import yaml  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--mode", choices=["bench", "shadow", "guided"], default="bench")
ap.add_argument("--hw", default=str(ROOT / "config" / "companion.yaml"), help="hardware config (camera, ToF, MAVLink)")
ap.add_argument("--config", default="", help="MiniFly variant YAML (default: hw config 'model')")
ap.add_argument("--url", default="", help="MAVLink URL (default: hw config mavlink.url)")
ap.add_argument("--camera", default="", help="uvc | sim | replay:<dir> | off | sitl_marker:<n,e,up,yaw[,id]> (SITL tests; default: hw config)")
ap.add_argument("--tof", default="", help="real | sim | off | sitl:<layout.json> (SITL tests; default: hw config)")
ap.add_argument("--seconds", type=float, default=0, help="stop after this long; 0 = until SIGINT / SIGTERM")
ap.add_argument("--cruise", type=float, default=0.5)
ap.add_argument("--max-forward", type=float, default=0.5)
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--no-fence-turn", action="store_true")
ap.add_argument("--affinity", default="", help="CPU list for the process, e.g. 6,7 (A76) or 0-5 (A55)")
ap.add_argument("--log-dir", default=str(ROOT / "logs"))
ap.add_argument("--tag", default="")
ap.add_argument("--quiet", action="store_true")
ap.add_argument("--pil", type=int, default=0, metavar="PORT",
                help="processor-in-the-loop: camera + ToF from Gazebo over UDP on this port (pil_link.py); control only while"
                     " the simulation PC sets go")
ap.add_argument("--set", action="append", default=[], metavar="KEY.PATH=VALUE", help="config override (YAML value)")
ap.add_argument("--track", action="store_true", help="tests: start target tracking at every hand-over (needs hw marker.enabled)")
ap.add_argument("--console", type=int, default=-1, metavar="PORT",
                help="ground console link on this UDP port (console_link.py); 0 = off (default: hw config console.port)")
a = ap.parse_args()


def cpus(spec: str) -> set[int]:
    out: set[int] = set()
    for part in spec.split(","):
        lo, _, hi = part.partition("-")
        out.update(range(int(lo), int(hi or lo) + 1))
    return out


if a.affinity:
    os.sched_setaffinity(0, cpus(a.affinity))  # before any thread starts: the threads inherit it

hw = yaml.safe_load(open(a.hw)) if Path(a.hw).exists() else {}
model = a.config or str(ROOT / "ecps295" / "minifly" / hw.get("model", "s7_w2map.yaml"))
cam_src = a.camera or hw.get("camera", {}).get("source", "uvc")
tof_src = a.tof or hw.get("tof", {}).get("source", "real")
url = a.url or hw.get("mavlink", {}).get("url", "udpin:127.0.0.1:14560")

import camera as camera_mod  # noqa: E402
import tof_vl53l7cx as tof_mod  # noqa: E402
from ecps_decoder import EcpsDecoder  # noqa: E402
from ecps_pilot import EcpsPilot  # noqa: E402
from ecps_retina import EcpsRetina  # noqa: E402
from envs import ECPS295_DYN, ECPS295_SAFETY_20FT  # noqa: E402
from real_drone import BenchDrone, RealDrone  # noqa: E402

from flydrones.brain import Brain, load_connectome  # noqa: E402
from flydrones.config import load_config  # noqa: E402
from flydrones.motor.command import FlightCommand  # noqa: E402

# ------------------------------------------------------------------ model (as run_g4.py --mode patrol --pilot/retina/decoder ecps)
cfg = load_config(model, {"safety": dict(ECPS295_SAFETY_20FT), "brain": {"seed": a.seed}})
for item in a.set:
    path, _, value = item.partition("=")
    node = cfg
    *parents, leaf = path.split(".")
    for k in parents:
        node = node.setdefault(k, {})
    node[leaf] = yaml.safe_load(value)
src = str(cfg["brain"]["source"])
if src.endswith(".npz") and not Path(src).is_absolute():
    cfg["brain"]["source"] = str(Path(model).resolve().parent / src)
cfg["decoder"]["cruise"] = a.cruise
cfg["safety"]["max_forward"] = a.max_forward
brain = Brain(load_connectome(cfg["brain"]["source"]), cfg)
dyn = {k: ECPS295_DYN[k] for k in ("v_max", "vz_max", "yaw_rate_max_dps")}
dyn.update(cfg.get("ecps_drone", {}) or {})

stop = {"flag": False}  # set by SIGTERM / SIGINT; checked while waiting for the FC as well as by the control loop
signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))
signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
# test hook (sitl_hil_test.py SCENARIO=tof_*): SIGUSR1 freezes the front ToF thread, as a hung I2C bus would: the last
# frame stays, its age grows
signal.signal(signal.SIGUSR1, lambda *_: [getattr(x, "_stop", None) and x._stop.set() for x in [tofs.get(0)] if x is not None])

# ------------------------------------------------------------------ sensors
link = None
if a.pil:  # Gazebo on the simulation PC instead of the camera and the ToF sensors
    from pil_link import PilLink

    link = PilLink(a.pil).start()
    cam_src = tof_src = f"pil:{a.pil}"
mk_cfg = dict(hw.get("marker") or {})
if link:
    cam = link.cam
elif cam_src.startswith("sitl_marker:"):  # SITL tests: the camera rendered from SIM_STATE + one marker (sitl_marker.py)
    import sitl_marker

    cam = sitl_marker.make(cam_src, hw.get("camera", {}), mk_cfg)
else:
    cam = camera_mod.make(cam_src, hw.get("camera", {}))
tofs = {}
for name, tc in (hw.get("tof", {}).get("sensors", {}) or {}).items():
    tc = dict(tc)
    bearing = int(tc.pop("bearing", 0)) % 360
    if link:
        s = link.tof(bearing, name, **tc)
    elif tof_src.startswith("sitl:"):  # SITL tests: ToF emulated from SIM_STATE + a world layout (sitl_world.py)
        import sitl_world

        s = sitl_world.make(tc, name, tof_src, bearing)
    else:
        s = tof_mod.make(tc, name, tof_src)
    if s is not None:
        tofs[bearing] = s
for s in [cam, *tofs.values()]:
    if s is not None:
        s.start()

if a.mode == "bench":
    drone = BenchDrone(cam, tofs, **dyn)
else:
    mv = hw.get("mavlink", {})
    # flight time limit: RealDrone hovers and hands back to the pilot instead of the governor's LAND in place
    max_flight_s = float(cfg["safety"].get("max_flight_s", 120))
    cfg["safety"]["max_flight_s"] = 1e9
    drone = RealDrone(url, cam, tofs, allow_cmd=(a.mode == "guided"), ext_height=bool(mv.get("ext_height", True)),
                      fc_sysid=int(mv.get("fc_sysid", 1)), stream_hz=float(mv.get("stream_hz", 20)),
                      handover_alt_m=float(mv.get("handover_alt_m", 0.3)),
                      rangefinder_hz=float(mv.get("rangefinder_hz", 20)), guard=hw.get("guard", {}),
                      max_flight_s=max_flight_s, stop_check=lambda: stop["flag"],
                      ekf_origin=tuple(mv.get("ekf_origin", (33.6430, -117.8420, 20.0))), **dyn)
for s in [*tofs.values(), cam]:  # sitl_world / sitl_marker sources read SIM_STATE through the drone's MAVLink connection
    if s is not None and hasattr(s, "bind"):
        s.bind(drone)
if link is not None and a.mode != "bench":  # PIL: take control only during the simulation PC's patrol phase
    _gate = drone.in_control
    drone.in_control = lambda: _gate() and link.go
pilot = EcpsPilot(brain, drone, cfg, fence_turn=not a.no_fence_turn, **(cfg.get("ecps_pilot", {}) or {}))
pilot.retina = EcpsRetina.from_config(cfg)
pilot.decoder = EcpsDecoder(cfg)
pilot.wp_active = False  # waypoint mission (if configured) starts when we take control
marker = None
if mk_cfg.get("enabled") and cam is not None and hasattr(pilot, "target"):
    from ecps_target import MarkerDetector, MarkerThread

    # full-resolution frame (camera.py keeps it as last_bgr); the detector thread moves itself to the A55 cores
    marker = MarkerThread(lambda: (cam.last_bgr, cam.stamp), MarkerDetector(**mk_cfg), hz=float(mk_cfg.get("hz", 10)),
                          cpus=str(mk_cfg.get("cpus", "0-5"))).start()
    pilot.target.source = marker
HZ = float(cfg["control"]["hz"])
DT = 1.0 / HZ
# Brain dt bound (2026-10-16): the brain simulates the wall time since the last tick, so one overrun makes the next tick
# dearer and the loop can spiral (A55: 265 ms ticks, report 4.3). Simulate at most 2 ticks' worth; the 116 Gazebo
# acceptance runs of s7_w2map never had a tick gap above 55.5 ms, so this changes nothing they exercised.
DT_MAX = 2 * DT

# ------------------------------------------------------------------ per-stage timers (wrap bound methods on the instances)
stage_ms: dict[str, float] = {}


def timed(obj, attr: str, name: str) -> None:
    fn = getattr(obj, attr)

    def w(*args, **kw):
        c0 = time.perf_counter()
        try:
            return fn(*args, **kw)
        finally:
            stage_ms[name] = stage_ms.get(name, 0.0) + (time.perf_counter() - c0) * 1000

    setattr(obj, attr, w)


timed(drone, "frame", "frame")
timed(pilot.retina, "encode", "retina")
timed(drone, "telemetry", "telemetry")
timed(pilot.encoder, "encode", "encoder")
timed(pilot.brain, "tick", "brain")
timed(pilot.decoder, "update", "decoder")
timed(pilot.safety, "filter", "governor")
shield = getattr(getattr(pilot, "struct", None), "shield", None)
if pilot.struct is not None:
    timed(pilot.struct, "filter", "struct")  # includes map + shield
if shield is not None and getattr(shield, "map", None) is not None:
    timed(shield, "_map_update", "map")
timed(drone, "send", "send")
timed(drone, "land", "land")
STAGES = ["frame", "retina", "telemetry", "encoder", "brain", "decoder", "governor", "struct", "map", "send"]

# ------------------------------------------------------------------ ground console (console_link.py; after the timers: it wraps them)
console = None
console_port = a.console if a.console >= 0 else int((hw.get("console") or {}).get("port", 0) or 0)
if console_port:
    import console_link

    console = console_link.make(console_port, pilot=pilot, drone=drone, cam=cam, tofs=tofs, cfg=cfg,
                                console_cfg=hw.get("console") or {},
                                run_info={"mode": a.mode, "model": Path(model).name, "seed": a.seed, "hz": HZ,
                                          "camera": cam_src, "tof": tof_src, "tag": a.tag})

# ------------------------------------------------------------------ logging
stamp = time.strftime("%Y%m%d_%H%M%S")
out = Path(a.log_dir) / f"{stamp}_{a.mode}{'_' + a.tag if a.tag else ''}"
out.mkdir(parents=True, exist_ok=True)
fcsv = open(out / "ticks.csv", "w", newline="")
fsys = open(out / "system.csv", "w", newline="")
wsys = None
TZ = sorted(Path("/sys/class/thermal").glob("thermal_zone*"))


def sysinfo() -> dict:
    d = {"t": round(time.monotonic() - T0, 2)}
    for z in TZ:
        try:
            d[f"T_{(z / 'type').read_text().strip().replace('_thermal_zone', '').replace('_zone', '')}"] = (
                int((z / "temp").read_text()) / 1000
            )
        except OSError:
            pass
    for c in (0, 6):
        try:
            d[f"MHz_cpu{c}"] = int(Path(f"/sys/devices/system/cpu/cpu{c}/cpufreq/scaling_cur_freq").read_text()) // 1000
        except OSError:
            pass
    d["load1"] = os.getloadavg()[0]
    try:
        d["rss_mb"] = round(int(open("/proc/self/statm").read().split()[1]) * os.sysconf("SC_PAGE_SIZE") / 2**20, 1)
    except OSError:
        pass
    return d


# ------------------------------------------------------------------ run
print(f"ecps_companion: mode={a.mode} model={Path(model).name} camera={cam_src} tof={tof_src} url={url if a.mode != 'bench' else '-'}"
      f" affinity={sorted(os.sched_getaffinity(0))} log={out}", flush=True)
drone.connect()
T0 = time.monotonic()
print("warming up the brain...", flush=True)
pilot.warmup(pilot.decoder.settle_s + 0.1, DT)
w = None
rows_n = 0
compute: list[float] = []
period: list[float] = []
stage_hist: dict[str, list[float]] = {k: [] for k in STAGES}
control_s = 0.0
was_in = False
wp_seen: list = [0]  # watchdog events already scanned for waypoint / target progress
was_track = False
events: list = []
t_flight = 0.0
nxt = time.monotonic()
last = nxt
last_sys = 0.0
deadline_miss = 0
dt_clamped = 0
try:
    while not stop["flag"] and (a.seconds <= 0 or time.monotonic() - T0 < a.seconds) and not (link is not None and link.finished):
        nxt += DT
        now = time.monotonic()
        dt = min(DT_MAX, max(1e-3, now - last))
        dt_clamped += now - last > DT_MAX
        period.append((now - last) * 1000)
        last = now
        stage_ms.clear()
        c0 = time.perf_counter()
        info = pilot.tick(t_flight, dt)
        cms = (time.perf_counter() - c0) * 1000
        t_flight += dt
        inc = drone.in_control()
        if inc != was_in:
            events.append((round(now - T0, 2), "companion in control" if inc else "control released"))
            print(f"[{now - T0:7.1f}] {'IN CONTROL (GUIDED, armed)' if inc else 'control released'}", flush=True)
            was_in = inc
        if a.track and marker is not None and inc and not was_track:
            pilot.track_start()  # tests: track from every hand-over (the console's TRACK does this otherwise)
        was_track = inc
        if not inc:  # not flying under our control: watchdog land / flight timer start only when we take over
            pilot.safety.land_requested = False
            pilot.safety._start = None
        else:
            control_s += dt
        pilot.wp_active = inc  # waypoints only advance under our control; the pilot's flying pauses the mission
        if getattr(pilot, "wp_on", False) or (marker is not None and pilot.target.active):
            for ev in pilot.watchdog_events[wp_seen[0]:]:
                if ev[1].startswith(("waypoint", "target")):  # tell the pilot (Mission Planner / OSD)
                    print(f"[{now - T0:7.1f}] {ev[1]}", flush=True)
                    if hasattr(drone, "_text"):
                        drone._text(f"ECPS: {ev[1]}", 0.0)
            wp_seen[0] = len(pilot.watchdog_events)
        compute.append(cms)
        if cms > DT * 1000:
            deadline_miss += 1
        for k in STAGES:
            stage_hist[k].append(stage_ms.get(k, 0.0))
        cmd = info.cmd
        tf = tofs.get(0)
        tr = tofs.get(180)
        row = {
            "t": round(now - T0, 3),
            "mono": round(now, 4),  # CLOCK_MONOTONIC, comparable with other processes on the Pi
            "dt_ms": round(dt * 1000, 2),
            "compute_ms": round(cms, 2),
            **{f"ms_{k}": round(stage_ms.get(k, 0.0), 3) for k in STAGES},
            "fc_mode": drone.fc_mode(),
            "armed": int(drone.armed()),
            "in_control": int(inc),
            "alt": info.tel.alt_m,
            "x": info.tel.x_m,
            "y": info.tel.y_m,
            "yaw": info.tel.yaw_deg,
            "rng": drone.rangefinder_m(),
            "ext_h": getattr(drone, "ext_h_m", None),
            "ext_src": getattr(drone, "ext_src", ""),
            "frame_age_ms": round(min(drone.frame_age_s(), 99) * 1000, 1),
            "tof_f_age_ms": round(min(tf.age_s(), 99) * 1000, 1) if tf else "",
            "tof_r_age_ms": round(min(tr.age_s(), 99) * 1000, 1) if tr else "",
            "tof_f_min": round(float(np.nanmin(tf.frame)), 3) if tf is not None and tf.frame is not None and np.isfinite(tf.frame).any() else "",
            "raw_forward": round(info.raw.forward, 3),
            "raw_yaw": round(info.raw.yaw, 3),
            "cmd_forward": round(cmd.forward, 3),
            "cmd_lateral": round(getattr(cmd, "lateral", 0.0), 3),
            "cmd_yaw": round(cmd.yaw, 3),
            "cmd_throttle": round(cmd.throttle, 3),
            "escape": int(cmd.escape),
            "note": cmd.note,
            "guard": getattr(drone, "guard_note", ""),
            **({"trk_state": pilot.target.state,
                "trk_range": round(pilot.target.last_det.range_m, 3) if pilot.target.last_det is not None else "",
                "trk_bearing": round(pilot.target.last_det.bearing_deg, 1) if pilot.target.last_det is not None else "",
                "trk_n": round(pilot.target.est[0], 3) if pilot.target.est is not None else "",
                "trk_e": round(pilot.target.est[1], 3) if pilot.target.est is not None else ""} if marker is not None else {}),
            **{k: round(v, 1) for k, v in info.rates.items() if k.startswith(("DN", "MDN"))},
        }
        if w is None:
            w = csv.DictWriter(fcsv, fieldnames=list(row.keys()), extrasaction="ignore")
            w.writeheader()
        w.writerow(row)
        rows_n += 1
        if console is not None:
            console.on_tick(info, row, inc, now)
        if link is not None:
            link.send_status({
                "t": row["t"], "ctl": int(inc), "cms": row["compute_ms"], "fage": row["frame_age_ms"],
                "tage": row["tof_f_age_ms"], "rtf": round(info.rtf, 3), "note": cmd.note[:40],
                "cmd": [row["cmd_forward"], row["cmd_lateral"], row["cmd_yaw"], row["cmd_throttle"], row["escape"]],
                "raw": [row["raw_forward"], row["raw_yaw"], round(info.raw.throttle, 3)],
                "rates": {k: round(v, 1) for k, v in info.rates.items() if k.startswith(("DN", "MDN"))},
            })
        if now - last_sys >= 1.0:
            last_sys = now
            si = sysinfo()
            if wsys is None:
                wsys = csv.DictWriter(fsys, fieldnames=list(si.keys()), extrasaction="ignore")
                wsys.writeheader()
            wsys.writerow(si)
            fsys.flush()
            fcsv.flush()
            if not a.quiet:
                print(f"[{si['t']:7.1f}] {drone.fc_mode():>8} ctl={int(inc)} tick p50/max {np.median(compute[-20:]):5.1f}/"
                      f"{max(compute[-20:]):5.1f} ms  fwd {cmd.forward:+.2f} yaw {cmd.yaw:+.2f} {cmd.note[:40]:40s}"
                      f" T_cpub {si.get('T_cpub', 0):.0f}C", flush=True)
        sl = nxt - time.monotonic()
        if sl > 0:
            time.sleep(sl)
        else:
            nxt = time.monotonic()  # overran: do not try to catch up with a burst of ticks
finally:
    try:
        if a.mode != "bench":
            drone.send(FlightCommand.hover("companion stop"))
    except Exception:
        pass
    sensors = [x for x in [marker, cam, *tofs.values()] if x is not None]
    for s in sensors:  # signal every thread first so their join timeouts overlap (systemd TimeoutStopSec)
        ev = getattr(s, "_stop", None)
        if ev is not None:
            ev.set()
    for s in sensors:
        s.stop()
    if link is not None:
        link.stop()
    if console is not None:
        console.stop()
    fcsv.close()
    fsys.close()


def pct(x, qs=(50, 95, 99, 100)):
    if not x:
        return {}
    v = np.asarray(x, float)
    return {f"p{q}": round(float(np.percentile(v, q)), 3) for q in qs} | {"mean": round(float(v.mean()), 3)}


summary = {
    "mode": a.mode,
    "model": model,
    "camera": cam_src,
    "tof": tof_src,
    "hz": HZ,
    "affinity": sorted(os.sched_getaffinity(0)),
    "ticks": rows_n,
    "wall_s": round(time.monotonic() - T0, 2),
    "deadline_miss": deadline_miss,
    "compute_ms": pct(compute),
    "period_ms": pct(period[1:]),
    "stage_ms": {k: pct(v) for k, v in stage_hist.items() if any(v)},
    "camera_frames": getattr(cam, "frames", 0),
    "camera_decode_ms": pct(getattr(cam, "decode_ms", [])),
    "tof": {
        n.name: {"frames": n.frames, "errors": n.errors, "status": n.status, "read_ms": pct(n.read_ms),
                 "init_s": getattr(n, "init_s", None)}
        for n in tofs.values()
    },
    "frames_new": drone.frames_new,
    "frames_repeated": drone.frames_repeated,
    "cmd_sent": drone.cmd_sent,
    "cmd_blocked": drone.cmd_blocked,
    "land_sent": drone.land_sent,
    "control_s": round(control_s, 2),
    "events": events,
    "watchdog_events": pilot.watchdog_events[-200:],
    "guard_events": getattr(drone, "guard_events", [])[-200:],
    "guard_ticks": getattr(drone, "guard_ticks", {}),
    "dt_clamped_ticks": dt_clamped,
    "safety_events": pilot.safety.events[-200:],
    "struct": pilot.struct.summary() if pilot.struct is not None else {},
    "pil": {"rx": link.rx, "go_changes": link.go_changes[:50]} if link is not None else None,
    "console": console.summary() if console is not None else None,
    "marker": {"frames": marker.frames, "hits": marker.hits, "errors": marker.errors, "status": marker.status,
               "det_ms": pct(marker.det_ms), "fixes": pilot.target.fixes, "state": pilot.target.state,
               "est": pilot.target.est, "log": pilot.target.log[-100:]} if marker is not None else None,
}
json.dump({"args": vars(a), "hw": hw, "summary": summary}, open(out / "run.json", "w"), indent=1, default=str)
print(json.dumps({k: summary[k] for k in ("ticks", "deadline_miss", "compute_ms", "period_ms")}), flush=True)
print(f"log: {out}", flush=True)
