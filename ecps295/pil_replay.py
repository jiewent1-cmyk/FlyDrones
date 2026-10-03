"""Per-tick compute budget of the MiniFly control loop on recorded camera frames (TechRoute §7.1, S10 prep).

Replays JPEG frames recorded by run_g4.py --record-frames (the ELP streams MJPEG, so decode is included) through
the same steps as flydrones.runtime.Pilot.tick, timing each stage. No Gazebo, SITL or MAVLink needed, so it runs on
a Raspberry Pi 5 (Cortex-A76 @ 2.4 GHz) as a stand-in for the Orange Pi Zero 3W (A733: 2x A76 @ 2.0 GHz).

    python pil_replay.py --frames g4_rec/frames --csv g4_rec/rec.csv [--threads 1] [--repeat 2] [--out pil.json]
"""

from __future__ import annotations

import argparse
import os
import sys

# numpy / BLAS thread count must be fixed before numpy is imported
_threads = sys.argv[sys.argv.index("--threads") + 1] if "--threads" in sys.argv else None
if _threads:
    for v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ[v] = _threads

import csv  # noqa: E402
import glob  # noqa: E402
import json  # noqa: E402
import platform  # noqa: E402
import time  # noqa: E402

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from envs import ECPS295_SAFETY_20FT  # noqa: E402

from flydrones.brain import Brain, load_connectome  # noqa: E402
from flydrones.config import load_config  # noqa: E402
from flydrones.motor import MotorDecoder  # noqa: E402
from flydrones.safety import SafetyGovernor, Telemetry  # noqa: E402
from flydrones.senses import InputEncoder, Retina  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--frames", required=True)
ap.add_argument("--csv", required=True, help="run_g4.py CSV of the same run (telemetry per tick)")
ap.add_argument("--threads", default=None)
ap.add_argument("--repeat", type=int, default=1)
ap.add_argument("--hz", type=float, default=20)
ap.add_argument("--out", default="pil_replay.json")
ap.add_argument(
    "--prep",
    choices=["none", "reduced"],
    default="none",
    help="reduced: JPEG decoded straight to half-size grayscale, then cv2 INTER_AREA to 192x144 "
    "(what a Pi camera driver would hand the Retina); none: full BGR frame, as upstream",
)
a = ap.parse_args()


def sysinfo() -> dict:
    def read(p):
        try:
            return open(p).read().strip()
        except OSError:
            return None

    freq = read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
    temp = read("/sys/class/thermal/thermal_zone0/temp")
    return {"cpu_mhz": int(freq) / 1000 if freq else None, "temp_c": int(temp) / 1000 if temp else None}


files = sorted(glob.glob(os.path.join(a.frames, "*.jpg")))
jpegs = [np.fromfile(f, dtype=np.uint8) for f in files]
tel_rows = list(csv.DictReader(open(a.csv)))
n = min(len(jpegs), len(tel_rows))
if n == 0:
    raise SystemExit("no frames")

cfg = load_config(None, {"safety": dict(ECPS295_SAFETY_20FT)})
brain = Brain(load_connectome(cfg["brain"]["source"]), cfg)
retina = Retina.from_config(cfg)
encoder = InputEncoder(brain.connectome, cfg)
decoder = MotorDecoder(cfg)
safety = SafetyGovernor(cfg)
dt = 1.0 / a.hz


def get_frame(buf: np.ndarray) -> np.ndarray:
    if a.prep == "reduced":  # 320x240 gray straight from the DCT, then area-average to 192x144
        return cv2.resize(cv2.imdecode(buf, cv2.IMREAD_REDUCED_GRAYSCALE_2), (192, 144), interpolation=cv2.INTER_AREA)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)  # MJPEG frame -> BGR 640x480


# warm-up on the first frame, like Pilot.warmup (decoder measures resting rates)
f0 = get_frame(jpegs[0])
t = -(decoder.settle_s + 0.1)
while t < 0:
    decoder.update(brain.tick(encoder.encode(retina.encode(f0), 0.0), ms=dt * 1000), dt)
    t += dt

stages = ["decode", "retina", "encode", "brain", "decode_cmd", "safety"]
outputs = []  # first pass only: decoded command + DN rates, to check that --prep does not change behaviour
times = {s: [] for s in stages}
total = []
before = sysinfo()
wall0 = time.perf_counter()
for _ in range(a.repeat):
    for i in range(n):
        r = tel_rows[i]
        tel = Telemetry(
            t=i * dt,
            alt_m=float(r["alt"]),
            yaw_rate_dps=float(r["yaw_rate_dps"]),
            x_m=float(r["north"]),
            y_m=float(r["east"]),
            flying=True,
        )
        c = [time.perf_counter()]
        frame = get_frame(jpegs[i])
        c.append(time.perf_counter())
        vision = retina.encode(frame)  # gray, area resize to 96x72, flow / loom grids
        c.append(time.perf_counter())
        inputs = encoder.encode(vision, tel.yaw_rate_dps)
        c.append(time.perf_counter())
        rates = brain.tick(inputs, ms=dt * 1000)  # LIF, 50 ms of brain time
        c.append(time.perf_counter())
        raw = decoder.update(rates, dt)
        c.append(time.perf_counter())
        safety.filter(raw, tel, dt)
        c.append(time.perf_counter())
        if len(outputs) < n:
            outputs.append(
                [raw.throttle, raw.yaw, raw.forward, *(rates.get(k, 0.0) for k in sorted(rates) if k.startswith("DN"))]
            )
        for k, s in enumerate(stages):
            times[s].append((c[k + 1] - c[k]) * 1000)
        total.append((c[-1] - c[0]) * 1000)
wall = time.perf_counter() - wall0
after = sysinfo()


def pct(v):
    return {q: round(float(np.percentile(v, q)), 2) for q in (50, 95, 99)} | {"max": round(float(np.max(v)), 2)}


res = {
    "host": platform.node(),
    "machine": platform.machine(),
    "cpu": platform.processor() or None,
    "python": platform.python_version(),
    "numpy": np.__version__,
    "threads": a.threads or "default",
    "prep": a.prep,
    "ticks": len(total),
    "frame_size": list(f0.shape[:2][::-1]),
    "budget_ms": 1000 / a.hz,
    "total_ms": pct(total),
    "stages_ms": {s: pct(v) for s, v in times.items()},
    "brain_realtime_x": round(n * a.repeat * dt / (sum(times["brain"]) / 1000), 2),
    "loop_utilisation_p95": round(pct(total)[95] / (1000 / a.hz), 3),
    "wall_s": round(wall, 1),
    "sys_before": before,
    "sys_after": after,
}
json.dump(res, open(a.out, "w"), indent=1)
np.save(a.out.replace(".json", "_outputs.npy"), np.array(outputs, dtype=np.float32))
print(json.dumps(res, indent=1))
