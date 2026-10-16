#!/usr/bin/env python3
"""VL53L7CX bench probe (Orange Pi): bring up one sensor and print its 8x8 frames as the companion will see them
(row 0 = top, column 0 = left, metres, '  -- ' = no valid target), plus frame rate, I2C read time and temperature.

    /opt/ecps295/venv/bin/python /opt/ecps295/scripts/tof_probe.py --sensor front [--frames 50]
    ... --orient     guided orientation check: hold a hand ~15 cm in front of the TOP-LEFT quarter of the field
                     of view (as seen from behind the sensor, looking where it looks); the script reports which
                     quarter is nearest and the flip_lr / flip_ud to put in config/companion.yaml
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, "/opt/ecps295/companion")
import tof_vl53l7cx as T  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--hw", default="/opt/ecps295/config/companion.yaml")
ap.add_argument("--sensor", default="front")
ap.add_argument("--frames", type=int, default=30)
ap.add_argument("--orient", action="store_true")
a = ap.parse_args()

cfg = dict(yaml.safe_load(open(a.hw))["tof"]["sensors"][a.sensor])
cfg.pop("bearing", None)
s = T.ToFSensor(a.sensor, **cfg).start()
t0 = time.monotonic()
while s.frame is None:
    if time.monotonic() - t0 > 30:
        print("no frame after 30 s:", s.status)
        sys.exit(1)
    time.sleep(0.1)
print(f"{a.sensor}: {s.status}, firmware upload + config {s.init_s:.2f} s, bus /dev/i2c-{s.bus} 0x{s.addr:02x}")


def show(f):
    g = f.reshape(8, 8)
    for row in g:
        print(" ".join("  -- " if not np.isfinite(v) else f"{v:5.2f}" for v in row))


last, n = None, 0
f0 = s.frames
t1 = time.monotonic()
quarters = []
while n < a.frames:
    f = s.frame
    if f is last:
        time.sleep(0.005)
        continue
    last, n = f, n + 1
    if not a.orient and (n % 10 == 1 or n == a.frames):
        print(f"--- frame {n}  temp {s.temp_c} C")
        show(f)
    if a.orient:
        g = np.where(np.isfinite(f), f, 9.0).reshape(8, 8)
        q = {"top-left": g[:4, :4], "top-right": g[:4, 4:], "bottom-left": g[4:, :4], "bottom-right": g[4:, 4:]}
        quarters.append(min(q, key=lambda k: np.median(q[k])))
hz = (s.frames - f0) / (time.monotonic() - t1)
print(f"rate {hz:.1f} Hz, I2C read per frame p50 {np.percentile(s.read_ms, 50):.2f} ms p95 {np.percentile(s.read_ms, 95):.2f} ms,"
      f" errors {s.errors}")
if a.orient:
    seen = max(set(quarters), key=quarters.count)
    fix = {"top-left": "no flip", "top-right": "flip_lr: true", "bottom-left": "flip_ud: true",
           "bottom-right": "flip_lr: true and flip_ud: true"}[seen]
    print(f"hand at top-left appears as {seen} with the current flags -> toggle: {fix}")
s.stop()
