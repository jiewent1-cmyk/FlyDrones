#!/usr/bin/env python3
"""USB camera bench probe (Orange Pi): list V4L2 formats, capture N frames through the companion's camera thread and
report the delivered frame rate, decode + downsample time, frame age and a saved 192x144 sample (what the retina sees).

    /opt/ecps295/venv/bin/python /opt/ecps295/scripts/cam_probe.py [--device /dev/video0] [--seconds 10]
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, "/opt/ecps295/companion")
import camera as C  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--device", default="/dev/video0")
ap.add_argument("--seconds", type=float, default=10)
ap.add_argument("--fps", type=float, default=30)
ap.add_argument("--out", default="/opt/ecps295/logs/cam_probe.png")
a = ap.parse_args()

print(subprocess.run(["v4l2-ctl", "-d", a.device, "--list-formats-ext"], capture_output=True, text=True).stdout[:2000])
cam = C.UVCCamera(a.device, fps=a.fps).start()
t0 = time.monotonic()
ages = []
while time.monotonic() - t0 < a.seconds:
    img, st = cam.latest()
    if img is not None:
        ages.append((time.monotonic() - st) * 1000)
    time.sleep(0.05)
cam.stop()
dt = time.monotonic() - t0
print(cam.status)
if not cam.frames:
    print("no frames")
    sys.exit(1)
print(f"{cam.frames / dt:.1f} fps delivered, decode+gray+resize p50 {np.percentile(cam.decode_ms, 50):.2f} ms "
      f"p95 {np.percentile(cam.decode_ms, 95):.2f} ms, frame age at 20 Hz sampling p50 {np.percentile(ages, 50):.1f} ms, errors {cam.errors}")
cv2.imwrite(a.out, cam.img)
print("sample:", a.out)
