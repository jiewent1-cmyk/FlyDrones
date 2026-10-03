"""Save frames from a Gazebo camera topic and report the achieved frame rate (G2 check).

Runs with the *system* python3 (gz.transport13 / gz.msgs10 bindings come from apt, not conda):
    /usr/bin/python3 gz_grab.py --topic /ecps295/camera --secs 5 --save 3 --out frames/
"""

import argparse
import pathlib
import threading
import time

import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node
from PIL import Image as PILImage

ap = argparse.ArgumentParser()
ap.add_argument("--topic", default="/ecps295/camera")
ap.add_argument("--secs", type=float, default=5.0)
ap.add_argument("--save", type=int, default=3, help="frames to save, spread over the window")
ap.add_argument("--out", default="frames")
a = ap.parse_args()

out = pathlib.Path(a.out)
out.mkdir(parents=True, exist_ok=True)
lock = threading.Lock()
stamps: list[float] = []
latest: dict = {}


def on_image(msg: Image) -> None:
    with lock:
        stamps.append(msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9)
        latest["msg"] = msg


node = Node()
if not node.subscribe(Image, a.topic, on_image):
    raise SystemExit(f"cannot subscribe to {a.topic}")

saved = 0
t0 = time.time()
while time.time() - t0 < a.secs:
    time.sleep(a.secs / max(a.save, 1))
    with lock:
        msg = latest.get("msg")
    if msg is None or saved >= a.save:
        continue
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    PILImage.fromarray(img).save(out / f"frame_{saved}.png")
    saved += 1

with lock:
    s = sorted(set(stamps))
if len(s) < 2:
    raise SystemExit(f"no frames on {a.topic}")
dt = np.diff(s)
print(f"{len(s)} frames in {s[-1] - s[0]:.2f} s sim time -> {1 / dt.mean():.1f} Hz (dt p95 {np.percentile(dt, 95) * 1000:.1f} ms); "
      f"size {msg.width}x{msg.height}; saved {saved} to {out}/")
