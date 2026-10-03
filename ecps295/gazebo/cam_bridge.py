"""Gazebo camera -> shared memory bridge (TechRoute §6A G4, option B variant).

gz.transport13 bindings only exist for the system python, FlyDrones runs in conda, so this process (system python)
subscribes to the Gazebo image topic, converts to grayscale, area-downsamples (§4.2 C7: render high, then average)
and publishes the latest frame in a seqlock-protected shared-memory file read by gz_camera_drone.py.

layout: header <QdII = seq (odd while writing), sim stamp [s], width, height; then width*height uint8
    /usr/bin/python3 cam_bridge.py --topic /ecps295/camera --size 192 144
"""

import argparse
import mmap
import os
import signal
import struct
import time

import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node
from PIL import Image as PILImage

HDR = struct.Struct("<QdII")

ap = argparse.ArgumentParser()
ap.add_argument("--topic", default="/ecps295/camera")
ap.add_argument("--size", type=int, nargs=2, default=[192, 144])
ap.add_argument("--shm", default="/dev/shm/ecps295_cam")
a = ap.parse_args()

W, H = a.size
fd = os.open(a.shm, os.O_CREAT | os.O_RDWR, 0o644)
os.ftruncate(fd, HDR.size + W * H)
mm = mmap.mmap(fd, HDR.size + W * H)
seq = 0
mm[: HDR.size] = HDR.pack(seq, 0.0, W, H)
count = {"n": 0}


def on_image(msg: Image) -> None:
    global seq
    rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    gray = PILImage.fromarray(rgb).convert("L").resize((W, H), PILImage.Resampling.BOX)
    stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    seq += 1                                   # odd: writer active
    mm[:8] = struct.pack("<Q", seq)
    mm[HDR.size:] = gray.tobytes()
    seq += 1                                   # even: frame complete
    mm[: HDR.size] = HDR.pack(seq, stamp, W, H)
    count["n"] += 1


node = Node()
if not node.subscribe(Image, a.topic, on_image):
    raise SystemExit(f"cannot subscribe to {a.topic}")
print(f"bridging {a.topic} -> {a.shm} as {W}x{H} gray", flush=True)
running = True
signal.signal(signal.SIGTERM, lambda *_: globals().update(running=False))
last, t_last = 0, time.time()
while running:
    time.sleep(5)
    n = count["n"]
    print(f"{(n - last) / (time.time() - t_last):.1f} frames/s wall", flush=True)
    last, t_last = n, time.time()
