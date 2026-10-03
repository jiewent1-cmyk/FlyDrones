"""Gazebo camera topics -> shared memory (TechRoute §6A G4).

gz.transport13 bindings only exist for the system python while FlyDrones runs in conda, so this process (system
python) subscribes to Gazebo image topics and publishes the latest frame of each in a seqlock-protected
shared-memory file:
  - the drone camera for the brain: grayscale, area-downsampled (§4.2 C7: render high, then average)
  - monitor views (chase / overview cameras): RGB, as rendered

layout per file: header <QdIII = seq (odd while writing), sim stamp [s], width, height, channels; then pixels (uint8)

    /usr/bin/python3 cam_bridge.py                                  # default: drone camera + both monitor views
    /usr/bin/python3 cam_bridge.py --stream /ecps295/camera:/dev/shm/ecps295_cam:192x144:gray
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

HDR = struct.Struct("<QdIII")
DEFAULT_STREAMS = [
    "/ecps295/camera:/dev/shm/ecps295_cam:192x144:gray",
    "/ecps295/camera:/dev/shm/ecps295_cam_rgb:640x480:rgb",  # same topic, colour, for monitor.py only
    "/ecps295/chase_camera:/dev/shm/ecps295_chase:960x540:rgb",
    "/ecps295/overview_camera:/dev/shm/ecps295_overview:960x540:rgb",
]

ap = argparse.ArgumentParser()
ap.add_argument("--stream", action="append", help="topic:shm_path:WxH:gray|rgb (repeatable)")
a = ap.parse_args()


class Stream:
    def __init__(self, spec: str):
        self.topic, self.path, size, self.mode = spec.split(":")
        self.w, self.h = (int(v) for v in size.split("x"))
        self.c = 1 if self.mode == "gray" else 3
        n = HDR.size + self.w * self.h * self.c
        fd = os.open(self.path, os.O_CREAT | os.O_RDWR, 0o644)
        os.ftruncate(fd, n)
        self.mm = mmap.mmap(fd, n)
        self.seq = 0
        self.mm[: HDR.size] = HDR.pack(0, 0.0, self.w, self.h, self.c)
        self.count = 0

    def on_image(self, msg: Image) -> None:
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        img = PILImage.fromarray(rgb)
        if self.mode == "gray":
            img = img.convert("L")
        if img.size != (self.w, self.h):
            img = img.resize((self.w, self.h), PILImage.Resampling.BOX)
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        self.seq += 1  # odd: writer active
        self.mm[:8] = struct.pack("<Q", self.seq)
        self.mm[HDR.size :] = img.tobytes()
        self.seq += 1  # even: frame complete
        self.mm[: HDR.size] = HDR.pack(self.seq, stamp, self.w, self.h, self.c)
        self.count += 1


node = Node()
streams = [Stream(s) for s in (a.stream or DEFAULT_STREAMS)]
by_topic: dict[str, list[Stream]] = {}
for st in streams:
    by_topic.setdefault(st.topic, []).append(st)
    print(f"bridging {st.topic} -> {st.path} as {st.w}x{st.h} {st.mode}", flush=True)
for topic, group in by_topic.items():  # a node subscribes once per topic; fan out to every stream on it
    if not node.subscribe(Image, topic, lambda msg, group=group: [st.on_image(msg) for st in group]):
        raise SystemExit(f"cannot subscribe to {topic}")
running = True
signal.signal(signal.SIGTERM, lambda *_: globals().update(running=False))
last, t_last = [0] * len(streams), time.time()
while running:
    time.sleep(5)
    now = time.time()
    print(
        "  ".join(f"{st.path.rsplit('/', 1)[-1]} {(st.count - prev) / (now - t_last):.1f}/s" for st, prev in zip(streams, last)),
        flush=True,
    )
    last, t_last = [st.count for st in streams], now
