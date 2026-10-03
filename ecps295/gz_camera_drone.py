"""Ecps295MavlinkDrone whose frame() comes from the Gazebo camera (TechRoute §6A G4).

Frames arrive through gazebo/cam_bridge.py (system python) in shared memory, already grayscale and area-downsampled;
FlyDrones' Retina does the final resize to its 96x72 grid.
"""

from __future__ import annotations

import mmap
import os
import struct
import time

import numpy as np

from mavlink_twin import Ecps295MavlinkDrone

HDR = struct.Struct("<QdII")


class GazeboCameraMavlinkDrone(Ecps295MavlinkDrone):
    has_camera = True

    def __init__(self, *args, shm: str = "/dev/shm/ecps295_cam", passive: bool = False, **kw):
        """passive=True: the brain sees and thinks but send() is ignored (open-loop probes)."""
        super().__init__(*args, **kw)
        self.shm_path = shm
        self.passive = passive
        self._mm = None
        self._last = None
        self._last_seq = 0
        self._last_new_wall = 0.0
        self.frame_stamp = float("nan")   # Gazebo sim time of the frame last returned
        self.frames_new = 0
        self.frames_repeated = 0

    def _open(self) -> None:
        t0 = time.time()
        while not os.path.exists(self.shm_path):
            if time.time() - t0 > 10:
                raise SystemExit(f"{self.shm_path} missing: is gazebo/cam_bridge.py running?")
            time.sleep(0.2)
        fd = os.open(self.shm_path, os.O_RDONLY)
        self._mm = mmap.mmap(fd, 0, prot=mmap.PROT_READ)

    def frame(self) -> np.ndarray | None:
        if self._mm is None:
            self._open()
        for _ in range(5):                     # seqlock read
            seq, stamp, w, h = HDR.unpack_from(self._mm, 0)
            if seq % 2:
                continue
            data = bytes(self._mm[HDR.size:HDR.size + w * h])
            if struct.unpack_from("<Q", self._mm, 0)[0] == seq:
                break
        else:
            return self._last
        if seq == 0:
            return self._last
        if seq != self._last_seq:
            self._last = np.frombuffer(data, dtype=np.uint8).reshape(h, w)
            self._last_seq, self.frame_stamp = seq, stamp
            self._last_new_wall = time.monotonic()
            self.frames_new += 1
        else:
            self.frames_repeated += 1
        return self._last

    def frame_age_s(self) -> float:
        return time.monotonic() - self._last_new_wall if self._last_new_wall else float("inf")

    def send(self, cmd) -> None:
        if self.passive:
            self._heartbeat()
            return
        super().send(cmd)
