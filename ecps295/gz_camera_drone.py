"""Ecps295MavlinkDrone whose frame() comes from the Gazebo camera (TechRoute §6A G4).

Frames arrive through gazebo/cam_bridge.py (system python) in shared memory, already grayscale and area-downsampled;
FlyDrones' Retina does the final resize to its 96x72 grid.
"""

from __future__ import annotations

import time

import numpy as np
from mavlink_twin import Ecps295MavlinkDrone
from shm_frames import ShmFrame


class GazeboCameraMavlinkDrone(Ecps295MavlinkDrone):
    has_camera = True

    def __init__(self, *args, shm: str = "/dev/shm/ecps295_cam", passive: bool = False, **kw):
        """passive=True: the brain sees and thinks but send() is ignored (open-loop probes)."""
        super().__init__(*args, **kw)
        self.cam = ShmFrame(shm)
        self.passive = passive
        self._last_new_wall = 0.0
        self.frames_new = 0
        self.frames_repeated = 0

    @property
    def frame_stamp(self) -> float:
        """Gazebo sim time of the frame last returned."""
        return self.cam.stamp

    def frame(self) -> np.ndarray | None:
        if self.cam.img is None:
            t0 = time.time()
            while not self.cam.read():
                if time.time() - t0 > 10:
                    raise SystemExit(f"no frames in {self.cam.path}: is gazebo/cam_bridge.py running?")
                time.sleep(0.05)
            self._last_new_wall = time.monotonic()
            self.frames_new += 1
        elif self.cam.read():
            self._last_new_wall = time.monotonic()
            self.frames_new += 1
        else:
            self.frames_repeated += 1
        return self.cam.img

    def frame_age_s(self) -> float:
        return time.monotonic() - self._last_new_wall if self._last_new_wall else float("inf")

    def send(self, cmd) -> None:
        if self.passive:
            self._heartbeat()
            return
        super().send(cmd)
