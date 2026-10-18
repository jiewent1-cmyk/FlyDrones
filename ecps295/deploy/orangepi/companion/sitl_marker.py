"""SITL-only camera (2026-10-18, target-tracking tests): the drone camera rendered from the simulator's ground truth
(SIM_STATE position and yaw) with one ArUco marker standing in the room, so detection -> TargetTrack -> waypoint layer
-> ArduPilot runs closed-loop on the Pi without Gazebo.

    ecps_companion.py --camera sitl_marker:<north>,<east>,<up>,<face yaw deg>[,<id>] ...

The background is a fixed panorama that turns with the yaw (the brain sees motion while yawing); the marker is drawn
with the detector's own pinhole model (ecps_target.render_marker, hw config `marker`: hfov, pitch, size, dictionary).
Like sitl_world.py, SIM_STATE only exists in SITL: on the real drone this source never delivers a frame and the camera
watchdog refuses the hand-over.
"""

from __future__ import annotations

import math
import time

import cv2
import numpy as np

from camera import _Cam


class SitlMarkerCamera(_Cam):
    def __init__(self, spec: str, marker_cfg: dict, width: int = 640, height: int = 480, fps: float = 15.0, **kw):
        super().__init__(fps=fps, **kw)
        from ecps_target import camera_matrix, marker_image

        v = [float(x) for x in spec.split(",")]
        self.marker = (v[0], v[1], v[2])
        self.marker_yaw = v[3]
        self.marker_id = int(v[4]) if len(v) > 4 else int(marker_cfg.get("marker_id", 0) or 0)
        self.size_m = float(marker_cfg.get("size_m", 0.15))
        self.pitch = float(marker_cfg.get("pitch_deg", 12.0))
        self.wh = (width, height)
        self.K = camera_matrix(width, height, float(marker_cfg.get("hfov_deg", 70.0)))
        self.tex = marker_image(marker_cfg.get("dictionary", "4x4_50"), self.marker_id)
        rng = np.random.default_rng(3)
        pano = cv2.resize((rng.random((40, 360)) * 160 + 40).astype(np.uint8), (int(width * 360 / 70), height),
                          interpolation=cv2.INTER_CUBIC)
        self.pano = cv2.GaussianBlur(pano, (0, 0), 2)
        self.drone = None
        self.status = "sitl_marker: not bound"

    def bind(self, drone) -> None:
        self.drone = drone
        self.status = "sitl_marker: waiting for SIM_STATE"

    def _truth(self):
        m = getattr(self.drone, "m", None)
        s = m.messages.get("SIM_STATE") if m is not None else None
        if s is None:
            return None
        from mavlink_twin import Ecps295MavlinkDrone

        ned = Ecps295MavlinkDrone.truth_ned(self.drone)
        return None if ned is None else (ned, math.degrees(s.yaw))

    def _run(self) -> None:
        from ecps_target import render_marker

        nxt = time.monotonic()
        w, h = self.wh
        while not self._stop.is_set():
            nxt += 1.0 / self.fps
            self._stop.wait(max(0.0, nxt - time.monotonic()))
            tr = self._truth() if self.drone is not None else None
            if tr is None:
                continue
            t0 = time.perf_counter()
            (n, e, up), yaw = tr
            off = int(((yaw % 360) / 360.0) * self.pano.shape[1])
            img = np.roll(self.pano, -off, axis=1)[:, :w].copy()
            render_marker(img, self.K, (n, e, up), yaw, self.pitch, self.marker, self.marker_yaw, self.size_m, self.tex)
            self.status = "sitl_marker: SIM_STATE"
            self._publish_gray(img, t0)


def make(source: str, cam_cfg: dict, marker_cfg: dict):
    """source 'sitl_marker:<n>,<e>,<up>,<yaw>[,<id>]'."""
    kw = {k: cam_cfg[k] for k in ("out_w", "out_h") if k in cam_cfg}
    return SitlMarkerCamera(source.split(":", 1)[1], marker_cfg, **kw)
