"""SITL-only ToF source (2026-10-17, ground-console tests): the two VL53L7CX emulated from the simulator's ground truth
(SIM_STATE) and a Gazebo world layout, exactly as mavlink_twin.Ecps295MavlinkDrone.zones() does in the local Gazebo
runs. The local 3D map, the range shield, the waypoint layer and the console then see a real room while the companion
flies an ArduPilot SITL on the Pi.

    ecps_companion.py --tof sitl:<world>.layout.json ...

SIM_STATE only exists in SITL: on the real drone this source never delivers a frame, the ToF guard refuses the
hand-over ("ToF ... stale") and nothing flies on it.
"""

from __future__ import annotations

import threading
import time

import numpy as np


class _TruthView:
    """The drone as mavlink_twin's zones() needs it: RealDrone deliberately hides the simulator truth
    (truth_ned() -> None, so nothing on the real drone can lean on it); this view puts it back for the emulated sensors
    only. zones() keeps its caches on the view, the drone itself is not changed."""

    def __init__(self, drone):
        self._drone = drone

    def __getattr__(self, name):
        return getattr(self._drone, name)

    def truth_ned(self):
        from mavlink_twin import Ecps295MavlinkDrone

        return Ecps295MavlinkDrone.truth_ned(self._drone)


class SitlWorldToF:
    """tof_vl53l7cx interface (frame / stamp / age_s / counters / start / stop), fed from SIM_STATE + the layout."""

    def __init__(self, name: str, layout: str, bearing: float = 0.0, hz: float = 15.0, n: int = 8, hfov: float = 60.0,
                 vfov: float = 60.0, max_m: float = 3.5, noise_m: float = 0.01, **_):
        self.name, self.layout, self.bearing = name, layout, float(bearing)
        self.hz, self.n, self.hfov, self.vfov, self.max_m, self.noise = float(hz), int(n), float(hfov), float(vfov), float(max_m), float(noise_m)
        self.frame = None
        self.stamp = 0.0
        self.frames = 0
        self.errors = 0
        self.read_ms: list[float] = []
        self.status = "sitl: not bound"
        self.init_s = 0.0
        self.drone = None
        self.view = None
        self._stop = threading.Event()
        self._th: threading.Thread | None = None

    def bind(self, drone) -> None:
        """Called once the drone backend exists; every sensor shares the room built from the layout."""
        if getattr(drone, "room", None) is None:
            from world_layout import room_from_layout

            drone.room = room_from_layout(self.layout)
        self.drone = drone
        self.view = getattr(drone, "_sitl_view", None) or _TruthView(drone)
        drone._sitl_view = self.view  # one view (and one zones() cache) for both sensors
        self.status = "sitl: waiting for SIM_STATE"

    def age_s(self) -> float:
        return time.monotonic() - self.stamp if self.stamp else float("inf")

    def start(self):
        self._th = threading.Thread(target=self._run, name=f"sitl-tof-{self.name}", daemon=True)
        self._th.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._th is not None:
            self._th.join(timeout=2)

    def _run(self) -> None:
        from mavlink_twin import Ecps295MavlinkDrone

        nxt = time.monotonic()
        while not self._stop.is_set():
            nxt += 1.0 / self.hz
            self._stop.wait(max(0.0, nxt - time.monotonic()))
            d = self.view
            if d is None or getattr(self.drone, "m", None) is None:
                continue
            c0 = time.perf_counter()
            try:  # rate 4x ours: our loop sets the pace, zones() recomputes on every call
                r = Ecps295MavlinkDrone.zones(d, self.bearing, self.hfov, self.vfov, self.n, 4 * self.hz, self.noise, self.max_m)
            except Exception as e:  # noqa: BLE001 - a test source: count and keep going
                self.errors += 1
                self.status = f"sitl: {type(e).__name__}: {e}"[:60]
                continue
            if r is None:
                continue
            self.frame = np.array(r, dtype=float)  # a new object per frame: the map update keys on identity
            self.stamp = time.monotonic()
            self.frames += 1
            self.status = "sitl: SIM_STATE + layout"
            if len(self.read_ms) < 50000:
                self.read_ms.append((time.perf_counter() - c0) * 1000)


def make(cfg: dict, name: str, source: str, bearing: float):
    """source 'sitl:<layout.json>'."""
    kw = {k: cfg[k] for k in ("hz", "n", "hfov", "vfov", "max_m") if k in cfg}
    return SitlWorldToF(name, source.split(":", 1)[1], bearing=bearing, **kw)
