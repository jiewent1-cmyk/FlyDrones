"""ECPS295 course twin for the L1 simulator (TechRoute v2.2 §4.1, §12).

Dynamics come from SITL system identification (sysid_tau.py on course params + ecps295.json):
first-order time constants differ per axis, so TwinSimDrone replaces SimDrone's single tau.
"""

from __future__ import annotations

import math

import numpy as np

from flydrones.drones.sim import SimDrone

# SITL sysid 2026-10-01, Copter-4.7.0, course_base.parm + ecps295.json (prior mass 295 g)
ECPS295_DYN = {
    "v_max": 0.5,
    "vz_max": 0.3,
    "yaw_rate_max_dps": 30.0,
    "tau_xy": 0.60,
    "tau_z": 0.35,
    "tau_yaw": 0.12,
}

TAKEOFF_ALT_M = 0.6

# §12 envelope, 20' cage (10' cage: geofence 0.8, v_max 0.3)
ECPS295_SAFETY_20FT = {
    "max_alt_m": 1.0,
    "min_alt_m": 0.4,
    "geofence_radius_m": 2.0,
    "max_flight_s": 120,
    "min_battery_pct": 30,
}


class TwinSimDrone(SimDrone):
    """SimDrone with per-axis first-order lag and the course takeoff height."""

    def __init__(
        self,
        *args,
        tau_xy: float = ECPS295_DYN["tau_xy"],
        tau_z: float = ECPS295_DYN["tau_z"],
        tau_yaw: float = ECPS295_DYN["tau_yaw"],
        takeoff_alt: float = TAKEOFF_ALT_M,
        **kw,
    ):
        kw.setdefault("v_max", ECPS295_DYN["v_max"])
        kw.setdefault("vz_max", ECPS295_DYN["vz_max"])
        kw.setdefault("yaw_rate_max_dps", ECPS295_DYN["yaw_rate_max_dps"])
        super().__init__(*args, tau=tau_xy, **kw)
        self.tau_vec = np.array([tau_xy, tau_xy, tau_z])
        self.tau_yaw = tau_yaw
        self.takeoff_alt = takeoff_alt

    def takeoff(self) -> None:
        super().takeoff()
        self._takeoff_target = self.takeoff_alt

    def start_hovering(self) -> None:
        """Begin already airborne at the current position (to align t=0 with a SITL run that waited for takeoff)."""
        self.flying = True
        self._takeoff_target = None
        self._landing = False

    def step(self, dt: float) -> None:
        self.t += dt
        self.battery = max(0.0, self.battery - dt * 100 / 600)
        if not self.flying:
            self.vel[:] = 0
            self.yaw_rate = 0
            return
        c = self.cmd
        if self._takeoff_target is not None:
            vz_t = 0.6 if self.pos[2] < self._takeoff_target else 0.0
            if self.pos[2] >= self._takeoff_target:
                self._takeoff_target = None
            target = np.array([0.0, 0.0, vz_t])
            yr_t = 0.0
        elif self._landing:
            target = np.array([0.0, 0.0, -0.5])
            yr_t = 0.0
        else:
            f = np.array([math.cos(self.yaw), math.sin(self.yaw)])
            r = np.array([math.sin(self.yaw), -math.cos(self.yaw)])
            vxy = (c.forward * f + c.lateral * r) * self.v_max
            target = np.array([vxy[0], vxy[1], c.throttle * self.vz_max])
            yr_t = -c.yaw * self.yr_max
        k = 1 - np.exp(-dt / self.tau_vec)
        self.vel += (target - self.vel) * k + self.rng.standard_normal(3) * self.wind * math.sqrt(dt)
        self.yaw_rate += (yr_t - self.yaw_rate) * (1 - math.exp(-dt / self.tau_yaw))
        self.yaw += self.yaw_rate * dt
        new, hit = self._collide(self.pos + self.vel * dt)
        if hit:
            if not self._touching:
                self.collisions += 1
            self.vel *= -0.2
        self._touching = hit
        self.pos = new
        if self._landing and self.pos[2] <= 0.02:
            self.flying = False
            self.pos[2] = 0.0
