"""S7 (route review 2026-10-14): local 3D occupancy map from multi-zone ToF sensors (VL53L5CX / VL53L7CX 8x8) for the
range shield (ecps_struct.RangeShield, config ecps_pilot.struct.shield.map). Off unless configured.

Why: two 8x8 boards (front + rear) see 2 x 45-60 deg at any instant. The map keeps what they saw while the drone turned,
so obstacles beside the drone still bound its motion, and it knows the height of what it saw: a low box top below the
propeller band is passable, a wall at flight height is not (raw multi-zone readings cannot tell the two apart).

Map: log-odds voxels (res_m, default 5 cm) on a fixed grid around the start point in the estimated (flow-EKF) frame.
Every new frame, each zone ray marks the voxels it crossed as free and its end voxel as occupied (no target -> free up to
max_m); returns below floor_m are the floor. Free voxels are found by sampling the ray every res/3, so a voxel that a ray
only clips by a corner can keep a stale hit until it decays (log-odds 0.85 -> below 0.6 in about 0.35 tau). Everything decays toward unknown with time constant tau_s, so content that
drift of the estimate has shifted fades instead of piling up (no SLAM / drift correction yet).
Query: occupied voxels within the flight band (altitude +- band_m) and query_m of the drone, nearest one per body-frame
sector -> a virtual 360-degree range ring that the shield treats like extra beams.
"""

from __future__ import annotations

import math

import numpy as np


def zone_dirs(bearing_deg: float, hfov_deg: float, vfov_deg: float, n: int) -> np.ndarray:
    """Body-frame unit vectors (x forward, y right, z up) of the n x n zone centres of a level-mounted multi-zone sensor
    facing bearing_deg; row 0 = top."""
    az = math.radians(bearing_deg) + (np.arange(n) - (n - 1) / 2) * math.radians(hfov_deg) / n
    el = ((n - 1) / 2 - np.arange(n)) * math.radians(vfov_deg) / n
    A, E = np.meshgrid(az, el)
    return np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], axis=-1).reshape(-1, 3)


class LocalMap:
    def __init__(self, cfg: dict):
        self.res = float(cfg.get("res_m", 0.05))
        self.half = float(cfg.get("half_m", 4.0))
        self.zmax = float(cfg.get("zmax_m", 1.6))
        self.n = int(round(2 * self.half / self.res))
        self.nz = int(round(self.zmax / self.res))
        self.L = np.zeros(self.n * self.n * self.nz, np.float32)
        self.l_hit, self.l_miss, self.l_lo, self.l_hi, self.l_occ = 0.85, -0.4, -2.0, 3.5, 0.6
        self.tau = float(cfg.get("tau_s", 10.0))
        self.floor = float(cfg.get("floor_m", 0.08))
        self.band = float(cfg.get("band_m", 0.10))
        # N4 (2026-10-15): depth of the band below the drone; deeper than band_m -> low box tops are obstacles (flying over
        # them corrupts the flow-EKF), except when the drone is already over one (within over_r_m), so it can leave
        self.band_lo = float(cfg.get("band_lo_m", self.band))
        self.over_r = float(cfg.get("over_r_m", 0.15))
        self.over_n = 0  # queries made while over a lower-band obstacle
        self.query = float(cfg.get("query_m", 1.0))
        self.n_sec = int(cfg.get("sectors", 36))
        self._t_decay = None
        self.frames = 0

    def _lin(self, P: np.ndarray) -> np.ndarray:
        i = np.floor((P[:, 0] + self.half) / self.res).astype(np.int64)
        j = np.floor((P[:, 1] + self.half) / self.res).astype(np.int64)
        k = np.floor(P[:, 2] / self.res).astype(np.int64)
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n) & (k >= 0) & (k < self.nz)
        return np.unique((i[ok] * self.n + j[ok]) * self.nz + k[ok])

    def integrate(self, origin, dirs: np.ndarray, ranges: np.ndarray, max_m: float, t: float) -> None:
        """origin (north, east, up) of the sensor, dirs (N, 3) world unit vectors, ranges (N,) slant ranges (nan = no
        target within max_m)."""
        if self._t_decay is None:
            self._t_decay = t
        elif t - self._t_decay >= 0.5:
            self.L *= math.exp(-(t - self._t_decay) / self.tau)
            self._t_decay = t
        o = np.asarray(origin, float)
        hit = np.isfinite(ranges)
        rr = np.where(hit, ranges, max_m)
        steps = np.arange(self.res / 3, max_m, self.res / 3)  # third-voxel steps: a ray skips only voxels it clips by a corner
        mask = steps[None, :] < (rr[:, None] - self.res)
        P = (o[None, None, :] + dirs[:, None, :] * steps[None, :, None])[mask]
        free = self._lin(P)
        self.L[free] = np.maximum(self.l_lo, self.L[free] + self.l_miss)
        H = o[None, :] + dirs[hit] * (ranges[hit][:, None] + self.res / 4)  # just inside the surface, not on a voxel face
        H = H[H[:, 2] > self.floor]
        occ = self._lin(H)
        self.L[occ] = np.minimum(self.l_hi, self.L[occ] + self.l_hit)
        self.frames += 1

    def sectors(self, x: float, y: float, z: float, yaw: float) -> list:
        """Nearest occupied voxel column in the flight band per body-frame sector within query_m: [((ux, uy), d)] with
        (ux, uy) the body-frame unit vector (x forward, y right) and d the horizontal centre-to-surface distance."""
        r = self.query + self.res
        i0, i1 = max(0, int((x - r + self.half) / self.res)), min(self.n, int((x + r + self.half) / self.res) + 1)
        j0, j1 = max(0, int((y - r + self.half) / self.res)), min(self.n, int((y + r + self.half) / self.res) + 1)
        k0, k1 = max(0, int((z - self.band) / self.res)), min(self.nz, int((z + self.band) / self.res) + 1)
        if i0 >= i1 or j0 >= j1 or k0 >= k1:
            return []
        W = self.L.reshape(self.n, self.n, self.nz)[i0:i1, j0:j1]
        G = W[:, :, k0:k1].max(axis=2) > self.l_occ
        kl = max(0, int((z - self.band_lo) / self.res))
        if kl < k0:  # N4 lower band
            Gl = W[:, :, kl:k0].max(axis=2) > self.l_occ
            il, jl = np.nonzero(Gl)
            dl = np.hypot((il + i0 + 0.5) * self.res - self.half - x, (jl + j0 + 0.5) * self.res - self.half - y)
            if len(dl) and dl.min() <= self.over_r:
                self.over_n += 1
            else:
                G = G | Gl
        ii, jj = np.nonzero(G)
        if not len(ii):
            return []
        dx = (ii + i0 + 0.5) * self.res - self.half - x
        dy = (jj + j0 + 0.5) * self.res - self.half - y
        cy, sy = math.cos(yaw), math.sin(yaw)
        bx, by = cy * dx + sy * dy, -sy * dx + cy * dy
        d = np.hypot(bx, by)
        keep = (d > 1e-6) & (d < self.query)
        if not keep.any():
            return []
        bx, by, d = bx[keep], by[keep], d[keep]
        sec = np.floor((np.arctan2(by, bx) + math.pi) / (2 * math.pi) * self.n_sec).astype(np.int64) % self.n_sec
        order = np.lexsort((d, sec))
        _, first = np.unique(sec[order], return_index=True)
        pick = order[first]
        return [((float(bx[q] / d[q]), float(by[q] / d[q])), float(d[q] - self.res / 2)) for q in pick]
