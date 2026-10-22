"""Obstacle-aware route planning for the ground console's GOTO / route (2026-10-22).

Why: the pilot's waypoint layer steers straight at the goal. With a waypoint next to an obstacle the brain escaped
(brake, retreat, saccade), the waypoint layer turned it back 2 s later, and after 20 s without progress the waypoint was
skipped (Gazebo console session 10-21). Here the console plans around what the local 3D map (ecps_map.LocalMap, the
range shield's map) has seen and hands the pilot a path of intermediate waypoints instead:

  grid     occupied voxels from just above the floor up to the top of the flight band, projected to 2D at plan_res_m;
           cells beyond the geofence (minus fence_margin_m) are blocked too (not inflated: the go-to range stays). Unknown space counts as free: the map only
           knows what the ToF sensors have seen, so the path is re-planned while flying (replan_s) as the map fills in.
  goal     a waypoint closer than clear_m to an obstacle moves to the nearest cell that is clear (at most move_max_m).
  path     A* (8-connected) on cells at least clear_m from obstacles, with an extra cost inside soft_m so the path keeps
           its distance where there is room; then shortened by line of sight (the shortcut must stay clear_m away).
  low      boxes lower than the flight band are avoided as well (flying over a box top makes the downward rangefinder
           and the flow-EKF jump, ecps_map N4); only when no path exists that way, the plan may cross low boxes
           (strict = False in the result).

All of this runs on a worker thread (RoutePlanner.request / result); the control thread only swaps waypoint lists.
"""

from __future__ import annotations

import heapq
import math
import threading

import numpy as np

DEFAULTS = {
    "plan_res_m": 0.10,
    "clear_m": 0.45,  # centre-to-surface: prop tip 0.13 + shield d_safe 0.25 + margin
    "soft_m": 0.80,
    "soft_w": 3.0,  # extra cost per metre inside soft_m, at the clear_m edge
    "move_max_m": 1.0,
    "fence_margin_m": 0.35,
    "low_floor_m": 0.12,  # voxels above this count as obstacles in the strict (no flying over boxes) grid
    "replan_s": 1.0,
    "path_spacing_m": 0.0,  # 0 = only the corners of the shortened path
}


class RoutePlanner:
    def __init__(self, cfg: dict | None = None):
        self.cfg = dict(DEFAULTS, **(cfg or {}))
        self._lock = threading.Lock()
        self._job = None
        self._result = None
        self._ev = threading.Event()
        self._th = threading.Thread(target=self._run, name="route-plan", daemon=True)
        self._th.start()
        self.plans = 0
        self.plan_ms: list[float] = []

    # ------------------------------------------------------------------ thread interface
    def request(self, lmap, z: float, fence_m: float, start, goal, tag) -> None:
        """Plan from start (n, e) to goal (n, e) at flight height z on a snapshot of the map; tag comes back with the
        result. A newer request replaces one that has not started yet."""
        snap = (lmap.L.copy(), lmap.n, lmap.nz, lmap.res, lmap.half, lmap.l_occ, lmap.band)
        with self._lock:
            self._job = (snap, float(z), float(fence_m), tuple(start), tuple(goal), tag)
        self._ev.set()

    def result(self):
        """The latest finished plan (and forget it), or None."""
        with self._lock:
            r, self._result = self._result, None
        return r

    def _run(self) -> None:
        import time

        while True:
            self._ev.wait()
            self._ev.clear()
            with self._lock:
                job, self._job = self._job, None
            if job is None:
                continue
            t0 = time.perf_counter()
            try:
                res = self.plan(*job[:5])
            except Exception as e:  # never kill the thread: report and go on
                res = {"ok": False, "why": f"{type(e).__name__}: {e}", "path": [], "goal": job[4], "moved": 0.0, "strict": True}
            res["tag"] = job[5]
            res["ms"] = round((time.perf_counter() - t0) * 1000, 1)
            self.plans += 1
            if len(self.plan_ms) < 5000:
                self.plan_ms.append(res["ms"])
            with self._lock:
                self._result = res

    # ------------------------------------------------------------------ planning
    def grids(self, snap, z: float, fence_m: float):
        """(strict, flight-band) occupancy at plan resolution, the geofence mask (True = inside), cell size, origin."""
        L, n, nz, res, half, l_occ, band = snap
        W = L.reshape(n, n, nz)
        f = max(1, int(round(self.cfg["plan_res_m"] / res)))
        m = n // f
        k_top = min(nz, int((z + band) / res) + 1)
        k_lo = max(0, int(self.cfg["low_floor_m"] / res) + 1)
        k_band = max(0, int((z - band) / res))
        occ_all = W[: m * f, : m * f, k_lo:k_top].max(axis=2) > l_occ
        occ_band = W[: m * f, : m * f, k_band:k_top].max(axis=2) > l_occ
        cell = res * f
        pool = lambda G: G.reshape(m, f, m, f).any(axis=(1, 3))  # noqa: E731
        A, B = pool(occ_all), pool(occ_band)
        c = (np.arange(m) + 0.5) * cell - half
        R = np.hypot(c[:, None], c[None, :])
        inside = R <= fence_m - self.cfg["fence_margin_m"]
        return A, B, inside, cell, half

    def plan(self, snap, z: float, fence_m: float, start, goal) -> dict:
        from scipy.ndimage import distance_transform_edt

        strict_grid, band_grid, inside, cell, half = self.grids(snap, z, fence_m)
        last = None
        for strict, occ in ((True, strict_grid), (False, band_grid)):
            dist = distance_transform_edt(~occ) * cell  # centre-to-nearest-obstacle-cell, metres
            clear = self.cfg["clear_m"] + cell / 2
            free = (dist >= clear) & inside  # the fence bounds the cells, it is not inflated like an obstacle
            occ = occ | ~inside
            idx = lambda p: (int((p[0] + half) / cell), int((p[1] + half) / cell))  # noqa: E731
            m = occ.shape[0]
            s, g = idx(start), idx(goal)
            if not (0 <= g[0] < m and 0 <= g[1] < m):
                return {"ok": False, "why": "goal outside the map", "path": [], "goal": goal, "moved": 0.0, "strict": strict}
            g2 = self._nearest_free(free, g)
            if g2 is None:
                last = {"ok": False, "why": "no clear cell near the goal", "path": [], "goal": goal, "moved": 0.0, "strict": strict}
                continue
            goal2 = self._centre(g2, cell, half) if g2 != g else tuple(goal)
            moved = math.hypot(goal2[0] - goal[0], goal2[1] - goal[1])
            if moved > self.cfg["move_max_m"]:
                last = {"ok": False, "why": f"goal blocked (nearest clear point {moved:.2f} m away)", "path": [], "goal": goal,
                        "moved": moved, "strict": strict}
                continue
            # the drone may start inside the clearance (it is next to the obstacle): let it leave through any free cell
            s = (min(max(s[0], 0), m - 1), min(max(s[1], 0), m - 1))
            cells = self._astar(occ, dist, free, s, g2, cell)
            if cells is None:
                last = {"ok": False, "why": "no path", "path": [], "goal": goal2, "moved": moved, "strict": strict}
                continue
            pts = self._shorten(cells, free, s)
            path = [self._centre(c, cell, half) for c in pts[1:]]
            if path:
                path[-1] = goal2
            else:
                path = [goal2]
            return {"ok": True, "why": "", "path": path, "goal": goal2, "moved": moved, "strict": strict,
                    "length": self._length(start, path)}
        return last

    @staticmethod
    def _centre(c, cell: float, half: float):
        return ((c[0] + 0.5) * cell - half, (c[1] + 0.5) * cell - half)

    @staticmethod
    def _length(start, path) -> float:
        d, p = 0.0, start
        for q in path:
            d += math.hypot(q[0] - p[0], q[1] - p[1])
            p = q
        return round(d, 2)

    @staticmethod
    def _nearest_free(free: np.ndarray, g):
        if free[g]:
            return g
        ii, jj = np.nonzero(free)
        if not len(ii):
            return None
        k = int(np.argmin((ii - g[0]) ** 2 + (jj - g[1]) ** 2))
        return int(ii[k]), int(jj[k])

    def _astar(self, occ, dist, free, s, g, cell: float):
        m = occ.shape[0]
        soft, w = self.cfg["soft_m"], self.cfg["soft_w"]
        clear = self.cfg["clear_m"]
        pen = np.where(dist < soft, w * (soft - np.minimum(dist, soft)) / max(1e-6, soft - clear), 0.0) * cell
        # leaving a start inside the clearance: cells that are not free are allowed while moving away from obstacles
        start_d = dist[s]
        steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
        h = lambda c: math.hypot(c[0] - g[0], c[1] - g[1]) * cell  # noqa: E731
        best = {s: 0.0}
        prev = {}
        q = [(h(s), 0.0, s)]
        while q:
            _, gc, c = heapq.heappop(q)
            if c == g:
                out = [c]
                while c in prev:
                    c = prev[c]
                    out.append(c)
                return out[::-1]
            if gc > best.get(c, math.inf) + 1e-9:
                continue
            for di, dj, k in steps:
                nb = (c[0] + di, c[1] + dj)
                if not (0 <= nb[0] < m and 0 <= nb[1] < m) or occ[nb]:
                    continue
                if not free[nb] and not (dist[nb] >= dist[c] - 1e-9 and dist[c] < start_d + cell):
                    continue  # inside the clearance only on the way out from a start that was already inside it
                ng = gc + k * cell + pen[nb]
                if ng < best.get(nb, math.inf) - 1e-9:
                    best[nb], prev[nb] = ng, c
                    heapq.heappush(q, (ng + h(nb), ng, nb))
        return None

    @staticmethod
    def _los(free, a, b) -> bool:
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
        for t in np.linspace(0.0, 1.0, n + 1):
            c = (int(round(a[0] + (b[0] - a[0]) * t)), int(round(a[1] + (b[1] - a[1]) * t)))
            if not free[c]:
                return False
        return True

    def _shorten(self, cells, free, s):
        """Greedy line of sight: from each kept point jump to the farthest later cell it sees through clear cells. The
        first stretch (leaving a start inside the clearance) keeps the A* cells until it reaches a clear one."""
        out = [cells[0]]
        i = 0
        while i < len(cells) and not free[cells[i]]:
            i += 1
        if i > 0 and i < len(cells):
            out.append(cells[i])
        i = max(i, 0)
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1 and not self._los(free, cells[i], cells[j]):
                j -= 1
            out.append(cells[j])
            i = j
        return out
