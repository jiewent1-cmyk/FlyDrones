"""Ventral obstacle cue from the downward ToF of the 3901-L0X (TechRoute §10 S7d, MiniFly v4 LCv cells).

Without GPS the EKF height comes from the downward ToF (or, with baro only, is coupled to it through the optical
flow terrain state). Flying low over a box shortens the range by the box height; the EKF reads it as the drone
sinking, ArduPilot climbs, and in Gazebo the drone crossed the 1.3 m hard fence and landed. VentralCue turns
"something under me" into a level for the brain, like a looming cue, so it brakes and backs off the obstacle.

Two parts, the level is the larger one:
- step: the range shortens by >= `drop_m` within `window_s`, faster than `min_rate_mps` (a box edge under the
  centre of the cone; the drone's own climb and sink are limited to vz_max 0.5 m/s). Immediate.
- rise: surface height T = baro height - range. The drone's own climb or sink moves both terms together, so T only
  changes when the surface under the drone does, abrupt or not: a box edge entering the 27 deg cone from the side
  shortens the min-of-cone range gradually, at the speed of the drone, and the step test misses it (offline check on
  the S7d runs: 22 of 48 crossings). T is low-passed (`smooth_s`) against baro noise and compared with a slow
  baseline (`baseline_s`) that only adapts while nothing is under the drone. The EKF height is NOT usable here:
  it is the quantity the box corrupts.
"""

from __future__ import annotations

from collections import deque


class VentralCue:
    def __init__(
        self,
        drop_m: float = 0.12,
        min_rate_mps: float = 0.8,
        window_s: float = 0.3,
        step_hold_s: float = 1.0,
        rise_m: float = 0.12,
        rise_span_m: float = 0.15,
        smooth_s: float = 0.4,
        baseline_s: float = 6.0,
        decay_s: float = 0.5,
        range_valid: tuple[float, float] = (0.05, 1.15),
    ):
        self.drop_m, self.min_rate, self.window_s, self.step_hold_s = drop_m, min_rate_mps, window_s, step_hold_s
        self.rise_m, self.rise_span, self.smooth_s, self.baseline_s = rise_m, rise_span_m, smooth_s, baseline_s
        self.decay_s = decay_s
        self.range_valid = range_valid
        self._hist: deque[tuple[float, float]] = deque()
        self._step_t: float | None = None
        self._tf: float | None = None  # low-passed surface height
        self._tb: float | None = None  # baseline surface height
        self.level = self.step = self.rise = 0.0
        self.events: list[tuple[float, str, float]] = []

    def height_above_floor(self, baro_m: float | None) -> float | None:
        """Baro height minus the floor's surface baseline. The baseline is frozen while something is under the drone,
        so this stays a height above the FLOOR over a box, independent of the flow EKF that the box corrupts."""
        if baro_m is None or self._tb is None:
            return None
        return baro_m - self._tb

    def _fast_drop(self, t: float, r: float) -> float:
        best = 0.0
        for t0, r0 in self._hist:
            dt = t - t0
            if 0 < dt <= self.window_s:
                d = r0 - r
                if d >= self.drop_m and d / dt >= self.min_rate and d > best:
                    best = d
        return best

    def update(self, t: float, range_m: float | None, baro_m: float | None, dt: float) -> float:
        ok = range_m is not None and range_m == range_m and self.range_valid[0] <= range_m <= self.range_valid[1]
        if ok:
            while self._hist and t - self._hist[0][0] > self.window_s:
                self._hist.popleft()
            d = self._fast_drop(t, range_m)
            if d > 0:
                if self._step_t is None or t - self._step_t > self.step_hold_s:
                    self.events.append((round(t, 2), "step", round(d, 3)))
                self._step_t = t
                self._hist.clear()
            self._hist.append((t, range_m))
        self.step = 1.0 if self._step_t is not None and t - self._step_t <= self.step_hold_s else 0.0

        if ok and baro_m is not None:
            surf = baro_m - range_m
            a = min(1.0, dt / self.smooth_s)
            self._tf = surf if self._tf is None else (1 - a) * self._tf + a * surf
            if self._tb is None:
                self._tb = self._tf
            rise = self._tf - self._tb
            new = min(1.0, max(0.0, (rise - self.rise_m) / self.rise_span + 0.5)) if rise > self.rise_m else 0.0
            if new > 0 and self.rise == 0:
                self.events.append((round(t, 2), "rise", round(rise, 3)))
            self.rise = new
            if self.rise == 0 and self.step == 0:  # adapt only while nothing is under the drone
                b = min(1.0, dt / self.baseline_s)
                self._tb = (1 - b) * self._tb + b * self._tf
        elif not ok:
            self.rise = max(0.0, self.rise - dt / self.decay_s)  # out of range: no evidence, fade
        target = max(self.step, self.rise)
        self.level = target if target >= self.level else max(target, self.level - dt / self.decay_s)
        return self.level
