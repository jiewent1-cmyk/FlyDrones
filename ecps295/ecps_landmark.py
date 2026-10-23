"""Landmark position fix (2026-10-22): ArUco markers at fixed places correct the optical-flow EKF's drift.

Without GPS the FC's horizontal position is integrated optical flow: it drifts (Gazebo hover: 7-8 cm still after the
position hold, more over a flight). A marker that does not move is a fixed point: the EKF position of the drone plus the
detector's range and bearing puts the marker at m_ekf; if its true place is L, the EKF has drifted by
    offset = m_ekf - L      (EKF frame minus world frame, north / east)
The companion does not touch the FC's EKF (an external position that comes and goes with the marker in view would make
the EKF switch sources); it applies the offset itself:
  - EcpsPilot sees the corrected pose (EKF - offset): waypoints, geofence, return home and target tracking all work in
    the world frame;
  - the hover hold (mavlink_twin ECPS_HOLD=pos) keeps its anchor in the world frame and sends anchor + offset as the
    EKF target, so a drift the marker reveals moves the target and the FC flies the drone back.
L comes from the config (positions: {id: [north, east]}) or, with auto_register, from the first register_n fixes of a
marker (their spread must stay below register_spread_m): from then on drift is measured against where it was first seen.
Fixes are gated: range <= max_range_m, |bearing| <= max_bearing_deg (lens edge), marker seen not too obliquely, and a fix
farther than gate_m from the current offset needs confirm_n in a row (a moved marker re-registers after that).
"""

from __future__ import annotations

import collections
import math


class LandmarkFix:
    def __init__(self, cfg: dict | None = None):
        c = cfg or {}
        self.enabled = bool(c.get("enabled", False))
        ids = c.get("ids", "all")
        self.ids = None if ids in (None, "all") else {int(i) for i in ids}
        self.known: dict[int, tuple[float, float]] = {int(k): (float(v[0]), float(v[1])) for k, v in (c.get("positions") or {}).items()}
        self._cfg_ids = set(self.known)  # surveyed: never re-registered
        self.auto = bool(c.get("auto_register", True))
        self.register_n = int(c.get("register_n", 10))
        self.register_spread = float(c.get("register_spread_m", 0.15))
        self.max_range = float(c.get("max_range_m", 3.0))
        self.min_range = float(c.get("min_range_m", 0.4))
        self.max_bearing = float(c.get("max_bearing_deg", 30.0))
        self.max_facing = float(c.get("max_facing_deg", 60.0))
        self.alpha = float(c.get("alpha", 0.15))  # EMA weight of a fix
        self.gate = float(c.get("gate_m", 0.4))
        self.confirm_n = int(c.get("confirm_n", 5))
        self.max_offset = float(c.get("max_offset_m", 1.5))  # beyond this the offset is not trusted (reset)
        self.offset = (0.0, 0.0)  # EKF minus world, north / east
        self.fixes = 0
        self.rejected = 0
        self.last_fix_t: float | None = None
        self._pending: dict[int, list[tuple[float, float]]] = collections.defaultdict(list)
        self._out_n = 0
        self._used: dict[int, float] = {}
        self._poses: collections.deque = collections.deque(maxlen=60)  # (monotonic, x, y, yaw_deg)
        self.log: list[tuple[float, str]] = []
        self.source = None  # MarkerThread: latest_all() -> ([Detection], stamp)

    # ------------------------------------------------------------------ helpers
    def _pose_at(self, stamp: float):
        best = None
        for p in self._poses:
            if best is None or abs(p[0] - stamp) < abs(best[0] - stamp):
                best = p
        return best if best is not None and abs(best[0] - stamp) < 0.25 else None

    def _note(self, t: float, what: str, on_event=None) -> None:
        self.log.append((round(t, 2), what))
        if on_event is not None:
            on_event(t, what)

    def correct(self, x: float, y: float) -> tuple[float, float]:
        """EKF position -> world position."""
        return x - self.offset[0], y - self.offset[1]

    # ------------------------------------------------------------------ per tick
    def update(self, now: float, x: float, y: float, yaw_deg: float, t: float, on_event=None) -> bool:
        """Fold the new detections in; True when the offset moved. now = monotonic (the detections' clock), x / y / yaw =
        the raw EKF pose of this tick, t = flight time (for the log)."""
        if not self.enabled or self.source is None or x is None or y is None or yaw_deg is None:
            return False
        self._poses.append((now, x, y, yaw_deg))
        get = getattr(self.source, "latest_all", None)
        dets, _ = get() if get is not None else ([self.source.latest()[0]] if self.source.latest()[0] else [], 0.0)
        moved = False
        for d in dets or []:
            if d is None or (self.ids is not None and d.marker_id not in self.ids):
                continue
            if d.stamp <= self._used.get(d.marker_id, 0.0) or now - d.stamp > 0.5:
                continue
            self._used[d.marker_id] = d.stamp
            if not (self.min_range <= d.range_m <= self.max_range and abs(d.bearing_deg) <= self.max_bearing
                    and getattr(d, "facing_deg", 0.0) <= self.max_facing):
                self.rejected += 1
                continue
            p = self._pose_at(d.stamp)
            if p is None:
                continue
            _, px, py, pyaw = p
            a = math.radians(pyaw + d.bearing_deg)
            mn, me = px + d.range_m * math.cos(a), py + d.range_m * math.sin(a)  # marker in the EKF frame
            L = self.known.get(d.marker_id)
            if L is None:
                if self.auto:
                    self._register(d.marker_id, mn - self.offset[0], me - self.offset[1], t, on_event)
                continue
            on, oe = mn - L[0], me - L[1]
            if math.hypot(on - self.offset[0], oe - self.offset[1]) > self.gate and self.fixes > 0:
                self._out_n += 1
                self.rejected += 1
                if self._out_n < self.confirm_n:
                    continue
                if d.marker_id not in self.known_cfg_ids():  # an auto-registered marker that moved: learn it again
                    del self.known[d.marker_id]
                    self._pending.pop(d.marker_id, None)
                    self._note(t, f"landmark {d.marker_id} moved -> registering again", on_event)
                    self._out_n = 0
                    continue
            self._out_n = 0
            k = self.alpha if self.fixes > 0 else 1.0
            o = (self.offset[0] + k * (on - self.offset[0]), self.offset[1] + k * (oe - self.offset[1]))
            if math.hypot(*o) > self.max_offset:
                self._note(t, f"landmark offset {math.hypot(*o):.2f} m > {self.max_offset:.1f} m -> reset", on_event)
                self.offset, self.fixes = (0.0, 0.0), 0
                continue
            self.offset = o
            self.fixes += 1
            self.last_fix_t = t
            moved = True
            if self.fixes == 1:
                self._note(t, f"landmark {d.marker_id}: first fix, offset {o[0]:+.2f}, {o[1]:+.2f}", on_event)
        return moved

    def known_cfg_ids(self) -> set[int]:
        return self._cfg_ids

    def _register(self, mid: int, wn: float, we: float, t: float, on_event) -> None:
        pts = self._pending[mid]
        pts.append((wn, we))
        if len(pts) < self.register_n:
            return
        mn = sum(p[0] for p in pts) / len(pts)
        me = sum(p[1] for p in pts) / len(pts)
        spread = max(math.hypot(p[0] - mn, p[1] - me) for p in pts)
        if spread > self.register_spread:
            pts.pop(0)  # still settling (or a bad view): slide the window
            return
        self.known[mid] = (mn, me)
        self._pending.pop(mid, None)
        self._note(t, f"landmark {mid} registered at {mn:+.2f}, {me:+.2f} (spread {spread:.2f} m)", on_event)

    def summary(self) -> dict:
        return {"enabled": self.enabled, "known": {k: [round(v[0], 3), round(v[1], 3)] for k, v in self.known.items()},
                "offset": [round(self.offset[0], 3), round(self.offset[1], 3)], "fixes": self.fixes,
                "rejected": self.rejected, "log": self.log[-50:]}
