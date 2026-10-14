"""Structural fixes for the blind-side contacts (RL roadmap H1 cause analysis, 2026-10-13). EcpsPilot safety layer,
config ecps_pilot.struct, everything off by default.

ROG lowbox analysis: 77 % of contacts happen during an escape (backing 46 %, turning in place 42 %), the escape starts
a median 0.06 m from the box that is then hit, i.e. the drone runs into what the camera cannot see. Decoder parameters
cannot fix that (H2/H3). Two fixes use only what the real drone has; the third needs extra hardware:

  memory   short-term spatial memory in the (flow-EKF) estimated frame: every escape onset stores a threat point
           r_m ahead of the drone, side_deg toward the threatened side (MDN retreat: a box under / just ahead); points
           live ttl_s. A forward or backward command whose direction lies within cone_deg of a point closer than
           veto_m is replaced by hover (forward 0). Turning in place stays allowed.
  rangers  side / rear time-of-flight rangers (VL53L0X class, 10 Hz, drone.side_ranges() in the twin): no backing when
           the rear reads < rear_m; a lateral push of push x (side_m - d) / side_m away from a side reading < side_m.
  shield   (S4, route review 2026-10-13) velocity safety filter on horizontal ToF beams, control-barrier style: every
           beam i (body bearing b_i, reading d_i) bounds the commanded body velocity along it, v . u_i <= alpha (d_i -
           d_safe), so below d_safe it forces a retreat and also counters flow-EKF drift. The command is projected onto
           the half-planes; yaw and throttle pass unchanged. Beam sets: 4 single-zone rangers (front / right / rear /
           left, VL53L1X-like 27 deg cone, 10 Hz), a multi-zone front array, or an oracle ring (drone.ranges()).
           unstick_s > 0: if forward stays clamped below 20 % of the request that long, yaw toward the more open side
           (the only new intent it adds; the fence turn does the same at the geofence).
"""

from __future__ import annotations

import dataclasses
import math


class StructGuard:
    def __init__(self, cfg: dict | None):
        cfg = cfg or {}
        m = cfg.get("memory", {}) or {}
        self.mem_on = bool(m.get("enabled", False))
        self.mem_r, self.mem_side = float(m.get("r_m", 0.35)), math.radians(float(m.get("side_deg", 30.0)))
        self.mem_ttl, self.mem_veto = float(m.get("ttl_s", 20.0)), float(m.get("veto_m", 0.5))
        self.mem_cos = math.cos(math.radians(float(m.get("cone_deg", 60.0))))
        g = cfg.get("rangers", {}) or {}
        self.rng_on = bool(g.get("enabled", False))
        self.rear_m, self.side_m, self.push = float(g.get("rear_m", 0.45)), float(g.get("side_m", 0.35)), float(g.get("push", 0.6))
        sh = cfg.get("shield", {}) or {}
        self.shield = RangeShield(sh) if sh.get("enabled", False) else None
        self.points: list[tuple[float, float, float]] = []
        self._n_esc = 0
        self.vetoes = self.rear_vetoes = self.pushes = 0

    @property
    def enabled(self) -> bool:
        return self.mem_on or self.rng_on or self.shield is not None

    def filter(self, cmd, tel, decoder, drone, t: float):
        if self.shield is not None and hasattr(drone, "ranges"):
            cmd = self.shield.filter(cmd, drone, t)
        if tel.x_m is None or tel.y_m is None or tel.yaw_deg is None:
            return cmd
        x, y, h = tel.x_m, tel.y_m, math.radians(tel.yaw_deg)  # NED: x north, y east, yaw clockwise from north
        if self.mem_on:
            for _, trig, d in decoder.escapes[self._n_esc :]:
                if trig == "MDN":
                    px, py = x + 0.2 * math.cos(h), y + 0.2 * math.sin(h)
                else:  # turn right (d = +1) -> the threat is on the left
                    a = h - d * self.mem_side
                    px, py = x + self.mem_r * math.cos(a), y + self.mem_r * math.sin(a)
                self.points.append((t, px, py))
            self._n_esc = len(decoder.escapes)
            self.points = [q for q in self.points if t - q[0] < self.mem_ttl]
            if cmd.forward != 0.0 and self.points:
                s = 1.0 if cmd.forward > 0 else -1.0
                ux, uy = s * math.cos(h), s * math.sin(h)
                for _, px, py in self.points:
                    dx, dy = px - x, py - y
                    dist = math.hypot(dx, dy)
                    if dist < self.mem_veto and (dist < 1e-6 or (dx * ux + dy * uy) > self.mem_cos * dist):
                        cmd = dataclasses.replace(cmd, forward=0.0, note="struct: remembered obstacle")
                        self.vetoes += 1
                        break
        if self.rng_on and hasattr(drone, "side_ranges"):
            r = drone.side_ranges()
            if cmd.forward < 0 and r["rear"] < self.rear_m:
                cmd = dataclasses.replace(cmd, forward=0.0, note="struct: rear ranger")
                self.rear_vetoes += 1
            lat = 0.0
            if r["left"] < self.side_m:
                lat += self.push * (self.side_m - r["left"]) / self.side_m  # + = right
            if r["right"] < self.side_m:
                lat -= self.push * (self.side_m - r["right"]) / self.side_m
            if lat:
                cmd = dataclasses.replace(cmd, lateral=max(-1.0, min(1.0, cmd.lateral + lat)), note="struct: side ranger")
                self.pushes += 1
        return cmd

    def summary(self) -> dict:
        out = {"struct_vetoes": self.vetoes, "struct_rear_vetoes": self.rear_vetoes, "struct_pushes": self.pushes}
        if self.shield is not None:
            out.update(self.shield.summary())
        return out


class RangeShield:
    def __init__(self, cfg: dict):
        self.beams = tuple(float(b) for b in cfg.get("beams", (0.0, 90.0, 180.0, -90.0)))
        self.fov, self.rate, self.noise = float(cfg.get("fov_deg", 27.0)), float(cfg.get("rate_hz", 10.0)), float(cfg.get("noise_m", 0.01))
        self.d_safe, self.alpha = float(cfg.get("d_safe", 0.25)), float(cfg.get("alpha", 1.0))
        self.unstick_s = float(cfg.get("unstick_s", 0.0))
        self.u = [(math.cos(math.radians(b)), math.sin(math.radians(b))) for b in self.beams]  # body: x forward, y right
        self.clamp_ticks = self.retreat_ticks = self.unsticks = 0
        self._blocked, self._t = 0.0, None

    def filter(self, cmd, drone, t: float):
        dt = 0.05 if self._t is None else max(0.0, t - self._t)
        self._t = t
        d = drone.ranges(self.beams, self.fov, self.rate, self.noise)
        vm = float(getattr(drone, "v_max", 0.5))
        req = cmd.forward
        vx, vy = cmd.forward * vm, cmd.lateral * vm
        retreat = False
        for _ in range(3):  # cyclic projection onto the half-planes v . u_i <= alpha (d_i - d_safe)
            for (ux, uy), di in zip(self.u, d):
                lim = self.alpha * (di - self.d_safe)
                s = vx * ux + vy * uy
                if s > lim + 1e-9:
                    vx, vy = vx - (s - lim) * ux, vy - (s - lim) * uy
                    retreat |= lim < 0
        fwd, lat = max(-1.0, min(1.0, vx / vm)), max(-1.0, min(1.0, vy / vm))
        if abs(fwd - cmd.forward) > 0.04 or abs(lat - cmd.lateral) > 0.04:
            self.clamp_ticks += 1
            self.retreat_ticks += retreat
            cmd = dataclasses.replace(cmd, forward=fwd, lateral=lat, note="struct: shield")
        if self.unstick_s > 0:
            blocked = req > 0.1 and fwd < 0.2 * req
            self._blocked = self._blocked + dt if blocked and not cmd.escape else 0.0
            if self._blocked > self.unstick_s:
                left = min((di for b, di in zip(self.beams, d) if b < 0), default=2.0)
                right = min((di for b, di in zip(self.beams, d) if b > 0), default=2.0)
                cmd = dataclasses.replace(cmd, yaw=1.0 if right >= left else -1.0, note="struct: shield unstick")
                self.unsticks += 1
        return cmd

    def summary(self) -> dict:
        return {"shield_clamp_ticks": self.clamp_ticks, "shield_retreat_ticks": self.retreat_ticks, "shield_unstick_ticks": self.unsticks}
