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
           memory (S6, sensor-layout screen 2026-10-14, for cheaper layouts with blind sectors): close hits are kept
           for a few seconds as points in the estimated frame and act as extra beams; with no rear-facing beam, backing
           is allowed only along the corridor flown in the last seconds. See RangeShield._memory().
           map (S7, 2026-10-14): front / rear multi-zone (8x8) boards build a local 3D occupancy map (ecps_map); the
           shield bounds motion by the nearest mapped obstacle per sector in the flight band (passable low box tops are
           left out), in addition to any single beams. See RangeShield._map_update().
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
            cmd = self.shield.filter(cmd, drone, t, tel)
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
        fv = cfg.get("fov_deg", 27.0)
        self.fov = tuple(float(f) for f in fv) if isinstance(fv, (list, tuple)) else float(fv)  # S6: per-beam list allowed
        self.rate, self.noise = float(cfg.get("rate_hz", 10.0)), float(cfg.get("noise_m", 0.01))
        self.model = {k: float(cfg[k]) for k in ("noise_k", "max_m", "p_miss", "p_net") if k in cfg}  # S6 camera-style model
        self.d_safe, self.alpha = float(cfg.get("d_safe", 0.25)), float(cfg.get("alpha", 1.0))
        self.unstick_s = float(cfg.get("unstick_s", 0.0))
        self.u = [(math.cos(math.radians(b)), math.sin(math.radians(b))) for b in self.beams]  # body: x forward, y right
        self.clamp_ticks = self.retreat_ticks = self.unsticks = 0
        self._blocked, self._t = 0.0, None
        m = cfg.get("memory", {}) or {}
        self.mem_on = bool(m.get("enabled", False))
        self.mem_r, self.mem_ttl, self.mem_cell = float(m.get("r_m", 1.2)), float(m.get("ttl_s", 4.0)), float(m.get("cell_m", 0.05))
        self.path_s, self.path_w = float(m.get("path_s", 6.0)), float(m.get("path_w", 0.12))
        fovs = self.fov if isinstance(self.fov, tuple) else (self.fov,) * len(self.beams)
        self.hf = [math.radians(f / 2 + 3.0) for f in fovs]  # carving half-angle per beam
        self.mem_points = bool(m.get("points", True))  # False: only the backing corridor
        mp = cfg.get("map", {}) or {}
        self.map = None
        self.map_sensors: list = []
        if mp.get("enabled", False):
            from ecps_map import LocalMap, zone_dirs

            self.map = LocalMap(mp)
            for sc in mp.get("sensors", [{}]):
                self.map_sensors.append(dict(bearing=float(sc.get("bearing", 0.0)), hfov=float(sc.get("hfov", 45.0)),
                                             vfov=float(sc.get("vfov", 45.0)), n=int(sc.get("n", 8)), rate_hz=float(sc.get("rate_hz", 15.0)),
                                             noise_m=float(sc.get("noise_m", 0.01)), max_m=float(sc.get("max_m", 3.5))))
            self._map_dirs = [zone_dirs(sc["bearing"], sc["hfov"], sc["vfov"], sc["n"]) for sc in self.map_sensors]
            self._map_last = [None] * len(self.map_sensors)
        self.map_ticks = 0
        dr = cfg.get("drift", {}) or {}
        self.drift_on = bool(dr.get("enabled", False))
        self.drift_tau, self.drift_resp = float(dr.get("tau_s", 1.0)), float(dr.get("resp_s", 0.5))
        self.drift_win, self.drift_max = float(dr.get("win_s", 0.5)), float(dr.get("max_mps", 0.15))
        self._dr_hist: list = []  # (t, x, y) estimate positions over win_s
        self._vmod = (0.0, 0.0)  # modelled response to the commands sent, estimate frame
        self.w_hat = (0.0, 0.0)  # drift estimate, estimate frame
        self._cmd_sent = (0.0, 0.0)  # last command after the filter, body frame (m/s)
        self.drift_ticks = 0
        self.rear_blind = not any(abs(b) >= 135.0 for b in self.beams) and not any(abs(sc["bearing"]) >= 135.0 for sc in self.map_sensors)
        self.cells: dict = {}
        self.path: list = []
        self._last_d = None
        self.mem_ticks = self.path_ticks = 0
        self.dbg = (None, None, 0)  # debug only: nearest constraint reading, its bearing, constraints the request violates

    def filter(self, cmd, drone, t: float, tel=None):
        dt = 0.05 if self._t is None else max(0.0, t - self._t)
        self._t = t
        d = drone.ranges(self.beams, self.fov, self.rate, self.noise, **self.model) if self.beams else []
        vm = float(getattr(drone, "v_max", 0.5))
        req = cmd.forward
        vx, vy = cmd.forward * vm, cmd.lateral * vm
        cons = list(zip(self.u, d))
        mem = []
        tel_ok = tel is not None and tel.x_m is not None and tel.y_m is not None and tel.yaw_deg is not None
        if self.mem_on and tel_ok:
            mem, path = self._memory(d, tel, t)
            if any(vx * ux + vy * uy > self.alpha * (di - self.d_safe) + 1e-9 for (ux, uy), di in mem):
                self.mem_ticks += 1
            if path is not None:
                if vx * path[0][0] + vy * path[0][1] > self.alpha * (path[1] - self.d_safe) + 1e-9:
                    self.path_ticks += 1
                mem = mem + [path]
            cons += mem
        if self.map is not None and tel_ok and hasattr(drone, "zones"):
            mc = self._map_update(drone, tel, t)
            if any(vx * ux + vy * uy > self.alpha * (di - self.d_safe) + 1e-9 for (ux, uy), di in mc):
                self.map_ticks += 1
            cons += mc
            mem = mem + mc
        if cons:
            k = min(range(len(cons)), key=lambda i: cons[i][1])
            (ux, uy), dk = cons[k]
            self.dbg = (round(dk, 3), round(math.degrees(math.atan2(uy, ux)), 1),
                        sum(vx * u0 + vy * u1 > self.alpha * (di - self.d_safe) + 1e-9 for (u0, u1), di in cons))
        else:
            self.dbg = (None, None, 0)
        if self.drift_on and tel_ok:
            wbx, wby = self._drift(tel, t, dt)
            tight = [max(0.0, wbx * ux + wby * uy) for (ux, uy), _ in cons]
            if any(w > 1e-6 and vx * ux + vy * uy > self.alpha * (di - self.d_safe) - w for w, ((ux, uy), di) in zip(tight, cons)):
                self.drift_ticks += 1
            cons = [((ux, uy), di - w / self.alpha) for w, ((ux, uy), di) in zip(tight, cons)]
        retreat = False
        for _ in range(3):  # cyclic projection onto the half-planes v . u_i <= alpha (d_i - d_safe)
            for (ux, uy), di in cons:
                lim = self.alpha * (di - self.d_safe)
                s = vx * ux + vy * uy
                if s > lim + 1e-9:
                    vx, vy = vx - (s - lim) * ux, vy - (s - lim) * uy
                    retreat |= lim < 0
        fwd, lat = max(-1.0, min(1.0, vx / vm)), max(-1.0, min(1.0, vy / vm))
        if self.drift_on:
            self._cmd_sent = (fwd * vm, lat * vm)
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
                if mem:  # S6: remembered obstacles count for the side choice too
                    left = min([left] + [di for (ux, uy), di in mem if uy < -1e-6])
                    right = min([right] + [di for (ux, uy), di in mem if uy > 1e-6])
                cmd = dataclasses.replace(cmd, yaw=1.0 if right >= left else -1.0, note="struct: shield unstick")
                self.unsticks += 1
        return cmd

    def _drift(self, tel, t: float, dt: float) -> tuple:
        """Drift estimate (see the module patch note), returned in the body frame (x forward, y right)."""
        x, y, h = tel.x_m, tel.y_m, math.radians(tel.yaw_deg)
        ch, sh = math.cos(h), math.sin(h)
        self._dr_hist.append((t, x, y))
        self._dr_hist = [q for q in self._dr_hist if t - q[0] <= self.drift_win + 1e-9]
        cx, cy = self._cmd_sent
        k = 1 - math.exp(-dt / max(self.drift_resp, 1e-3)) if dt > 0 else 0.0
        self._vmod = (self._vmod[0] + (ch * cx - sh * cy - self._vmod[0]) * k, self._vmod[1] + (sh * cx + ch * cy - self._vmod[1]) * k)
        t0, x0, y0 = self._dr_hist[0]
        if t - t0 > 0.5 * self.drift_win:
            vmx, vmy = (x - x0) / (t - t0), (y - y0) / (t - t0)
            kw = 1 - math.exp(-dt / max(self.drift_tau, 1e-3)) if dt > 0 else 0.0
            wx = self.w_hat[0] + (vmx - self._vmod[0] - self.w_hat[0]) * kw
            wy = self.w_hat[1] + (vmy - self._vmod[1] - self.w_hat[1]) * kw
            n = math.hypot(wx, wy)
            if n > self.drift_max:
                wx, wy = wx * self.drift_max / n, wy * self.drift_max / n
            self.w_hat = (wx, wy)
        wx, wy = self.w_hat
        return (ch * wx + sh * wy, -sh * wx + ch * wy)

    def _memory(self, d, tel, t: float):
        """S6 short-term range memory (route review 2026-10-14) for layouts with blind sectors (front-only arrays, one
        8x8 board, the camera). Hits closer than r_m are stored as points in the (flow-EKF) estimated frame, one per
        cell_m grid cell, for ttl_s; a stored point inside a beam's cone that the beam now reads > 0.1 m farther is
        erased (free-space carving). Points within reach act as extra beams in the same barrier filter, so obstacles
        seen on the way in still bound sideways / backward motion after the drone turns. With no rear-facing beam,
        backing is allowed only along the corridor flown in the last path_s (it was free a moment ago): an extra
        constraint on the rear direction with reading = corridor length + d_safe. Drift of the estimate shifts the
        remembered points, which is why they expire quickly."""
        x, y, h = tel.x_m, tel.y_m, math.radians(tel.yaw_deg)
        ch, sh = math.cos(h), math.sin(h)
        if self.mem_points and d is not self._last_d:  # a new sensor sample
            self._last_d = d
            if self.cells:
                drop = []
                for k, (_, px, py) in self.cells.items():
                    dx, dy = px - x, py - y
                    bx, by = ch * dx + sh * dy, -sh * dx + ch * dy
                    r = math.hypot(bx, by)
                    a = math.atan2(by, bx)
                    for b, hf, di in zip(self.beams, self.hf, d):
                        if abs((a - math.radians(b) + math.pi) % (2 * math.pi) - math.pi) <= hf and r - 0.06 < di - 0.10:
                            drop.append(k)
                            break
                for k in drop:
                    del self.cells[k]
            for (ux, uy), di in zip(self.u, d):
                if di < self.mem_r:
                    r = di + 0.06
                    px, py = x + r * (ch * ux - sh * uy), y + r * (sh * ux + ch * uy)
                    self.cells[(round(px / self.mem_cell), round(py / self.mem_cell))] = (t, px, py)
        if not self.path or t - self.path[-1][0] >= 0.1:
            self.path.append((t, x, y))
            self.path = [q for q in self.path if t - q[0] <= self.path_s]
            self.cells = {k: v for k, v in self.cells.items() if t - v[0] <= self.mem_ttl}
        mem = []
        for _, px, py in self.cells.values():
            dx, dy = px - x, py - y
            bx, by = ch * dx + sh * dy, -sh * dx + ch * dy
            r = math.hypot(bx, by)
            if 1e-6 < r < 0.9:  # farther points can never bind at v_max 0.5, alpha 1
                mem.append(((bx / r, by / r), r - 0.06))
        path = None
        if self.rear_blind:
            s = 0.0
            for _, px, py in self.path:
                dx, dy = px - x, py - y
                al = -(ch * dx + sh * dy)  # along the body-rear direction
                if al > s and abs(-sh * dx + ch * dy) < self.path_w:
                    s = al
            path = ((-1.0, 0.0), s + self.d_safe)
        return mem, path

    def _map_update(self, drone, tel, t: float) -> list:
        """S7: feed every new multi-zone frame into the local map at the estimated pose (yaw only: the frame is taken as
        level), then return the nearest mapped obstacle per sector in the flight band as extra beams (reading = centre
        distance - 0.06, the beam convention)."""
        import numpy as np

        x, y, h = tel.x_m, tel.y_m, math.radians(tel.yaw_deg)
        z = tel.alt_m if getattr(tel, "alt_m", None) is not None else 0.8
        ch, sh = math.cos(h), math.sin(h)
        for i, sc in enumerate(self.map_sensors):
            r = drone.zones(**sc)
            if r is None or r is self._map_last[i]:
                continue
            self._map_last[i] = r
            B = self._map_dirs[i]
            D = np.stack([ch * B[:, 0] - sh * B[:, 1], sh * B[:, 0] + ch * B[:, 1], B[:, 2]], axis=1)
            b = h + math.radians(sc["bearing"])
            self.map.integrate((x + 0.06 * math.cos(b), y + 0.06 * math.sin(b), z), D, np.asarray(r, float), sc["max_m"], t)
        return [(u, di - 0.06) for u, di in self.map.sectors(x, y, z, h)]

    def summary(self) -> dict:
        out = {"shield_clamp_ticks": self.clamp_ticks, "shield_retreat_ticks": self.retreat_ticks, "shield_unstick_ticks": self.unsticks}
        if self.mem_on:
            out.update({"shield_mem_ticks": self.mem_ticks, "shield_path_ticks": self.path_ticks})
        if self.map is not None:
            out.update({"shield_map_ticks": self.map_ticks, "shield_map_frames": self.map.frames})
        return out
