"""Fast twin of the G4 Gazebo setup for optimisation (RL plan P0).

Everything is in the telemetry frame the pilot sees in Gazebo: x = north, y = east, z = up, yaw clockwise from north
(the upstream SimDrone uses a counter-clockwise yaw, which would make EcpsPilot's fence turn-back turn the wrong way).

Camera matches the Gazebo model (gazebo/models/ecps295_quad): 120 deg horizontal FOV, pitched 12 deg down, grey
image. Scene matches make_quad.py: two-tone 0.61 m EVA mat inside the cage, plain cardboard boxes shaded per face by
the world's sun, see-through nets (dark strands darken what is behind them) and far scenery / sky outside.
Dynamics are TwinSimDrone's per-axis first-order lags (envs.ECPS295_DYN).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

import numpy as np

from flydrones.drones.base import Drone
from flydrones.motor.command import FlightCommand
from flydrones.safety import Telemetry

CARDBOARD = 0.51  # luminance of rgb (0.65, 0.48, 0.30)
SUN = np.array([0.1, -0.5, -0.9])  # Gazebo direction (-0.5, 0.1, -0.9) in ENU -> (north, east, up)
SUN = -SUN / np.linalg.norm(SUN)  # towards the sun


@dataclass
class Obstacle:
    lo: tuple[float, float, float]  # (north, east, up)
    hi: tuple[float, float, float]
    albedo: float = CARDBOARD
    net: bool = False
    name: str = "box"
    taped: bool = False  # make_quad.py tape_bands: dark 5 cm grid at +-0.25 of each size


@dataclass
class World:
    obstacles: list[Obstacle] = field(default_factory=list)
    floor_half: float = 3.05  # EVA mat half size (cage)
    tile: float = 0.61
    tile_lo: float = 0.30
    tile_hi: float = 0.55
    ground: float = 0.5
    sky: float = 0.8
    scenery_seed: int = 3
    name: str = "world"
    floor_tape: list = field(default_factory=list)  # (north, east, length, angle_from_north) light strips, 5 cm wide

    @staticmethod
    def from_layout(path: str, **kw) -> World:
        """<world>.layout.json written by make_quad.py (ENU centres / sizes)."""
        import re
        from pathlib import Path

        lay = json.load(open(path))
        sdf = Path(str(path).replace(".layout.json", ".sdf"))
        taped, ftape = set(), []
        if sdf.exists():
            s = sdf.read_text()
            for m in re.finditer(r'<model name="([^"]+)">(.*?)</model>', s, re.S):
                if "tape_ring0" in m.group(2):
                    taped.add(m.group(1))
            if "floor_tape" in s:
                ftape = _floor_tape(6.1)
        obs = []
        for b in lay["boxes"]:
            (e, n, z), (se, sn, sz) = b["center_enu"], b["size_enu"]
            net = b["name"].startswith("net_")
            obs.append(
                Obstacle((n - sn / 2, e - se / 2, z - sz / 2), (n + sn / 2, e + se / 2, z + sz / 2), 0.16 if net else CARDBOARD, net, b["name"], b["name"] in taped)
            )
        half = max((max(abs(o.lo[0]), abs(o.hi[0])) for o in obs if o.net), default=lay["floor_m"] / 2)
        return World(obs, floor_half=half, name=lay.get("world", path), floor_tape=ftape, **kw)

    def solids(self) -> list[Obstacle]:
        return [o for o in self.obstacles if not o.net]


def _floor_tape(side: float, n: int = 10, seed: int = 7) -> list:
    """Same draws as make_quad.floor_tape (ENU x, y -> north = y, east = x; yaw from +x east)."""
    import random

    rnd = random.Random(seed)
    out = []
    for _ in range(n):
        x, y = rnd.uniform(-side / 2 + 0.3, side / 2 - 0.3), rnd.uniform(-side / 2 + 0.3, side / 2 - 0.3)
        length, ang = rnd.uniform(0.5, 1.4), rnd.uniform(0, math.pi)
        out.append((y, x, length, ang))
    return out


def _clouds(seed: int, w: int = 720, h: int = 180) -> np.ndarray:
    """Smooth value noise over (elevation 0..90 deg, azimuth 0..360 deg) in [-1, 1]: Gazebo <sky/> clouds."""
    rnd = np.random.default_rng(seed)
    out = np.zeros((h, w))
    for cell, amp in ((60, 1.0), (24, 0.5), (10, 0.25)):
        g = rnd.uniform(-1, 1, (h // cell + 2, w // cell + 2))
        yi, xi = np.arange(h) / cell, np.arange(w) / cell
        y0, x0 = yi.astype(int), xi.astype(int)
        fy, fx = (yi - y0)[:, None], (xi - x0)[None, :]
        fy, fx = fy * fy * (3 - 2 * fy), fx * fx * (3 - 2 * fx)
        a, b = g[y0][:, x0], g[y0][:, x0 + 1]
        c, d = g[y0 + 1][:, x0], g[y0 + 1][:, x0 + 1]
        out += amp * ((a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy)
    return out / 1.75


def _scenery(seed: int, n: int = 24) -> np.ndarray:
    """Horizon profile: (azimuth centre, half width, top elevation, shade) of far buildings / trees."""
    rnd = np.random.default_rng(seed)
    az = rnd.uniform(0, 2 * np.pi, n)
    r = rnd.uniform(18, 60, n)
    w = rnd.uniform(4, 22, n)
    h = rnd.uniform(4, 22, n)
    g = rnd.uniform(0.3, 0.8, n)
    return np.stack([az, np.arctan2(w / 2, r), np.arctan2(h, r), g], 1)


class TwinCamera:
    """Equidistant fisheye (r = f * theta) scaled so the image width spans hfov, as the Gazebo wideanglecamera."""

    def __init__(self, width: int = 96, height: int = 72, hfov_deg: float = 120.0, pitch_down_deg: float = 12.0,
                 noise: float = 0.007, seed: int = 0):
        self.w, self.h = width, height
        u = (np.arange(width) + 0.5) / width * 2 - 1
        v = (1 - (np.arange(height) + 0.5) / height * 2) * height / width
        U, V = np.meshgrid(u, v)
        theta = np.hypot(U, V) * math.radians(hfov_deg) / 2
        phi = np.arctan2(V, U)
        self.cf, self.cr, self.cu = np.cos(theta), np.sin(theta) * np.cos(phi), np.sin(theta) * np.sin(phi)
        self.pix = math.radians(hfov_deg) / width  # rad per pixel (centre)
        self.pitch = math.radians(pitch_down_deg)
        import os

        self.noise = float(os.environ.get("TWIN_NOISE", noise))
        self.net_strands = os.environ.get("TWIN_NETS", "1") == "1"  # ablation switches for the twin calibration
        self.cloud_amp = float(os.environ.get("TWIN_CLOUDS", 0.12))  # fitted to Gazebo frames (blank statistic)
        self.lamp_amp = float(os.environ.get("TWIN_LAMPS", 0.25))
        self.net_k = float(os.environ.get("TWIN_NET_K", 10.0))  # strand coverage gain: GPU rasterises thin strands >= ~1 px
        self.rng = np.random.default_rng(seed)
        self._scen_seed = None
        self._scen = None

    @staticmethod
    def _shade(albedo, lam):
        """Linear radiance; render() applies the display gamma. Fitted to a Gazebo frame of cage20: mat tiles
        0.30 / 0.55 -> ~0.78 / 1.0, a cardboard face away from the sun -> ~0.6."""
        return albedo * (0.55 + 1.6 * lam)

    def render(self, world: World, pos: np.ndarray, yaw: float) -> np.ndarray:
        if self._scen_seed != world.scenery_seed:
            self._scen_seed, self._scen = world.scenery_seed, _scenery(world.scenery_seed)
            self._cloud = _clouds(world.scenery_seed)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        f0 = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        right = np.array([-math.sin(yaw), math.cos(yaw), 0.0])  # yaw clockwise in (north, east)
        up0 = np.array([0.0, 0.0, 1.0])
        fwd, up = f0 * cp - up0 * sp, f0 * sp + up0 * cp
        d = self.cf[..., None] * fwd + self.cr[..., None] * right + self.cu[..., None] * up
        dx, dy, dz = d[..., 0], d[..., 1], d[..., 2]
        px, py, pz = pos
        INF = 1e9
        t_best = np.full(dx.shape, INF)
        shade = np.zeros(dx.shape)
        # floor (EVA mat inside the cage, ground outside)
        with np.errstate(divide="ignore", invalid="ignore"):
            tf = np.where(dz < -1e-6, -pz / dz, INF)
        fx, fy = px + tf * dx, py + tf * dy
        inside = (np.abs(fx) < world.floor_half) & (np.abs(fy) < world.floor_half)
        tiles = ((np.floor(fx / world.tile) + np.floor(fy / world.tile)) % 2).astype(bool)
        floor = np.where(inside, np.where(tiles, world.tile_hi, world.tile_lo), world.ground)
        for tn_, te_, ln_, ang_ in world.floor_tape:
            ca, sa = math.cos(ang_), math.sin(ang_)  # strip axis in ENU: (east, north) = (ca, sa)
            rn, re_ = fx - tn_, fy - te_
            along_ = re_ * ca + rn * sa
            across_ = -re_ * sa + rn * ca
            floor = np.where((np.abs(along_) < ln_ / 2) & (np.abs(across_) < 0.025), 0.80, floor)
        # four ceiling lamps 4.05 m up at (+-1.52, +-1.52): soft pools of light on the mat
        lamp = sum(4.05**2 / ((fx - ln) ** 2 + (fy - le) ** 2 + 4.05**2) for ln in (-1.52, 1.52) for le in (-1.52, 1.52)) / 4
        m = tf < 30.0
        shade = np.where(m, self._shade(floor, SUN[2]) * (1 - self.lamp_amp + self.lamp_amp * lamp), shade)
        t_best = np.where(m, tf, t_best)
        # far scenery and sky where nothing nearer was hit
        az = np.arctan2(dy, dx)
        el = np.arcsin(np.clip(dz, -1, 1))
        ci = np.clip((el / (np.pi / 2) * self._cloud.shape[0]).astype(int), 0, self._cloud.shape[0] - 1)
        cj = ((az % (2 * np.pi)) / (2 * np.pi) * self._cloud.shape[1]).astype(int) % self._cloud.shape[1]
        bg = np.clip(world.sky * (1 + self.cloud_amp * self._cloud[ci, cj]), 0, 1) ** 2.2
        for a0, hw, top, g in self._scen:
            da = np.abs((az - a0 + np.pi) % (2 * np.pi) - np.pi)
            bg = np.where((da < hw) & (el < top) & (el > -0.05), self._shade(g, 0.4), bg)
        shade = np.where(t_best >= INF, bg, shade)
        nets = []
        for o in world.obstacles:
            lo, hi = np.array(o.lo), np.array(o.hi)
            with np.errstate(divide="ignore", invalid="ignore"):
                t1, t2 = (lo[0] - px) / dx, (hi[0] - px) / dx
                t3, t4 = (lo[1] - py) / dy, (hi[1] - py) / dy
                t5, t6 = (lo[2] - pz) / dz, (hi[2] - pz) / dz
            ax_, ay_, az_ = np.minimum(t1, t2), np.minimum(t3, t4), np.minimum(t5, t6)
            tmin = np.maximum(np.maximum(ax_, ay_), az_)
            tmax = np.minimum(np.minimum(np.maximum(t1, t2), np.maximum(t3, t4)), np.maximum(t5, t6))
            hit = (tmax >= tmin) & (tmin > 1e-6) & (tmin < t_best)
            if o.net:
                nets.append((o, hit, tmin))
                continue
            if hit.any():
                nx = np.where(ax_ >= np.maximum(ay_, az_), -np.sign(dx), 0.0)
                ny = np.where((ay_ > ax_) & (ay_ >= az_), -np.sign(dy), 0.0)
                nz = np.where((az_ > ax_) & (az_ > ay_), -np.sign(dz), 0.0)
                lam = np.clip(nx * SUN[0] + ny * SUN[1] + nz * SUN[2], 0.0, 1.0)
                alb = np.full(dx.shape, o.albedo)
                if o.taped:
                    c, sz = (lo + hi) / 2, hi - lo
                    hp = [(p + tmin * dd - cc) / ss for p, dd, cc, ss in zip((px, py, pz), (dx, dy, dz), c, sz)]
                    band = [np.abs(np.abs(q) - 0.25) * ss < 0.025 for q, ss in zip(hp, sz)]
                    tape = ((nx != 0) & (band[1] | band[2])) | ((ny != 0) & (band[0] | band[2])) | ((nz != 0) & (band[0] | band[1]))
                    alb = np.where(tape, 0.18, alb)
                shade = np.where(hit, self._shade(alb, lam), shade)
                t_best = np.where(hit, tmin, t_best)
        # nets: 4 mm strands on a 10 cm grid, area-coverage per pixel (thin, so they alias into a fine texture)
        for o, hit, tn in nets:
            if not self.net_strands:
                shade = np.where(hit, shade * 0.9 + self._shade(o.albedo, 0.5) * 0.1, shade)
                continue
            if not hit.any():
                continue
            along = py + tn * dy if (o.hi[0] - o.lo[0]) < (o.hi[1] - o.lo[1]) else px + tn * dx
            zz = pz + tn * dz
            half_fp = np.maximum(tn * self.pix / 2, 1e-4)
            cov = np.zeros(dx.shape)
            for s in (along, zz):
                ph = np.mod(s, 0.10)
                dist = np.minimum(ph, 0.10 - ph)
                c = np.where(dist < half_fp + 0.002, np.clip(self.net_k * 0.004 / (2 * half_fp), 0, 1), 0.0)
                cov = 1 - (1 - cov) * (1 - c)
            shade = np.where(hit, shade * (1 - cov) + self._shade(o.albedo, 0.5) * cov, shade)
        img = np.clip(shade, 0, 1) ** (1 / 2.2)
        if self.noise:
            img = img + self.rng.standard_normal(img.shape) * self.noise
        return np.clip(img * 255, 0, 255).astype(np.uint8)


class TwinDrone(Drone):
    """Velocity-mode quad (ArduPilot GUIDED stand-in) with per-axis first-order lags, in the NED telemetry frame."""

    name = "twin"
    has_camera = True

    def __init__(
        self,
        world: World,
        start=(0.0, 0.0, 0.05),
        yaw_deg: float = 0.0,
        seed: int = 0,
        v_max: float = 0.5,
        vz_max: float = 0.3,
        yaw_rate_max_dps: float = 30.0,
        tau_xy: float = 0.60,
        tau_z: float = 0.35,
        tau_yaw: float = 0.12,
        wind: float = 0.03,
        cmd_delay_ticks: int = 0,
        camera: TwinCamera | None = None,
        radius: float = 0.12,
    ):
        self.world = world
        self.pos = np.array(start, dtype=float)
        self.vel = np.zeros(3)
        self.yaw = math.radians(yaw_deg)
        self.yaw_rate = 0.0  # rad/s, clockwise positive
        self.v_max, self.vz_max, self.yr_max = v_max, vz_max, math.radians(yaw_rate_max_dps)
        self.tau = np.array([tau_xy, tau_xy, tau_z])
        self.tau_yaw = tau_yaw
        self.wind = wind
        self.rng = np.random.default_rng(seed)
        import os

        ss = int(os.environ.get("TWIN_SS", 1))
        self.cam = camera or TwinCamera(96 * ss, 72 * ss, seed=seed)
        self.cam.supersample = ss
        self.queue = [FlightCommand()] * (cmd_delay_ticks + 1)
        self.flying = False
        self.t = 0.0
        self.collisions = 0
        self._touching = False
        self.r = radius
        self.battery = 100.0

    def takeoff(self) -> None:  # Gazebo run_g4: drone.takeoff() blocks until airborne; the brain does not tick
        self.flying = True

    def hover_at(self, alt: float) -> None:
        self.pos[2] = alt
        self.vel[:] = 0
        self.flying = True

    def land(self) -> None:
        self.flying = False

    def send(self, cmd: FlightCommand) -> None:
        self.queue.append(cmd)
        self.queue.pop(0)

    def telemetry(self) -> Telemetry:
        return Telemetry(
            t=self.t,
            alt_m=float(self.pos[2]),
            vz_mps=float(self.vel[2]),
            yaw_deg=math.degrees(self.yaw) % 360,
            yaw_rate_dps=math.degrees(self.yaw_rate),
            x_m=float(self.pos[0]),
            y_m=float(self.pos[1]),
            battery_pct=self.battery,
            flying=self.flying,
        )

    def frame(self) -> np.ndarray:
        img = self.cam.render(self.world, self.pos, self.yaw)
        ss = self.cam.supersample
        if ss > 1:  # Gazebo renders 640x480 and the bridge / Retina area-average it down
            h, w = img.shape
            img = img.reshape(h // ss, ss, w // ss, ss).mean(axis=(1, 3)).astype(np.uint8)
        return img

    def step(self, dt: float) -> None:
        self.t += dt
        if not self.flying:
            self.vel[:] = 0
            self.yaw_rate = 0.0
            return
        c = self.queue[0]
        f = np.array([math.cos(self.yaw), math.sin(self.yaw)])
        r = np.array([-math.sin(self.yaw), math.cos(self.yaw)])
        vxy = (c.forward * f + c.lateral * r) * self.v_max
        target = np.array([vxy[0], vxy[1], c.throttle * self.vz_max])
        k = 1 - np.exp(-dt / self.tau)
        self.vel += (target - self.vel) * k + self.rng.standard_normal(3) * self.wind * math.sqrt(dt)
        self.yaw_rate += (c.yaw * self.yr_max - self.yaw_rate) * (1 - math.exp(-dt / self.tau_yaw))
        self.yaw += self.yaw_rate * dt
        # axis by axis: a blocked axis bounces (restitution 0.2), the others keep moving -> the drone slides along a
        # box face like the Gazebo body does, instead of sticking to it while the brain still commands forward
        hit = False
        for ax in range(3):
            trial = self.pos.copy()
            trial[ax] += self.vel[ax] * dt
            if self._inside(trial):
                self.vel[ax] *= -0.2
                hit = True
            else:
                self.pos = trial
        if self.pos[2] < 0.05:
            self.pos[2], self.vel[2] = 0.05, 0.0
        if hit and not self._touching:
            self.collisions += 1
        self._touching = hit

    def _inside(self, q: np.ndarray) -> bool:
        for o in self.world.obstacles:
            lo, hi = np.array(o.lo) - self.r, np.array(o.hi) + self.r
            if np.all(q > lo) and np.all(q < hi):
                return True
        return False
