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
        self.gain = 1.0  # D low light: exposure gain on linear radiance (Gazebo: lower light diffuse / camera gain)
        self.flat = False  # D textureless: every solid face one albedo, no sun shading, plain mat, no tape
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
        if self.flat:
            floor = np.where(inside, (world.tile_hi + world.tile_lo) / 2, world.ground)
        for tn_, te_, ln_, ang_ in ([] if self.flat else world.floor_tape):
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
                if self.flat:
                    lam = np.full(dx.shape, 0.5)
                alb = np.full(dx.shape, o.albedo)
                if o.taped and not self.flat:
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
        if self.gain != 1.0:
            shade = shade * self.gain
        img = np.clip(shade, 0, 1) ** (1 / 2.2)
        if self.noise:
            img = img + self.rng.standard_normal(img.shape) * self.noise
        return np.clip(img * 255, 0, 255).astype(np.uint8)


def flow_dr(seed: int) -> dict:
    """H1 domain randomisation of the flow-EKF model (2026-10-08): the S8 lowbox calibration could not pin one setting
    (Gazebo itself cannot rank the models on contacts with 30 worlds), so every episode draws its estimator from the
    region that reproduced the reliable metrics: terrain lag 0.5-4 s (log-uniform), random walk 0.02-0.06, cruise
    0.8 +- 0.1 m."""
    r = np.random.default_rng(70_000 + seed)
    return {"nav": "flow", "tau_terr": float(np.exp(r.uniform(np.log(0.5), np.log(4.0)))), "flow_rw": float(r.uniform(0.02, 0.06)),
            "flow_bias": 0.94, "hover_alt": 0.8, "hover_jitter": 0.1}


def flow_dr2(seed: int, fc: str = "default") -> dict:
    """N4 twin v2 (2026-10-15): downward ToF as the Gazebo 5 x 5 ray cone (min), EKF terrain state as ArduPilot's
    EstimateTerrainOffset with the flight controller's EK3_TERR_GRAD / EK3_RNG_M_NSE (fc "default" 0.1 / 0.5, "tf10"
    10 / 0.1), flow random walk 0.005-0.015 and scale 0.94-1.0 (Gazebo g7_w2map on lowbox_s0-19: max error 0.28 m, error
    growth away from boxes 0.008 m/s; twin with 0.01 / 1.0: 0.33 m, 0.007 m/s), P66 0.002-0.005 (fitted on 9 Gazebo logs)."""
    r = np.random.default_rng(71_000 + seed)
    g, n = {"default": (0.1, 0.5), "tf10": (10.0, 0.1)}[fc]
    return {"nav": "flow", "ventral": "rays", "terr_model": "ekf3", "terr_grad": g, "rng_m_nse": n,
            "terr_p66": float(r.uniform(0.002, 0.005)), "flow_rw": float(r.uniform(0.005, 0.015)),
            "flow_bias": float(r.uniform(0.94, 1.0)), "hover_alt": 0.8, "hover_jitter": 0.1}


V3_MOTION = {"tau_xy": 0.40, "tau_dec": 0.25, "cmd_delay_ticks": 3, "jit0": 0.04, "jit_yaw": 0.12, "jit_tau": 0.5}


V3_PRESETS = {"full": {}, "det": {"jit0": 0.0, "jit_yaw": 0.0}, "half": {"jit0": 0.02, "jit_yaw": 0.06}}


def flow_dr3(seed: int, fc: str = "default") -> dict:
    """twin v3 (2026-10-16): flow_dr2 + the motion model fitted on sim-paced Gazebo (V3_MOTION); env TWIN_V3 picks a
    preset (full = fitted on speeds, which over-states contacts ~10x; det = no velocity jitter; half)."""
    import os

    return {**flow_dr2(seed, fc), **V3_MOTION, **V3_PRESETS[os.environ.get("TWIN_V3", "full")]}


OOD_DRIFT, OOD_GUST = 0.03, 0.10  # wind: steady drift (m/s) left by GUIDED, gust noise (default wind 0.03); v4 on
# lowbox_s0-19 (2/20 runs with contact in distribution): drift 0.12 / 0.06 / 0.03 / 0 -> 20 / 19 / 12 / 5, so 0.03
OOD_MODES = {
    "lowlight": "camera gain x0.3 on linear radiance, image noise 0.007 -> 0.02 (dim room, high camera gain)",
    "notex": "textureless: one albedo per box, flat (unlit) faces, no tape, plain mat, no floor strips",
    "halfcam": "left or right half of the image covered (dark, value 0.08), side drawn per episode seed",
    "wind": "steady drift OOD_DRIFT m/s from a per-seed direction + gusts OOD_GUST (default 0.03) m/s/sqrt(s)",
    "tof": "downward ToF faults: per s a 10 % chance of a 1-4 s stuck (stale) reading, plus 3 % out-of-range samples"
           " (1.2 m); the flow-EKF scale uses the same faulty range",
    "ghost": "boxes not rendered but still collide (garyb9 ghost check: is the avoidance seen?)",
}


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
        nav: str = "truth",
        flow_bias: float = 0.94,
        tau_terr: float = 1.5,
        flow_rw: float = 0.02,
        tof_hz: float = 10.0,
        hover_alt: float | None = None,
        hover_jitter: float = 0.0,
        ood: str | None = None,
        ventral: str = "box",
        terr_model: str = "lag",
        terr_grad: float = 0.1,
        rng_m_nse: float = 0.5,
        terr_p66: float = 0.003,
        tau_dec: float | None = None,
        jit0: float = 0.0,
        jit_yaw: float = 0.0,
        jit_tau: float = 0.5,
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
        # nav="flow" (RL roadmap H1): the real drone has no GPS; EKF3 velocity = optical flow x height above terrain,
        # height from the companion (route B, ~truth). Over a box the flow sensor sees a closer surface while the EKF
        # terrain estimate lags, so the speed is over-estimated (Gazebo S8 logs: x1.2-1.7 at range/height 0.4-0.75);
        # GUIDED then flies the true drone slower and the position estimate runs ahead. Telemetry x/y = estimate.
        self.nav, self.flow_bias, self.tau_terr, self.flow_rw = nav, flow_bias, tau_terr, flow_rw
        self.est = np.array(start[:2], dtype=float)
        self.terr = 0.0  # EKF terrain height estimate
        self.tof_dt, self._tof_t, self._tof = 1.0 / tof_hz, -1.0, None
        self.nrng = np.random.default_rng(50_000 + seed)
        self._seed, self._side_t, self._side, self._srng = seed, -1.0, None, None  # side_ranges(), built on first use
        # flow + route B in Gazebo cruises higher than the GPS runs (S8 lowbox: median 0.81 m, p10-p90 0.62-0.98 m)
        self.hover_alt = None if hover_alt is None else hover_alt + float(np.random.default_rng(60_000 + seed).uniform(-hover_jitter, hover_jitter))
        # D stage (out of distribution); own RNG streams so the in-distribution path is untouched
        assert ood in (None, *OOD_MODES), ood
        self.ood = ood
        # N4 (2026-10-15) fidelity options, defaults = the old model:
        #  ventral="rays": the downward ToF reads the nearest of 5 x 5 rays across its 27 deg cone, like the Gazebo
        #    gpu_lidar reduced by ArduPilotPlugin RangeCb (min over the scan). Approaching a box the reading ramps down
        #    along the box face instead of jumping to the top ("box": vertical distance to the highest top inside the
        #    cone footprint), so the ventral step fires later, as in Gazebo (backward escapes near low boxes start
        #    0.15 m from the edge there, 0.24 m in the old twin, which therefore almost never flies over a box).
        #  terr_model="ekf3": the EKF terrain state is ArduPilot 4.7 NavEKF3_core::EstimateTerrainOffset (1-state filter on
        #    the 10 Hz range, median of 3 at 20 Hz; process noise = moved distance^2 * EK3_TERR_GRAD^2 + dt^2 * P66,
        #    noise EK3_RNG_M_NSE) instead of a first-order lag; offline it reproduces the logged Gazebo state to ~1 cm.
        assert ventral in ("box", "rays") and terr_model in ("lag", "ekf3"), (ventral, terr_model)
        self.ventral, self.terr_model = ventral, terr_model
        self.terr_grad, self.rng_m_nse, self.terr_p66 = terr_grad, rng_m_nse, terr_p66
        self._rt = (None, 0.0)  # _range_rays cache (position / yaw key, value)
        self._tk = {"t": -1.0, "P": 0.0036, "xy": None, "buf": [], "s_t": -1.0, "s_v": None, "rng": np.random.default_rng(88_000 + seed)}
        if ventral == "rays":
            a = np.radians(np.linspace(-13.5, 13.5, 5))
            H, V = np.meshgrid(a, a)
            # gpu_lidar ray (cos v cos h, cos v sin h, sin v) in the sensor frame, sensor pitched 90 deg down (x -> -z)
            self._vrays = np.stack([np.sin(V).ravel(), (np.cos(V) * np.sin(H)).ravel(), -(np.cos(V) * np.cos(H)).ravel()], axis=1)
        # v3 (2026-10-16, sim-paced Gazebo g7_sc2 lowbox_s0-19): GUIDED stops faster than it starts (tau_dec), and the
        # vehicle wanders: rms speed 0.07 m/s with no command, ~0.2 m/s while yawing at > 10 deg/s (random direction;
        # the EKF sees it too, so it is not an estimation error). Modelled as a world-frame OU velocity offset added to
        # the target, per-axis sd jit0 + jit_yaw * min(|yaw rate| / 20 deg/s, 1), correlation time jit_tau.
        self.tau_dec, self.jit0, self.jit_yaw, self.jit_tau = tau_dec, jit0, jit_yaw, jit_tau
        self._jit, self._jrng = np.zeros(2), np.random.default_rng(90_000 + seed)
        self.view = world  # what the camera sees
        orng = np.random.default_rng(80_000 + seed)
        self.drift = np.zeros(3)
        self.half = None
        self._faults = []
        if ood == "lowlight":
            self.cam.gain, self.cam.noise = 0.3, 0.02
        elif ood == "notex":
            self.cam.flat = True
        elif ood == "halfcam":
            self.half = "L" if orng.random() < 0.5 else "R"
        elif ood == "wind":
            a = orng.uniform(0, 2 * math.pi)
            self.drift = OOD_DRIFT * np.array([math.cos(a), math.sin(a), 0.0])
            self.wind = OOD_GUST
        elif ood == "tof":
            t = 0.0
            while t < 600.0:  # fault windows (start, end), one draw per second
                if orng.random() < 0.10:
                    d = orng.uniform(1.0, 4.0)
                    self._faults.append((t, t + d))
                    t += d
                t += 1.0
            self._orng = orng
        elif ood == "ghost":
            import copy

            self.view = copy.copy(world)
            self.view.obstacles = [o for o in world.obstacles if o.net]

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

    def _range_true(self) -> float:
        """Downward ToF (VL53L0X, ~27 deg cone): distance to the highest surface inside the cone footprint."""
        if self.ventral == "rays":
            return self._range_rays()
        n, e, a = self.pos
        rad = max(a, 0.05) * 0.24
        top = 0.0
        for o in self.world.obstacles:
            if o.net or o.hi[2] >= a:
                continue
            if o.lo[0] - rad < n < o.hi[0] + rad and o.lo[1] - rad < e < o.hi[1] + rad:
                top = max(top, o.hi[2])
        return a - top

    def _range_rays(self) -> float:
        """N4 ventral="rays": nearest of the 25 cone rays (slant range to the first box face / top, net or the floor), as
        the Gazebo 5 x 5 gpu_lidar scan reduced by ArduPilotPlugin.RangeCb (min). Sensor 0.045 m behind and 0.015 m below
        the centre (gazebo/make_quad.py). Cached per position / yaw."""
        key = (self.pos[0], self.pos[1], self.pos[2], self.yaw)
        if self._rt[0] == key:
            return self._rt[1]
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        B = self._vrays
        D = np.stack([cy * B[:, 0] - sy * B[:, 1], sy * B[:, 0] + cy * B[:, 1], B[:, 2]], axis=1)
        o = self.pos + np.array([-0.045 * cy, -0.045 * sy, -0.015])
        v = float(min(np.min(self._rays3(o, D)), 2.0))
        self._rt = (key, v)
        return v

    def _terr_ekf3(self) -> None:
        """N4 terr_model="ekf3": ArduPilot 4.7 NavEKF3_core::EstimateTerrainOffset on the downward range (z up here;
        the EKF height is the companion's floor height ~ truth, the moved distance comes from the EKF position)."""
        k = self._tk
        if k["s_t"] < 0 or self.t - k["s_t"] >= self.tof_dt - 1e-9:  # its own 10 Hz sample of the sensor
            k["s_t"] = self.t
            k["s_v"] = float(np.clip(self._range_true() + k["rng"].normal(0, 0.01), 0.03, 1.2))
        if k["t"] >= 0 and self.t - k["t"] < 0.05 - 1e-9:  # readRangeFinder stores at 20 Hz
            return
        dt = 0.05 if k["t"] < 0 else self.t - k["t"]
        k["t"] = self.t
        k["buf"] = (k["buf"] + [k["s_v"]])[-3:]
        meas = max(float(np.median(k["buf"])), 0.05)
        if k["xy"] is not None:
            d2 = min(float(np.sum((self.est - k["xy"]) ** 2)), 100.0)
            k["P"] += d2 * self.terr_grad ** 2 + dt * dt * self.terr_p66
        k["xy"] = self.est.copy()
        innov = max(self.pos[2] - self.terr, 0.05) - meas
        var = self.rng_m_nse + k["P"]
        if innov * innov / (25.0 * var) < 1.0:  # EK3_RNG_I_GATE 500
            self.terr += k["P"] / var * innov
            k["P"] = max(k["P"] - k["P"] ** 2 / var, 0.0)
        self.terr = min(self.terr, self.pos[2] - 0.05)

    def rangefinder_m(self) -> float | None:
        if self.t - self._tof_t >= self.tof_dt - 1e-9:  # 10 Hz sample and hold
            self._tof_t = self.t
            v = float(np.clip(self._range_true() + self.nrng.normal(0, 0.01), 0.03, 1.2))
            if self.ood == "tof":
                if any(a <= self.t < b for a, b in self._faults):
                    v = self._tof if self._tof is not None else v  # stuck: keep the stale reading
                elif self._orng.random() < 0.03:
                    v = 1.2  # out of range
            self._tof = v
        return self._tof

    def _ray(self, ang: float, skip: frozenset = frozenset()) -> float:
        """Horizontal distance from the body centre to the first obstacle (box or net) at flight height, max 2 m.
        `skip`: obstacle names this ray does not see (S6 camera model: nets it fails to segment)."""
        n, e, a = self.pos
        dn, de = math.cos(ang), math.sin(ang)
        best = 2.0
        for o in self.world.obstacles:
            if not o.lo[2] <= a <= o.hi[2] or o.name in skip:
                continue
            t0, t1 = 0.0, best
            for p, d, lo, hi in ((n, dn, o.lo[0], o.hi[0]), (e, de, o.lo[1], o.hi[1])):
                if abs(d) < 1e-9:
                    if not lo <= p <= hi:
                        t0 = math.inf
                        break
                else:
                    u, v = (lo - p) / d, (hi - p) / d
                    t0, t1 = max(t0, min(u, v)), min(t1, max(u, v))
            if t0 <= t1:
                best = min(best, t0)
        return best

    def side_ranges(self) -> dict:
        """Structural fix S3 (ecps_struct rangers): rear / left / right VL53L0X-class rangers on the frame edge
        (0.06 m from the centre), 10 Hz sample and hold, 1 cm noise, own RNG stream."""
        if self._side is None or self.t - self._side_t >= self.tof_dt - 1e-9:
            if self._srng is None:
                self._srng = np.random.default_rng(85_000 + self._seed)
            self._side_t = self.t
            self._side = {k: float(np.clip(self._ray(self.yaw + off) - 0.06 + self._srng.normal(0, 0.01), 0.0, 2.0))
                          for k, off in (("rear", math.pi), ("left", -math.pi / 2), ("right", math.pi / 2))}
        return self._side

    def ranges(self, beams: tuple, fov_deg=0.0, rate_hz: float = 10.0, noise_m: float = 0.01, **model) -> list[float]:
        """S4 shield (ecps_struct, route review 2026-10-13): horizontal ToF beams at body bearings `beams` (deg, 0 = nose,
        + = right), sensor 0.06 m from the centre. A beam with fov_deg > 0 reads the nearest of 3 rays across its cone
        (single-zone VL53L1X ~27 deg); sample and hold at rate_hz, Gaussian noise from its own RNG stream per beam set,
        so side_ranges() and the flow / ToF noise are untouched.
        S6 (sensor-layout screen, 2026-10-14): fov_deg may be a per-beam list (mixed sensors on one spec), and `model`
        adds a camera-style range model (ground-contact-line ranging), see _ranges_model(). Without either the readings
        are bit-identical to before."""
        if not hasattr(self, "_beams"):
            self._beams = {}
        fk = tuple(float(f) for f in fov_deg) if isinstance(fov_deg, (list, tuple)) else fov_deg
        key = (tuple(beams), fk, rate_hz, noise_m, tuple(sorted(model.items())))
        st = self._beams.get(key)
        if st is None:
            st = self._beams[key] = [-1.0, None, np.random.default_rng(86_000 + self._seed)]
        if st[1] is None or self.t - st[0] >= 1.0 / rate_hz - 1e-9:
            st[0] = self.t
            if model or isinstance(fk, tuple):
                st[1] = self._ranges_model(beams, fk, noise_m, st, **model)
            else:
                h = math.radians(fov_deg) / 2
                offs = (-h, 0.0, h) if h > 0 else (0.0,)
                st[1] = [float(np.clip(min(self._ray(self.yaw + math.radians(b) + o) for o in offs) - 0.06
                                       + (st[2].normal(0, noise_m) if noise_m > 0 else 0.0), 0.0, 2.0)) for b in beams]
        return st[1]

    def _ranges_model(self, beams, fov, noise_m: float, st: list, noise_k: float = 0.0, max_m: float = 2.0,
                      p_miss: float = 0.0, p_net: float = 1.0) -> list[float]:
        """S6 range model: sigma = noise_m + noise_k d^2 (ground-contact-line ranging, d = h / tan(angle below the
        horizon), so the error grows with d^2); no detection beyond max_m; each reading is lost with p_miss (-> 2 m);
        each net wall is visible to this sensor for the whole episode with probability p_net (contrast-limited
        segmentation), an unseen net is read through. Boxes count only at flight height, as for the ToF beams, i.e. the
        model assumes the vision side also estimates the obstacle height."""
        rng = st[2]
        if len(st) < 4:
            st.append(frozenset(o.name for o in self.world.obstacles if o.net and rng.random() >= p_net))
        skip = st[3]
        fovs = fov if isinstance(fov, tuple) else (float(fov),) * len(beams)
        out = []
        for b, fv in zip(beams, fovs):
            h = math.radians(fv) / 2
            offs = (-h, 0.0, h) if h > 0 else (0.0,)
            d = min(self._ray(self.yaw + math.radians(b) + o, skip) for o in offs) - 0.06
            sd = noise_m + noise_k * d * d
            if sd > 0:
                d += rng.normal(0, sd)
            if (p_miss > 0 and rng.random() < p_miss) or d > max_m:
                d = 2.0
            out.append(float(np.clip(d, 0.0, 2.0)))
        return out

    def zones(self, bearing: float, hfov: float, vfov: float, n: int = 8, rate_hz: float = 15.0, noise_m: float = 0.01,
              max_m: float = 3.5) -> np.ndarray:
        """S7 (route review 2026-10-14): one level-mounted multi-zone ToF (VL53L5CX / L7CX, n x n zones) facing body
        bearing `bearing` (deg, + = right), 0.06 m from the centre: slant range of every zone-centre ray to the first
        box, net or the floor (3D slab test against the world boxes), sample and hold at rate_hz, Gaussian noise from its
        own RNG stream, nan beyond max_m (no target). Row-major, row 0 = top (ecps_map.zone_dirs order)."""
        from ecps_map import zone_dirs

        if not hasattr(self, "_zones"):
            self._zones = {}
        key = (bearing, hfov, vfov, n, rate_hz, noise_m, max_m)
        st = self._zones.get(key)
        if st is None:
            st = self._zones[key] = [-1.0, None, np.random.default_rng(87_000 + self._seed + len(self._zones)), zone_dirs(bearing, hfov, vfov, n)]
        if st[1] is None or self.t - st[0] >= 1.0 / rate_hz - 1e-9:
            st[0] = self.t
            cy, sy = math.cos(self.yaw), math.sin(self.yaw)
            B = st[3]
            D = np.stack([cy * B[:, 0] - sy * B[:, 1], sy * B[:, 0] + cy * B[:, 1], B[:, 2]], axis=1)
            b = self.yaw + math.radians(bearing)
            o = self.pos + np.array([0.06 * math.cos(b), 0.06 * math.sin(b), 0.0])
            r = self._rays3(o, D)
            if noise_m > 0:
                r = r + st[2].normal(0.0, noise_m, len(r))
            st[1] = np.where(r > max_m, np.nan, np.maximum(r, 0.0))
        return st[1]

    def _rays3(self, o: np.ndarray, D: np.ndarray) -> np.ndarray:
        """Distance along each unit ray D (N, 3) from o to the first world box (nets included) or the floor, inf if none."""
        if not hasattr(self, "_box_lo"):
            self._box_lo = np.array([ob.lo for ob in self.world.obstacles], float).reshape(-1, 3)
            self._box_hi = np.array([ob.hi for ob in self.world.obstacles], float).reshape(-1, 3)
        t = np.full(len(D), np.inf)
        if len(self._box_lo):
            Dz = np.where(np.abs(D) < 1e-12, 1e-12, D)
            t1 = (self._box_lo[None, :, :] - o[None, None, :]) / Dz[:, None, :]
            t2 = (self._box_hi[None, :, :] - o[None, None, :]) / Dz[:, None, :]
            tmin = np.maximum(np.minimum(t1, t2).max(axis=2), 0.0)
            tmax = np.maximum(t1, t2).min(axis=2)
            t = np.where(tmax >= tmin, tmin, np.inf).min(axis=1)
        floor = np.where(D[:, 2] < -1e-9, o[2] / np.maximum(-D[:, 2], 1e-9), np.inf)
        return np.minimum(t, floor)

    def baro_m(self) -> float | None:
        return float(self.pos[2] + self.nrng.normal(0, 0.03))

    def _flow_scale(self) -> float:
        rng = max(self._range_true(), 0.05)
        if self.ood == "tof" and self._tof is not None:
            rng = max(self._tof, 0.05)  # the EKF scales flow by the (faulty) measured range
        return self.flow_bias * max(self.pos[2] - self.terr, 0.05) / rng

    def telemetry(self) -> Telemetry:
        if self.nav == "flow":
            return Telemetry(
                t=self.t, alt_m=float(self.pos[2]), vz_mps=float(self.vel[2]), yaw_deg=math.degrees(self.yaw) % 360,
                yaw_rate_dps=math.degrees(self.yaw_rate), x_m=float(self.est[0]), y_m=float(self.est[1]),
                battery_pct=self.battery, flying=self.flying,
            )
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
        img = self.cam.render(self.view, self.pos, self.yaw)
        if self.half:
            w = img.shape[1] // 2
            img = img.copy()
            if self.half == "L":
                img[:, :w] = 20
            else:
                img[:, w:] = 20
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
        if self.nav == "flow":
            vxy = vxy / max(self._flow_scale(), 0.3)  # GUIDED drives the (biased) estimated velocity to the target
        target = np.array([vxy[0], vxy[1], c.throttle * self.vz_max])
        if self.ood == "wind":
            target = target + self.drift  # GUIDED leaves a steady-state drift under a steady wind
        if self.jit0 > 0 or self.jit_yaw > 0:
            sd = self.jit0 + self.jit_yaw * min(abs(math.degrees(self.yaw_rate)) / 20.0, 1.0)
            a = dt / self.jit_tau
            self._jit += -self._jit * a + self._jrng.standard_normal(2) * sd * math.sqrt(2 * a)
            target[:2] = target[:2] + self._jit
        tau = self.tau
        if self.tau_dec is not None and np.hypot(*target[:2]) < np.hypot(*self.vel[:2]):
            tau = np.array([self.tau_dec, self.tau_dec, self.tau[2]])
        k = 1 - np.exp(-dt / tau)
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
        if self.nav == "flow":
            if self.terr_model == "ekf3":
                self._terr_ekf3()
            else:
                surf = self.pos[2] - self._range_true()
                self.terr += (surf - self.terr) * (1 - math.exp(-dt / self.tau_terr))
            self.est += self._flow_scale() * self.vel[:2] * dt + self.nrng.standard_normal(2) * self.flow_rw * math.sqrt(dt)
        if hit and not self._touching:
            self.collisions += 1
        self._touching = hit

    def _inside(self, q: np.ndarray) -> bool:
        for o in self.world.obstacles:
            lo, hi = np.array(o.lo) - self.r, np.array(o.hi) + self.r
            if np.all(q > lo) and np.all(q < hi):
                return True
        return False
