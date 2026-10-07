"""One G4-style patrol episode in the fast twin, scored exactly like run_g4.py (patrol_metrics / flight_metrics).

Build order and config handling copy run_g4.py: load_config(variant, ECPS295_SAFETY_20FT, brain seed), brain file next
to the variant YAML, cruise / max_forward overrides, ECPS295_DYN + cfg ecps_drone, EcpsPilot(fence_turn) +
EcpsRetina + EcpsDecoder. `overrides` (a nested dict) is deep-merged last; that is where optimised parameters go.
"""

from __future__ import annotations

import copy
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))  # ecps295/

from ecps_decoder import EcpsDecoder  # noqa: E402
from ecps_pilot import EcpsPilot  # noqa: E402
from ecps_retina import EcpsRetina  # noqa: E402
from envs import ECPS295_DYN, ECPS295_SAFETY_20FT, TAKEOFF_ALT_M  # noqa: E402

from flydrones.brain import Brain, load_connectome  # noqa: E402
from flydrones.config import load_config  # noqa: E402
from rl import arms  # noqa: E402
from rl.trigger_metrics import trigger_metrics  # noqa: E402
from rl.twin import TwinDrone, World  # noqa: E402

PROP_TIP_R = 0.13
HZ = 20
DT = 1.0 / HZ
SUBSTEPS = 4
_CONNECTOMES: dict[str, object] = {}


def deep_merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


class SimClockPilot(EcpsPilot):
    """F2 watchdog measures wall time between ticks; in a faster-than-real-time twin that is meaningless -> age 0."""

    def tick(self, t, dt):
        self._last_tick_end = None
        return super().tick(t, dt)


def build_cfg(variant: str, seed: int, cruise: float = 0.5, max_forward: float = 0.5, overrides: dict | None = None) -> dict:
    cfg = load_config(variant, {"safety": dict(ECPS295_SAFETY_20FT), "brain": {"seed": seed}})
    src = str(cfg["brain"]["source"])
    if src.endswith(".npz") and not Path(src).is_absolute():
        cfg["brain"]["source"] = str(Path(variant).resolve().parent / src)
    cfg["decoder"]["cruise"] = cruise
    cfg["safety"]["max_forward"] = max_forward
    return deep_merge(cfg, overrides or {})


def clearance(world: World, n: float, e: float, alt: float) -> float:
    best = float("inf")
    for b in world.obstacles:
        if b.hi[2] < alt - 0.03:
            continue
        dx = max(b.lo[0] - n, 0.0, n - b.hi[0])
        dy = max(b.lo[1] - e, 0.0, e - b.hi[1])
        best = min(best, (dx * dx + dy * dy) ** 0.5 - PROP_TIP_R)
    return best


def _gap(o, n, e):
    return math.hypot(max(o.lo[0] - n, 0.0, n - o.hi[0]), max(o.lo[1] - e, 0.0, e - o.hi[1]))


def run_episode(
    variant: str,
    world: World,
    seed: int = 0,
    seconds: float = 120.0,
    overrides: dict | None = None,
    start=(0.0, 0.0),
    yaw_deg: float = 0.0,
    dyn_over: dict | None = None,
    keep_rows: bool = False,
    arm: str = "intact",
) -> dict:
    cfg = build_cfg(variant, seed, overrides=overrides)
    src = cfg["brain"]["source"]
    if src not in _CONNECTOMES:
        _CONNECTOMES[src] = load_connectome(src)
    brain = Brain(arms.connectome_for(_CONNECTOMES[src], arm), cfg)
    dyn = {k: ECPS295_DYN[k] for k in ("v_max", "vz_max", "yaw_rate_max_dps")}
    dyn.update(cfg.get("ecps_drone", {}) or {})
    dyn.update({k: ECPS295_DYN[k] for k in ("tau_xy", "tau_z", "tau_yaw")})
    dyn.update(dyn_over or {})
    drone = TwinDrone(world, start=(start[0], start[1], 0.05), yaw_deg=yaw_deg, seed=seed, **dyn)
    pilot = SimClockPilot(brain, drone, cfg, fence_turn=True, **(cfg.get("ecps_pilot", {}) or {}))
    pilot.retina = EcpsRetina.from_config(cfg)
    pilot.decoder = EcpsDecoder(cfg)
    arms.install(arm, brain, pilot, drone, cfg, seed)

    drone.connect()
    pilot.warmup(pilot.decoder.settle_s + 0.1, DT)
    drone.hover_at(TAKEOFF_ALT_M)
    rows = []
    escs = []
    dn = []
    esc_at = []
    escape_prev = False
    escapes = 0
    for k in range(int(seconds * HZ)):
        info = pilot.tick(k * DT, DT)
        for _ in range(SUBSTEPS):
            drone.step(DT / SUBSTEPS)
        n, e, a = drone.pos
        esc = bool(info.cmd.escape)
        if esc and not escape_prev:
            lb = getattr(pilot.retina, "last_blank", {}) or {}
            esc_at.append((round(k * DT, 1), round(clearance(world, n, e, a), 2), round(math.hypot(n, e), 2),
                           pilot.decoder.escapes[-1][1] if pilot.decoder.escapes else "?", round(info.tel.yaw_rate_dps, 1),
                           int(getattr(pilot, "_turning_back", False)), round(max(lb.get("frac_L", 0), lb.get("frac_R", 0)), 2),
                           round(max(info.rates.get("DNp03_L", 0), info.rates.get("DNp03_R", 0)), 1)))
        escs.append(esc)
        escapes += esc and not escape_prev
        escape_prev = esc
        rows.append((n, e, a, clearance(world, n, e, a), info.cmd.forward, info.cmd.yaw))
        dn.append((max(info.rates.get("DNp03_L", 0), info.rates.get("DNp03_R", 0)), max(info.rates.get("DNp01_L", 0), info.rates.get("DNp01_R", 0)),
                   info.rates.get("DNg02_L", 0) + info.rates.get("DNg02_R", 0)))
        if pilot.safety.land_requested:
            break

    cl = [r[3] for r in rows]
    episodes = sum(1 for p, q in zip(cl, cl[1:]) if p >= 0 > q) + (1 if cl[0] < 0 else 0)
    near = sum(1 for p, q in zip(cl, cl[1:]) if p >= 0.1 > q)
    path = sum(math.hypot(b[0] - a_[0], b[1] - a_[1]) for a_, b in zip(rows, rows[1:]))
    fence, cell = cfg["safety"]["geofence_radius_m"], 0.5
    kk = int(math.ceil(fence / cell))
    allowed = {(i, j) for i in range(-kk, kk) for j in range(-kk, kk) if math.hypot((i + 0.5) * cell, (j + 0.5) * cell) <= fence}
    visited = {(math.floor(r[0] / cell), math.floor(r[1] / cell)) for r in rows} & allowed
    dcmd = sum((b[4] - a_[4]) ** 2 + (b[5] - a_[5]) ** 2 for a_, b in zip(rows, rows[1:]))
    out = {
        "world": world.name,
        "arm": arm,
        "seed": seed,
        "ticks": len(rows),
        "contact_episodes": episodes,
        "near_miss_episodes": near,
        "min_clearance_m": round(min(cl), 3),
        "path_m": round(path, 1),
        "coverage": round(len(visited) / max(1, len(allowed)), 3),
        "fence_turn_s": round(pilot.fence_turn_ticks / HZ, 1),
        "escapes": escapes,
        "radius_true_max_m": round(max(math.hypot(r[0], r[1]) for r in rows), 2),
        "alt_min": round(min(r[2] for r in rows), 2),
        "alt_max": round(max(r[2] for r in rows), 2),
        "dcmd2": round(dcmd, 2),
        "proximity": round(sum(math.exp(-((max(c, 0.0) / 0.3) ** 2)) for c in cl) / len(cl), 4),
        "landed_early": bool(pilot.safety.land_requested),
        "escape_triggers": [e[1] for e in pilot.decoder.escapes],
        "escape_at": esc_at,
        "dnp03_mean": round(sum(x[0] for x in dn) / len(dn), 2),
        "dnp01_mean": round(sum(x[1] for x in dn) / len(dn), 2),
        "dng02_mean": round(sum(x[2] for x in dn) / len(dn), 1),
        "fwd_mean": round(sum(r[4] for r in rows) / len(rows), 3),
        "contact_where": sorted({min(world.obstacles, key=lambda o: _gap(o, r[0], r[1])).name for r in rows if r[3] < 0}),
    }
    out.update(trigger_metrics([k * DT for k in range(len(rows))], [r[0] for r in rows], [r[1] for r in rows], cl, escs))
    if keep_rows:
        out["rows"] = rows
    return out


if __name__ == "__main__":
    import argparse
    import json
    import time

    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(HERE.parent / "minifly" / "v3_2.yaml"))
    ap.add_argument("--layout", default=str(HERE.parent / "gazebo" / "worlds" / "ecps295_cage20.layout.json"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--seconds", type=float, default=120)
    a = ap.parse_args()
    t0 = time.perf_counter()
    r = run_episode(a.config, World.from_layout(a.layout), a.seed, a.seconds)
    r["wall_s"] = round(time.perf_counter() - t0, 1)
    print(json.dumps(r))
