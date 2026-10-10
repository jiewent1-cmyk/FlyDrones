"""Procedural training cages (RL plan P0.3). The G4 Gazebo worlds stay out of training (test set).

20 ft cage (nets at +-3.05 m), EVA mat, 3-6 cardboard boxes or stacks (0.35-0.65 m, stacks up to ~1.2 m), half of
them taped, sometimes a plain (untextured, light) wall panel like wall_offset_plain. Nothing within 0.7 m of the
take-off point. Seeded, so a generation can reuse the same worlds for every candidate (common random numbers).
"""

from __future__ import annotations

import math

import numpy as np

from rl.twin import CARDBOARD, Obstacle, World, _floor_tape

HALF = 3.05


def nets(half: float = HALF) -> list[Obstacle]:
    h = 3.05
    return [
        Obstacle((half - 0.01, -half, 0), (half + 0.01, half, h), 0.16, True, "net_n"),
        Obstacle((-half - 0.01, -half, 0), (-half + 0.01, half, h), 0.16, True, "net_s"),
        Obstacle((-half, half - 0.01, 0), (half, half + 0.01, h), 0.16, True, "net_e"),
        Obstacle((-half, -half - 0.01, 0), (half, -half + 0.01, h), 0.16, True, "net_w"),
    ]


def random_world(seed: int, fence_stack: bool = True) -> World:
    rnd = np.random.default_rng(10_000 + seed)
    obs = nets()
    placed: list[tuple[float, float, float]] = []
    n_items = int(rnd.integers(3, 7))
    tries = 0
    while len(placed) < n_items and tries < 200:
        tries += 1
        r, a = rnd.uniform(0.9, 2.6), rnd.uniform(0, 2 * math.pi)
        cn, ce = r * math.cos(a), r * math.sin(a)
        s = rnd.uniform(0.35, 0.65)
        if any(math.hypot(cn - pn, ce - pe) < (s + ps) / 2 + 0.45 for pn, pe, ps in placed):
            continue  # keep a passable gap
        placed.append((cn, ce, s))
        taped = bool(rnd.random() < 0.5)
        if rnd.random() < 0.3:  # stack of two
            for k in range(2):
                obs.append(Obstacle((cn - s / 2, ce - s / 2, k * s), (cn + s / 2, ce + s / 2, (k + 1) * s), CARDBOARD, False, f"stack{len(placed)}_{k}", taped))
        else:
            obs.append(Obstacle((cn - s / 2, ce - s / 2, 0), (cn + s / 2, ce + s / 2, s), CARDBOARD, False, f"box{len(placed)}", taped))
    for _ in range(10 if fence_stack and rnd.random() < 0.5 else 0):  # cage20's hard case: stack at the fence edge
        r, a = rnd.uniform(1.7, 2.1), rnd.uniform(0, 2 * math.pi)
        cn, ce, s = r * math.cos(a), r * math.sin(a), rnd.uniform(0.45, 0.6)
        if all(math.hypot(cn - pn, ce - pe) > (s + ps) / 2 + 0.45 for pn, pe, ps in placed):
            placed.append((cn, ce, s))
            taped = bool(rnd.random() < 0.5)
            for k in range(2):
                obs.append(Obstacle((cn - s / 2, ce - s / 2, k * s), (cn + s / 2, ce + s / 2, (k + 1) * s), CARDBOARD, False, f"edge_stack_{k}", taped))
            break
    if rnd.random() < 0.3:  # plain wall panel, 1.2 x 0.3 x 1.5 m, light and untextured
        r, a = rnd.uniform(1.4, 2.3), rnd.uniform(0, 2 * math.pi)
        cn, ce = r * math.cos(a), r * math.sin(a)
        if all(math.hypot(cn - pn, ce - pe) > 1.2 for pn, pe, _ in placed):
            along_n = bool(rnd.random() < 0.5)
            hn, he = (0.6, 0.15) if along_n else (0.15, 0.6)
            obs.append(Obstacle((cn - hn, ce - he, 0), (cn + hn, ce + he, 1.5), 0.75, False, "plain_wall"))
    return World(obs, floor_half=HALF, name=f"proc{seed}", floor_tape=_floor_tape(6.1, seed=7 + seed), scenery_seed=3 + seed % 5)


def lowbox_world(seed: int, n: int = 6) -> World:
    """H1: the S8 lowbox distribution (gazebo/make_quad.py lowbox_objects): boxes 0.2-0.75 m tall (flown over at the
    0.6-0.9 m cruise height: the ventral cue / flow-EKF case), one in four a 1.1 m stack, footprints 0.3-0.6 m, centres
    1.0-2.3 m from take-off, >= 0.5 m apart. Seeds are offset by 10**6 so lowbox_s0..49 (Gazebo test set) never recur."""
    import random

    rng = random.Random(1000 + 10**6 + seed)
    obs, centres = nets(), []
    while len(centres) < n:
        r, th = rng.uniform(1.0, 2.3), rng.uniform(-math.pi, math.pi)
        x, y = r * math.cos(th), r * math.sin(th)  # ENU, as make_quad
        w, d = rng.uniform(0.3, 0.6), rng.uniform(0.3, 0.6)
        if any(math.hypot(x - cx, y - cy) < 0.5 + 0.5 * (w + cw) for cx, cy, cw in centres):
            continue
        h = 1.1 if rng.random() < 0.25 else rng.uniform(0.2, 0.75)
        centres.append((x, y, max(w, d)))
        taped = rng.random() < 0.5
        n_, e_ = y, x  # ENU -> NED
        obs.append(Obstacle((n_ - d / 2, e_ - w / 2, 0), (n_ + d / 2, e_ + w / 2, h), CARDBOARD, False, f"lb{len(centres) - 1}", taped))
    return World(obs, floor_half=HALF, name=f"lowproc{seed}", floor_tape=_floor_tape(6.1, seed=7 + seed), scenery_seed=3 + seed % 5)


def empty_world(seed: int = 0) -> World:
    """Nets and mat only: every saccade here is a false alarm (B1 empty-field saccade rate)."""
    return World(nets(), floor_half=HALF, name=f"empty{seed}", floor_tape=_floor_tape(6.1, seed=7 + seed), scenery_seed=3 + seed % 5)


def validation_set(k: int = 24) -> list[dict]:
    """Fixed held-out procedural episodes for model selection (world seeds disjoint from training draws in practice)."""
    rnd = np.random.default_rng(424242)
    return [{"world_seed": 5_000_000 + i, "yaw_deg": float(rnd.uniform(0, 360)), "seed": 900_000 + i} for i in range(k)]


def episode_set(gen: int, k: int, stream: int = 0) -> list[dict]:
    """k (world seed, start yaw, brain seed) triples for generation `gen`; identical for every candidate of the gen.
    `stream` selects an independent sequence of training worlds (replicate runs, RL roadmap A4); 0 = es1-es3 / A1."""
    rnd = np.random.default_rng(777 + gen + 100_003 * stream)
    return [{"world_seed": int(rnd.integers(0, 10**6)), "yaw_deg": float(rnd.uniform(0, 360)), "seed": int(rnd.integers(0, 10**6))} for _ in range(k)]
