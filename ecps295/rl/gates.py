"""Pre-flight gates for any optimisation run or code change (RL roadmap A3).

  sensory   open-loop causal gate on the brain itself (garyb9 sensory_assay): one-sided looming must lateralise DNp03
            in opposite directions for the two sides, two-sided looming must drive DNp01, one-sided front-to-back flow
            must lateralise DNg02, and silencing every input must give exactly the dark response (same noise seed)
  smoke     closed-loop semantics in the fast twin: a box ahead-left makes the first saccade turn right (clockwise)
            and ahead-right turn left; hovering (no cruise) in front of the textured net never saccades; flying at a
            plain untextured wall triggers an escape before contact
  anchor    SHA-256 of fixed twin episodes (v3_2 and es3, cage20, seeds 0/1, 30 s); --update writes rl/anchors.json,
            --check fails if any number changed (intended changes must update the anchors in the same commit)

python -m rl.gates all --config minifly/v3_2.yaml      (exit code 1 when a gate fails)
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

from flydrones.brain import Brain, load_connectome
from rl.episode import build_cfg, run_episode
from rl.twin import CARDBOARD, Obstacle, World
from rl.worlds import HALF, empty_world, nets

E = Path(__file__).resolve().parent.parent
ANCHORS = Path(__file__).resolve().parent / "anchors.json"
STIM_HZ, MS = 120.0, 600.0


def _brain(variant: str) -> tuple[Brain, dict]:
    cfg = build_cfg(variant, 0)
    return Brain(load_connectome(cfg["brain"]["source"]), cfg, seed=7), cfg


def _respond(variant: str, inputs: dict) -> dict:
    b, _ = _brain(variant)
    b.tick({}, 200.0)  # settle
    acc: dict = {}
    for _ in range(int(MS / 50)):
        for k, v in b.tick(inputs, 50.0).items():
            acc[k] = acc.get(k, 0.0) + v / (MS / 50)
    return acc


def sensory(variant: str) -> list[tuple[str, bool, str]]:
    dark = _respond(variant, {})
    silenced = _respond(variant, {g: 0.0 for g in _brain(variant)[0].input_specs})
    loom_l = _respond(variant, {"LPLC2_L": STIM_HZ, "LC4_L": STIM_HZ})
    loom_r = _respond(variant, {"LPLC2_R": STIM_HZ, "LC4_R": STIM_HZ})
    loom_b = _respond(variant, {"LPLC2_L": STIM_HZ, "LC4_L": STIM_HZ, "LPLC2_R": STIM_HZ, "LC4_R": STIM_HZ})
    flow_l = _respond(variant, {"T4a_L": STIM_HZ})
    flow_r = _respond(variant, {"T4a_R": STIM_HZ})
    lat = lambda r, g: r[f"{g}_R"] - r[f"{g}_L"]  # noqa: E731
    out = []
    a, b = lat(loom_l, "DNp03"), lat(loom_r, "DNp03")
    out.append(("one-sided looming lateralises DNp03 oppositely", a * b < 0 and min(abs(a), abs(b)) > 1.0, f"R-L: left {a:+.1f} Hz, right {b:+.1f} Hz"))
    gf = loom_b["DNp01_L"] + loom_b["DNp01_R"] - dark["DNp01_L"] - dark["DNp01_R"]
    out.append(("two-sided looming drives DNp01", gf > 5.0, f"DNp01 L+R above dark {gf:+.1f} Hz"))
    a, b = lat(flow_l, "DNg02"), lat(flow_r, "DNg02")
    out.append(("one-sided flow lateralises DNg02 oppositely", a * b < 0 and min(abs(a), abs(b)) > 1.0, f"R-L: left {a:+.1f} Hz, right {b:+.1f} Hz"))
    d = max(abs(silenced[k] - dark[k]) for k in dark)
    out.append(("silenced inputs == dark (same noise seed)", d < 1e-9, f"max |diff| {d:.2e} Hz"))
    return out


def _box_world(side: float) -> World:
    """One cardboard box 1.2 m ahead, offset to the east (side=+1, right of a north-facing drone) or west (-1)."""
    n, e, s = 1.3, 0.30 * side, 0.5
    return World(nets() + [Obstacle((n - s / 2, e - s / 2, 0), (n + s / 2, e + s / 2, 1.1), CARDBOARD, False, "box")], floor_half=HALF, name=f"box{side:+.0f}")


def smoke(variant: str) -> list[tuple[str, bool, str]]:
    out = []
    for side, want in ((-1, +1), (+1, -1)):  # box left -> turn right (clockwise, +yaw); box right -> left
        r = run_episode(variant, _box_world(side), seed=0, seconds=12, keep_rows=True)
        rows = r["rows"]
        # sign of the commanded yaw during the first saccade (+ = clockwise = right turn): rows[k][5] is cmd.yaw.
        # (The position track does not show it: the saccade backs off first and then turns almost in place.)
        t0 = int(round(r["escape_at"][0][0] / 0.05)) if r["escape_at"] else None
        if t0 is None:
            out.append((f"box {'left' if side < 0 else 'right'}: saccade away", False, "no saccade"))
            continue
        yaw = float(np.mean([row[5] for row in rows[t0 : t0 + 20]]))
        ok = yaw * want > 0.1 and r["contact_episodes"] == 0
        turn = "right" if yaw > 0 else "left"
        out.append((f"box {'left' if side < 0 else 'right'}: saccade away, no contact", ok, f"first saccade turns {turn} (mean yaw cmd {yaw:+.2f}), contacts {r['contact_episodes']}"))
    r = run_episode(variant, empty_world(0), seed=0, seconds=20, overrides={"decoder": {"cruise": 0.0}})
    out.append(("hover at the textured net: no saccade", r["saccade_onsets"] == 0, f"{r['saccade_onsets']} saccade(s) in 20 s"))
    wall = World(nets() + [Obstacle((1.6, -0.6, 0), (1.9, 0.6, 1.5), 0.75, False, "plain_wall")], floor_half=HALF, name="plain_wall")
    r = run_episode(variant, wall, seed=0, seconds=15)
    out.append(("plain untextured wall: escape before contact", r["escapes"] > 0 and r["contact_episodes"] == 0, f"escapes {r['escapes']}, contacts {r['contact_episodes']}, min clearance {r['min_clearance_m']} m"))
    return out


def _anchor_runs() -> dict:
    w = World.from_layout(str(E / "gazebo" / "worlds" / "ecps295_cage20.layout.json"))
    res = {}
    for name, v in (("v3_2", str(E / "minifly" / "v3_2.yaml")), ("es3", str(E / "rl" / "configs" / "v3_2_es3.yaml"))):
        for seed in (0, 1):
            r = run_episode(v, w, seed=seed, seconds=30)
            r.pop("wall_s", None)
            res[f"{name}_s{seed}"] = hashlib.sha256(json.dumps(r, sort_keys=True).encode()).hexdigest()
    return res


def anchor(update: bool) -> list[tuple[str, bool, str]]:
    now = _anchor_runs()
    if update or not ANCHORS.exists():
        ANCHORS.write_text(json.dumps(now, indent=1) + "\n")
        return [("anchors written", True, str(ANCHORS))]
    old = json.loads(ANCHORS.read_text())
    return [(f"anchor {k}", old.get(k) == v, v[:16]) for k, v in now.items()]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["sensory", "smoke", "anchor", "all"])
    ap.add_argument("--config", default=str(E / "minifly" / "v3_2.yaml"))
    ap.add_argument("--update", action="store_true", help="anchor: rewrite rl/anchors.json")
    a = ap.parse_args()
    checks = []
    if a.what in ("sensory", "all"):
        checks += sensory(a.config)
    if a.what in ("smoke", "all"):
        checks += smoke(a.config)
    if a.what in ("anchor", "all"):
        checks += anchor(a.update)
    for name, ok, info in checks:
        print(f"{'PASS' if ok else 'FAIL'}  {name:48s} {info}")
    sys.exit(0 if all(ok for _, ok, _ in checks) else 1)


if __name__ == "__main__":
    main()
