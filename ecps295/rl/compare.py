"""Twin vs Gazebo per variant: contact runs, near misses, coverage, path, escapes, fence turn (RL plan P0.5).

python -m rl.compare rl/out/consistency_cage20.jsonl v3=matrix_gps v3_1=matrix_v31 v3_2=matrix_v32
"""

from __future__ import annotations

import json
import re
import statistics as st
import sys
from pathlib import Path

RUNS = Path.home() / "sim" / "runs"
KEYS = ["contact_episodes", "near_miss_episodes", "coverage", "path_m", "escapes", "fence_turn_s", "min_clearance_m"]


def gz(log: str) -> list[dict]:
    rows = []
    for line in open(RUNS / f"{log}.log"):
        if "contact_episodes=" not in line:
            continue
        rows.append({k: float(m.group(1)) for k in KEYS if (m := re.search(k + r"=(-?[\d.]+)", line))})
    return rows


def summ(rows: list[dict]) -> str:
    def m(k):
        v = [r[k] for r in rows if k in r]
        return st.mean(v) if v else float("nan")

    pc = sum(r["contact_episodes"] > 0 for r in rows)
    return (
        f"n={len(rows):2d} contact {pc:2d}/{len(rows):<2d} ({pc / len(rows):.0%}) near {m('near_miss_episodes'):.1f} "
        f"cov {m('coverage'):.3f} path {m('path_m'):.1f} esc {m('escapes'):.1f} fence {m('fence_turn_s'):.1f}s"
    )


if __name__ == "__main__":
    twin = [json.loads(line) for line in open(sys.argv[1])]
    for pair in sys.argv[2:]:
        v, log = pair.split("=")
        t = [r for r in twin if Path(r["variant"]).stem == v and "error" not in r]
        print(f"{v:6s} twin   {summ(t)}")
        print(f"{'':6s} gazebo {summ(gz(log))}")
