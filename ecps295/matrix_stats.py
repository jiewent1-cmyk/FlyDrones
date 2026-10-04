"""Aggregate run_matrix.sh logs: per prefix (e.g. patrol_v2_1, patrol_v3) mean / min / max of each metric.

python matrix_stats.py ~/sim/runs/matrix_v3.log [--prefix patrol_]
"""

import argparse
import re
import statistics as st
from collections import defaultdict

ap = argparse.ArgumentParser()
ap.add_argument("log")
ap.add_argument("--prefix", default="patrol_")
a = ap.parse_args()

groups: dict[str, list[dict]] = defaultdict(list)
for line in open(a.log):
    if not line.startswith(a.prefix) or "FAILED" in line:
        continue
    tag = line.split()[0]
    group = re.sub(r"_s\d+$", "", tag)
    vals = {k: v for k, v in re.findall(r"(\w+)=(\S+)", line)}
    groups[group].append(vals)

keys = ["contact_episodes", "near_miss_episodes", "min_clearance_m", "path_m", "coverage", "fence_turn_s", "escapes"]
print("| group | runs | " + " | ".join(keys) + " |")
print("|---|---|" + "---|" * len(keys))
for g, runs in groups.items():
    cells = []
    for k in keys:
        xs = [float(r[k]) for r in runs if r.get(k) not in (None, "None")]
        if not xs:
            cells.append("-")
        elif k in ("contact_episodes", "near_miss_episodes"):
            cells.append(f"{sum(xs):.0f} total ({'/'.join(f'{x:.0f}' for x in xs)})")
        elif k == "min_clearance_m":
            cells.append(f"min {min(xs):.3f}, median {st.median(xs):.3f}")
        else:
            cells.append(f"{st.mean(xs):.2f} ({min(xs):.2f}-{max(xs):.2f})")
    print(f"| {g} | {len(runs)} | " + " | ".join(cells) + " |")
