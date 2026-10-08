"""Compact view of ablation_stats.py outputs: python slew_summary.py OUT_PREFIX metric[,metric...]"""

import csv
import json
import sys
from collections import defaultdict

pre, metrics = sys.argv[1], sys.argv[2].split(",")
j = json.load(open(pre + ".json"))
ref = j["ref"]
vals = defaultdict(lambda: defaultdict(list))
for r in csv.DictReader(open(pre + ".csv")):
    for m in metrics:
        if r.get(m) not in (None, "", "None"):
            vals[r["condition"]][m].append(float(r[m] == "True") if r[m] in ("True", "False") else float(r[m]))
print(f"{pre.split('/')[-1]}  ref={ref} n={len(vals[ref][metrics[0]])}  " + "  ".join(f"{m}={sum(vals[ref][m]) / max(1, len(vals[ref][m])):.3f}" for m in metrics))
for c, d in j["conditions"].items():
    cells = []
    for m in metrics:
        x = d.get(m)
        if not x:
            cells.append(f"{m}=-")
            continue
        star = "*" if x["p_holm"] < 0.05 else ""
        cells.append(f"{m}={x['mean']:.3f} ({x['diff']:+.3f}, p={x['p_holm']:.3g}{star})")
    print(f"  {c:18s} n={d[metrics[0]]['n'] if metrics[0] in d else '-':>3}  " + "  ".join(cells))
