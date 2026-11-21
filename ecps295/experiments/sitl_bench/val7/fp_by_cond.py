#!/usr/bin/env python3
"""fp_by_cond.py -- 从 fp_s1.csv 里挑出某个参数组，按气压条件统计误报。

用法: fp_by_cond.py fp_s1.csv '<params json>'
"""
import csv
import json
import sys
from collections import defaultdict

fp_file = sys.argv[1] if len(sys.argv) > 1 else "fp_s1.csv"
want = sys.argv[2] if len(sys.argv) > 2 else None

rows = list(csv.DictReader(open(fp_file)))
if want is None:
    # 误报最少的参数组
    rows.sort(key=lambda r: (int(r["n_trigger"]), float(r["fp_per_h"] or 0)))
    want = rows[0]["params"]
row = next(r for r in rows if r["params"] == want)
print("参数组:", want)
print("误报 %s/%s 飞行, %.3f 飞行小时, %.2f 次/小时, A=%s B=%s AB=%s" %
      (row["n_trigger"], row["n_runs"], float(row["flight_h"]), float(row["fp_per_h"] or 0),
       row.get("n_trig_A"), row.get("n_trig_B"), row.get("n_trig_AB")))

runs = json.loads(row["runs"])
by_cond = defaultdict(lambda: [0, 0, []])
for r in runs:
    tag = r["run"]
    cond = tag.split("_")[2]
    by_cond[cond][1] += 1
    if r.get("t_trig") is not None:
        by_cond[cond][0] += 1
        by_cond[cond][2].append((tag, r["t_trig"], r.get("reason")))
print("%-10s %6s %6s  %s" % ("条件", "触发", "飞行", "触发明细"))
for c in sorted(by_cond):
    n, tot, det = by_cond[c]
    print("%-10s %6d %6d  %s" % (c, n, tot, "; ".join("%s@%.1fs(%s)" % d for d in det[:6])))
