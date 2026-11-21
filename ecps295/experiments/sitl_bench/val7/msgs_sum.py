#!/usr/bin/env python3
# msgs_sum.py -- 汇总第4步各组出现过的飞控 STATUSTEXT
import json
import os
import re
from collections import Counter

D = "/home/jacky/sim/val7"
RUNS = os.path.join(D, "runs")
groups = {}
for tag in sorted(os.listdir(RUNS)):
    m = re.match(r"v7_(s4|s5)_((?:d3_)?.+?)_r\d+$", tag)
    if not m:
        continue
    p = os.path.join(RUNS, tag, "metrics.json")
    if not os.path.exists(p):
        continue
    try:
        j = json.load(open(p))
    except Exception:
        continue
    groups.setdefault(m.group(2), []).append(j)

print("%-16s %-6s %s" % ("group", "n", "STATUSTEXT (count)"))
for g in sorted(groups):
    c = Counter()
    for j in groups[g]:
        for x in (j.get("msgs") or []):
            c[x] += 1
    print("%-16s %-6d %s" % (g, len(groups[g]), "; ".join("%s(%d)" % (k, v) for k, v in c.most_common(8))))
