#!/usr/bin/env python3
"""Print selected metrics from a results.csv for tags matching a substring."""
import csv
import sys

path = sys.argv[1]
pat = sys.argv[2]
cols = sys.argv[3].split(",") if len(sys.argv) > 3 else None
rows = list(csv.DictReader(open(path)))
for r in rows:
    if pat in r.get("tag", ""):
        out = {c: r.get(c) for c in cols} if cols else r
        print(out)
