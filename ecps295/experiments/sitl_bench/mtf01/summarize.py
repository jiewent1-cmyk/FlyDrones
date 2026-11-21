#!/usr/bin/env python3
"""results.csv -> per-configuration median [min-max] (tags grouped without the _rN suffix)."""
import csv, re, statistics, sys
rows = list(csv.DictReader(open("/home/jacky/sim/mtf01/results.csv")))
groups = {}
for r in rows:
    groups.setdefault(re.sub(r"_r\d+$", "", r["tag"]), []).append(r)
cols = sys.argv[1:] or ["alt_rms", "alt_max", "hgt_est_rms", "hover_hgt_est_rms", "track_rms", "track_max", "land_offset", "pos_est_err_end"]
print("| 配置 | n | 完成 | " + " | ".join(cols) + " |")
print("|" + "---|" * (len(cols) + 3))
for g, rs in groups.items():
    def f(c):
        v = [float(r[c]) for r in rs if r.get(c) not in ("", "None", None)]
        if not v: return "-"
        return "%.3f [%.3f–%.3f]" % (statistics.median(v), min(v), max(v)) if len(v) > 1 else "%.3f" % v[0]
    done = sum(r["complete"] == "True" for r in rs)
    print("| %s | %d | %d/%d | %s |" % (g, len(rs), done, len(rs), " | ".join(f(c) for c in cols)))
