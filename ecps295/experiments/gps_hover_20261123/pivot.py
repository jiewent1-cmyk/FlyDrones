#!/usr/bin/env python3
"""Pivot of results_h15_.csv: median of a metric per arm x cell, paired Wilcoxon against arm F (+ better, x worse, p < 0.05)."""
import csv, sys, statistics as st, collections
from scipy.stats import wilcoxon
M = sys.argv[1] if len(sys.argv) > 1 else "exc_max_900"
R = collections.defaultdict(dict)
for r in csv.DictReader(open("results_h15_.csv")):
    if r.get(M):
        R[(r["arm"].replace("h15_", ""), r["flow"], r["gps"])][r["rep"]] = float(r[M])
cells = [(f, g) for f in ("N05", "N15", "N25") for g in ("Gg", "Gm", "Gp")]
print("arm      " + " ".join("%-9s" % ("%s_%s" % c) for c in cells) + " worst/F mean/F")
for a in sorted({k[0] for k in R}):
    row, ratios = [], []
    for fl, g in cells:
        d, f = R.get((a, fl, g)), R[("F", fl, "Gm")]
        if not d:
            row.append("   -     "); continue
        k = sorted(set(d) & set(f)); m, mf = st.median(d.values()), st.median(f.values()); ratios.append(m / mf)
        try:
            p = wilcoxon([d[x] for x in k], [f[x] for x in k]).pvalue
        except Exception:
            p = 1
        row.append("%5.2f%s(%d)" % (m, "+" if p < 0.05 and m < mf else "x" if p < 0.05 else " ", len(d)))
    print("%-8s " % a + " ".join(row) + "  %.2f  %.2f" % (max(ratios), sum(ratios) / len(ratios)))
