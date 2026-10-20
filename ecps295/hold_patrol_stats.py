"""Hover drift inside patrol runs (ECPS_HOLD=vel vs pos), 2026-10-20.

usage: hold_patrol_stats.py <runs_dir> <tagA> <tagB> [...]     run dirs are <runs_dir>/<tag>__<world>/run.csv

A hover segment = consecutive ticks whose commanded horizontal speed is below ECPS_HOLD_EPS_MPS (0.03 m/s, the
mavlink_twin hold threshold; cmd_forward / cmd_lateral are in units of v_max = 0.5 m/s), lasting at least MIN_S.
Per segment: max truth horizontal displacement from the segment start. Per run: hover time and the median / max of
the segment drifts. Paired over common worlds against the first tag (Wilcoxon).
"""

import csv
import glob
import math
import os
import sys

import numpy as np

V_MAX, EPS, MIN_S = 0.5, 0.03, 1.0


def segments(path: str) -> list[float]:
    rows = list(csv.DictReader(open(path)))
    out, start = [], None
    for i, r in enumerate(rows + [None]):
        hov = r is not None and math.hypot(float(r["cmd_forward"]), float(r["cmd_lateral"])) * V_MAX < EPS
        if hov and start is None:
            start = i
        elif not hov and start is not None:
            seg = rows[start:i]
            if float(seg[-1]["t"]) - float(seg[0]["t"]) >= MIN_S:
                n0, e0 = float(seg[0]["north"]), float(seg[0]["east"])
                out.append((float(seg[-1]["t"]) - float(seg[0]["t"]),
                            max(math.hypot(float(s["north"]) - n0, float(s["east"]) - e0) for s in seg)))
            start = None
    return out


def load(runs: str, tag: str) -> dict:
    D = {}
    for p in sorted(glob.glob(f"{runs}/{tag}__*/run.csv")):
        S = segments(p)
        if S:
            d = np.array([s[1] for s in S])
            D[os.path.basename(os.path.dirname(p)).split("__", 1)[1]] = {
                "hover_s": sum(s[0] for s in S), "n_seg": len(S), "drift_med": float(np.median(d)), "drift_max": float(d.max())}
    return D


if __name__ == "__main__":
    from scipy.stats import wilcoxon

    runs, tags = sys.argv[1], sys.argv[2:]
    T = {t: load(runs, t) for t in tags}
    for t, D in T.items():
        M = list(D.values())
        print(f"== {t}: {len(M)} runs  " + "  ".join(
            f"{k} {np.median([m[k] for m in M]) * (100 if 'drift' in k else 1):.1f}" for k in ("hover_s", "n_seg", "drift_med", "drift_max"))
            + "  (median over runs; drift in cm)")
    A = T[tags[0]]
    for t in tags[1:]:
        W = sorted(set(A) & set(T[t]))
        for k in ("drift_med", "drift_max"):
            x = np.array([A[w][k] for w in W]); y = np.array([T[t][w][k] for w in W])
            print(f"  {tags[0]} -> {t} {k}: {np.median(x) * 100:.1f} -> {np.median(y) * 100:.1f} cm  n={len(W)}  p={wilcoxon(y - x).pvalue:.3g}")
