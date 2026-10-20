"""Hover-precision metrics from run_g4 --mode hovertest runs (hover_test.py), 2026-10-18.

usage: hover_stats.py <runs_dir> <tagA> [<tagB>]     run dirs are <runs_dir>/<tag>__<world>/run.csv

Per block (A = H1+Y1+H2, B = H3+Y2+H4), relative to the block start:
  true   horizontal displacement of the drone (SIM_STATE truth)             -> what we want small
  ekf    displacement of the EKF position (what the FC's position loop sees) -> control part
  est    |true - ekf| displacement                                          -> estimator drift part
and the hover altitude spread (std of truth alt). With two tags: paired comparison over common worlds (Wilcoxon).
"""

import csv
import glob
import math
import os
import sys

import numpy as np

BLOCKS = {"A": ("H1", "Y1", "H2"), "B": ("H3", "Y2", "H4")}


def run_metrics(path: str) -> dict:
    rows = list(csv.DictReader(open(path)))
    out = {}
    for b, phases in BLOCKS.items():
        R = [r for r in rows if r["phase"] in phases]
        if len(R) < 20:
            continue
        f = lambda k: np.array([float(r[k]) for r in R])
        n, e, en, ee, alt = f("north"), f("east"), f("ekf_north"), f("ekf_east"), f("alt")
        dn, de = n - n[0], e - e[0]
        kn, ke = en - en[0], ee - ee[0]
        true = np.hypot(dn, de)
        ekf = np.hypot(kn, ke)
        est = np.hypot(dn - kn, de - ke)
        out[b] = {
            "true_rms": float(np.sqrt(np.mean(true**2))), "true_max": float(true.max()), "true_end": float(true[-1]),
            "ekf_rms": float(np.sqrt(np.mean(ekf**2))), "est_end": float(est[-1]), "alt_std": float(alt.std()),
        }
        for ph in phases:  # end-of-segment truth displacement
            idx = [i for i, r in enumerate(R) if r["phase"] == ph]
            if idx:
                out[b][f"true_end_{ph}"] = float(true[idx[-1]])
    return out


def load(runs: str, tag: str) -> dict:
    D = {}
    for p in sorted(glob.glob(f"{runs}/{tag}__*/run.csv")):
        w = os.path.basename(os.path.dirname(p)).split("__", 1)[1]
        try:
            D[w] = run_metrics(p)
        except (KeyError, ValueError) as ex:
            print(f"skip {p}: {ex}")
    return D


def summary(D: dict, label: str) -> None:
    print(f"== {label}: {len(D)} runs")
    for b in BLOCKS:
        M = [d[b] for d in D.values() if b in d]
        if not M:
            continue
        keys = [k for k in M[0] if all(k in m for m in M)]
        print(f"  block {b} (n={len(M)}): " + "  ".join(
            f"{k} {np.median([m[k] for m in M]) * 100:.1f}/{np.percentile([m[k] for m in M], 90) * 100:.1f}" for k in keys))
    print("  (cm, median/p90 over runs)")


def paired(A: dict, B: dict, la: str, lb: str) -> None:
    from scipy.stats import wilcoxon

    W = sorted(set(A) & set(B))
    print(f"== paired {la} -> {lb}, {len(W)} worlds")
    for b in BLOCKS:
        K = [w for w in W if b in A[w] and b in B[w]]
        if len(K) < 5:
            continue
        for k in ("true_rms", "true_max", "true_end", "ekf_rms", "est_end", "alt_std"):
            x = np.array([A[w][b][k] for w in K]); y = np.array([B[w][b][k] for w in K])
            p = wilcoxon(y - x).pvalue if np.any(y != x) else 1.0
            print(f"  {b} {k:9s} {np.median(x) * 100:6.1f} -> {np.median(y) * 100:6.1f} cm (median)  p={p:.3g}")


if __name__ == "__main__":
    runs = sys.argv[1]
    A = load(runs, sys.argv[2])
    summary(A, sys.argv[2])
    if len(sys.argv) > 3:
        B = load(runs, sys.argv[3])
        summary(B, sys.argv[3])
        paired(A, B, sys.argv[2], sys.argv[3])
