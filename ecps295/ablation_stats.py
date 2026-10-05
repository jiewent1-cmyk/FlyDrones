"""Paired statistics for the S8 ablation (every condition flies the same episodes: same world, same brain seed).

    python ablation_stats.py RUNS_DIR --ref full --conds a,b,c [--episodes lowbox_s0..19] [--out ablation_patrol]

A run directory is <cond>__<episode> (run_matrix tag), holding run.json and rtf.log. A run is valid if it finished
and its whole-run RTF is >= 0.95; a pair is used only if both the reference and the condition are valid there.

Per condition and metric: mean with a 95% bootstrap CI (10,000 resamples of episodes), the paired difference to the
reference with its bootstrap CI, and a test:
- continuous metrics: Wilcoxon signed-rank (two-sided, zero differences dropped as in Wilcoxon's original method),
  effect size = matched-pairs rank-biserial correlation;
- binary metrics: exact McNemar on the discordant pairs, effect = difference in proportions.
p-values are Holm-corrected across conditions within each metric. Writes <out>.md (tables), <out>.csv (per run) and
<out>.json (everything).
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os

import numpy as np
from scipy import stats

METRICS = [  # (key, label, kind, better)
    ("success", "success (no contact, no FC landing)", "bin", "higher"),
    ("contact_any", "episodes with contact", "bin", "lower"),
    ("fc_land", "FC landings", "bin", "lower"),
    ("contact_episodes", "contacts per episode", "num", "lower"),
    ("near_miss_episodes", "near misses per episode", "num", "lower"),
    ("min_clearance_m", "min clearance m", "num", "higher"),
    ("path_m", "path m", "num", "higher"),
    ("coverage", "coverage", "num", "higher"),
    ("over_box_s", "time over boxes s", "num", "lower"),
    ("alt_true_max_m", "max true height m", "num", "lower"),
    ("ekf_err_alt_max_m", "FC height error max m", "num", "lower"),
]
RNG = np.random.default_rng(0)


def load(d: str) -> dict | None:
    try:
        j = json.load(open(os.path.join(d, "run.json")))
        rtf = [float(ln.split()[2]) for ln in open(os.path.join(d, "rtf.log")) if ln.startswith("RTF run")]
    except (FileNotFoundError, ValueError, IndexError):
        return None
    if not rtf or rtf[0] < 0.95:
        return None
    j["rtf_run"] = rtf[0]
    j["contact_any"] = int(j.get("contact_episodes", 0) > 0)
    j["fc_land"] = int(bool(j.get("fc_land", False)))
    j["success"] = int(not j["contact_any"] and not j["fc_land"])
    return j


def boot_ci(x: np.ndarray, n: int = 10000) -> tuple[float, float]:
    if len(x) == 0:
        return (math.nan, math.nan)
    idx = RNG.integers(0, len(x), size=(n, len(x)))
    m = x[idx].mean(axis=1)
    return float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))


def holm(ps: list[float]) -> list[float]:
    order = sorted(range(len(ps)), key=lambda i: ps[i])
    adj, running = [0.0] * len(ps), 0.0
    for rank, i in enumerate(order):
        running = max(running, min(1.0, (len(ps) - rank) * ps[i]))
        adj[i] = running
    return adj


def paired_test(ref: np.ndarray, x: np.ndarray, kind: str) -> tuple[float, float]:
    """(p, effect): Wilcoxon + rank-biserial, or exact McNemar + difference in proportions."""
    if kind == "bin":
        b = int(((ref == 1) & (x == 0)).sum())
        c = int(((ref == 0) & (x == 1)).sum())
        p = 1.0 if b + c == 0 else float(stats.binomtest(b, b + c, 0.5).pvalue)
        return p, float(x.mean() - ref.mean())
    d = x - ref
    d = d[d != 0]
    if len(d) == 0:
        return 1.0, 0.0
    p = float(stats.wilcoxon(d).pvalue)
    r = stats.rankdata(np.abs(d))
    rb = float((r[d > 0].sum() - r[d < 0].sum()) / r.sum())
    return p, rb


ap = argparse.ArgumentParser()
ap.add_argument("runs")
ap.add_argument("--ref", required=True)
ap.add_argument("--conds", required=True, help="comma-separated condition names (without the reference)")
ap.add_argument("--labels", default="", help="cond=label,... for the tables")
ap.add_argument("--out", default="ablation")
a = ap.parse_args()
conds = [c for c in a.conds.split(",") if c]
labels = dict(kv.split("=", 1) for kv in a.labels.split(",") if "=" in kv)

data: dict[str, dict[str, dict]] = {}
invalid: dict[str, list[str]] = {}
for c in [a.ref, *conds]:
    data[c], invalid[c] = {}, []
    for d in sorted(glob.glob(os.path.join(a.runs, f"{c}__*"))):
        ep = os.path.basename(d).split("__", 1)[1]
        j = load(d)
        if j is None:
            invalid[c].append(ep)
        else:
            data[c][ep] = j

with open(a.out + ".csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["condition", "episode", "rtf_run", *[m[0] for m in METRICS]])
    for c, eps in data.items():
        for ep, j in sorted(eps.items()):
            w.writerow([c, ep, j["rtf_run"], *[j.get(m[0]) for m in METRICS]])

result = {"ref": a.ref, "conditions": {}, "invalid": invalid}
lines = [f"# Ablation vs `{a.ref}`", ""]
for key, label, kind, better in METRICS:
    rows, ps = [], []
    for c in conds:
        eps = sorted(set(data[a.ref]) & set(data[c]))
        if not eps:
            rows.append((c, 0, None))
            ps.append(1.0)
            continue
        ref = np.array([float(data[a.ref][e].get(key) or 0.0) for e in eps])
        x = np.array([float(data[c][e].get(key) or 0.0) for e in eps])
        p, eff = paired_test(ref, x, kind)
        rows.append((c, len(eps), (x.mean(), boot_ci(x), (x - ref).mean(), boot_ci(x - ref), p, eff)))
        ps.append(p)
    adj = holm(ps)
    full = np.array([float(j.get(key) or 0.0) for j in data[a.ref].values()])
    lines += [f"## {label} ({'higher' if better == 'higher' else 'lower'} is better)", ""]
    lo, hi = boot_ci(full)
    lines += [f"reference `{a.ref}`: {full.mean():.3f} [{lo:.3f}, {hi:.3f}] (n={len(full)})", ""]
    eff_name = "diff in proportion" if kind == "bin" else "rank-biserial r"
    lines += [
        f"| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | {eff_name} | p (Holm) |",
        "|---|---|---|---|---|---|",
    ]
    for (c, n, r), padj in zip(rows, adj):
        name = labels.get(c, c)
        if r is None:
            lines.append(f"| {name} | 0 | - | - | - | - |")
            continue
        m, (l1, h1), dm, (l2, h2), _p, eff = r
        star = "**" if padj < 0.05 else ""
        lines.append(
            f"| {name} | {n} | {m:.3f} [{l1:.3f}, {h1:.3f}] | {star}{dm:+.3f} [{l2:+.3f}, {h2:+.3f}]{star} | {eff:+.2f} | {padj:.3g} |"
        )
        result["conditions"].setdefault(c, {})[key] = {
            "n": n,
            "mean": m,
            "ci": [l1, h1],
            "diff": dm,
            "diff_ci": [l2, h2],
            "effect": eff,
            "p_holm": padj,
        }
    lines.append("")
lines += ["## invalid runs (excluded with their pair)", ""] + [f"- {c}: {', '.join(v)}" for c, v in invalid.items() if v]
open(a.out + ".md", "w").write("\n".join(lines) + "\n")
json.dump(result, open(a.out + ".json", "w"), indent=1)
print("\n".join(lines[:40]))
