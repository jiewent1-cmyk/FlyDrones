"""Paired statistics for Gazebo arms (RL roadmap A2): every arm flies the same seeds, so compare seed by seed.

  binary   contact run (any contact episode): exact McNemar on discordant seeds, plus "saved" (reference had a contact,
           candidate none) and "new" (candidate had a contact, reference none)
  paired   contacts, contacts per 100 m, near misses, coverage, min clearance: mean paired difference with a 95%
           bootstrap CI (10,000 resamples of seeds) and a two-sided Wilcoxon signed-rank p
  Holm     within each metric across all candidate-vs-reference comparisons (as S8)
  shuffle  --family shuffle: the real wiring against K rewired instances -> rank of the reference among instance means
           and a two-level bootstrap (instances, then seeds) of "shuffled minus reference"

python -m rl.stats --runs ~/sim/runs --ref fix_v3_2 --arms fix_v4 fix_v4oc5 fix_es3
python -m rl.stats --runs ~/sim/runs --ref a1_intact --arms a1_bypass a1_brainoff ... --family a1_shuffle1 ... a1_shuffle5
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import re

import numpy as np
from scipy.stats import binomtest, wilcoxon

METRICS = [
    ("contact_episodes", "contacts / run", -1),
    ("contacts_per_100m", "contacts / 100 m", -1),
    ("near_miss_episodes", "near misses / run", -1),
    ("coverage", "coverage", +1),
    ("min_clearance_m", "min clearance m", +1),
]


def load(runs: str, group: str) -> dict[int, dict]:
    out = {}
    for p in glob.glob(os.path.join(runs, f"{group}_s*", "run.json")):
        seed = int(re.search(r"_s(\d+)$", os.path.dirname(p)).group(1))
        j = json.load(open(p))
        j["contacts_per_100m"] = 100.0 * j["contact_episodes"] / max(1.0, j["path_m"])
        out[seed] = j
    return out


def boot_ci(d: np.ndarray, n: int = 10_000, seed: int = 0) -> tuple[float, float]:
    rng = np.random.default_rng(seed)
    m = d[rng.integers(0, d.size, (n, d.size))].mean(axis=1)
    return float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))


def holm(ps: list[float]) -> list[float]:
    order = np.argsort(ps)
    adj, run = [0.0] * len(ps), 0.0
    for rank, i in enumerate(order):
        run = max(run, min(1.0, (len(ps) - rank) * ps[i]))
        adj[i] = run
    return adj


def compare(ref: dict, cand: dict) -> dict:
    seeds = sorted(set(ref) & set(cand))
    r = {"n": len(seeds)}
    a = np.array([ref[s]["contact_episodes"] > 0 for s in seeds])
    b = np.array([cand[s]["contact_episodes"] > 0 for s in seeds])
    saved, new = int((a & ~b).sum()), int((~a & b).sum())
    r["contact_runs"] = (int(a.sum()), int(b.sum()))
    r["saved"], r["new"] = saved, new
    r["mcnemar_p"] = binomtest(saved, saved + new, 0.5).pvalue if saved + new else 1.0
    for key, _, _ in METRICS:
        x = np.array([ref[s][key] for s in seeds], float)
        y = np.array([cand[s][key] for s in seeds], float)
        d = y - x
        lo, hi = boot_ci(d)
        p = wilcoxon(d).pvalue if np.any(d != 0) else 1.0
        r[key] = {"ref": x.mean(), "cand": y.mean(), "diff": d.mean(), "ci": (lo, hi), "p": float(p)}
    return r


def shuffle_family(ref: dict, family: list[dict], key: str = "contacts_per_100m", n: int = 10_000) -> dict:
    """Rank of the reference among instance means + bootstrap over instances then seeds of mean(shuffled - reference)."""
    inst_means = [np.mean([f[s][key] for s in f if s in ref]) for f in family]
    ref_mean = np.mean([ref[s][key] for s in ref])
    rng = np.random.default_rng(1)
    diffs = []
    for _ in range(n):
        f = family[rng.integers(len(family))]
        seeds = [s for s in f if s in ref]
        pick = rng.choice(seeds, len(seeds))
        diffs.append(np.mean([f[s][key] - ref[s][key] for s in pick]))
    lo, hi = np.percentile(diffs, [2.5, 97.5])
    better = sum(ref_mean < m for m in inst_means) if key != "coverage" else sum(ref_mean > m for m in inst_means)
    return {"metric": key, "ref": ref_mean, "instances": inst_means, "ref_better_than": f"{better}/{len(family)}", "diff_ci": (lo, hi)}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", default=os.path.expanduser("~/sim/runs"))
    ap.add_argument("--ref", required=True)
    ap.add_argument("--arms", nargs="*", default=[])
    ap.add_argument("--family", nargs="*", default=[], help="rewired instances of the reference (shuffle<k>)")
    a = ap.parse_args()
    ref = load(a.runs, a.ref)
    res = {g: compare(ref, load(a.runs, g)) for g in a.arms + a.family}
    print(f"reference {a.ref}: n={len(ref)}\n")
    print("| arm | n | contact runs ref→arm | saved / new | McNemar p (Holm) |")
    print("|---|---|---|---|---|")
    padj = dict(zip(res, holm([r["mcnemar_p"] for r in res.values()]))) if res else {}
    for g, r in res.items():
        print(f"| {g} | {r['n']} | {r['contact_runs'][0]} → {r['contact_runs'][1]} | {r['saved']} / {r['new']} | {r['mcnemar_p']:.3f} ({padj[g]:.3f}) |")
    for key, label, _ in METRICS:
        print(f"\n{label}: mean paired difference arm − ref [95% bootstrap CI], Wilcoxon p (Holm)")
        padj = dict(zip(res, holm([r[key]["p"] for r in res.values()]))) if res else {}
        for g, r in res.items():
            m = r[key]
            print(f"  {g:14s} {m['ref']:.3f} → {m['cand']:.3f}  diff {m['diff']:+.3f} [{m['ci'][0]:+.3f}, {m['ci'][1]:+.3f}]  p={m['p']:.4f} ({padj[g]:.4f})")
    if a.family:
        fam = [load(a.runs, g) for g in a.family]
        for key in ("contacts_per_100m", "coverage"):
            s = shuffle_family(ref, fam, key)
            print(f"\nshuffle family ({key}): ref {s['ref']:.3f}, instances {[round(x, 3) for x in s['instances']]}, "
                  f"ref better than {s['ref_better_than']}, shuffled − ref 95% CI [{s['diff_ci'][0]:+.3f}, {s['diff_ci'][1]:+.3f}]")


if __name__ == "__main__":
    main()
