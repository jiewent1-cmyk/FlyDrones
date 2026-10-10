"""Twin pre-screen in the hardware-aligned setting (flow nav with per-episode DR, cruise 0.8 m): A1 control arms on the
v4 base + C1/H3 candidates. Decides which arms are worth a Gazebo run for the paper (C0/C2/C4 retest under H0).

  lowbox_s0..49 x brain seeds {k, k+100, k+200}  (120 s)   cage20 x 32 (120 s)   empty field x 32 (60 s, all saccades false)
python -m rl.hw_arms --procs 10 [--out rl/out/hw_arms.jsonl]
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.stats import binomtest, wilcoxon

from rl.batch import run_jobs
from rl.stats import boot_ci
from rl.twin import flow_dr

E = Path(__file__).resolve().parent.parent
V4 = str(E / "minifly" / "v4.yaml")
COND = {"v4": (V4, "intact"), "brainoff": (V4, "brainoff"), "noloom": (V4, "noloom"),
        **{f"shuffle{k}": (V4, f"shuffle{k}") for k in range(1, 6)},
        "randread1": (V4, "randread1"), "randread2": (V4, "randread2"),
        "v4_eff": (str(E / "minifly" / "v4_eff.yaml"), "intact"),
        "h2_s3": (str(E / "minifly" / "h2_s3.yaml"), "intact"),
        "h2_s3_eff": (str(E / "minifly" / "h2_s3_eff.yaml"), "intact")}


def jobs() -> list[dict]:
    J = []
    for c, (v, arm) in COND.items():
        for k in range(50):
            for b in (0, 100, 200):
                J.append({"tag": c, "set": "lowbox", "key": f"{k}_{b}", "variant": v, "arm": arm, "seed": k + b, "seconds": 120.0,
                          "world": f"lowbox_s{k}", "dyn_over": flow_dr(k + b)})
        for s in range(32):
            J.append({"tag": c, "set": "cage20", "key": str(s), "variant": v, "arm": arm, "seed": s, "seconds": 120.0,
                      "world": "cage20", "dyn_over": flow_dr(1000 + s)})
            J.append({"tag": c, "set": "empty", "key": str(s), "variant": v, "arm": arm, "seed": s, "seconds": 60.0,
                      "world": f"empty:{s}", "yaw_deg": (s * 37) % 360, "dyn_over": flow_dr(2000 + s)})
    return J


def report(res: list[dict]) -> None:
    D = {}
    for r in res:
        if "error" in r:
            continue
        r["cp100"] = 100.0 * r["contact_episodes"] / max(1.0, r["path_m"])
        D.setdefault(r["set"], {}).setdefault(r["tag"], {})[r["key"]] = r
    for st in ("lowbox", "cage20"):
        ref = D[st]["v4"]
        print(f"===== {st} (paired vs v4 intact)")
        for c in COND:
            A = D[st].get(c, {})
            ks = sorted(set(ref) & set(A))
            a = np.array([ref[k]["contact_episodes"] > 0 for k in ks]); b = np.array([A[k]["contact_episodes"] > 0 for k in ks])
            sv, nw = int((a & ~b).sum()), int((~a & b).sum())
            p = binomtest(sv, sv + nw).pvalue if sv + nw else 1.0
            line = f"  {c:10s} n={len(ks):3d} contact_runs {int(b.sum()):3d} (v4 {int(a.sum())}) McN p={p:.3f}"
            for m in ("cp100", "coverage", "min_clearance_m"):
                d = np.array([A[k][m] - ref[k][m] for k in ks])
                lo, hi = boot_ci(d) if len(d) else (0, 0)
                pw = wilcoxon(d).pvalue if np.any(d != 0) else 1.0
                line += f" | {m} {np.mean([A[k][m] for k in ks]):.3f} d{np.mean(d):+.3f}[{lo:+.3f},{hi:+.3f}] p={pw:.2g}"
            print(line)
    print("===== empty field (every saccade is false)")
    for c in COND:
        A = list(D["empty"].get(c, {}).values())
        print(f"  {c:10s} n={len(A)} FA/min {np.mean([r['false_alarms_per_min'] for r in A]):.2f}  saccades/min {np.mean([r['saccade_onsets'] for r in A]):.2f}  cov {np.mean([r['coverage'] for r in A]):.3f}")
    print("errors:", sum("error" in r for r in res))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--procs", type=int, default=10)
    ap.add_argument("--out", default=str(E / "rl" / "out" / "hw_arms.jsonl"))
    ap.add_argument("--report", action="store_true", help="only summarise an existing --out")
    a = ap.parse_args()
    if a.report:
        report([json.loads(l) for l in open(a.out)]); return
    done = set()
    if Path(a.out).exists():
        done = {(r["tag"], r["set"], r["key"]) for r in map(json.loads, open(a.out)) if "error" not in r}
    J = [j for j in jobs() if (j["tag"], j["set"], j["key"]) not in done]
    print(len(J), "jobs to run", flush=True)
    run_jobs(J, a.procs, a.out)
    report([json.loads(l) for l in open(a.out)])


if __name__ == "__main__":
    main()
