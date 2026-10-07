"""B1: empty-field saccade rate (nets only; every saccade is a false alarm). 32 seeds x 60 s per variant."""
import numpy as np

from rl.batch import run_jobs

variants = {"v3_2": "v3_2", "es3": "minifly/v3_2_es3.yaml", "a1_intact": "minifly/a1_intact.yaml"}
jobs = [{"tag": k, "variant": v, "world": f"empty:{s}", "seed": s, "yaw_deg": (s * 37) % 360, "seconds": 60} for k, v in variants.items() for s in range(32)]
res = run_jobs(jobs, 12, "rl/out/b1_empty.jsonl")
for k in variants:
    rs = [r for r in res if r.get("tag") == k and "error" not in r]
    sac = np.array([r["saccade_onsets"] for r in rs])
    turns = [x for r in rs for x in r["escape_triggers"]]
    print(f"{k:10s} n={len(rs)} saccades/min={sac.mean():.2f} (runs with any: {(sac > 0).mean():.0%})  fa/min={np.mean([r['false_alarms_per_min'] for r in rs]):.2f}  cov={np.mean([r['coverage'] for r in rs]):.3f}  speed={np.mean([r['mean_speed'] for r in rs]):.3f}  triggers={dict((t, turns.count(t)) for t in set(turns))}")
print("errors:", sum("error" in r for r in res))
