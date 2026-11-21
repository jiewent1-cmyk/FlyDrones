"""python pairs.py RUNS REGEX REF ARM [ARM..]   REGEX has groups (arm, key, subset)"""
import glob, json, os, re, sys
import numpy as np
sys.path.insert(0, os.environ.get("TREE", "ecps295"))
from rl.stats import compare, holm
from rl.trigger_metrics import from_run_csv
runs, rx, ref, arms = sys.argv[1], re.compile(sys.argv[2]), sys.argv[3], sys.argv[4:]
D = {}
for p in glob.glob(os.path.join(runs, "*", "run.json")):
    m = rx.match(os.path.basename(os.path.dirname(p)))
    if not m: continue
    arm, key, sub = m.group(1), m.group(2), m.group(3)
    j = json.load(open(p)); j["contacts_per_100m"] = 100.0 * j["contact_episodes"] / max(1.0, j["path_m"])
    c = p.replace("run.json", "run.csv")
    try: j["fa"] = from_run_csv(c).get("false_alarms_per_min", np.nan)
    except Exception: j["fa"] = np.nan
    D.setdefault(sub, {}).setdefault(arm, {})[key] = j
for sub, A in sorted(D.items()):
    print("=====", sub, {a: len(v) for a, v in A.items()})
    if ref not in A: continue
    for a in arms:
        if a not in A: continue
        r = compare(A[ref], A[a]); s = sorted(set(A[ref]) & set(A[a]))
        fa = [np.nanmean([A[x][k]["fa"] for k in s]) for x in (ref, a)]
        print(f"  {a:10s} n={r['n']} contact_runs {r['contact_runs']} saved {r['saved']} new {r['new']} p={r['mcnemar_p']:.3f}  FA/min {fa[0]:.2f}->{fa[1]:.2f}")
        for k in ("contacts_per_100m", "near_miss_episodes", "coverage", "min_clearance_m"):
            v = r[k]; print(f"      {k:20s} {v['ref']:.3f} -> {v['cand']:.3f}  d={v['diff']:+.3f} CI[{v['ci'][0]:+.3f},{v['ci'][1]:+.3f}] p={v['p']:.3g}")
