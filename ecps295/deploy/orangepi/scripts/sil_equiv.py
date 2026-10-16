"""Software-in-the-loop equivalence on the Orange Pi: re-run fast-twin episodes of the final config (s7_w2map) that the
Mac mini already ran (rl/out/s6.jsonl, shield_screen) and compare every metric field by field.

Run from the twin tree:  cd /opt/ecps295/twin/ecps295 && PYTHONPATH=../src:. python sil_equiv.py REF.jsonl OUT.jsonl
    [--n-lowbox 10] [--n-cage 4] [--n-empty 2] [--procs 6]
Prints the number of identical episodes, any differing fields, and the per-episode wall time on the Pi.
"""

from __future__ import annotations

import argparse
import json
import math

from rl import hw_arms
from rl.batch import run_jobs
from rl.shield_screen import CONDS

ap = argparse.ArgumentParser()
ap.add_argument("ref")
ap.add_argument("out")
ap.add_argument("--n-lowbox", type=int, default=10)
ap.add_argument("--n-cage", type=int, default=4)
ap.add_argument("--n-empty", type=int, default=2)
ap.add_argument("--procs", type=int, default=6)
a = ap.parse_args()

REF = {(r["set"], r["key"]): r for r in map(json.loads, open(a.ref)) if r.get("tag") == "s7_w2map"}
want = {"lowbox": a.n_lowbox, "cage20": a.n_cage, "empty": a.n_empty}
hw_arms.COND.clear()
hw_arms.COND.update({"s7_w2map": CONDS["s7_w2map"][:2]})
J = []
for j in hw_arms.jobs():
    if (j["set"], j["key"]) not in REF or want[j["set"]] <= 0:
        continue
    want[j["set"]] -= 1
    c = CONDS["s7_w2map"]
    J.append({**j, "variant": c[0], "arm": c[1], "overrides": c[2], "dyn_over_local": j["dyn_over"],
              "dyn_over": REF[(j["set"], j["key"])]["dyn_over"]})  # the reference's exact randomised values
print(len(J), "episodes", flush=True)
res = run_jobs(J, a.procs, a.out)
same, diff = 0, []
ulp = sum(j["dyn_over_local"] != j["dyn_over"] for j in J)
print(f"flow_dr on this machine differs from the reference in {ulp}/{len(J)} jobs (libm last-bit differences)")
for r in res:
    r = json.loads(json.dumps(r))  # same JSON normalisation as the reference file
    ref = REF[(r["set"], r["key"])]
    bad = []
    for k, v in ref.items():
        if k in ("wall_s", "variant"):
            continue
        w = r.get(k)
        if isinstance(v, float) and isinstance(w, float) and math.isnan(v) and math.isnan(w):
            continue
        if v != w:
            bad.append((k, v, w))
    if bad:
        diff.append((r["set"], r["key"], bad[:5]))
    else:
        same += 1
print(f"identical {same}/{len(res)}")
for d in diff:
    print("DIFF", d)
ws = sorted(r.get("wall_s", float("nan")) for r in res if r["set"] != "empty")
print("wall_s per 120 s episode on the Pi (procs=%d): median %.1f  max %.1f" % (a.procs, ws[len(ws) // 2], ws[-1]))
