"""N4 twin fidelity check (2026-10-15): the rl.n3m_eval reference job (s7_w2map, lowbox_s0..49, brain seed k + b,
flow model hw_arms.flow_dr(k + b)) with optional TwinDrone extras, to compare old / new twin models with Gazebo g7_w2map
on the same layouts and seeds. python -m rl.fid_test <out.jsonl> <procs> '<extras json>' <b> [<b> ...]"""
import json
import sys

from rl import hw_arms
from rl import shield_screen as ss
from rl.batch import run_jobs

if __name__ == "__main__":
    out, procs, extra = sys.argv[1], int(sys.argv[2]), json.loads(sys.argv[3])
    J = []
    for b in map(int, sys.argv[4:]):
        for k in range(50):
            J.append({"tag": "fid", "variant": ss.V4, "arm": "intact", "overrides": ss.CONDS["s7_w2map"][2], "seconds": 120.0, "set": "lowbox",
                      "key": f"{k}_{b}", "seed": k + b, "world": f"lowbox_s{k}", "dyn_over": {**hw_arms.flow_dr(k + b), **extra}, "extra": extra})
    run_jobs(J, procs, out)
