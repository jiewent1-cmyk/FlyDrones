"""S6 batch 2 (2026-10-14): more brain seeds for the leading sensor layouts and a camera-model sensitivity check.
Same episode recipe as rl.hw_arms (flow nav with per-episode dynamics randomisation), new seeds only:
  lowbox_s0..49 x brain seeds {k+300, k+400}   cage20 seeds 32..63
Conds: rl.shield_screen CONDS plus
  s6_camm_bad  camera + memory with a pessimistic model (sigma 0.03 + 0.10 d^2, nothing beyond 1.2 m, 30 % of readings
               lost, each net wall seen with p 0.2)
python rl/out/s6_more.py --conds v4 s5 s6_m1m ... --procs 10 --out rl/out/s6b.jsonl"""
import argparse, json
from pathlib import Path
from rl import hw_arms, shield_screen as ss
from rl.batch import run_jobs
ss.CONDS["s6_camm_bad"] = (ss.V4, "intact", ss.s6(ss.CAM12, 10.0, rate=15.0, mem=True, noise=0.03, noise_k=0.10, max_m=1.2, p_miss=0.3, p_net=0.2))
ss.CONDS["v4"] = (ss.V4, "intact", {})
ap = argparse.ArgumentParser()
ap.add_argument("--conds", nargs="+", required=True)
ap.add_argument("--procs", type=int, default=10)
ap.add_argument("--out", required=True)
a = ap.parse_args()
J = []
for c in a.conds:
    v, arm, ov = ss.CONDS[c]
    for k in range(50):
        for b in (300, 400):
            J.append({"tag": c, "set": "lowbox", "key": f"{k}_{b}", "variant": v, "arm": arm, "seed": k + b, "seconds": 120.0,
                      "world": f"lowbox_s{k}", "dyn_over": hw_arms.flow_dr(k + b), "overrides": ov})
    for s in range(32, 64):
        J.append({"tag": c, "set": "cage20", "key": str(s), "variant": v, "arm": arm, "seed": s, "seconds": 120.0,
                  "world": "cage20", "dyn_over": hw_arms.flow_dr(1000 + s), "overrides": ov})
out = Path(a.out)
if out.exists():
    done = {(r["tag"], r["set"], r["key"]) for r in map(json.loads, open(out)) if "error" not in r}
    J = [j for j in J if (j["tag"], j["set"], j["key"]) not in done]
print(len(J), "jobs to run", flush=True)
run_jobs(J, a.procs, str(out))
