"""Export each run's best held-out-validation point as a variant YAML (base + optimised decoder [+ bypass gains]).

Search space and arm are read from the run's config.json.
python rl/export_arms.py                      (A1 / B2 runs)
python rl/export_arms.py a1v2_bypass b2v2_s1  (any runs under rl/out)
"""
import json
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, ".")
from rl import params
from rl.episode import deep_merge

DEFAULT = ["a1_intact", "a1_bypass", "a1_shuffle1", "a1_shuffle2", "a1_shuffle3", "a1_shuffle4", "a1_shuffle5", "a1_randread1",
           "b2_wcov2", "b2_wcov4", "b2_wcov8"]

for run in sys.argv[1:] or DEFAULT:
    d = Path("rl/out") / run
    if not (d / "best_val_z.npy").exists():
        print(run, "MISSING")
        continue
    cfg = json.load(open(d / "config.json"))
    arm = cfg.get("arm", "intact")
    base = yaml.safe_load(open(cfg.get("base") or "minifly/v3_2.yaml"))  # H2 runs are based on minifly/v4.yaml
    params.set_space(cfg.get("space", "wide"))
    z = np.load(d / "best_val_z.npy")
    bv = json.load(open(d / "best_val.json"))
    merged = deep_merge(base, params.full_overrides(z, arm))
    head = f"# {run} (arm {arm}, space {cfg.get('space', 'wide')}) best held-out validation gen {bv['gen']}: F={bv['mean']['F']} pC={bv['mean']['p_contact']} cov={bv['mean']['coverage']}\n"
    head += "".join(f"#   {k}: {v}\n" for k, v in params.describe_full(z, arm).items())
    Path(f"minifly/{run}.yaml").write_text(head + yaml.safe_dump(merged, sort_keys=False))
    print(run, arm, "gen", bv["gen"], "F", bv["mean"]["F"], "pC", bv["mean"]["p_contact"], "contacts", bv["mean"]["contacts"], "cov", bv["mean"]["coverage"])
