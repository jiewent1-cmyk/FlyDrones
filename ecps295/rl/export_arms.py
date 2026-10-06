"""Export each A1/B2 run's best held-out-validation point as a variant YAML (v3_2 + optimised decoder [+ bypass gains])."""
import json
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, ".")
from rl import params
from rl.episode import deep_merge

base = yaml.safe_load(open("minifly/v3_2.yaml"))
params.set_space("wide")
for run, arm in [("a1_intact", "intact"), ("a1_bypass", "bypass"), ("a1_shuffle1", "shuffle1"), ("a1_shuffle2", "shuffle2"), ("a1_shuffle3", "shuffle3"),
                 ("a1_shuffle4", "shuffle4"), ("a1_shuffle5", "shuffle5"), ("a1_randread1", "randread1"), ("b2_wcov2", "intact"), ("b2_wcov4", "intact"), ("b2_wcov8", "intact")]:
    d = Path("rl/out") / run
    if not (d / "best_val_z.npy").exists():
        print(run, "MISSING")
        continue
    z = np.load(d / "best_val_z.npy")
    bv = json.load(open(d / "best_val.json"))
    merged = deep_merge(base, params.full_overrides(z, arm))
    head = f"# {run} (arm {arm}) best held-out validation gen {bv['gen']}: F={bv['mean']['F']} pC={bv['mean']['p_contact']} cov={bv['mean']['coverage']}\n"
    head += "".join(f"#   {k}: {v}\n" for k, v in params.describe_full(z, arm).items())
    Path(f"minifly/{run}.yaml").write_text(head + yaml.safe_dump(merged, sort_keys=False))
    print(run, "gen", bv["gen"], "F", bv["mean"]["F"], "pC", bv["mean"]["p_contact"], "cov", bv["mean"]["coverage"])
