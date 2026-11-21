"""N3 round 2 (2026-10-15): turn a CMA candidate into a Gazebo variant on ROG.
Input: JSON {"name": ..., "source": ..., "overrides": {...}}: a CMA candidate (params.full_overrides: decoder / ToF / ventral)
or any other s7_w2map variant (e.g. N4 map band_lo_m).
Output: ~/sim/rl_h2/ecps295/minifly/<name>.yaml = minifly/s7_w2map.yaml deep-merged with the overrides, and the cruise to
pass to run_g4.py (--cruise / --max-forward come from the command line there, so they are printed, not only stored).
usage (on ROG): python3 gz_export.py cand.json"""
import copy
import json
import sys
from pathlib import Path

import yaml

M = Path.home() / "sim" / "rl_h2" / "ecps295" / "minifly"


def deep_merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        out[k] = deep_merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else copy.deepcopy(v)
    return out


c = json.load(open(sys.argv[1]))
name, ov = c["name"], c["overrides"]
base = yaml.safe_load(open(M / "s7_w2map.yaml"))
cfg = deep_merge(base, ov)
dst = M / f"{name}.yaml"
assert not dst.exists(), f"{dst} exists"
cr = (ov.get("decoder") or {}).get("cruise")
head = f"# {name}: s7_w2map + {c.get('source', '')}" + (f"; cruise {cr:.3f} -> pass --cruise/--max-forward" if cr else "") + "\n"
dst.write_text(head + yaml.safe_dump(cfg, sort_keys=False))
print(name, "cruise", round(cr, 3) if cr else "unchanged (matrix passes 0.5)")
