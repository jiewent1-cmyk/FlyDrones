"""Patrol regression pf1 (fence turn skipped when forward <= 0) vs g7_sc2_tf10, same worlds/seeds. Run on ROG."""
import json, glob, os, statistics as st
R = os.path.expanduser("~/sim/runs")
def load(pfx):
    out = {}
    for p in glob.glob(f"{R}/{pfx}__*/run.json"):
        try: out[p.split("/")[-2].split("__", 1)[1]] = json.load(open(p))
        except Exception: pass
    return out
new, base = load("pf1"), load("g7_sc2_tf10")
keys = sorted(set(new) & set(base))
print(f"pf1 runs: {len(new)}, paired with baseline: {len(keys)}")
for grp in ("lowbox", "cage20"):
    K = [k for k in keys if k.startswith(grp)]
    if not K: continue
    for name, D in (("pf1 ", new), ("base", base)):
        g = lambda f: [D[k].get(f) for k in K if D[k].get(f) is not None]
        print(f"{grp:7s} {name}: n={len(K)} contacts={sum(g('contact_episodes'))} runs_w_contact={sum(1 for x in g('contact_episodes') if x)} "
              f"near={sum(g('near_miss_episodes'))} minclr med={st.median(g('min_clearance_m')):.2f} min={min(g('min_clearance_m')):.2f} "
              f"path={sum(g('path_m')):.0f} cov={st.mean(g('coverage')):.3f} r_true_max={max(g('radius_true_max_m')):.2f} "
              f"fence_turn_s={sum(g('fence_turn_s')):.0f} escapes={sum(g('escapes'))}")
