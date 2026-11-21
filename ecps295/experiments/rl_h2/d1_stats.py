"""D-stage Gazebo: notex (lowbox_s0-19) vs in-distribution lowbox runs, ghost cage20 vs cage20; seed level bypass vs intact."""
import glob, json, os, re
import numpy as np
from scipy.stats import mannwhitneyu, binomtest
R = os.path.expanduser("~/sim/runs")
def load(name):
    p = os.path.join(R, name, "run.json")
    return json.load(open(p)) if os.path.exists(p) else None
C = ["v4"] + [f"c4_s{i}" for i in range(1, 9)] + [f"h2_s{i}" for i in range(1, 9)]
def base(c, k):
    for pre in ("mc1", "mc4", "fx", "mdb"):
        r = load(f"{pre}_{c}__lowbox_s{k}_k{k}")
        if r: return r
def runs(c, k, w):
    return load(f"md_{c}__{w}_k{k}")
def summary(title, pairs_fn, cfgs, ks):
    print(f"===== {title}")
    V = {}
    for c in cfgs:
        a, b = [], []
        for k in ks:
            x, y = pairs_fn(c, k)
            if x is None or y is None: continue
            a.append(x["contact_episodes"] > 0); b.append(y["contact_episodes"] > 0)
        a, b = np.array(a), np.array(b)
        sv, nw = int((a & ~b).sum()), int((~a & b).sum())
        p = binomtest(nw, sv + nw).pvalue if sv + nw else 1.0
        V[c] = (int(a.sum()), int(b.sum()), len(a))
        print(f"  {c:7s} n={len(a):2d} in-dist {a.sum():2d} -> ood {b.sum():2d}  (+{nw} / -{sv}, McNemar p={p:.3f})")
    for what, f in (("ood contact runs", lambda v: v[1]), ("change vs in-dist", lambda v: v[1] - v[0]), ("in-dist", lambda v: v[0])):
        g = {s: [f(V[c]) for c in cfgs if c.startswith(s)] for s in ("c4", "h2")}
        p = mannwhitneyu(g["c4"], g["h2"], method="exact").pvalue
        print(f"  seed level {what:18s}: bypass {g['c4']} median {np.median(g['c4']):+.1f} | intact {g['h2']} median {np.median(g['h2']):+.1f} | p={p:.3f}")
summary("notex lowbox (Gazebo)", lambda c, k: (base(c, k), runs(c, k, f"lowbox_s{k}_notex")), C, range(int(os.environ.get("NW", 20))))
G = ["v4"] + [f"c4_s{i}" for i in range(1, 5)] + [f"h2_s{i}" for i in range(1, 5)]
summary("ghost cage20 x 8 (Gazebo)", lambda c, k: (runs(c, k, "cage20"), runs(c, k, "cage20_ghost")), G, range(8))
