"""PIL hover-hold comparison (2026-10-21): pil4p/pil4v (+ pil5p/pil5v reruns) vs Gazebo-local hb2 / hp0, paired by world."""
import csv, glob, json, math, sys
import numpy as np
from scipy.stats import wilcoxon
R = "/home/jacky/sim/runs"; SKIP = set(sys.argv[1].split(",")) if len(sys.argv) > 1 and sys.argv[1] else set()
REPL = dict(a.split("=") for a in sys.argv[2:])  # e.g. pil4p=pil5p: take that tag's run where it exists

def segs(p, settle):
    rows = list(csv.DictReader(open(p))); out = []; st = None
    for i, r in enumerate(rows + [None]):
        h = r is not None and math.hypot(float(r["cmd_forward"]), float(r["cmd_lateral"])) * 0.5 < 0.03
        if h and st is None: st = i
        elif not h and st is not None:
            s = rows[st:i]; t0 = float(s[0]["t"])
            if float(s[-1]["t"]) - t0 >= (3 if settle else 1):
                s2 = [x for x in s if float(x["t"]) >= t0 + (1.5 if settle else 0)]
                n0, e0 = float(s2[0]["north"]), float(s2[0]["east"])
                out.append(max(math.hypot(float(x["north"]) - n0, float(x["east"]) - e0) for x in s2))
            st = None
    return out

def load(tag):
    D = {}
    tags = [tag] + ([REPL[tag]] if tag in REPL else [])
    for t in tags:
        for p in glob.glob(f"{R}/{t}__*/run.json"):
            w = p.split("__", 1)[1].split("/")[0]
            if t == tag and w in SKIP: continue
            s = json.load(open(p)); s = s.get("summary", s)
            if "coverage" not in s: continue
            c = p.replace("run.json", "run.csv"); a, b = segs(c, False), segs(c, True)
            rows = list(csv.DictReader(open(c)))
            ctl = np.mean([float(x.get("pil_ctl") or 0) for x in rows]) if "pil_ctl" in rows[0] else 1.0
            D[w] = dict(contact=s["contact_episodes"] > 0, nm=s["near_miss_episodes"] > 0, coverage=s["coverage"],
                        path=s["path_m"], rtf=s.get("rtf_run", np.nan), ctl=ctl,
                        d_med=np.median(a) if a else np.nan, d_max=max(a) if a else np.nan,
                        s_med=np.median(b) if b else np.nan, s_max=max(b) if b else np.nan)
    return D

T = {t: load(t) for t in ("pil4p", "pil4v", "hb2", "hp0")}
for t, D in T.items():
    V = list(D.values())
    print(f"{t}: n={len(V)} contacts={sum(v['contact'] for v in V)} nm_runs={sum(v['nm'] for v in V)} "
          f"rtf_min={min(v['rtf'] for v in V):.3f} ctl_min={min(v['ctl'] for v in V):.3f}")
for a, b in (("pil4v", "pil4p"), ("hp0", "pil4v"), ("hb2", "pil4p")):
    W = sorted(set(T[a]) & set(T[b])); print(f"== {a} -> {b}  n={len(W)}")
    for k, sc in (("d_med", 100), ("d_max", 100), ("s_med", 100), ("s_max", 100), ("coverage", 1), ("path", 1)):
        x = np.array([T[a][w][k] for w in W]); y = np.array([T[b][w][k] for w in W]); m = ~(np.isnan(x) | np.isnan(y))
        p = wilcoxon(y[m] - x[m]).pvalue if m.sum() > 5 and np.any(y[m] != x[m]) else float("nan")
        print(f"   {k:8s} {np.median(x[m]) * sc:7.2f} -> {np.median(y[m]) * sc:7.2f}  p={p:.3g}")
