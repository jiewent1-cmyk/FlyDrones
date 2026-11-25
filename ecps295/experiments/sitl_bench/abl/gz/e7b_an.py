"""e7b (2026-11-22): e7 again with SIM_FLOW_RND 0.25. Landmark fix off / on, 240 s hold. Corrected estimate = EKF - (lm_dn, lm_de)."""
import csv, json, math, sys, statistics as st, collections
from pathlib import Path
from scipy.stats import wilcoxon
R = Path.home() / "sim/runs"; PRE = sys.argv[1] if len(sys.argv) > 1 else "e7b__"
def q(v, p): v = sorted(v); return v[min(len(v) - 1, int(p * len(v)))]
T = (30, 60, 120, 180, 230)
D = {"off": {}, "on": {}}
f = lambda r, k: float(r[k]) if r.get(k) not in ("", "None", None) else 0.0
for d in sorted(R.glob(PRE + "*")):
    key, _, m = d.name[len(PRE):].rpartition("_")
    allr = list(csv.DictReader(open(d / "run.csv")))
    i0 = next((i for i, r in enumerate(allr) if r.get("trk") == "ARRIVED"), None)
    j = json.load(open(d / "run.json"))
    if i0 is None:
        print("never arrived", d.name, collections.Counter(r.get("trk") for r in allr)); continue
    t0 = f(allr[i0], "t") + 5; rows = [r for r in allr[i0:] if f(r, "t") >= t0]
    n0, e0 = f(rows[0], "north"), f(rows[0], "east")
    st_ = collections.Counter(r.get("trk") for r in rows)
    lm = j.get("landmark") or {}
    o = {"contact": j["contact_episodes"], "fixes": lm.get("fixes"), "rej": lm.get("rejected"), "dur": f(rows[-1], "t") - t0,
         "lost": 1 - st_.get("ARRIVED", 0) / len(rows), "land": bool(j.get("fc_land")), "states": dict(st_)}
    for T1 in T:
        r = min(rows, key=lambda r: abs(f(r, "t") - t0 - T1))
        w = [x for x in rows if f(x, "t") - t0 <= T1]
        o[T1] = dict(drift=math.hypot(f(r, "north") - n0, f(r, "east") - e0),
                     dmax=max(math.hypot(f(x, "north") - n0, f(x, "east") - e0) for x in w),
                     ekf=math.hypot(f(r, "ekf_north") - f(r, "north"), f(r, "ekf_east") - f(r, "east")),
                     cor=math.hypot(f(r, "ekf_north") - f(r, "lm_dn") - f(r, "north"), f(r, "ekf_east") - f(r, "lm_de") - f(r, "east")))
    D[m][key] = o
K = sorted(set(D["off"]) & set(D["on"]))
print({m: len(v) for m, v in D.items()}, "pairs", len(K))
for m in D:
    v = D[m].values()
    print(m, "contacts", sum(o["contact"] for o in v), "fc_land", sum(o["land"] for o in v), "dur min %.0f" % min(o["dur"] for o in v),
          "runs with non-ARRIVED rows", sum(o["lost"] > 0 for o in v), "lost frac med %.3f max %.3f" % (st.median(o["lost"] for o in v), max(o["lost"] for o in v)),
          "fixes med", st.median(o["fixes"] or 0 for o in v), "rejected", sum(o["rej"] or 0 for o in v))
for k, a_, b_ in (("drift", "drift", "drift"), ("dmax", "dmax", "dmax"), ("ekf", "ekf", "ekf"), ("used", "ekf", "cor")):
    for T1 in T:
        a = [D["off"][x][T1][a_] for x in K]; b = [D["on"][x][T1][b_] for x in K]
        try: p = wilcoxon(a, b).pvalue
        except Exception: p = float("nan")
        print(f"{k:6s} t={T1:3d}s off med {st.median(a):.3f} p90 {q(a,.9):.3f} max {max(a):.3f} | on med {st.median(b):.3f} p90 {q(b,.9):.3f} max {max(b):.3f} | on better {sum(y < x for x, y in zip(a, b))}/{len(a)} p={p:.3g}")
for x in K:
    if D["on"][x]["lost"] > 0.05 or D["off"][x]["lost"] > 0.05: print("  lost", x, D["off"][x]["states"], D["on"][x]["states"])
