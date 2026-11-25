"""Gazebo experiments 6 and 7 (2026-11-22). e6: front ToF (w1map) vs front + rear ToF (w2map), paired by world and seed.
e7: landmark fix off / on, 240 s hold: true drift from the hold point and estimate error over time."""
import csv, json, math, statistics as st
from pathlib import Path
from scipy.stats import wilcoxon
R = Path.home() / "sim/runs"
def q(v, p): v = sorted(v); return v[min(len(v) - 1, int(p * len(v)))]
print("== e6 ==")
P = {}
for d in sorted(R.glob("e6__*")):
    key, _, c = d.name[4:].rpartition("__")
    j = json.load(open(d / "run.json"))
    P.setdefault(key, {})[c] = j
print(len(P), "pairs; sim/wall min", min(j["sim_s"] / j["wall_s"] for v in P.values() for j in v.values()))
for c in ("w1map", "w2map"):
    J = [v[c] for v in P.values()]
    n = len(J)
    print(c, "n", n, "trk_pass", sum(bool(j.get("trk_pass")) for j in J), "contact runs", sum(j["contact_episodes"] > 0 for j in J),
          "contacts", sum(j["contact_episodes"] for j in J), "near-miss runs", sum(j["near_miss_episodes"] > 0 for j in J),
          "near", sum(j["near_miss_episodes"] for j in J), "fc_land", sum(bool(j["fc_land"]) for j in J))
    for k in ("min_clearance_m", "coverage", "path_m", "trk_t_arrived", "trk_end_dist_m", "ekf_err_xy_max_m", "shield_clamp_ticks", "shield_map_ticks", "struct_rear_vetoes", "escapes", "sim_s"):
        x = [j[k] for j in J if j.get(k) is not None]
        print(f"   {k:22s} n {len(x)} min {min(x):.3f} med {st.median(x):.3f} p90 {q(x,.9):.3f} max {max(x):.3f}")
for k in ("min_clearance_m", "trk_t_arrived", "coverage", "path_m"):
    a = [v["w1map"].get(k) for v in P.values()]; b = [v["w2map"].get(k) for v in P.values()]
    z = [(x, y) for x, y in zip(a, b) if x is not None and y is not None]
    try: p = wilcoxon([x for x, _ in z], [y for _, y in z]).pvalue
    except Exception as e: p = float("nan")
    print(f"paired {k}: n {len(z)} w2>w1 in {sum(y > x for x, y in z)} p={p:.3g}")
for key, v in sorted(P.items()):
    a, b = v["w1map"], v["w2map"]
    if a["contact_episodes"] or b["contact_episodes"] or not a.get("trk_pass") or not b.get("trk_pass") or a["near_miss_episodes"] or b["near_miss_episodes"]:
        print("  ", key, *[(c, j.get("trk_final_state"), j.get("trk_pass"), j["contact_episodes"], j["near_miss_episodes"], j["min_clearance_m"]) for c, j in (("w1", a), ("w2", b))])
print("== e7 ==")
T = (30, 60, 120, 180, 230)
D = {"off": {}, "on": {}}
for d in sorted(R.glob("e7__*")):
    key, _, m = d.name[4:].rpartition("_")
    rows = [r for r in csv.DictReader(open(d / "run.csv")) if r.get("trk") == "ARRIVED"]
    j = json.load(open(d / "run.json"))
    f = lambda r, k: float(r[k]) if r[k] not in ("", "None") else 0.0
    t0 = f(rows[0], "t") + 5; rows = [r for r in rows if f(r, "t") >= t0]
    n0, e0 = f(rows[0], "north"), f(rows[0], "east")
    o = {"contact": j["contact_episodes"], "fixes": (j.get("landmark") or {}).get("fixes"), "rej": (j.get("landmark") or {}).get("rejected")}
    for T1 in T:
        r = min(rows, key=lambda r: abs(f(r, "t") - t0 - T1))
        w = [x for x in rows if f(x, "t") - t0 <= T1]
        o[T1] = dict(drift=math.hypot(f(r, "north") - n0, f(r, "east") - e0),
                     dmax=max(math.hypot(f(x, "north") - n0, f(x, "east") - e0) for x in w),
                     ekf=math.hypot(f(r, "ekf_north") - f(r, "north"), f(r, "ekf_east") - f(r, "east")),
                     cor=math.hypot(f(r, "ekf_north") + f(r, "lm_dn") - f(r, "north"), f(r, "ekf_east") + f(r, "lm_de") - f(r, "east")),
                     est=math.hypot(f(r, "est_n") - f(r, "north"), f(r, "est_e") - f(r, "east")) if r.get("est_n") not in ("", None) else float("nan"),
                     mkerr=abs(f(r, "mk_range") - f(r, "true_range")) if r.get("mk_range") not in ("", None) else float("nan"))
    D[m][key] = o
print({m: len(v) for m, v in D.items()}, "contacts", {m: sum(o["contact"] for o in v.values()) for m, v in D.items()},
      "fixes med", st.median(o["fixes"] for o in D["on"].values()), "rejected", sum(o["rej"] for o in D["on"].values()))
for k in ("drift", "dmax", "ekf", "cor", "est", "mkerr"):
    for T1 in T:
        a = [D["off"][x][T1][k] for x in D["off"]]; b = [D["on"][x][T1][k] for x in D["off"]]
        try: p = wilcoxon(a, b).pvalue
        except Exception: p = float("nan")
        print(f"{k:6s} t={T1:3d}s off med {st.median(a):.3f} p90 {q(a,.9):.3f} max {max(a):.3f} | on med {st.median(b):.3f} p90 {q(b,.9):.3f} max {max(b):.3f} | on better {sum(y < x for x, y in zip(a, b))}/{len(a)} p={p:.3g}")
r = next(csv.DictReader(open(R / "e7__cage10_k0_on/run.csv")))
print({k: r[k] for k in ("est_n", "est_e", "goal_n", "goal_e", "lm_dn", "lm_de", "mk_range", "true_range")})
