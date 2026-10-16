"""N3 on the chosen hardware (2026-10-14): held-out twin test of CMA candidates against the s7_w2map reference.
Each candidate = a cmaes_run output dir (its space from config.json, z from best_val_z.npy or mean_z.npy), flown with the
same s7_w2map shield / ToF overrides as in training. Test episodes are disjoint from training (procedural worlds) and from
the CMA validation set:
  std  : rl.hw_arms set - lowbox_s0..49 x brain seeds {k, k+100, k+200}, cage20 seeds 0..31, empty 0..31
  new  : rl/s6_more.py set - lowbox_s0..49 x {k+300, k+400}, cage20 seeds 32..63
  stress (N4): lowbox_s0..49 (seed k) + cage20 0..31 under rl.twin OOD modes wind / tof / notex, plus drift2 (flow-EKF
         random walk x2, scale bias 0.94 -> 0.88); sets are named <mode>:lowbox / <mode>:cage20
python rl/n3m_eval.py --runs n3m_s1 n3m_s2 --procs 9 --out rl/out/n3m_eval.jsonl            (reference s7_w2map always included)
python rl/n3m_eval.py --report --out rl/out/n3m_eval.jsonl"""
import argparse
import json
from pathlib import Path

import numpy as np

from rl import hw_arms, params, shield_screen as ss
from rl.batch import run_jobs
from rl.episode import deep_merge

E = Path(__file__).resolve().parent.parent
REF = "s7_w2map"


def cand_overrides(run: str, which: str) -> dict:
    d = E / "rl" / "out" / run
    cfg = json.loads((d / "config.json").read_text())
    params.set_space(cfg["space"])
    z = np.load(d / ("best_val_z.npy" if which == "best" else "mean_z.npy"))
    return deep_merge(ss.CONDS[cfg["shield"]][2], params.full_overrides(z, cfg["arm"]))


FLOW = hw_arms.flow_dr  # --twin: v1 = rl.twin.flow_dr (the N3 round-2 twin); v2 / v2tf = rl.twin.flow_dr2


def jobs(tag: str, ov: dict, sets: list[str]) -> list[dict]:
    J = []
    base = {"tag": tag, "variant": ss.V4, "arm": "intact", "overrides": ov, "seconds": 120.0}
    if "std" in sets:
        for k in range(50):
            for b in (0, 100, 200):
                J.append({**base, "set": "lowbox", "key": f"{k}_{b}", "seed": k + b, "world": f"lowbox_s{k}", "dyn_over": FLOW(k + b)})
        for s in range(32):
            J.append({**base, "set": "cage20", "key": str(s), "seed": s, "world": "cage20", "dyn_over": FLOW(1000 + s)})
            J.append({**base, "set": "empty", "key": str(s), "seed": s, "seconds": 60.0, "world": f"empty:{s}", "yaw_deg": (s * 37) % 360,
                      "dyn_over": FLOW(2000 + s)})
    if "new" in sets:
        for k in range(50):
            for b in (300, 400):
                J.append({**base, "set": "lowbox", "key": f"{k}_{b}", "seed": k + b, "world": f"lowbox_s{k}", "dyn_over": FLOW(k + b)})
        for s in range(32, 64):
            J.append({**base, "set": "cage20", "key": str(s), "seed": s, "world": "cage20", "dyn_over": FLOW(1000 + s)})
    if "stress" in sets:
        for m in STRESS:
            for k in range(50):
                J.append({**base, "set": f"{m}:lowbox", "key": str(k), "seed": k, "world": f"lowbox_s{k}", "dyn_over": stress_dyn(m, k)})
            for s in range(32):
                J.append({**base, "set": f"{m}:cage20", "key": str(s), "seed": s, "world": "cage20", "dyn_over": stress_dyn(m, 1000 + s)})
    return J


STRESS = ("wind", "tof", "notex", "drift2")


def stress_dyn(mode: str, seed: int) -> dict:
    d = FLOW(seed)
    if mode == "drift2":  # map and shield live in EKF coordinates: twice the flow random walk, twice the scale bias
        return {**d, "flow_rw": 2.0 * d["flow_rw"], "flow_bias": 0.88}
    return {**d, "ood": mode}


def report(out: Path) -> None:
    from scipy.stats import binomtest, wilcoxon

    R = {}
    for r in map(json.loads, open(out)):
        if "error" not in r:
            R.setdefault(r["tag"], {})[(r["set"], r["key"])] = r
    ref = R.get(REF, {})
    sets = sorted({k[0] for t in R.values() for k in t}, key=lambda x: (":" in x, x))
    for st in sets:
        print(f"===== {st} (paired vs {REF})")
        for tag in [REF] + sorted(t for t in R if t != REF):
            keys = sorted(k for k in R[tag] if k[0] == st and k in ref)
            if not keys:
                continue
            a = np.array([ref[k]["contact_episodes"] > 0 for k in keys]); c = np.array([R[tag][k]["contact_episodes"] > 0 for k in keys])
            sv, nw = int((a & ~c).sum()), int((~a & c).sum()); p = binomtest(nw, sv + nw).pvalue if sv + nw else 1.0
            def d(f, fn=lambda r, f: r.get(f) or 0.0):
                x = np.array([fn(ref[k], f) for k in keys], float); y = np.array([fn(R[tag][k], f) for k in keys], float)
                pw = wilcoxon(y - x).pvalue if np.any(y != x) else 1.0
                return y.mean(), (y - x).mean(), pw
            cov, dcov, pcov = d("coverage")
            sh, dsh, _ = d("sh", lambda r, f: r.get("shield_clamp_ticks", 0) / max(1, r.get("ticks", 1)))
            nm = np.mean([R[tag][k]["near_miss_episodes"] > 0 for k in keys])
            mc = np.percentile([R[tag][k]["min_clearance_m"] for k in keys], 5)
            est = np.mean([R[tag][k].get("est_err_max_m") or 0 for k in keys])
            sp = np.mean([R[tag][k].get("mean_speed") or 0 for k in keys])
            print(f"{tag:12s} n={len(keys):3d} contact {int(c.sum()):3d} (ref {int(a.sum())}, McNemar p={p:.3f}) | cov {cov:.3f} ({dcov:+.3f}, p={pcov:.3f})"
                  f" | near-miss runs {nm:.2f} | minclr p05 {mc:+.3f} | shield share {sh:.3f} ({dsh:+.3f}) | est {est:.2f} | speed {sp:.2f}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", nargs="*", default=[])
    ap.add_argument("--which", default="best", choices=["best", "mean"])
    ap.add_argument("--sets", nargs="+", default=["std", "new"])
    ap.add_argument("--procs", type=int, default=9)
    ap.add_argument("--out", default=str(E / "rl" / "out" / "n3m_eval.jsonl"))
    ap.add_argument("--report", action="store_true")
    ap.add_argument("--twin", default="v1", choices=["v1", "v2", "v2tf", "v3", "v3tf"],
                    help="v2 / v2tf: rl.twin.flow_dr2 (default / fast EKF terrain); v3 / v3tf: rl.twin.flow_dr3 (+ Gazebo-fitted motion)")
    a = ap.parse_args()
    global FLOW
    if a.twin != "v1":
        from rl.twin import flow_dr2, flow_dr3

        fd = flow_dr3 if a.twin.startswith("v3") else flow_dr2
        FLOW = lambda s: fd(s, "tf10" if a.twin.endswith("tf") else "default")  # noqa: E731
    out = Path(a.out)
    if a.report:
        report(out)
        return
    J = jobs(REF, ss.CONDS[REF][2], a.sets)
    for run in a.runs:
        J += jobs(run if a.which == "best" else f"{run}_mean", cand_overrides(run, a.which), a.sets)
    if out.exists():
        done = {(r["tag"], r["set"], r["key"]) for r in map(json.loads, open(out)) if "error" not in r}
        J = [j for j in J if (j["tag"], j["set"], j["key"]) not in done]
    print(len(J), "jobs to run", flush=True)
    run_jobs(J, a.procs, str(out))
    report(out)


if __name__ == "__main__":
    main()
