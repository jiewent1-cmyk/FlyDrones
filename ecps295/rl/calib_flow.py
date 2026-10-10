"""H1 calibration gate: does the fast twin with nav="flow" reproduce the Gazebo flow + route-B results on lowbox_s0..29?

Targets are the slew-fixed ROG matrix (gen_slewcheck.py phase B, fx_* runs, 30 paired worlds, brain seed k on lowbox_s<k>).
For every estimator setting (tau_terr x flow_rw) the twin flies the same 8 models on the same worlds; the gate passes when
the Spearman rank correlation across models is >= 0.6 for contact runs AND the twin ranks wcov2 and wcov4 worse than v3_2
(pre-registered 2026-10-07). Over-box time and the max position-estimate error are reported for the magnitudes.

python -m rl.calib_flow --procs 40 [--tau 0.5,1.5,4 --rw 0.01,0.03,0.06] [--nav truth]
"""

from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.stats import spearmanr

from rl.batch import run_jobs

E = Path(__file__).resolve().parent.parent
MODELS = {
    "v4": "minifly/v4.yaml",
    "v4_oc5": "rl/configs/v4_oc5.yaml",
    "v3_2": "minifly/v3_2.yaml",
    "es2": "rl/configs/v3_2_es2.yaml",
    "es3": "rl/configs/v3_2_es3.yaml",
    "wcov2": "rl/configs/b2_wcov2.yaml",
    "wcov4": "rl/configs/b2_wcov4.yaml",
    "wcov8": "rl/configs/b2_wcov8.yaml",
}
# Gazebo (ROG, flow + route B, lowbox_s0..29): contact runs /30, over-box s, EKF xy error max m, coverage
GAZEBO = {
    "v4": (4, 9.9, 0.44, 0.38),
    "v4_oc5": (5, 10.4, 0.32, 0.39),
    "v3_2": (4, 23.6, 1.17, 0.52),
    "es2": (6, 21.9, 0.94, 0.48),
    "es3": (6, 24.6, 0.82, 0.47),
    "wcov2": (7, 24.3, 1.03, 0.54),
    "wcov4": (10, 18.8, 1.12, 0.52),
    "wcov8": (4, 23.0, 0.83, 0.54),
}


# Amended gate (2026-10-08, post hoc, user-approved): split-half reliability of the 30-world Gazebo set is ~0 for contacts
# but 0.75-0.95 for over-box time, EKF error, coverage, min clearance, path and escapes. Gate v2: mean Spearman rho
# across models >= 0.6 over over-box time, max estimate error, coverage and min clearance.
GAZEBO_MINCLR = {"v4": 0.32, "v4_oc5": 0.34, "v3_2": 0.19, "es2": 0.23, "es3": 0.26, "wcov2": 0.16, "wcov4": 0.14, "wcov8": 0.28}


def gate_v2(res: list[dict]) -> dict:
    ok = [m for m in MODELS if any(r["model"] == m and "error" not in r for r in res)]
    mean = lambda m, k: float(np.mean([r[k] for r in res if r["model"] == m and "error" not in r]))  # noqa: E731
    rho = {
        "over_box": spearmanr([GAZEBO[m][1] for m in ok], [mean(m, "over_box_s") for m in ok]).statistic,
        "est_err": spearmanr([GAZEBO[m][2] for m in ok], [mean(m, "est_err_max_m") for m in ok]).statistic,
        "coverage": spearmanr([GAZEBO[m][3] for m in ok], [mean(m, "coverage") for m in ok]).statistic,
        "min_clearance": spearmanr([GAZEBO_MINCLR[m] for m in ok], [mean(m, "min_clearance_m") for m in ok]).statistic,
    }
    rho = {k: round(float(v), 3) for k, v in rho.items()}
    return {"rho": rho, "mean_rho": round(float(np.mean(list(rho.values()))), 3), "gate_v2": bool(np.mean(list(rho.values())) >= 0.6)}


def summarise(res: list[dict]) -> dict:
    out = {}
    for m in MODELS:
        L = [r for r in res if r["model"] == m and "error" not in r]
        if not L:
            out[m] = None
            continue
        out[m] = (
            sum(r["contact_episodes"] > 0 for r in L),
            round(float(np.mean([r["over_box_s"] for r in L])), 1),
            round(float(np.mean([r["est_err_max_m"] for r in L])), 2),
            round(float(np.mean([r["coverage"] for r in L])), 3),
        )
    ok = [m for m in MODELS if out[m]]
    rho = spearmanr([GAZEBO[m][0] for m in ok], [out[m][0] for m in ok]).statistic if len(ok) > 2 else float("nan")
    rho_ob = spearmanr([GAZEBO[m][1] for m in ok], [out[m][1] for m in ok]).statistic if len(ok) > 2 else float("nan")
    worse = all(out[w] and out["v3_2"] and out[w][0] > out["v3_2"][0] for w in ("wcov2", "wcov4"))
    return {"models": out, "rho_contact_runs": round(float(rho), 3), "rho_over_box": round(float(rho_ob), 3),
            "wcov2_wcov4_worse_than_v3_2": worse, "gate": bool(rho >= 0.6 and worse)}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--procs", type=int, default=10)
    ap.add_argument("--tau", default="0.5,1.5,4")
    ap.add_argument("--rw", default="0.01,0.03,0.06")
    ap.add_argument("--bias", default="0.94")
    ap.add_argument("--nav", default="flow")
    ap.add_argument("--dr", action="store_true", help="per-episode domain randomisation (rl.twin.flow_dr) instead of the grid")
    ap.add_argument("--alt", type=float, default=0.0, help="flow: cruise altitude (0 = TAKEOFF_ALT_M 0.6)")
    ap.add_argument("--alt-jitter", type=float, default=0.0)
    ap.add_argument("--worlds", default="0-29")
    ap.add_argument("--out", default=str(E / "rl" / "out" / "calib_flow.json"))
    a = ap.parse_args()
    lo, hi = map(int, a.worlds.split("-"))
    grid = [(None, None, None)] if a.nav == "truth" or a.dr else list(
        itertools.product(map(float, a.tau.split(",")), map(float, a.rw.split(",")), map(float, a.bias.split(","))))
    jobs = []
    for g, (tau, rw, bias) in enumerate(grid):
        dyn = {"nav": a.nav} if a.nav == "truth" else {"nav": "flow", "tau_terr": tau, "flow_rw": rw, "flow_bias": bias}
        if a.alt:
            dyn.update(hover_alt=a.alt, hover_jitter=a.alt_jitter)
        if a.dr:
            from rl.twin import flow_dr
        for m, v in MODELS.items():
            for k in range(lo, hi + 1):
                jobs.append({"tag": f"g{g}", "model": m, "variant": str(E / v), "world": str(E / "gazebo" / "worlds" / f"ecps295_lowbox_s{k}.layout.json"),
                             "seed": k, "seconds": 120.0, "dyn_over": flow_dr(k) if a.dr else dyn, "grid": [tau, rw, bias]})
    res = run_jobs(jobs, a.procs)
    report = []
    for g, setting in enumerate(grid):
        s = summarise([r for r in res if r["tag"] == f"g{g}"])
        s.update(gate_v2([r for r in res if r["tag"] == f"g{g}"]))
        s["setting"] = {"tau_terr": setting[0], "flow_rw": setting[1], "flow_bias": setting[2], "nav": a.nav, "alt": a.alt, "alt_jitter": a.alt_jitter}
        report.append(s)
        print(json.dumps(s["setting"]), "rho_cr", s["rho_contact_runs"], "rho_ob", s["rho_over_box"], "worse", s["wcov2_wcov4_worse_than_v3_2"], "GATE", s["gate"], "| v2", s["rho"], "mean", s["mean_rho"], "GATE_V2", s["gate_v2"])
        for m in MODELS:
            print(f"   {m:7s} twin {s['models'][m]}  gazebo {GAZEBO[m]}")
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    json.dump({"report": report, "episodes": [{k: r.get(k) for k in ("tag", "model", "seed", "contact_episodes", "over_box_s", "est_err_max_m", "coverage", "min_clearance_m", "path_m", "escapes", "error")} for r in res]},
              open(a.out, "w"))


if __name__ == "__main__":
    main()
