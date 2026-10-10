"""Pilot experiment (RL plan P1, decoder group A): CMA-ES over 17 decoder parameters, started at MiniFly v3_2.

Each generation draws K procedural cages (rl.worlds; never the G4 worlds) + start yaw + brain seed, and every
candidate - and the v3_2 reference - flies the same K episodes (common random numbers). Fitness (maximised):

  J_ep = coverage + 0.01 path_m - 0.15 near_misses - 0.5 fence_turn_s / T - 0.0025 sum|dcmd|^2
  F    = 0.5 mean(J) + 0.5 CVaR_25%(J) - 1.0 P(contact) - 0.3 mean(contact episodes)

Checkpoints every generation to rl/out/<run>/ (es.pkl, gens.jsonl, episodes.jsonl), so it can be resumed.

nohup nice -n 19 python -m rl.cmaes_run --run es1 --gens 25 --k 6 --seconds 90 --procs 10 &
"""

from __future__ import annotations

import argparse
import json
import pickle
import time
from pathlib import Path

import cma
import numpy as np

from rl import params
from rl.batch import run_jobs

E = Path(__file__).resolve().parent.parent
V32 = str(E / "minifly" / "v3_2.yaml")


W_COV = 1.0  # coverage weight in J (B2 sweeps it; 1.0 = es1-es3 / A1 fitness)
FITNESS = "v1"  # v2 (B, fixed 2026-10-07): + 0.5 mean proximity cost, - 3 x shortfall below coverage 0.40
COV_MIN, W_PROX, W_SHORT = 0.40, 0.5, 3.0
W_BACK, W_EST, W_FA = 0.3, 0.5, 0.1  # v3 (H2, fixed 2026-10-07): per m of commanded backing, per m of max position-estimate error
BASE = V32  # reference / base variant (H2: minifly/v4.yaml); the reference arm keeps the tag "v3_2" in the logs
DYN: dict | None = None  # TwinDrone overrides (H2: nav=flow + calibrated estimator)
LOW_FRAC = 0.0  # share of training / validation episodes in lowbox-style worlds
W_INT = 0.0  # B3: cost per unit of intervention_frac (share of ticks where governor / fence turn overrode the decoder)


def ep_score(r: dict, seconds: float) -> float:
    s = ep_score_v1(r, seconds) - W_INT * r.get("intervention_frac", 0.0)
    if FITNESS in ("v3", "v4"):
        s -= W_BACK * r.get("back_m", 0.0) + W_EST * r.get("est_err_max_m", 0.0)
    if FITNESS == "v4":  # C1 (2026-10-08): + 0.1 per false alarm a minute (rl.trigger_metrics, pre-registered definition)
        s -= W_FA * (r.get("false_alarms_per_min") or 0.0)
    if FITNESS in ("v2", "v3", "v4"):
        cov_min = 0.35 if FITNESS in ("v3", "v4") else COV_MIN  # v4 covers ~0.38 on lowbox (revision doc: floor 0.35)
        return s - W_PROX * r.get("proximity", 0.0) - W_SHORT * max(0.0, cov_min - r["coverage"])
    return s


def ep_score_v1(r: dict, seconds: float) -> float:
    return (
        W_COV * r["coverage"]
        + 0.01 * r["path_m"]
        - 0.15 * r["near_miss_episodes"]
        - 0.5 * r["fence_turn_s"] / seconds
        - 0.0025 * r["dcmd2"]
    )


def fitness(eps: list[dict], seconds: float) -> dict:
    ok = [r for r in eps if "error" not in r]
    if len(ok) < len(eps):
        return {"F": -10.0, "errors": len(eps) - len(ok)}
    j = np.array([ep_score(r, seconds) for r in ok])
    q = max(1, int(round(0.25 * len(j))))
    cvar = float(np.sort(j)[:q].mean())
    contact = np.array([r["contact_episodes"] for r in ok])
    F = 0.5 * j.mean() + 0.5 * cvar - 1.0 * float((contact > 0).mean()) - 0.3 * float(contact.mean())
    return {
        "F": round(float(F), 4),
        "J_mean": round(float(j.mean()), 4),
        "J_cvar": round(cvar, 4),
        "p_contact": round(float((contact > 0).mean()), 3),
        "contacts": int(contact.sum()),
        "near": round(float(np.mean([r["near_miss_episodes"] for r in ok])), 2),
        "coverage": round(float(np.mean([r["coverage"] for r in ok])), 3),
        "path": round(float(np.mean([r["path_m"] for r in ok])), 1),
        "minclr_p05": round(float(np.percentile([r["min_clearance_m"] for r in ok], 5)), 3),
        "interv": round(float(np.mean([r.get("intervention_frac", 0.0) for r in ok])), 4),
    }


def _world(e: dict) -> str:
    """Training / validation world: lowbox-style for a LOW_FRAC share of the episodes (decided from the world seed)."""
    low = (e["world_seed"] * 2654435761 % 2**32) / 2**32 < LOW_FRAC
    return f"lowproc:{e['world_seed']}" if low else f"proc:{e['world_seed']}"


def _dyn(e: dict) -> dict | None:
    if DYN == "dr":  # H2: flow-EKF domain randomisation, drawn per episode from its world seed (common to all candidates)
        from rl.twin import flow_dr

        return flow_dr(e["world_seed"])
    return DYN


def jobs_for(tag: str, overrides: dict | None, eps: list[dict], seconds: float, arm: str = "intact") -> list[dict]:
    return [
        {
            "tag": tag,
            "variant": BASE,
            "world": _world(e),
            "dyn_over": _dyn(e),
            "seed": e["seed"],
            "yaw_deg": e["yaw_deg"],
            "seconds": seconds,
            "overrides": overrides,
            "arm": arm,
        }
        for e in eps
    ]


def main() -> None:
    from rl.worlds import episode_set, validation_set

    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="es1")
    ap.add_argument("--gens", type=int, default=25)
    ap.add_argument("--k", type=int, default=6, help="episodes per candidate per generation")
    ap.add_argument("--popsize", type=int, default=16)
    ap.add_argument("--sigma0", type=float, default=0.2)
    ap.add_argument("--seconds", type=float, default=90)
    ap.add_argument("--procs", type=int, default=10)
    ap.add_argument("--val-every", type=int, default=5, help="evaluate the distribution mean on the held-out set")
    ap.add_argument("--val-k", type=int, default=24)
    ap.add_argument("--space", default="default", choices=["default", "wide", "turncap", "v4hw", "v4hw_eff"])
    ap.add_argument("--arm", default="intact", help="rl.arms control arm optimised with the same budget")
    ap.add_argument("--w-cov", type=float, default=1.0, help="coverage weight in the episode score (B2 Pareto sweep)")
    ap.add_argument("--fitness", default="v1", choices=["v1", "v2", "v3", "v4"])
    ap.add_argument("--base", default="", help="base / reference variant YAML (default minifly/v3_2.yaml; H2: minifly/v4.yaml)")
    ap.add_argument("--nav", default="truth", choices=["truth", "flow"], help="TwinDrone navigation (H1 flow estimator)")
    ap.add_argument("--flow", default="", help='estimator settings for --nav flow: JSON, e.g. {"tau_terr":1.5,"flow_rw":0.03}, or "dr" (rl.twin.flow_dr per episode)')
    ap.add_argument("--lowbox-frac", type=float, default=0.0, help="share of episodes in lowbox-style worlds (H1)")
    ap.add_argument("--w-int", type=float, default=0.0, help="B3: cost of governor / fence-turn intervention (share of ticks)")
    ap.add_argument("--cma-seed", type=int, default=1, help="CMA-ES seed; also selects the training-world stream (seed-1)")
    ap.add_argument("--x0-yaml", default="", help="warm start from the decoder values in this variant YAML (rl.validate export)")
    a = ap.parse_args()
    params.set_space(a.space)
    global W_COV, FITNESS, W_INT, BASE, DYN, LOW_FRAC
    W_COV, FITNESS, W_INT = a.w_cov, a.fitness, a.w_int
    BASE = str(E / a.base) if a.base else V32
    if a.nav == "flow":
        DYN = "dr" if a.flow == "dr" else {"nav": "flow", **(json.loads(a.flow) if a.flow else {})}
    else:
        DYN = None
    LOW_FRAC = a.lowbox_frac
    x0 = params.Z0
    if a.x0_yaml:

        from rl.episode import build_cfg

        cfg = build_cfg(a.x0_yaml, 0)
        d, e = cfg["decoder"], cfg["decoder"]["ecps"]
        vals = {
            "yaw_gain": d["axes"]["yaw"]["gain"], "yaw_dnp03_w": d["axes"]["yaw"]["terms"]["DNp03_L"], "smoothing": d["smoothing"],
            "brake_dnp03": d["brake_terms"]["DNp03_L"], "brake_dnp01": d["brake_terms"]["DNp01_L"], "brake_decay": e["brake_decay"],
            "escape_thr": d["escape"]["threshold_hz"], "sacc_thr": e["saccade"]["threshold_hz"], "sacc_dur": e["saccade"]["duration_s"],
            "sacc_backoff": e["saccade"]["backoff"], "sacc_backoff_s": e["saccade"]["backoff_s"], "sacc_on_brake": e["saccade"]["on_brake"],
            "sacc_refr": e["saccade"]["refractory_s"], "sacc_keep_dir": e["saccade"]["keep_direction_s"], "caut_hold": e["caution"]["hold_s"],
            "caut_ramp": e["caution"]["ramp_s"], "caut_on_brake": e["caution"]["on_brake"],
        }
        for n, path, v0, *_ in params.SPEC:  # extra (v4hw / eff) parameters: read by config path, else the spec default
            if n not in vals:
                node = cfg
                try:
                    for k in path.split("."):
                        node = node[k]
                except (KeyError, TypeError):
                    node = v0
                vals[n] = node
        x0 = params.to_z(np.array([vals[n] for n in params.NAMES]))
        print("warm start", params.describe(x0), flush=True)
    x0 = params.x0_for(a.arm, x0)
    out = E / "rl" / "out" / a.run
    out.mkdir(parents=True, exist_ok=True)
    ck = out / "es.pkl"
    if ck.exists():
        es, gen0 = pickle.load(open(ck, "rb"))
        print(f"resumed at generation {gen0}", flush=True)
    else:
        es = cma.CMAEvolutionStrategy(
            x0.tolist(), a.sigma0, {"bounds": [0.0, 1.0], "popsize": a.popsize, "seed": a.cma_seed, "verbose": -9}
        )
        gen0 = 0
        (out / "config.json").write_text(json.dumps({**vars(a), "names": params.NAMES, "z0": x0.tolist(), "spec": params.SPEC}, indent=1))
    for gen in range(gen0, a.gens):
        t0 = time.time()
        eps = episode_set(gen, a.k, stream=a.cma_seed - 1)
        Z = es.ask()
        jobs = jobs_for("v3_2", None, eps, a.seconds)
        for i, z in enumerate(Z):
            jobs += jobs_for(f"c{i}", params.full_overrides(np.array(z), a.arm), eps, a.seconds, a.arm)
        res = run_jobs(jobs, a.procs)
        with open(out / "episodes.jsonl", "a") as f:
            for r in res:
                f.write(json.dumps({"gen": gen, **{k: v for k, v in r.items() if k != "rows"}}) + "\n")
        by = {}
        for r in res:
            by.setdefault(r["tag"], []).append(r)
        fits = [fitness(by[f"c{i}"], a.seconds) for i in range(len(Z))]
        es.tell(Z, [-f["F"] for f in fits])
        base = fitness(by["v3_2"], a.seconds)
        best = int(np.argmax([f["F"] for f in fits]))
        rec = {
            "gen": gen,
            "wall_s": round(time.time() - t0),
            "v3_2": base,
            "best": fits[best],
            "best_params": params.describe_full(np.array(Z[best]), a.arm),
            "mean_params": params.describe_full(es.mean, a.arm),
            "sigma": round(float(es.sigma), 4),
            "F_median": round(float(np.median([f["F"] for f in fits])), 4),
        }
        with open(out / "gens.jsonl", "a") as f:
            f.write(json.dumps(rec) + "\n")
        pickle.dump((es, gen + 1), open(ck, "wb"))
        if a.val_every and ((gen + 1) % a.val_every == 0 or gen + 1 == a.gens):
            vset = validation_set(a.val_k)
            vjobs = jobs_for("mean", params.full_overrides(es.mean, a.arm), vset, a.seconds, a.arm)
            vbase = out / "val_v3_2.json"
            if not vbase.exists():
                vjobs += jobs_for("v3_2", None, vset, a.seconds)
            vres = run_jobs(vjobs, a.procs)
            vb = {}
            for r in vres:
                vb.setdefault(r["tag"], []).append(r)
            if "v3_2" in vb:
                vbase.write_text(json.dumps(fitness(vb["v3_2"], a.seconds)))
            vm = fitness(vb["mean"], a.seconds)
            vrec = {"gen": gen, "mean": vm, "v3_2": json.loads(vbase.read_text()), "z": [float(x) for x in es.mean]}
            with open(out / "val.jsonl", "a") as f:
                f.write(json.dumps(vrec) + "\n")
            best_path = out / "best_val.json"
            if not best_path.exists() or vm["F"] > json.loads(best_path.read_text())["mean"]["F"]:
                best_path.write_text(json.dumps(vrec))
                np.save(out / "best_val_z.npy", es.mean)
            print(f"  VAL gen {gen}: mean F={vm['F']:+.3f} pC={vm['p_contact']:.2f} near={vm['near']:.1f} cov={vm['coverage']:.2f}"
                  f" | v3_2 F={vrec['v3_2']['F']:+.3f} pC={vrec['v3_2']['p_contact']:.2f} near={vrec['v3_2']['near']:.1f} cov={vrec['v3_2']['coverage']:.2f}", flush=True)
        print(
            f"gen {gen:2d} {rec['wall_s']:4d}s  v3_2 F={base['F']:+.3f} pC={base['p_contact']:.2f} near={base['near']:.1f} cov={base['coverage']:.2f}"
            f" | best F={fits[best]['F']:+.3f} pC={fits[best]['p_contact']:.2f} near={fits[best]['near']:.1f} cov={fits[best]['coverage']:.2f}"
            f" | median F={rec['F_median']:+.3f} sigma={rec['sigma']:.3f}",
            flush=True,
        )
    np.save(out / "mean_z.npy", es.mean)


if __name__ == "__main__":
    main()
