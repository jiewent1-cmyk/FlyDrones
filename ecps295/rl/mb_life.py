"""Stage E: mushroom-body lifetimes in the twin, then a frozen-g test (mb.py).

train  one lifetime = n_ep episodes in a row (90 s each, flow nav + DR) on procedural worlds that are never tested
       (lowproc / proc, seeds from 5000), g carried from episode to episode, learning on. Lifetimes run in parallel.
test   frozen g on the rl.hw_arms k_0 set (lowbox_s0-49, cage20 x 32, empty x 32), learning off, two arms per lifetime:
         learned   the lifetime's g
         shuffled  the same g values permuted over the KCs (same amount of slowing, no scene specificity)
       the plain variant (g = 1, bit-identical to no MB) is the in-distribution base from rl.hw_arms (--base)
python -m rl.mb_life --tag e1 --lives 0-7 --procs 8 [--mb '{"k_sac": 30}']           train
python -m rl.mb_life --tag e1 --lives 0-7 --test --procs 10                          test
python -m rl.mb_life --tag e1 --report --base rl/out/seeds_a.jsonl
"""
from __future__ import annotations

import argparse
import json
from multiprocessing import get_context
from pathlib import Path

import numpy as np
from scipy.stats import wilcoxon

from rl.batch import run_jobs, seeds
from rl.hw_arms import V4
from rl.twin import flow_dr

E = Path(__file__).resolve().parent.parent
OUT = E / "rl" / "out" / "mb"


def _life(args) -> dict:
    from rl.batch import _job

    tag, L, n_ep, cfg, variant = args
    cfg = {**cfg, "seed": L}
    g, log = None, []
    for i in range(n_ep):
        s = 5000 + 100 * L + i
        w = f"lowproc:{s}" if i % 2 == 0 else f"proc:{s}"
        r = _job({"variant": variant, "arm": "intact", "seed": s, "seconds": 90.0, "world": w, "dyn_over": flow_dr(s),
                  "mb": {"cfg": cfg, "g": g, "learn": True}})
        if "error" in r:
            raise RuntimeError(r["error"])
        g = r.pop("mb_g")
        log.append({k: r[k] for k in ("world", "contact_episodes", "coverage", "mb_m_mean", "mb_g_mean", "mb_g_low", "mb_ppl1_s", "mb_active_frac")})
    res = {"tag": tag, "life": L, "cfg": cfg, "variant": variant, "g": g, "log": log}
    (OUT / f"{tag}_L{L}.json").write_text(json.dumps(res))
    return {"life": L, "g_mean": float(np.mean(g)), "g_low": int(np.sum(np.array(g) < 0.5)),
            "contacts_first_half": sum(x["contact_episodes"] > 0 for x in log[: n_ep // 2]),
            "contacts_second_half": sum(x["contact_episodes"] > 0 for x in log[n_ep // 2 :])}


def test_jobs(tag: str, lives: list[int]) -> list[dict]:
    J = []
    for L in lives:
        life = json.loads((OUT / f"{tag}_L{L}.json").read_text())
        g = np.array(life["g"])
        arms_ = {"learned": g, "shuffled": np.random.default_rng(7000 + L).permutation(g)}
        for name, gg in arms_.items():
            base = {"tag": f"{tag}_L{L}_{name}", "life": L, "mbarm": name, "variant": life["variant"], "arm": "intact",
                    "mb": {"cfg": life["cfg"], "g": gg.tolist(), "learn": False}}
            for k in range(50):
                J.append({**base, "set": "lowbox", "key": f"{k}_0", "seed": k, "seconds": 120.0, "world": f"lowbox_s{k}", "dyn_over": flow_dr(k)})
            for s in range(32):
                J.append({**base, "set": "cage20", "key": str(s), "seed": s, "seconds": 120.0, "world": "cage20", "dyn_over": flow_dr(1000 + s)})
                J.append({**base, "set": "empty", "key": str(s), "seed": s, "seconds": 60.0, "world": f"empty:{s}",
                          "yaw_deg": (s * 37) % 360, "dyn_over": flow_dr(2000 + s)})
    return J


def report(tag: str, base: list[str]) -> None:
    out = OUT / f"{tag}_test.jsonl"
    R = [json.loads(l) for l in open(out)]
    B = {}
    for p in base:
        for r in map(json.loads, open(p)):
            if r.get("tag") == "v4" and "error" not in r and (r["set"] != "lowbox" or r["key"].endswith("_0")):
                B[(r["set"], r["key"])] = r
    lives = sorted({r["life"] for r in R})
    print(f"===== {tag}: frozen g, {len(lives)} lifetimes; plain v4 (g = 1) from --base")
    for st in ("lowbox", "cage20", "empty"):
        f = (lambda r: r["false_alarms_per_min"]) if st == "empty" else (lambda r: float(r["contact_episodes"] > 0))
        v4 = np.mean([f(r) for (s, _), r in B.items() if s == st]) * (1 if st == "empty" else len([1 for (s, _) in B if s == st]))
        per = {a: [] for a in ("learned", "shuffled")}
        cov = {a: [] for a in ("learned", "shuffled")}
        for L in lives:
            for a in per:
                X = [r for r in R if r["life"] == L and r["mbarm"] == a and r["set"] == st and "error" not in r]
                per[a].append(np.mean([f(r) for r in X]) * (1 if st == "empty" else len(X)))
                cov[a].append(np.mean([r["coverage"] for r in X]))
        what = "false alarms / min" if st == "empty" else "runs with contact"
        d = np.array(per["learned"]) - np.array(per["shuffled"])
        p = wilcoxon(d).pvalue if np.any(d != 0) else 1.0
        print(f"  {st:7s} {what}: v4 {v4:.2f} | learned {np.round(per['learned'], 2).tolist()} median {np.median(per['learned']):.2f}"
              f" | shuffled median {np.median(per['shuffled']):.2f} | learned - shuffled Wilcoxon p={p:.3f}")
        print(f"          coverage: learned {np.median(cov['learned']):.3f} shuffled {np.median(cov['shuffled']):.3f}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--lives", default="0-7")
    ap.add_argument("--n-ep", type=int, default=30)
    ap.add_argument("--mb", default="{}", help="JSON overrides of mb.DEFAULT")
    ap.add_argument("--variant", default=V4)
    ap.add_argument("--procs", type=int, default=8)
    ap.add_argument("--test", action="store_true")
    ap.add_argument("--report", action="store_true")
    ap.add_argument("--base", nargs="+", default=[])
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    lives = seeds(a.lives)
    if a.report:
        report(a.tag, a.base); return
    if a.test:
        J = test_jobs(a.tag, lives)
        out = OUT / f"{a.tag}_test.jsonl"
        if out.exists():
            done = {(r["tag"], r["set"], r["key"]) for r in map(json.loads, open(out)) if "error" not in r}
            J = [j for j in J if (j["tag"], j["set"], j["key"]) not in done]
        print(len(J), "test jobs", flush=True)
        run_jobs(J, a.procs, str(out))
        return
    cfg = json.loads(a.mb)
    with get_context("fork").Pool(min(a.procs, len(lives))) as pool:
        for r in pool.imap_unordered(_life, [(a.tag, L, a.n_ep, cfg, a.variant) for L in lives]):
            print(json.dumps(r), flush=True)


if __name__ == "__main__":
    main()
