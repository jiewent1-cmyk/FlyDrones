"""D stage: out-of-distribution twin test of the exported C4 seeds (bypass, c4_s*) against the H2 seeds (intact, h2_s*).
FLYNN-style prediction: if the connectome helps, the intact arm degrades less than the bypass arm away from the
training distribution. In-distribution = the hardware-aligned setting (flow nav + per-episode DR) of rl.hw_arms.

  modes rl.twin.OOD_MODES (lowlight, notex, halfcam, wind, tof, ghost)
  per mode: lowbox_s0..49 (brain seed k, 120 s)   cage20 x 32 (120 s)   empty field x 32 (60 s; not for ghost)
  same worlds / seeds / flow DR as the k_0 subset of rl.hw_arms, so its --out files are the paired in-distribution base
python -m rl.ood --conds c4_s1 h2_s1 ... --out rl/out/ood_a.jsonl [--modes ...]
python -m rl.ood --report --out rl/out/ood_a.jsonl rl/out/ood_b.jsonl --base rl/out/seeds_a.jsonl rl/out/seeds_b.jsonl
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.stats import mannwhitneyu

from rl.batch import run_jobs
from rl.hw_arms import V4, exported
from rl.twin import OOD_MODES, flow_dr

E = Path(__file__).resolve().parent.parent


def jobs(conds: dict, modes: list[str]) -> list[dict]:
    J = []
    for m in modes:
        for c, (v, arm) in conds.items():
            base = {"tag": c, "ood": m, "variant": v, "arm": arm}
            for k in range(50):
                J.append({**base, "set": "lowbox", "key": f"{k}_0", "seed": k, "seconds": 120.0, "world": f"lowbox_s{k}",
                          "dyn_over": {**flow_dr(k), "ood": m}})
            for s in range(32):
                J.append({**base, "set": "cage20", "key": str(s), "seed": s, "seconds": 120.0, "world": "cage20",
                          "dyn_over": {**flow_dr(1000 + s), "ood": m}})
                if m != "ghost":
                    J.append({**base, "set": "empty", "key": str(s), "seed": s, "seconds": 60.0, "world": f"empty:{s}",
                              "yaw_deg": (s * 37) % 360, "dyn_over": {**flow_dr(2000 + s), "ood": m}})
    return J


def _load(paths: list[str], ood: bool) -> dict:
    """{mode: {set: {tag: {key: record}}}}; base records (rl.hw_arms) go under mode 'in'."""
    D = {}
    for p in paths:
        for r in map(json.loads, open(p)):
            if "error" in r or (not ood and r["set"] == "lowbox" and not r["key"].endswith("_0")):
                continue
            D.setdefault(r.get("ood", "in") if ood else "in", {}).setdefault(r["set"], {}).setdefault(r["tag"], {})[r["key"]] = r
    return D


def _metric(R: dict, st: str) -> float:
    """lowbox / cage20: runs with contact; empty: false alarms per minute (mean)."""
    if not R:
        return float("nan")
    if st == "empty":
        return float(np.mean([r["false_alarms_per_min"] for r in R.values()]))
    return float(sum(r["contact_episodes"] > 0 for r in R.values()))


def report(out: list[str], base: list[str]) -> None:
    D = {**_load(base, False), **_load(out, True)}
    modes = ["in"] + [m for m in OOD_MODES if m in D]
    tags = sorted({t for m in D.values() for s in m.values() for t in s}, key=lambda t: (t != "v4", t[:2], t))
    for st, what in (("lowbox", "runs with contact / 50"), ("cage20", "runs with contact / 32"), ("empty", "false alarms / min")):
        print(f"===== {st}: {what}")
        print("  " + "".join(f"{m:>9s}" for m in ["cond"] + modes))
        for t in tags:
            print("  " + f"{t:>9s}" + "".join(f"{_metric(D.get(m, {}).get(st, {}).get(t, {}), st):9.2f}" for m in modes))
        print("  seed level, bypass (c4_*) vs intact (h2_*), exact Mann-Whitney:")
        for m in modes:
            v = {g: [_metric(D.get(m, {}).get(st, {}).get(t, {}), st) for t in tags if t.startswith(g)] for g in ("c4", "h2")}
            d = {g: [_metric(D.get(m, {}).get(st, {}).get(t, {}), st) - _metric(D["in"].get(st, {}).get(t, {}), st)
                     for t in tags if t.startswith(g)] for g in ("c4", "h2")}
            if not all(map(len, v.values())) or any(np.isnan(x) for g in v.values() for x in g):
                continue
            p = mannwhitneyu(v["c4"], v["h2"], method="exact").pvalue
            line = f"    {m:9s} median bypass {np.median(v['c4']):6.2f} intact {np.median(v['h2']):6.2f} p={p:.3f}"
            if m != "in":
                pd = mannwhitneyu(d["c4"], d["h2"], method="exact").pvalue
                line += f" | change vs in-dist: bypass {np.median(d['c4']):+6.2f} intact {np.median(d['h2']):+6.2f} p={pd:.3f}"
            print(line)
    print("errors:", sum("error" in json.loads(l) for p in out for l in open(p)))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--procs", type=int, default=10)
    ap.add_argument("--out", nargs="+", default=[str(E / "rl" / "out" / "ood.jsonl")])
    ap.add_argument("--conds", nargs="+", default=[], help="exported configs (minifly/<name>.yaml); v4 is added")
    ap.add_argument("--modes", nargs="+", default=list(OOD_MODES), choices=list(OOD_MODES))
    ap.add_argument("--report", action="store_true", help="only summarise existing --out files")
    ap.add_argument("--base", nargs="+", default=[], help="rl.hw_arms --out files with the in-distribution runs")
    a = ap.parse_args()
    if a.report:
        report(a.out, a.base); return
    conds = {"v4": (V4, "intact"), **{c: exported(c) for c in a.conds if c != "v4"}}
    done = set()
    if Path(a.out[0]).exists():
        done = {(r["ood"], r["tag"], r["set"], r["key"]) for r in map(json.loads, open(a.out[0])) if "error" not in r}
    J = [j for j in jobs(conds, a.modes) if (j["ood"], j["tag"], j["set"], j["key"]) not in done]
    print(len(J), "jobs to run", flush=True)
    run_jobs(J, a.procs, a.out[0])


if __name__ == "__main__":
    main()
