#!/usr/bin/env python3
"""Brain-step micro-benchmark on one core: MiniFly v4 (s7_w2map) ticked at 50 ms with fixed input rates.
    taskset -c 6 python brain_bench.py [--ticks 200] [--profile]"""
import argparse, cProfile, pstats, sys, time
sys.path[:0] = ["/opt/ecps295/ecps295", "/opt/ecps295/src"]
import numpy as np
from envs import ECPS295_SAFETY_20FT
from flydrones.brain import Brain, load_connectome
from flydrones.config import load_config
ap = argparse.ArgumentParser(); ap.add_argument("--ticks", type=int, default=200); ap.add_argument("--profile", action="store_true")
a = ap.parse_args()
m = "/opt/ecps295/ecps295/minifly/s7_w2map.yaml"
cfg = load_config(m, {"safety": dict(ECPS295_SAFETY_20FT), "brain": {"seed": 0}})
cfg["brain"]["source"] = "/opt/ecps295/ecps295/minifly/minifly_v4.npz"
b = Brain(load_connectome(cfg["brain"]["source"]), cfg)
rng = np.random.default_rng(0)
inp = {k: rng.uniform(5, 60, size=b.connectome.group(k).size) for k in b.input_specs if b.connectome.group(k).size}
print("neurons", b.net.n, "synapses", b.net.W.nnz, "dt_ms", b.net.p.dt)
for _ in range(20): b.tick(inp, ms=50.0)
pr = cProfile.Profile() if a.profile else None
t = []
if pr: pr.enable()
for _ in range(a.ticks):
    c = time.perf_counter(); b.tick(inp, ms=50.0); t.append((time.perf_counter() - c) * 1000)
if pr:
    pr.disable(); pstats.Stats(pr).sort_stats("tottime").print_stats(8)
print(f"brain tick p50 {np.percentile(t,50):.2f} ms p95 {np.percentile(t,95):.2f} ms")
