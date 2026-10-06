"""Run many gz_inst.sh jobs in parallel slots on the server (one slot = one instance index I).

python gz_batch.py --slots 12 --world ecps295_cage20.sdf --seeds 0-31 --arm v3_2=minifly/v3_2.yaml --arm es2=minifly/v3_2_es2.yaml
Jobs are interleaved seed by seed across arms; a job whose runs/<tag>/run.json exists is skipped (resumable).
"""

import argparse
import itertools
import os
import subprocess
import threading
import time
from pathlib import Path

S = Path.home() / "sim"
ARGS = "--mode patrol --seconds {secs} --cruise 0.5 --max-forward 0.5 --fence-turn --decoder ecps --pilot ecps --retina ecps --out run"

ap = argparse.ArgumentParser()
ap.add_argument("--slots", type=int, default=12)
ap.add_argument("--world", default="ecps295_cage20.sdf")
ap.add_argument("--seeds", default="0-31")
ap.add_argument("--seconds", type=int, default=120)
ap.add_argument("--arm", action="append", required=True, help="name=variant.yaml[@rl.arms arm] (relative to ~/sim/ecps295)")
ap.add_argument("--prefix", default="val")
ap.add_argument("--nav", default="gps")
a = ap.parse_args()
lo, _, hi = a.seeds.partition("-")
arms = [x.split("=", 1) for x in a.arm]
jobs = [(f"{a.prefix}_{n}_s{s}", s, S / "ecps295" / cfg.split("@")[0], (cfg.split("@") + ["intact"])[1]) for s in range(int(lo), int(hi or lo) + 1) for n, cfg in arms]
jobs = [j for j in jobs if not (S / "runs" / j[0] / "run.json").exists()]
print(f"{len(jobs)} jobs, {a.slots} slots", flush=True)
lock = threading.Lock()
it = iter(jobs)
done = itertools.count(1)
t0 = time.time()


def worker(slot: int) -> None:
    while True:
        with lock:
            job = next(it, None)
        if job is None:
            return
        tag, seed, cfg, arm = job
        args = ARGS.format(secs=a.seconds) + f" --seed {seed} --config {cfg}"
        subprocess.run(["bash", str(S / "gz_inst.sh"), str(slot), a.world, tag, args], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, env={**os.environ, "NAV": a.nav, "ECPS_ARM": arm})
        ok = (S / "runs" / tag / "run.json").exists()
        print(f"[{next(done)}/{len(jobs)} {time.time() - t0:5.0f}s] slot {slot:2d} {tag} {'ok' if ok else 'FAILED'}", flush=True)


ts = [threading.Thread(target=worker, args=(i,)) for i in range(a.slots)]
for t in ts:
    t.start()
    time.sleep(3)  # stagger Gazebo start-ups
for t in ts:
    t.join()
print(f"all done in {time.time() - t0:.0f} s", flush=True)
