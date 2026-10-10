"""Run many gz_inst.sh jobs in parallel slots on the server (one slot = one instance index I).

python gz_batch.py --slots 12 --world ecps295_cage20.sdf --seeds 0-31 --arm v3_2=minifly/v3_2.yaml --arm es2=minifly/v3_2_es2.yaml
Jobs are interleaved seed by seed across arms; a job whose runs/<tag>/run.json exists is skipped (resumable).

Hardware-aligned setting (RL roadmap H0, S8 E1/E2 flags of gen_slewcheck.py):
python gz_batch.py --hw --world 'ecps295_lowbox_s{seed}.sdf' --seeds 0-29 --prefix h0 --arm v4=minifly/v4.yaml ...
  --hw = --tree ~/sim/hw/ecps295 --nav flow --extra-parm G_extnav_height + companion_fs --extra "--ext-height"
python gz_batch.py --hw --mode approach --seconds 20 --world 'ecps295_{wall}.sdf' ... (wall approaches, see --walls)
"""

import argparse
import itertools
import os
import subprocess
import threading
import time
from pathlib import Path

S = Path.home() / "sim"
MODE = "--mode {mode} --seconds {secs} --cruise 0.5 --max-forward 0.5 {fence}--decoder ecps --pilot ecps --retina ecps --out run"

ap = argparse.ArgumentParser()
ap.add_argument("--slots", type=int, default=12)
ap.add_argument("--slot-offset", type=int, default=0, help="first instance index; concurrent batches need disjoint ranges")
ap.add_argument("--world", default="ecps295_cage20.sdf", help="may contain {seed} (e.g. ecps295_lowbox_s{seed}.sdf)")
ap.add_argument("--walls", action="store_true", help="S8 E2: seed s -> world wall_<tape|plain|offset|offset_plain>, brain seed s %% 8")
ap.add_argument("--seeds", default="0-31")
ap.add_argument("--seconds", type=int, default=120)
ap.add_argument("--mode", default="patrol", choices=["patrol", "approach"])
ap.add_argument("--arm", action="append", required=True, help="name=variant.yaml[@rl.arms arm] (relative to the tree)")
ap.add_argument("--prefix", default="val")
ap.add_argument("--nav", default="gps")
ap.add_argument("--tree", default=str(S / "ecps295"), help="ecps295 tree (gz_inst.sh ECPS_E)")
ap.add_argument("--extra", default="", help="extra run_g4.py arguments")
ap.add_argument("--extra-parm", default="", help="comma-separated SITL parm files appended last (gz_inst.sh EXTRA_PARM)")
ap.add_argument("--hw", action="store_true", help="hardware-aligned: hw tree, flow nav, route B (companion height)")
a = ap.parse_args()
if a.hw:
    a.tree = str(S / "hw" / "ecps295")
    a.nav = "flow"
    v = Path(a.tree) / "sitl" / "variants"
    a.extra_parm = a.extra_parm or f"{v / 'G_extnav_height.parm'},{v / 'companion_fs.parm'}"
    a.extra = (a.extra + " --ext-height").strip()
TREE = Path(a.tree)
WALLS = ["wall_tape", "wall_plain", "wall_offset", "wall_offset_plain"]
lo, _, hi = a.seeds.partition("-")
arms = [x.split("=", 1) for x in a.arm]


def world_of(s: int) -> tuple[str, int]:
    if a.walls:
        return f"ecps295_{WALLS[s // 8]}.sdf", s % 8
    return a.world.format(seed=s), s


jobs = [(f"{a.prefix}_{n}_s{s}", s, TREE / cfg.split("@")[0], (cfg.split("@") + ["intact"])[1]) for s in range(int(lo), int(hi or lo) + 1) for n, cfg in arms]
jobs = [j for j in jobs if not (S / "runs" / j[0] / "run.json").exists()]
print(f"{len(jobs)} jobs, {a.slots} slots, tree {TREE}, nav {a.nav}, extra '{a.extra}'", flush=True)
lock = threading.Lock()
it = iter(jobs)
done = itertools.count(1)
t0 = time.time()
fence = "--fence-turn " if a.mode == "patrol" else ""
env0 = {**os.environ, "NAV": a.nav, "ECPS_E": str(TREE), "EXTRA_PARM": a.extra_parm}


def worker(slot: int) -> None:
    while True:
        with lock:
            job = next(it, None)
        if job is None:
            return
        tag, s, cfg, arm = job
        world, seed = world_of(s)
        args = MODE.format(mode=a.mode, secs=a.seconds, fence=fence) + f" --seed {seed} --config {cfg} {a.extra}".rstrip()
        subprocess.run(["bash", str(S / "gz_inst.sh"), str(a.slot_offset + slot), world, tag, args], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, env={**env0, "ECPS_ARM": arm})
        ok = (S / "runs" / tag / "run.json").exists()
        print(f"[{next(done)}/{len(jobs)} {time.time() - t0:5.0f}s] slot {slot:2d} {tag} {world} {'ok' if ok else 'FAILED'}", flush=True)


ts = [threading.Thread(target=worker, args=(i,)) for i in range(a.slots)]
for t in ts:
    t.start()
    time.sleep(3)  # stagger Gazebo start-ups
for t in ts:
    t.join()
print(f"all done in {time.time() - t0:.0f} s", flush=True)
