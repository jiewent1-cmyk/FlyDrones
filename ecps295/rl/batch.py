"""Run many twin episodes in parallel (one process per core, numpy single-threaded), append results as JSON lines.

python -m rl.batch --variants v3 v3_1 v3_2 --worlds cage20 --seeds 0-19 --out rl/out/consistency.jsonl
"""

from __future__ import annotations

import argparse
import json
import os
import time
from multiprocessing import get_context
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

E = Path(__file__).resolve().parent.parent  # ecps295/
WORLDS = E / "gazebo" / "worlds"


def variant_path(v: str) -> str:
    return v if v.endswith(".yaml") else str(E / "minifly" / f"{v}.yaml")


def world_path(w: str) -> str:
    return w if w.endswith(".json") else str(WORLDS / f"ecps295_{w}.layout.json")


def _job(job: dict) -> dict:
    from rl.episode import run_episode
    from rl.twin import World

    t0 = time.perf_counter()
    try:
        if job["world"].startswith("empty:"):
            from rl.worlds import empty_world

            world = empty_world(int(job["world"][6:]))
        elif job["world"].startswith("lowproc:"):
            from rl.worlds import lowbox_world

            world = lowbox_world(int(job["world"][8:]))
        elif job["world"].startswith("proc:"):
            from rl.worlds import random_world

            world = random_world(int(job["world"][5:]))
        else:
            world = World.from_layout(world_path(job["world"]))
        r = run_episode(
            variant_path(job["variant"]),
            world,
            seed=job["seed"],
            seconds=job.get("seconds", 120.0),
            overrides=job.get("overrides"),
            yaw_deg=job.get("yaw_deg", 0.0),
            start=tuple(job.get("start", (0.0, 0.0))),
            dyn_over=job.get("dyn_over"),
            arm=job.get("arm", "intact"),
        )
    except Exception as ex:  # keep the batch alive; the failure is in the record
        r = {"error": repr(ex)}
    r.update({k: job[k] for k in job if k not in ("overrides",)})
    r["wall_s"] = round(time.perf_counter() - t0, 1)
    return r


def run_jobs(jobs: list[dict], procs: int, out: str | None = None) -> list[dict]:
    res = []
    f = open(out, "a") if out else None
    with get_context("fork").Pool(procs, maxtasksperchild=20) as pool:
        for r in pool.imap_unordered(_job, jobs):
            res.append(r)
            if f:
                f.write(json.dumps(r) + "\n")
                f.flush()
    if f:
        f.close()
    return res


def seeds(s: str) -> list[int]:
    a, _, b = s.partition("-")
    return list(range(int(a), int(b or a) + 1))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--variants", nargs="+", default=["v3_2"])
    ap.add_argument("--worlds", nargs="+", default=["cage20"])
    ap.add_argument("--seeds", default="0-4")
    ap.add_argument("--seconds", type=float, default=120)
    ap.add_argument("--procs", type=int, default=10)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    jobs = [{"variant": v, "world": w, "seed": s, "seconds": a.seconds} for v in a.variants for w in a.worlds for s in seeds(a.seeds)]
    t0 = time.time()
    run_jobs(jobs, a.procs, a.out)
    print(f"{len(jobs)} episodes in {time.time() - t0:.0f} s")
