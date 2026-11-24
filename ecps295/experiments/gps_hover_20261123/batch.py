#!/usr/bin/env python3
"""Run jobs (lines 'tag|parm|speedup|home|gh_sim args') on SITL instances BASE..BASE+NINST-1. Tags with metrics.json are skipped."""
import subprocess, sys, time, os
D = "/home/jacky/sim/gps_hover"
jobs = [l.strip() for l in open(sys.argv[1]) if l.strip() and not l.startswith("#")]
jobs = [j for j in jobs if not os.path.exists("%s/runs/%s/metrics.json" % (D, j.split("|")[0]))]
base, n = int(os.environ.get("BASE", "40")), int(os.environ.get("NINST", "10"))
free, running = list(range(base, base + n)), []
print(len(jobs), "jobs", flush=True)
while jobs or running:
    while jobs and free:
        tag, parm, spd, home, args = jobs.pop(0).split("|")
        inst = free.pop(0)
        p = subprocess.Popen(["bash", D + "/run_one.sh", tag, str(inst), spd, parm, home] + args.split())
        running.append((p, inst, tag))
    time.sleep(1)
    for r in running[:]:
        if r[0].poll() is not None:
            running.remove(r); free.append(r[1])
            print(time.strftime("%H:%M:%S"), "done", r[2], len(jobs), "left", flush=True)
