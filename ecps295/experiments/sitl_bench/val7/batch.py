#!/usr/bin/env python3
"""Run jobs 'tag|parm|speedup|yaw|overlay|args' on SITL instances 30..33 (at most 4 at a time). Skips tags with metrics.json unless FORCE=1."""
import subprocess, sys, time, os
D = "/home/jacky/sim/val7"
jobs = [l.strip() for f in sys.argv[1:] for l in open(f) if l.strip() and not l.startswith("#")]  # several job files run in order
if not os.environ.get("FORCE"):
    jobs = [j for j in jobs if not os.path.exists(f"{D}/runs/{j.split('|')[0]}/metrics.json")]
free = [30, 31, 32, 33]
running = []
while jobs or running:
    while jobs and free:
        tag, parm, spd, yaw, ovl, args = jobs.pop(0).split("|")
        inst = free.pop(0)
        p = subprocess.Popen(["bash", D + "/run_one.sh", tag, str(inst), spd, D + "/" + parm, yaw, ovl] + args.split())
        running.append((p, inst, tag))
        print(time.strftime("%H:%M:%S"), "start", tag, "I%d" % inst, flush=True)
    time.sleep(2)
    for r in running[:]:
        if r[0].poll() is not None:
            running.remove(r); free.append(r[1])
            print(time.strftime("%H:%M:%S"), "done ", r[2], flush=True)
