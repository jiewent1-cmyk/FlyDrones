#!/usr/bin/env python3
"""Run jobs (lines 'tag|parm|speedup|mtf_sim args') on SITL instances 3..6, at most 4 at a time."""
import subprocess, sys, time, os
D = "/home/jacky/sim/mtf01"
jobs = [l.strip() for l in open(sys.argv[1]) if l.strip() and not l.startswith("#")]
free = [int(i) for i in os.environ.get("SLOTS", "3,4,5,6").split(",")]
running = []
while jobs or running:
    while jobs and free:
        tag, parm, spd, args = jobs.pop(0).split("|")
        inst = free.pop(0)
        p = subprocess.Popen(["bash", D + "/run_one.sh", tag, str(inst), spd, D + "/" + parm] + args.split())
        running.append((p, inst, tag))
        print(time.strftime("%H:%M:%S"), "start", tag, "I%d" % inst, flush=True)
    time.sleep(2)
    for r in running[:]:
        if r[0].poll() is not None:
            running.remove(r); free.append(r[1])
            print(time.strftime("%H:%M:%S"), "done ", r[2], flush=True)
