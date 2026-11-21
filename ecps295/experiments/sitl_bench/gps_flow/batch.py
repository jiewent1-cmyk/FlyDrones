#!/usr/bin/env python3
"""Run jobs (lines 'tag|parm|speedup|home|gf_sim args') on SITL instances 20..23, at most 4 at a time. (copy of mtf01/batch.py)"""
import subprocess, sys, time, os
D = "/home/jacky/sim/gps_flow"
jobs = [l.strip() for l in open(sys.argv[1]) if l.strip() and not l.startswith("#")]
free = [int(i) for i in os.environ.get("SLOTS", "20,21,22,23").split(",")]
running = []
while jobs or running:
    while jobs and free:
        tag, parm, spd, home, args = jobs.pop(0).split("|")
        inst = free.pop(0)
        p = subprocess.Popen(["bash", D + "/run_one.sh", tag, str(inst), spd, parm, home] + args.split())
        running.append((p, inst, tag))
        print(time.strftime("%H:%M:%S"), "start", tag, "I%d" % inst, flush=True)
    time.sleep(2)
    for r in running[:]:
        if r[0].poll() is not None:
            running.remove(r); free.append(r[1])
            print(time.strftime("%H:%M:%S"), "done ", r[2], flush=True)
