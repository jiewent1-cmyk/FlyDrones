#!/usr/bin/env python3
"""Run jobs on SITL instances 10..13 (only these), at most 4 at a time.

Job line:  tag|parmfile|speedup|home_yaw|opt_sim args
"""
import os
import subprocess
import sys
import time

D = "/home/jacky/sim/opt_flow"
jobs = [l.strip() for l in open(sys.argv[1]) if l.strip() and not l.startswith("#")]
free = [int(i) for i in os.environ.get("SLOTS", "10,11,12,13").split(",")]
running = []
while jobs or running:
    while jobs and free:
        tag, parm, spd, yaw, args = jobs.pop(0).split("|", 4)
        inst = free.pop(0)
        p = subprocess.Popen(["bash", D + "/run_one.sh", tag, str(inst), spd, D + "/" + parm, yaw] + args.split())
        running.append((p, inst, tag))
        print(time.strftime("%H:%M:%S"), "start", tag, "I%d" % inst, flush=True)
    time.sleep(2)
    for r in running[:]:
        if r[0].poll() is not None:
            running.remove(r)
            free.append(r[1])
            print(time.strftime("%H:%M:%S"), "done ", r[2], flush=True)
