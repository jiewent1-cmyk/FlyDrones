"""One row per Gazebo run on the server (val_*, fix_*, a1_*, ab_*): outcome, trigger metrics, saccade turn, RTF."""
import csv
import glob
import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, "ecps295")
from rl.trigger_metrics import from_run_csv


def turns(p):
    """Integrated |yaw| per saccade (deg) from a run.csv."""
    rows = list(csv.DictReader(open(p)))
    t = np.array([float(r["t"]) for r in rows])
    yr = np.array([float(r["yaw_rate_dps"]) for r in rows])
    esc = np.array([r["escape"] == "1" for r in rows])
    dt = np.diff(t, prepend=t[0])
    out, i = [], 0
    while i < len(esc):
        if esc[i]:
            j = i
            while j < len(esc) and esc[j]:
                j += 1
            out.append(abs(float(np.sum(yr[i:j] * dt[i:j]))))
            i = j
        else:
            i += 1
    return out


keys = ["contact_episodes", "near_miss_episodes", "min_clearance_m", "path_m", "coverage", "escapes", "fence_turn_s", "radius_true_max_m", "alt_max", "nav", "variant", "arm"]
tkeys = ["threat_episodes", "saccade_recall", "saccade_latency_s", "false_alarms_per_min", "chain_rate", "slow_fraction", "mean_speed"]
w = csv.writer(open(sys.argv[1], "w", newline=""))
w.writerow(["run", "group", "seed"] + keys + tkeys + ["saccade_turn_deg_median", "rtf_run"])
n = 0
for d in sorted(glob.glob("runs/*/run.json")):
    run = d.split("/")[1]
    m = re.match(r"(.*)_s(\d+)$", run) or re.match(r"(.*)_i(\d+)$", run)
    j = json.load(open(d))
    c = d.replace("run.json", "run.csv")
    tm = from_run_csv(c) if os.path.exists(c) else {}
    tu = turns(c) if os.path.exists(c) else []
    rtf = ""
    rtf_log = d.replace("run.json", "rtf.log")
    if os.path.exists(rtf_log):
        for line in open(rtf_log):
            if line.startswith("RTF run"):
                rtf = line.split()[2]
    w.writerow(
        [run, m.group(1) if m else run, m.group(2) if m else ""]
        + [j.get(k, "") for k in keys]
        + [tm.get(k, "") for k in tkeys]
        + [round(float(np.median(tu)), 1) if tu else "", rtf]
    )
    n += 1
print(n, "runs")
