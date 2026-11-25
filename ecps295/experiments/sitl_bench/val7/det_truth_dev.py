#!/usr/bin/env python3
"""det_truth_dev.py -- truth deviation at the trigger for the step-2 fault cells, selected parameter group.

For each run runs/v7_s2_<cell>_r<k> it replays the recorded monitor inputs offline (offline_eval.py det) with the
selected group fc=0.5, tau=30, E=0.7, T=1.0, R=0.6 and writes det_s2_truth_dev.csv: detection delay and the true
height minus the 2 m mission height at the moment of the trigger. The injection time is taken from the run's own
metrics.json (online trigger time minus online detection delay).

Needs the run directories (not in the repository) and pymavlink.  Run:  python det_truth_dev.py
"""
import csv
import json
import subprocess
import sys

SET = "fc=0.5,tau=30,E=0.7,T=1.0,R=0.6"
CELLS = ["a", "c", "g3", "g5", "g10", "gN", "h", "i", "j1", "j2", "k"]
rows = []
for c in CELLS:
    for r in range(1, 6):
        d = f"runs/v7_s2_{c}_r{r}"
        m = json.load(open(d + "/metrics.json"))
        inj = round(m["mon_trigger_t"] - m["mon_detect_delay"], 3)
        p = subprocess.run([sys.executable, "offline_eval.py", "det", "--run", d, "--inject-t", str(inj), "--set", SET],
                           capture_output=True, text=True)
        j = json.loads(p.stdout)
        rows.append({"cell": c, "rep": r, "set": SET, "inject_t": inj, "t_trig": j.get("t_trig"),
                     "reason": j.get("reason"), "delay_s": j.get("delay_s"),
                     "truth_at_trig": j.get("truth_at_trig"), "truth_dev_at_trig": j.get("truth_dev_at_trig")})
with open("det_s2_truth_dev.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)
for c in CELLS:
    v = sorted(x["truth_dev_at_trig"] for x in rows if x["cell"] == c and x["truth_dev_at_trig"] is not None)
    dl = sorted(x["delay_s"] for x in rows if x["cell"] == c and x["delay_s"] is not None)
    print(f"{c:4s} n={len(v)} dev {v[0]:+.3f} .. {v[-1]:+.3f}   delay {dl}")
