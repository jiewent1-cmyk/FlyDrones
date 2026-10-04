"""Navigation-source comparison for patrol runs (TechRoute §10 S7d): GPS vs 3901-L0X flow EKF, scored on simulator truth.

    python nav_compare.py ~/sim/runs patrolG_v3 patrolF_v3 patrolFB_v3

Per group: contacts, near misses, path, EKF horizontal error, how far past the geofence the drone really went, height
excursions (truth above the 1.3 m hard fence / below 0.3 m) and the EKF height error, plus mean RTF.
"""

import csv
import glob
import json
import os
import statistics as st
import sys

root, groups = sys.argv[1], sys.argv[2:]
FENCE_R, ALT_HARD = 2.0, 1.3
print(
    "| group | runs | runs with contact | contacts | near misses | path m (mean) | EKF xy err max m (median / max) "
    "| true radius max m (max) | runs past fence | runs > 1.3 m | runs on the floor | EKF-truth height min m | RTF mean |"
)
print("|---|" + "---|" * 12)
for g in groups:
    runs = sorted(glob.glob(os.path.join(root, f"{g}_s*")))
    js, rows = [], []
    for d in runs:
        try:
            js.append(json.load(open(os.path.join(d, "run.json"))))
            rows.append(list(csv.DictReader(open(os.path.join(d, "run.csv")))))
        except FileNotFoundError:
            continue
    rtf = []
    for d in runs:
        try:
            rtf.append(float(open(os.path.join(d, "rtf.log")).read().split()[2]))
        except (FileNotFoundError, IndexError, ValueError):
            pass
    contacts = [j.get("contact_episodes", 0) for j in js]
    err = [j["ekf_err_xy_max_m"] for j in js]
    rad = [j["radius_true_max_m"] for j in js]
    high = sum(1 for r in rows if max(float(x["alt"]) for x in r) > ALT_HARD)
    floor = sum(1 for r in rows if sum(float(x["alt"]) < 0.1 for x in r) > 20)  # > 1 s near the floor
    dz = min(min(float(x["ekf_alt"] or 0) - float(x["alt"]) for x in r) for r in rows)
    print(
        f"| {g} | {len(js)} | {sum(c > 0 for c in contacts)} | {sum(contacts)} | {sum(j.get('near_miss_episodes', 0) for j in js)} "
        f"| {st.mean(j['path_m'] for j in js):.1f} | {st.median(err):.2f} / {max(err):.2f} | {max(rad):.2f} "
        f"| {sum(r > FENCE_R + 0.1 for r in rad)} | {high} | {floor} | {dz:+.2f} "
        f"| {st.mean(rtf):.3f} ({min(rtf):.2f}-{max(rtf):.2f}) |"
    )
