"""S7e: brain v3/v4 x FC height (flow EKF / companion ExternalNav) on the random low-box cages, scored on truth.

python lowbox_compare.py ~/sim/runs lb_v3 lb_v4 lb_v3B lb_v4B
"""

import glob
import json
import os
import statistics as st
import sys

root, groups = sys.argv[1], sys.argv[2:]
cols = (
    "| group | valid runs | runs with contact | contacts | near misses | path m (mean) | over boxes s (mean) "
    "| max true height m (max) | FC landings | FC height error max m (median / max) | RTF whole run |"
)
print(cols)
print("|---|" + "---|" * (cols.count("|") - 2))
for g in groups:
    js, rtf, invalid = [], [], []
    for d in sorted(glob.glob(os.path.join(root, f"{g}_s*"))):
        try:
            j = json.load(open(os.path.join(d, "run.json")))
        except FileNotFoundError:
            invalid.append(os.path.basename(d) + " (no result)")
            continue
        r = [float(ln.split()[2]) for ln in open(os.path.join(d, "rtf.log")) if ln.startswith("RTF run")]
        if not r or r[0] < 0.95:
            invalid.append(f"{os.path.basename(d)} (RTF {r[0] if r else '?'})")
            continue
        js.append(j)
        rtf.append(r[0])
    if not js:
        print(f"| {g} | 0 | | | | | | | | | invalid: {', '.join(invalid)} |")
        continue
    err = [j["ekf_err_alt_max_m"] for j in js]
    print(
        f"| {g} | {len(js)} | {sum(j['contact_episodes'] > 0 for j in js)} | {sum(j['contact_episodes'] for j in js)} "
        f"| {sum(j['near_miss_episodes'] for j in js)} | {st.mean(j['path_m'] for j in js):.1f} "
        f"| {st.mean(j['over_box_s'] for j in js):.1f} | {max(j['alt_true_max_m'] for j in js):.2f} "
        f"| {sum(bool(j['fc_land']) for j in js)} | {st.median(err):.2f} / {max(err):.2f} "
        f"| {min(rtf):.3f}-{max(rtf):.3f}{'; invalid: ' + ', '.join(invalid) if invalid else ''} |"
    )
