#!/usr/bin/env python3
"""runs/*/metrics.json -> results.csv (one row per run) + summary.md (median [min-max] per config)."""
import csv
import glob
import json
import os
import re
import statistics

D = "/home/jacky/sim/gps_flow"
COLS = ["mission_complete", "auto_armed", "track_rms", "track_max", "hold_rms_to_wp", "hold_max_to_wp", "hold_wander_max",
        "alt_rms", "alt_max", "hgt_est_rms", "pos_est_err_max", "land_offset", "radius_max", "yaw_mean", "yaw_maxabs", "yaw_change"]
rows = []
for mj in sorted(glob.glob(D + "/runs/*/metrics.json")):
    rd = os.path.dirname(mj)
    try:
        d = json.load(open(mj))
    except Exception as e:
        d = {"tag": os.path.basename(rd), "error": "metrics unreadable: %s" % e}
    cmd = open(rd + "/cmd.txt").read().strip() if os.path.exists(rd + "/cmd.txt") else ""
    r = {"tag": d.get("tag"), "config": re.sub(r"_r\d+$", "", d.get("tag", "")), "home": re.search(r"home=(\S+)", cmd).group(1) if "home=" in cmd else ""}
    sc = d.get("script") or {}
    r["auto_armed"] = sc.get("auto_armed")
    for k in COLS:
        if k in d:
            r[k] = d[k]
    ya = d.get("yaw_err_auto") or {}
    r["yaw_mean"], r["yaw_maxabs"], r["yaw_change"] = ya.get("mean"), ya.get("maxabs"), ya.get("max_change_from_arm")
    if d.get("hover"):
        r["hover_true_drift"] = d["hover"]["true_drift_max"]
        r["hover_est_drift"] = d["hover"]["est_drift_max"]
    sw = d.get("switches") or []
    for i, s in enumerate(sw[:2]):
        for k in ("t_rel_arm", "ch8", "est_err_before", "ekf_minus_true_jump_2s", "true_move_10s", "true_hspd_max_10s", "true_hspd_max_5s_before", "true_alt_dev_10s"):
            r["sw%d_%s" % (i + 1, k)] = s.get(k)
        r["sw%d_msgs" % (i + 1)] = " / ".join(s.get("msgs_10s", []))
    r["fc_msgs"] = " / ".join(m for m in d.get("fc_msgs", []) if not re.match(r"^[0-9A-Za-z]{32}$", m))[:400]
    r["error"] = d.get("error", "")
    rows.append(r)
keys = []
for r in rows:
    for k in r:
        if k not in keys:
            keys.append(k)
with open(D + "/results.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=keys)
    w.writeheader()
    w.writerows(rows)

SUM = ["track_rms", "track_max", "hold_rms_to_wp", "hold_wander_max", "alt_rms", "alt_max", "hgt_est_rms", "pos_est_err_max", "land_offset", "yaw_maxabs", "yaw_change",
       "sw1_ekf_minus_true_jump_2s", "sw1_true_move_10s", "sw1_true_hspd_max_10s", "sw1_true_alt_dev_10s", "hover_true_drift"]


def fmt(v):
    v = [x for x in v if isinstance(x, (int, float)) and x is not None]
    if not v:
        return "-"
    return "%.3g [%.3g–%.3g]" % (statistics.median(v), min(v), max(v)) if len(v) > 1 else "%.3g" % v[0]


cfgs = []
for r in rows:
    if r["config"] not in cfgs:
        cfgs.append(r["config"])
with open(D + "/summary.md", "w") as f:
    used = [k for k in SUM if any(isinstance(r.get(k), (int, float)) for r in rows)]
    f.write("| config | n | complete | " + " | ".join(used) + " |\n|" + "---|" * (len(used) + 3) + "\n")
    for c in cfgs:
        rr = [r for r in rows if r["config"] == c]
        f.write("| %s | %d | %d/%d | " % (c, len(rr), sum(1 for r in rr if r.get("mission_complete")), len(rr)) + " | ".join(fmt([r.get(k) for r in rr]) for k in used) + " |\n")
print(open(D + "/summary.md").read())
