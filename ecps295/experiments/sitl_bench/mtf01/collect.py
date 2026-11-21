#!/usr/bin/env python3
"""runs/*/metrics.json -> results.csv (one row per run) + a compact table on stdout. Usage: collect.py [prefix...]"""
import csv
import json
import os
import sys

D = "/home/jacky/sim/mtf01/runs"
pref = sys.argv[1:]
rows = []
for tag in sorted(os.listdir(D)):
    if pref and not any(tag.startswith(p) for p in pref):
        continue
    f = os.path.join(D, tag, "metrics.json")
    cmd = open(os.path.join(D, tag, "cmd.txt")).read().strip() if os.path.exists(os.path.join(D, tag, "cmd.txt")) else ""
    try:
        r = json.load(open(f))
    except Exception as e:
        rows.append({"tag": tag, "config": cmd, "complete": "ANALYSIS_FAILED", "note": str(e)[:80]})
        continue
    sc = r.get("script") or {}
    h = r.get("hover", {})
    rows.append({
        "tag": tag, "config": cmd.split("args=")[-1] if cmd else "", "speedup": cmd.split("speedup=")[1].split()[0] if "speedup=" in cmd else "",
        "parm": os.path.basename(cmd.split("parm=")[1].split()[0]) if "parm=" in cmd else "",
        "complete": r.get("mission_complete"), "reached": "".join(str(k) for k in r.get("reached", [])),
        "loiter_armed": sc.get("loiter_armed"), "auto_armed": sc.get("auto_armed"),
        "hover_agl": h.get("true_agl_mean"), "hover_hgt_est_rms": h.get("hgt_est_rms"), "hover_drift": h.get("true_drift_max"),
        "alt_rms": r.get("alt_rms"), "alt_mean": r.get("alt_mean"), "alt_max": r.get("alt_max"), "hgt_est_rms": r.get("hgt_est_rms"),
        "track_rms": r.get("track_rms"), "track_max": r.get("track_max"), "radius_max": r.get("radius_max"),
        "land_offset": r.get("land_offset"), "pos_est_err_end": r.get("pos_est_err_end"),
        "auto_s": r.get("auto_seconds"), "on_ground": r.get("disarmed_on_ground"),
        "note": "; ".join(m for m in r.get("fc_msgs", []) if m and not m.startswith(("Arming", "Disarming", "EKF3 IMU1", "Barometer", "SIM Hit", "Field", "AHRS", "RC7", "ArduPilot")) and not m[1:].startswith("fdc10e"))[:200],
    })
keys = list(rows[0].keys()) if rows else []
for r in rows:
    for k in r:
        if k not in keys:
            keys.append(k)
out = "/home/jacky/sim/mtf01/results.csv" if not pref else "/dev/null"
with open(out, "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=keys)
    w.writeheader()
    w.writerows(rows)
short = ["tag", "complete", "reached", "auto_armed", "hover_agl", "hover_hgt_est_rms", "alt_mean", "alt_rms", "alt_max", "hgt_est_rms", "track_rms", "track_max", "land_offset", "pos_est_err_end", "auto_s"]
print(" ".join("%-9s" % k[:9] if k != "tag" else "%-22s" % k for k in short))
for r in rows:
    print(" ".join(("%-22s" % r.get(k, "")) if k == "tag" else "%-9s" % str(r.get(k, ""))[:9] for k in short), "|", r.get("note", "")[:90])
