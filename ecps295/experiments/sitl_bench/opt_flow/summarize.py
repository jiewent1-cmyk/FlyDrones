#!/usr/bin/env python3
"""summarize.py - collect runs/*/metrics.json into results.csv (one row per run)."""
import csv
import json
import os
import sys

D = "/home/jacky/sim/opt_flow"
COLS = ["tag", "scenario", "alt_cmd", "side", "laps", "parm", "yaw", "args",
        "true_drift_final", "true_drift_max", "drift_rate_ms", "est_drift_final", "est_drift_max",
        "pos_est_err_final", "pos_est_err_max", "true_agl_mean", "true_agl_min", "true_agl_max",
        "alt_err_rms", "hgt_est_rms", "hgt_est_maxabs", "true_radius_max", "hover_window_s",
        "track_rms", "track_max", "alt_rms", "alt_max", "alt_mean", "wp_err", "lap_err",
        "land_offset", "pos_est_err_end", "radius_max", "mission_complete", "auto_seconds",
        "fault_max_agl", "fault_min_agl", "pre_fault_agl", "fault_dev_max", "fault_dev_min", "fault_dev_absmax", "fault_spec", "armed", "disarmed", "note"]

rows = []
for tag in sorted(os.listdir(D + "/runs")):
    rd = D + "/runs/" + tag
    if not os.path.isdir(rd) or not os.path.exists(rd + "/metrics.json"):
        continue
    try:
        d = json.load(open(rd + "/metrics.json"))
    except Exception as e:
        rows.append({"tag": tag, "note": "metrics parse error: %s" % e})
        continue
    cmd = open(rd + "/cmd.txt").read() if os.path.exists(rd + "/cmd.txt") else ""
    parm = cmd.split("parm=")[1].split()[0] if "parm=" in cmd else ""
    yaw = cmd.split("yaw=")[1].split()[0] if "yaw=" in cmd else ""
    args = cmd.split("args=")[1].strip() if "args=" in cmd else ""
    r = {"tag": tag, "parm": parm, "yaw": yaw, "args": args}
    for c in COLS:
        if c in d:
            r[c] = d[c]
    sc = d.get("script") or {}
    if "armed" in sc:
        r["armed"] = sc["armed"]
    if "disarmed" in sc:
        r["disarmed"] = sc["disarmed"]
    msgs = d.get("fc_msgs") or []
    if msgs:
        r["note"] = "; ".join(msgs[:4])
    if d.get("error"):
        r["note"] = str(d.get("note", "")) + " ERR:" + str(d["error"])
    rows.append(r)

with open(D + "/results.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=COLS, extrasaction="ignore")
    w.writeheader()
    for r in rows:
        w.writerow(r)
print("wrote results.csv with %d runs" % len(rows))
for r in rows:
    if "hover_window_s" in r:
        print("%-14s %-8s drift_final=%-6s max=%-6s rate=%-7s poserr=%-6s agl=%-6s hgt_rms=%-6s %s" % (
            r["tag"], r.get("parm", "").split("/")[-1].replace(".parm", ""),
            r.get("true_drift_final"), r.get("true_drift_max"), r.get("drift_rate_ms"),
            r.get("pos_est_err_final"), r.get("true_agl_mean"), r.get("hgt_est_rms"), r.get("note", "")[:60]))
    elif "track_rms" in r:
        print("%-14s %-8s track_rms=%-6s max=%-6s alt_rms=%-6s land=%-6s lap=%-30s %s" % (
            r["tag"], r.get("parm", "").split("/")[-1].replace(".parm", ""),
            r.get("track_rms"), r.get("track_max"), r.get("alt_rms"), r.get("land_offset"),
            str(r.get("lap_err")), r.get("note", "")[:40]))
