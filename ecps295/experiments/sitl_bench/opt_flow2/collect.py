#!/usr/bin/env python3
"""runs/*/metrics.json -> results.csv (one row per run) and summary.md (median [min-max] per group = tag without _rN)."""
import csv, glob, json, os, re, statistics as stt, sys
D = "/home/jacky/sim/opt_flow2"
rows = []
for f in sorted(glob.glob(D + "/runs/*/metrics.json")):
    try:
        r = json.load(open(f))
    except Exception as e:
        rows.append({"tag": os.path.basename(os.path.dirname(f)), "error": "bad json"}); continue
    cmd = open(os.path.join(os.path.dirname(f), "cmd.txt")).read().strip()
    r["group"] = re.sub(r"_r\d+$", "", r["tag"])
    r["ovl"] = re.search(r"ovl=(\S+)", cmd).group(1) if "ovl=" in cmd else ""
    r["yaw"] = re.search(r"yaw=(\S+)", cmd).group(1) if "yaw=" in cmd else "0"
    r["msgs"] = " / ".join(r.get("msgs") or r.get("fc_msgs") or [])
    r.pop("script", None); r.pop("fc_msgs", None); r.pop("wp_err", None); r.pop("reached", None); r.pop("bin", None)
    rows.append(r)
keys = ["tag", "group", "yaw", "ovl"] + sorted({k for r in rows for k in r} - {"tag", "group", "yaw", "ovl", "msgs"}) + ["msgs"]
with open(D + "/results.csv", "w", newline="") as fh:
    w = csv.DictWriter(fh, keys); w.writeheader(); [w.writerow(r) for r in rows]
groups = {}
for r in rows:
    groups.setdefault(r.get("group", r["tag"]), []).append(r)
cols = sys.argv[1].split(",") if len(sys.argv) > 1 else ["hov_alt_mean", "hov_alt_rms", "hov_drift_max", "hov_pos_est_end", "track_rms", "alt_leg_rms", "alt_cnr_rms"]
pat = sys.argv[2] if len(sys.argv) > 2 else ""
print("| group | n | ok | " + " | ".join(cols) + " |")
print("|---" * (len(cols) + 3) + "|")
for g in sorted(groups):
    if not re.search(pat, g):
        continue
    rs = groups[g]
    ok = sum(1 for r in rs if r.get("mission_complete"))
    cells = []
    for c in cols:
        v = [r[c] for r in rs if isinstance(r.get(c), (int, float)) and not isinstance(r.get(c), bool)]
        cells.append("-" if not v else (f"{stt.median(v):.3f}" if len(v) == 1 else f"{stt.median(v):.3f} [{min(v):.3f}–{max(v):.3f}]"))
    print(f"| {g} | {len(rs)} | {ok}/{len(rs)} | " + " | ".join(cells) + " |")
