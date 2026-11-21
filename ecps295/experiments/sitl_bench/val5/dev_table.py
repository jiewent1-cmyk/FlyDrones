#!/usr/bin/env python3
"""Add truth-deviation-vs-time around the fault to fault_rows.json and print a table."""
import glob
import json
import os
import re
import statistics as stt
import subprocess

D = "/home/jacky/sim/val5"
PY = "/home/jacky/miniconda3/envs/flydrones/bin/python"
FAULTS = {
    "bad1m_lap1": (15.0, 25.0, 3.0), "bad1m_lap2": (37.5, 48.0, 3.0), "bad03_10s": (30.0, 44.0, 10.0),
    "yaw20_5s": (44.0, 46.0, 5.0), "block30": (33.0, 33.0, 30.0), "stop10": (50.0, 45.0, 10.0),
    "stop_norec": (45.0, 45.0, 60.0), "lat500_20s": (40.0, 44.0, 20.0),
}
rows = json.load(open(D + "/fault_rows.json"))
for r in rows:
    rel, dur = r["rel"], min(r["dur"], 60.0)
    out = subprocess.run([PY, D + "/dev_at_fault.py", D + "/runs/" + r["tag"], str(rel), str(dur)],
                         capture_output=True, text=True).stdout.strip()
    d = json.loads(out) if out.startswith("{") else {}
    r.update(d)
json.dump(rows, open(D + "/fault_rows.json", "w"), indent=1)

g = {}
for r in rows:
    g.setdefault(r["lay"] + "_" + r["flt"], []).append(r)


def c(rs, k):
    v = [r[k] for r in rs if isinstance(r.get(k), (int, float))]
    if not v:
        return "-"
    return f"{stt.median(v):.3f} [{min(v):.3f}-{max(v):.3f}]"


out = ["| 布置 × 故障 | 故障前最大偏离 | 故障时刻偏离 | 故障窗口最大偏离 | 恢复后最大偏离 | 全程最大偏离(发生时刻) |",
       "|---|---|---|---|---|---|"]
for k in sorted(g, key=lambda x: (x.split("_")[0], list(FAULTS).index(x.split("_", 1)[1]))):
    rs = g[k]
    tmax = ", ".join("%.0f" % (r["dev_max_rel"] or -1) for r in rs)
    out.append("| %s | %s | %s | %s | %s | %s (%s) |" % (
        k, c(rs, "dev_pre_max"), c(rs, "dev_at_fault"), c(rs, "dev_fault_window_max"),
        c(rs, "dev_after_max"), c(rs, "dev_max"), tmax))
open(D + "/table_dev_window.md", "w").write("\n".join(out) + "\n")
print("\n".join(out))
