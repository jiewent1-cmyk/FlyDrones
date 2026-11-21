#!/usr/bin/env python3
"""Group table: EKF jump at the fault moment, true speed in the fault window, gate counts."""
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
    "stop_norec": (45.0, 45.0, 999.0), "lat500_20s": (40.0, 44.0, 20.0),
}
rows = []
for f in sorted(glob.glob(D + "/runs/*/metrics.json")):
    r = json.load(open(f))
    tag = r["tag"]
    m = re.match(r"(sq4|sq1|base)_(.*)", tag)
    if not m:
        continue
    lay = m.group(1)
    flt = re.sub(r"_r\d+$", "", m.group(2))
    if flt not in FAULTS:
        continue
    rel = FAULTS[flt][0] if lay == "sq4" else FAULTS[flt][1]
    dur = FAULTS[flt][2]
    out = subprocess.run([PY, D + "/jump_at_fault.py", os.path.dirname(f), str(rel), str(min(dur, 60))],
                         capture_output=True, text=True).stdout.strip()
    j = json.loads(out) if out.startswith("{") else {}
    inj = json.loads(r.get("inj") or "{}")
    rows.append({"tag": tag, "lay": lay, "flt": flt, "rel": rel, "dur": dur,
                 "rms": r.get("track_rms"), "tmax": r.get("track_max"), "lap3": r.get("lap3"),
                 "land": r.get("land_offset"), "jump": j.get("jump_at_fault"), "jt": j.get("jump_t_rel"),
                 "spd": j.get("true_spd_fault"), "disp": j.get("true_disp_fault"),
                 "gated": r.get("n_gated"), "innov": r.get("gate_innov_rej"), "inj": inj,
                 "jm": r.get("est_jump_max"), "spdm": r.get("spd_max")})
json.dump(rows, open(D + "/fault_rows.json", "w"), indent=1)

g = {}
for r in rows:
    g.setdefault(r["lay"] + "_" + r["flt"], []).append(r)


def c(rs, k):
    v = [r[k] for r in rs if isinstance(r.get(k), (int, float))]
    if not v:
        return "-"
    return f"{stt.median(v):.3f} [{min(v):.3f}-{max(v):.3f}]"


out = ["| 布置 × 故障 | 故障时刻 EKF 跳变 | 该窗口真值速度峰值 | 该窗口真值位移 | 全程最大跳变 | 门限 v1 拒 | 门限 v2 拒 |",
       "|---|---|---|---|---|---|---|"]
for k in sorted(g, key=lambda x: (x.split("_")[0], list(FAULTS).index(x.split("_", 1)[1]))):
    rs = g[k]
    out.append("| %s | %s | %s | %s | %s | %s | %s |" % (
        k, c(rs, "jump"), c(rs, "spd"), c(rs, "disp"), c(rs, "jm"),
        sum(r["gated"] or 0 for r in rs), sum(r["innov"] or 0 for r in rs)))
open(D + "/table_fault_window.md", "w").write("\n".join(out) + "\n")
print("\n".join(out))
