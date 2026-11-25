#!/usr/bin/env python3
"""collect.py -- gather the V8 run outputs into two tables.

Reads  runs/<tag>/metrics.json (written by analyze7.py) and runs/<tag>/hg.log (printed by hg_loop.py).
Writes results.csv       one row per run: the scalar fields of metrics.json, plus the guard summary
       guard_status.csv  one row per guard status line of hg.log (about 1 Hz): time since hg_loop start, guard state

Run:  python collect.py [runs directory]   (default: runs/ next to this script)
"""
import csv
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
RUNS = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "runs"


def cond(tag):
    return re.sub(r"^v8_|_hg_r\d+$", "", tag)


rows, status = [], []
for d in sorted(RUNS.iterdir()):
    mf = d / "metrics.json"
    if not mf.exists():
        continue
    m = json.loads(mf.read_text())
    row = {"cond": cond(m["tag"]), "rep": int(re.search(r"_r(\d+)$", m["tag"]).group(1))}
    for k, v in m.items():
        if isinstance(v, (list, dict)):
            continue
        row[k] = v
    row["n_msgs"] = len(m.get("msgs") or [])
    trip_t = trip_reason = trip_mode = None
    e_max = None
    cfg = ""
    disarm_t = None
    hg = d / "hg.log"
    if hg.exists():
        for line in hg.read_text(errors="replace").splitlines():
            mm = re.match(r"\s*([\d.]+) (st|guard cfg) (\{.*\})\s*$", line)
            if mm and mm.group(2) == "guard cfg":
                cfg = mm.group(3)
            elif mm:
                t = float(mm.group(1))
                s = json.loads(mm.group(3))
                status.append({"tag": m["tag"], "cond": row["cond"], "rep": row["rep"], "t": t,
                               "active": int(bool(s.get("active"))), "tripped": int(bool(s.get("tripped"))),
                               "reason": s.get("reason", ""), "blind": s.get("blind", ""), "e": s.get("e"),
                               "mode": s.get("mode"), "rng": s.get("rng")})
                if s.get("e") is not None:
                    e_max = s["e"] if e_max is None else max(e_max, abs(s["e"]))
                if s.get("tripped") and trip_t is None:
                    trip_t, trip_reason, trip_mode = t, s.get("reason", ""), s.get("mode")
            elif "FC disarmed" in line:
                dm = re.match(r"\s*([\d.]+) ", line)
                disarm_t = float(dm.group(1)) if dm else None
    row.update({"guard_tripped": int(trip_t is not None), "guard_first_trip_log_t": trip_t,
                "guard_reason": trip_reason or "", "guard_mode_at_first_trip_line": trip_mode or "",
                "guard_e_abs_max": e_max, "guard_disarm_log_t": disarm_t, "guard_cfg": cfg})
    rows.append(row)

keys = []
for r in rows:
    for k in r:
        if k not in keys:
            keys.append(k)
with open(HERE / "results.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=keys)
    w.writeheader()
    w.writerows(rows)
with open(HERE / "guard_status.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(status[0].keys()))
    w.writeheader()
    w.writerows(status)
print(len(rows), "runs ->", HERE / "results.csv", "|", len(status), "status lines ->", HERE / "guard_status.csv")
