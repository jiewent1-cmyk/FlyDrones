#!/usr/bin/env python3
# mk_tables.py -- 生成报告里的总表
import csv
import json
import os

D = "/home/jacky/sim/val7"
OUT = open(os.path.join(D, "total_table.md"), "w")


def w(s=""):
    OUT.write(s + "\n")


def fp_rows():
    rows = list(csv.DictReader(open(os.path.join(D, "fp_s1.csv"))))
    out = []
    for r in rows:
        p = json.loads(r["params"])
        out.append((p["fc"], p["tau"], p["E"], p["T"], p["R"], int(r["n_trigger"]), int(r["n_runs"]),
                    int(r["n_trig_A"]), int(r["n_trig_B"]), float(r["fp_per_h"])))
    out.sort(key=lambda x: (x[9], x[5]))
    return out


def det_rows():
    rows = list(csv.DictReader(open(os.path.join(D, "det_s2.csv"))))
    out = []
    for r in rows:
        p = json.loads(r["params"])
        out.append((p["fc"], p["tau"], p["E"], p["T"], p["R"], int(r["n_det"]), int(r["n_runs"]),
                    r["delay_med"] or "-", r["delay_max"] or "-"))
    out.sort(key=lambda x: (-x[5], float(x[7]) if x[7] != "-" else 99))
    return out
