#!/usr/bin/env python3
"""mk_k_fault.py -- 生成 k 格的 fault 列表（读数钉在 0.2 m）。

从基线飞行（无异常）的 monitor.csv 取出真实高度曲线（测距读数 ≈ 真值离地高度），
按 fault 时钟（相对 AUTO 开始 = FB_TB0）生成跟随爬升的 SIM_SONAR_POS_Z 台阶：
    读数 = 真值高度 - SIM_SONAR_POS_Z  =>  要读数 = 0.2  =>  pos_z = 高度 - 0.2
"""
import csv
import os
import re
import sys

D = "/home/jacky/sim/val7"
RUN = sys.argv[1] if len(sys.argv) > 1 else os.path.join(D, "runs", "v7_base_hov_r1")
STEP = 0.25          # 台阶步长 [s]
END_DT = 60.0        # 生成到 fault 时刻 [s]

rows = list(csv.DictReader(open(os.path.join(RUN, "monitor.csv"))))
curve = [(float(r["t"]), float(r["distance"])) for r in rows if r["distance"] not in ("", None)]

log = open(os.path.join(RUN, "run.log")).read()
tb0 = float(re.search(r"FB_TB0 ([\d.]+)", log).group(1))


def alt_at(dt):
    t = tb0 + dt
    best = None
    for (tt, d) in curve:
        if best is None or abs(tt - t) < abs(best[0] - t):
            best = (tt, d)
    return best[1] if best else 0.0


# 起飞时刻：高度开始上升
t_lo = None
for dt in [i * 0.05 for i in range(0, int(END_DT / 0.05))]:
    if alt_at(dt) > 0.15:
        t_lo = dt
        break
if t_lo is None:
    t_lo = 3.0

out = []
dt = t_lo
while dt <= END_DT:
    v = round(alt_at(dt) - 0.2, 2)
    out.append("--fault %s:SIM_SONAR_POS_Z=%s" % (round(dt, 2), v))
    dt += STEP
open(os.path.join(D, "k_faults.txt"), "w").write(" ".join(out))
print("tb0=%.3f t_lo=%.2f n=%d" % (tb0, t_lo, len(out)))
print("first:", out[0], "last:", out[-1])
print("alt at t_lo=%.3f  at +10s=%.3f" % (alt_at(t_lo), alt_at(t_lo + 10)))
