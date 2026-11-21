#!/usr/bin/env python3
"""q3.py -- 打印某个运行的数据flash 时序：真值高度 / EKF 高度 / 测距读数 / 模式。

用法: python3 q3.py runs/v7_s4_gN_none_r1 [step_s]
"""
import os
import sys

from pymavlink import mavutil

rd = sys.argv[1].rstrip("/")
step = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
ld = os.path.join(rd, "logs")
logs = sorted(f for f in os.listdir(ld) if f.endswith(".BIN"))

rfnd, sim, ekf, modes, arms = [], [], [], [], []
for f in logs:
    m = mavutil.mavlink_connection(os.path.join(ld, f))
    while True:
        x = m.recv_match(type=["SIM2", "XKF1", "RFND", "MODE", "ARM"])
        if x is None:
            break
        k = x.get_type()
        t = x.TimeUS / 1e6
        if k == "SIM2":
            sim.append((t, -x.PD))
        elif k == "XKF1" and x.C == 0:
            ekf.append((t, -x.PD))
        elif k == "RFND":
            rfnd.append((t, x.Dist))
        elif k == "MODE":
            modes.append((t, x.ModeNum))
        elif k == "ARM":
            arms.append((t, x.ArmState))


def last(series, t):
    v = None
    for s in series:
        if s[0] <= t:
            v = s[1]
        else:
            break
    return v


t0 = min(s[0] for s in sim)
t1 = max(s[0] for s in sim)
print("%-8s %-8s %-8s %-8s %-6s %-4s" % ("t", "truth", "ekf", "rfnd", "mode", "arm"))
t = t0
while t <= t1:
    print("%-8.2f %-8s %-8s %-8s %-6s %-4s" % (
        t - t0,
        "%.3f" % last(sim, t) if last(sim, t) is not None else "-",
        "%.3f" % last(ekf, t) if last(ekf, t) is not None else "-",
        "%.3f" % last(rfnd, t) if last(rfnd, t) is not None else "-",
        last(modes, t), last(arms, t)))
    t += step
