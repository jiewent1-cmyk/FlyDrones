#!/usr/bin/env python3
"""Per-run: truth deviation from the planned square vs time, around the fault moment.

Usage: dev_at_fault.py <rundir> <fault_rel_s> [<dur_s>]
"""
import json
import math
import os
import sys

from pymavlink import mavutil

rd = sys.argv[1].rstrip("/")
FREL = float(sys.argv[2])
FDUR = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
SIDE = 2.0

sim, att, modes, arms = [], [], [], []
ld = os.path.join(rd, "logs")
for f in sorted(os.listdir(ld)):
    if not f.endswith(".BIN"):
        continue
    m = mavutil.mavlink_connection(os.path.join(ld, f))
    while True:
        x = m.recv_match(type=["SIM2", "SIM", "MODE", "ARM"])
        if x is None:
            break
        k = x.get_type()
        t = x.TimeUS / 1e6
        if k == "SIM2":
            sim.append((t, x.PN, x.PE, x.PD))
        elif k == "SIM":
            att.append((t, x.Yaw))
        elif k == "MODE":
            modes.append((t, x.ModeNum))
        elif k == "ARM":
            arms.append((t, x.ArmState))

t_arm = next((t for t, s in arms if s == 1), sim[0][0] if sim else None)
t_auto = next((t for t, mn in modes if mn == 3 and t_arm is not None and t >= t_arm), None)
if t_auto is None or not sim or not att:
    print(json.dumps({"tag": os.path.basename(rd), "error": "no AUTO"}))
    sys.exit(0)


def at(series, t):
    lo, hi = 0, len(series) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    return series[lo]


T0 = at(sim, t_arm)
yaw0 = at(att, t_arm)[1]
psi = math.radians(yaw0)
c, s = math.cos(psi), math.sin(psi)


def fld(p):
    n, e = p[1] - T0[1], p[2] - T0[2]
    return (c * n + s * e, -s * n + c * e)


def seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = dx * dx + dy * dy
    u = 0 if L == 0 else max(0, min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L))
    return math.hypot(p[0] - a[0] - u * dx, p[1] - a[1] - u * dy)


pts = [(0, 0), (SIDE, 0), (SIDE, SIDE), (0, SIDE), (0, 0)]
f0 = t_auto + FREL
f1 = f0 + FDUR
dev = [(p[0], min(seg_dist(fld(p), pts[i], pts[i + 1]) for i in range(4))) for p in sim if t_auto <= p[0] <= t_auto + 120]
mx = max(dev, key=lambda x: x[1])
pre = [d for t, d in dev if t <= f0 - 1]
win = [d for t, d in dev if f0 - 1 <= t <= f1 + 5]
post = [d for t, d in dev if t >= f1 + 5]
print(json.dumps({"tag": os.path.basename(rd), "t_auto": round(t_auto, 2),
                  "dev_max": round(mx[1], 3), "dev_max_rel": round(mx[0] - t_auto, 1),
                  "dev_pre_max": round(max(pre), 3) if pre else None,
                  "dev_fault_window_max": round(max(win), 3) if win else None,
                  "dev_after_max": round(max(post), 3) if post else None,
                  "dev_at_fault": round(min(dev, key=lambda x: abs(x[0] - f0))[1], 3)}))
