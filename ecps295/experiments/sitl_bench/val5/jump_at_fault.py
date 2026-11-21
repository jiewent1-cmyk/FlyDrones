#!/usr/bin/env python3
"""Per-run: EKF position jump and true speed around the fault moment.

Usage: jump_at_fault.py <rundir> <fault_rel_s> [<dur_s>]
Fault time is relative to AUTO entry, as used by the injection switches.
"""
import json
import math
import os
import sys

from pymavlink import mavutil

rd = sys.argv[1].rstrip("/")
FREL = float(sys.argv[2])
FDUR = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

sim, ekf, modes, arms = [], [], [], []
ld = os.path.join(rd, "logs")
for f in sorted(os.listdir(ld)):
    if not f.endswith(".BIN"):
        continue
    m = mavutil.mavlink_connection(os.path.join(ld, f))
    while True:
        x = m.recv_match(type=["SIM2", "XKF1", "MODE", "ARM"])
        if x is None:
            break
        k = x.get_type()
        t = x.TimeUS / 1e6
        if k == "SIM2":
            sim.append((t, x.PN, x.PE))
        elif k == "XKF1" and x.C == 0:
            ekf.append((t, x.PN, x.PE, x.VN, x.VE))
        elif k == "MODE":
            modes.append((t, x.ModeNum))
        elif k == "ARM":
            arms.append((t, x.ArmState))

t_arm = next((t for t, s in arms if s == 1), sim[0][0] if sim else None)
t_auto = next((t for t, mn in modes if mn == 3 and t_arm is not None and t >= t_arm), None)
if t_auto is None:
    print("no AUTO")
    sys.exit(0)
f0 = t_auto + FREL
f1 = f0 + (FDUR if FDUR else 1.0)
w = [q for q in ekf if f0 - 1.0 <= q[0] <= f1 + 1.0]
jm, jt = 0.0, None
for q0, q1 in zip(w, w[1:]):
    dt = q1[0] - q0[0]
    j = math.hypot(q1[1] - q0[1] - 0.5 * (q0[3] + q1[3]) * dt, q1[2] - q0[2] - 0.5 * (q0[4] + q1[4]) * dt)
    if j > jm:
        jm, jt = j, q1[0]
# true speed in the fault window (0.2 s baseline)
sw = [p for p in sim if f0 <= p[0] <= f1]
spd = 0.0
j = 0
for i, p in enumerate(sw):
    while j < i and sw[j][0] < p[0] - 0.2:
        j += 1
    if j < i and p[0] - sw[j][0] > 0.1:
        spd = max(spd, math.dist(p[1:], sw[j][1:]) / (p[0] - sw[j][0]))
# true displacement in the window
disp = math.dist(sw[-1][1:], sw[0][1:]) if len(sw) > 1 else None
out = {"tag": os.path.basename(rd), "t_auto": round(t_auto, 2), "fault_abs": round(f0, 2),
       "jump_at_fault": round(jm, 3), "jump_t_rel": round(jt - t_auto, 2) if jt else None,
       "true_spd_fault": round(spd, 3), "true_disp_fault": round(disp, 3) if disp is not None else None}
print(json.dumps(out))
