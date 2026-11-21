#!/usr/bin/env python3
"""Flow-orientation runs: time from arming until the TRUE horizontal distance from the arming point exceeds 0.5/1/3 m,
and the EKF's own estimate of that distance at the same moment (what the pilot/GCS would see)."""
import sys, os, json, math
from pymavlink import mavutil
for rd in sys.argv[1:]:
    f = sorted(x for x in os.listdir(rd + "/logs") if x.endswith(".BIN"))[0]
    m = mavutil.mavlink_connection(rd + "/logs/" + f)
    sim, ekf, t0 = [], [], None
    while True:
        x = m.recv_match(type=["SIM2", "XKF1"])
        if x is None: break
        if x.get_type() == "SIM2": sim.append((x.TimeUS / 1e6, x.PN, x.PE, x.PD))
        elif x.C == 0: ekf.append((x.TimeUS / 1e6, x.PN, x.PE))
    s0, e0 = sim[0], ekf[0]
    out = {}
    for th in (0.5, 1.0, 3.0):
        for p in sim:
            if -(p[3] - s0[3]) > 0.3 and math.hypot(p[1] - s0[1], p[2] - s0[2]) > th:
                e = min(ekf, key=lambda q: abs(q[0] - p[0]))
                out["t_%.1fm" % th] = round(p[0] - s0[0], 1)
                out["est_at_%.1fm" % th] = round(math.hypot(e[1] - e0[1], e[2] - e0[2]), 2)
                break
    print(os.path.basename(rd), json.dumps(out))
