"""Identify first-order velocity time constants in SITL (TechRoute v2.2 §5.5) for L1 SimDrone.tau."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time

import numpy as np
import sitl_util as su
from scipy.optimize import curve_fit

ap = argparse.ArgumentParser()
ap.add_argument("--url", default="tcp:127.0.0.1:5760")
ap.add_argument("--alt", type=float, default=1.0)
ap.add_argument("--vxy", type=float, default=0.3)
ap.add_argument("--vz", type=float, default=0.3)
ap.add_argument("--yaw-dps", type=float, default=30)
ap.add_argument("--hold", type=float, default=3.0)
ap.add_argument("--out", default="sysid")
ap.add_argument("--analyze", help="skip flying; analyze an existing CSV")
a = ap.parse_args()

# (axis, command sequence of (value, seconds)); z positive = up
H = a.hold
PLAN = [
    ("x", [(0, 2), (a.vxy, H), (0, H), (-a.vxy, H), (0, H)]),
    ("z", [(0, 2), (a.vz, 2), (0, H), (-a.vz, 2), (0, H)]),
    ("yaw", [(0, 2), (a.yaw_dps, H), (0, H), (-a.yaw_dps, H), (0, H)]),
]

if a.analyze:
    with open(a.analyze) as f:
        rd = csv.reader(f)
        next(rd)
        rows = [(float(r[0]), r[1], float(r[2]), float(r[3]), float(r[4]), float(r[5]), float(r[6])) for r in rd]
else:
    m = su.connect(a.url)
    print(f"EKF ready after {su.wait_ekf(m):.0f}s", flush=True)
    su.arm_takeoff(m, a.alt)
    print(f"steady after {su.wait_steady(m, a.alt):.1f}s", flush=True)
    print("airborne", flush=True)

    rows = []  # t, axis, cmd, x_body_v, z_up_v, yaw_rate_dps, alt
    state = {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0, "yr": 0.0, "alt": 0.0}
    t0 = time.time()
    for axis, seq in PLAN:
        for val, dur in seq:
            t_end = time.time() + dur
            next_send = 0.0
            while time.time() < t_end:
                now = time.time()
                if now >= next_send:
                    if axis == "x":
                        su.send_vel(m, val, 0, 0, 0)
                    elif axis == "z":
                        su.send_vel(m, 0, 0, -val, 0)
                    else:
                        su.send_vel(m, 0, 0, 0, math.radians(val))
                    next_send = now + 0.05
                msg = m.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=True, timeout=0.02)
                if msg is None:
                    continue
                if msg.get_type() == "LOCAL_POSITION_NED":
                    state.update(vx=msg.vx, vy=msg.vy, vz=msg.vz, alt=-msg.z)
                    c, s = math.cos(state["yaw"]), math.sin(state["yaw"])
                    vbx = c * state["vx"] + s * state["vy"]
                    rows.append((time.time() - t0, axis, val, vbx, -state["vz"], math.degrees(state["yr"]), state["alt"]))
                else:
                    state.update(yaw=msg.yaw, yr=msg.yawspeed)
        print(f"axis {axis} done", flush=True)
    print("landed:", su.land(m), flush=True)

    with open(f"{a.out}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "axis", "cmd", "vbx", "vup", "yaw_rate_dps", "alt"])
        w.writerows(rows)


def fo(t, tau):
    return 1 - np.exp(-t / tau)


def fopdt(t, tau, td):
    return np.where(t > td, 1 - np.exp(-(t - td) / tau), 0.0)


col = {"x": 3, "z": 4, "yaw": 5}
res = {}
for axis, _ in PLAN:
    r = [row for row in rows if row[1] == axis]
    t = np.array([row[0] for row in r])
    cmd = np.array([row[2] for row in r])
    y = np.array([row[col[axis]] for row in r])
    edges = np.flatnonzero(np.diff(cmd) != 0) + 1
    fits = []
    for i, e in enumerate(edges):
        stop = edges[i + 1] if i + 1 < len(edges) else len(t)
        y0, y1 = y[max(e - 1, 0)], cmd[e]
        if abs(y1 - y0) < 1e-6 or abs(cmd[e] - cmd[e - 1]) < 1e-6:
            continue
        tt = t[e:stop] - t[e]
        yy = (y[e:stop] - y0) / (y1 - y0)
        p1, _ = curve_fit(fo, tt, yy, p0=[0.3], bounds=([0.01], [5]))
        p2, _ = curve_fit(fopdt, tt, yy, p0=[0.3, 0.05], bounds=([0.01, 0], [5, 1]))
        rms1 = float(np.sqrt(np.mean((fo(tt, *p1) - yy) ** 2)))
        rms2 = float(np.sqrt(np.mean((fopdt(tt, *p2) - yy) ** 2)))
        fits.append(
            {
                "from": float(cmd[e - 1]),
                "to": float(cmd[e]),
                "tau": float(p1[0]),
                "rms": rms1,
                "tau_pd": float(p2[0]),
                "td": float(p2[1]),
                "rms_pd": rms2,
            }
        )
    res[axis] = {
        "steps": fits,
        "tau_median": float(np.median([f["tau"] for f in fits])),
        "tau_pd_median": float(np.median([f["tau_pd"] for f in fits])),
        "td_median": float(np.median([f["td"] for f in fits])),
    }
    print(
        f"{axis:>3}: tau={res[axis]['tau_median']:.3f}s  (FOPDT tau={res[axis]['tau_pd_median']:.3f}s td={res[axis]['td_median']:.3f}s)"
    )
    for f in fits:
        print(
            f"      {f['from']:+.2f}->{f['to']:+.2f}  tau={f['tau']:.3f} rms={f['rms']:.3f} | tau_pd={f['tau_pd']:.3f} td={f['td']:.3f} rms={f['rms_pd']:.3f}"
        )
json.dump(res, open(f"{a.out}.json", "w"), indent=1)
