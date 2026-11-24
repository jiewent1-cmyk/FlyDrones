#!/usr/bin/env python3
"""Metrics of one hover run against SITL truth (dataflash SIM2 / SIM), window = HOVER_START..HOVER_END (FC time_boot).
  exc_*    true horizontal distance from the true position at the start of the hold: max over the first T s, rms, end
  ekf_*    |EKF displacement - true displacement| (both from the start of the hold)
  yaw_*    ATT.Yaw - SIM.Yaw: value at start, max change during the hold
"""
import json, math, os, re, sys
from pymavlink import mavutil
rd = sys.argv[1]
res = {"tag": os.path.basename(rd)}
log = open(rd + "/run.log").read() if os.path.exists(rd + "/run.log") else ""
mj = re.search(r"RESULT_JSON (.*)", log)
res["script"] = sc = json.loads(mj.group(1)) if mj else None
bins = sorted(f for f in os.listdir(rd + "/logs") if f.endswith(".BIN")) if os.path.isdir(rd + "/logs") else []
if not sc or "t0" not in sc or not bins:
    res["error"] = "no hover or no log"
    print(json.dumps(res)); sys.exit(0)
t0, t1 = sc["t0"], sc["t1"]
sim, ekf, sy, ay, msgs = [], [], [], [], []
m = mavutil.mavlink_connection(rd + "/logs/" + bins[-1])
while True:
    x = m.recv_match(type=["SIM2", "XKF1", "SIM", "ATT", "MSG"])
    if x is None:
        break
    k, t = x.get_type(), x.TimeUS / 1e6
    if k == "MSG":
        msgs.append((round(t - t0, 1), x.Message))
    elif t0 - 1 <= t <= t1 + 60:
        if k == "SIM2":
            sim.append((t, x.PN, x.PE, x.PD, math.hypot(x.VN, x.VE)))
        elif k == "XKF1" and x.C == 0:
            ekf.append((t, x.PN, x.PE))
        elif k == "SIM":
            sy.append((t, x.Yaw))
        elif k == "ATT":
            ay.append((t, x.Yaw))


def at(s, t):
    return min(s, key=lambda r: abs(r[0] - t))


s0, e0 = at(sim, t0), at(ekf, t0)
H = [r for r in sim if t0 <= r[0] <= t1]
exc = [(r[0] - t0, math.hypot(r[1] - s0[1], r[2] - s0[2])) for r in H]
res["hover_s"] = round(t1 - t0, 1)
res["complete"] = bool(sc.get("end_mode") == "GUIDED" and sc.get("hover_s", 0) >= 0.98 * (H[-1][0] - H[0][0]) and sc.get("disarmed"))
for T in (60, 120, 180, 240, 300, 600, 900):
    v = [d for t, d in exc if t <= T]
    if exc[-1][0] >= T - 2:
        res["exc_max_%d" % T] = round(max(v), 3)
        res["exc_at_%d" % T] = round(v[-1], 3)
res["exc_max"] = round(max(d for _, d in exc), 3)
res["exc_rms"] = round(math.sqrt(sum(d * d for _, d in exc) / len(exc)), 3)
res["exc_end"] = round(exc[-1][1], 3)
res["spd_max"] = round(max(r[4] for r in H), 3)
res["alt_min"], res["alt_max"] = round(min(-(r[3] - s0[3]) for r in H), 3), round(max(-(r[3] - s0[3]) for r in H), 3)
E = [r for r in ekf if t0 <= r[0] <= t1]
err, j, jump = [], 0, 0.0
for i, r in enumerate(E):
    while j < len(H) - 1 and H[j][0] < r[0]:
        j += 1
    err.append(math.hypot((r[1] - e0[1]) - (H[j][1] - s0[1]), (r[2] - e0[2]) - (H[j][2] - s0[2])))
    if i:
        jump = max(jump, math.hypot(r[1] - E[i - 1][1], r[2] - E[i - 1][2]))
res["ekf_err_max"], res["ekf_err_end"], res["ekf_step_max"] = round(max(err), 3), round(err[-1], 3), round(jump, 3)


def wrap(d):
    return (d + 180) % 360 - 180


yy = [(t, wrap(v - at(sy, t)[1])) for t, v in ay[::5] if t0 <= t <= t1]
res["yaw_err0"] = round(yy[0][1], 1)
res["yaw_chg_max"] = round(max(abs(wrap(v - yy[0][1])) for _, v in yy), 1)
land = [r for r in sim if r[0] > t1]
res["land_offset"] = round(math.hypot(land[-1][1] - s0[1], land[-1][2] - s0[2]), 3) if land else None
bad = ("variance", "Failsafe", "failsafe", "Glitch", "glitch", "lane", "reset", "stopped aiding", "fence", "Fence", "Crash", "anomaly", "Hit ground")
res["events"] = [(t, s) for t, s in msgs if t > -5 and any(b in s for b in bad)][:20]
res["rtf"] = round(sc["sim_s"] / sc["wall_s"], 2)
print(json.dumps(res))
