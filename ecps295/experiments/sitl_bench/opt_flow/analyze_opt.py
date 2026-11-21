#!/usr/bin/env python3
"""analyze_opt.py - truth-based metrics for one run (dataflash SIM2 = true NED, XKF1 core0 = EKF).

hover scenario:
  window  = [HOVER_START, HOVER_END] (sim markers in run.log, mapped through the arming time)
  drift   = horizontal distance of the truth from the truth position at HOVER_START
  rate    = |linear fit of the truth NE displacement over the last 60 s|  (m/s)
  est_err = |EKF NE - truth NE| (both relative to arming)
  height  = truth AGL vs target, and EKF height error
mission scenario:
  window  = "Mission: 2 WP" .. "Mission: n Land"
  track   = distance of the truth to the planned square polyline (true frame at arming)
  wp_err  = truth distance to each reached waypoint
  lap_err = truth distance to the home waypoint at the end of each lap
"""
import json
import math
import os
import re
import sys

from pymavlink import mavutil

rd = sys.argv[1]
cmdtxt = open(os.path.join(rd, "cmd.txt")).read()
args = cmdtxt.split("args=")[1].split()


def arg(name, default):
    return float(args[args.index(name) + 1]) if name in args else default


ALT, SIDE = arg("--alt", 2.0), arg("--side", 2.0)
LAPS = int(arg("--laps", 1))
SCEN = args[args.index("--scenario") + 1] if "--scenario" in args else "hover"
logs = sorted(f for f in os.listdir(os.path.join(rd, "logs")) if f.endswith(".BIN")) if os.path.isdir(os.path.join(rd, "logs")) else []
res = {"tag": os.path.basename(rd.rstrip("/")), "scenario": SCEN, "alt_cmd": ALT, "side": SIDE, "laps": LAPS}
runlog = open(os.path.join(rd, "run.log")).read() if os.path.exists(os.path.join(rd, "run.log")) else ""
mj = re.search(r"RESULT_JSON (.*)", runlog)
res["script"] = json.loads(mj.group(1)) if mj else None
if not logs:
    res["error"] = "no dataflash log"
    print(json.dumps(res, indent=1))
    sys.exit(0)

sim, ekf, msgs, arms = [], [], [], []
for f in logs:
    m = mavutil.mavlink_connection(os.path.join(rd, "logs", f))
    while True:
        x = m.recv_match(type=["SIM2", "XKF1", "MSG", "ARM"])
        if x is None:
            break
        k = x.get_type()
        if k == "SIM2":
            sim.append((x.TimeUS / 1e6, x.PN, x.PE, x.PD))
        elif k == "XKF1" and x.C == 0:
            ekf.append((x.TimeUS / 1e6, x.PN, x.PE, x.PD))
        elif k == "MSG":
            msgs.append((x.TimeUS / 1e6, x.Message))
        elif k == "ARM":
            arms.append((x.TimeUS / 1e6, x.ArmState))


def at(series, t):
    lo, hi = 0, len(series) - 1
    if hi < 0:
        return None
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    return series[lo]


def rms(v):
    return round(math.sqrt(sum(e * e for e in v) / len(v)), 3) if v else None


def seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    u = 0 if L == 0 else max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / L))
    return math.hypot(px - ax - u * dx, py - ay - u * dy)


def fit_slope(ts, vs):
    n = len(ts)
    if n < 3:
        return 0.0
    mt = sum(ts) / n
    mv = sum(vs) / n
    num = sum((t - mt) * (v - mv) for t, v in zip(ts, vs))
    den = sum((t - mt) ** 2 for t in ts)
    return num / den if den else 0.0


segs, t_arm = [], None
for t, s in arms:
    if s == 1:
        t_arm = t
    elif t_arm is not None:
        segs.append((t_arm, t))
        t_arm = None
if t_arm is not None and sim:
    segs.append((t_arm, sim[-1][0]))
if not segs and sim:  # this dataflash build logs no usable ARM message: the log starts at arming
    segs.append((sim[0][0], sim[-1][0]))
res["armed_segments"] = len(segs)

markers = {}
for v, k in re.findall(r"\[\s*([\d.]+)\] (FC mode LOITER ARMED|HOVER_START|HOVER_END)", runlog):
    markers.setdefault(k, float(v))
faults = [(float(v), k) for v, k in re.findall(r"\[\s*([\d.]+)\] FAULT: (\S+) = (\S+)" if False else r"\[\s*([\d.]+)\] FAULT: (\S+)", runlog)]

if not sim:
    res["error"] = "empty log"
    print(json.dumps(res, indent=1))
    sys.exit(0)

if SCEN == "hover":
    s0 = segs[0][0] if segs else sim[0][0]
    s1 = segs[0][1] if segs else sim[-1][0]
    t0, e0 = at(sim, s0), at(ekf, s0)
    if "HOVER_START" in markers and "FC mode LOITER ARMED" in markers:
        off = markers["HOVER_START"] - markers["FC mode LOITER ARMED"]
        h0 = s0 + off
        h1 = h0 + (markers["HOVER_END"] - markers["HOVER_START"])
    else:
        h0, h1 = s0, s1
    res["hover_window_s"] = round(h1 - h0, 1)
    w = [p for p in sim if h0 <= p[0] <= h1]
    we = [p for p in ekf if h0 <= p[0] <= h1]
    if not w:
        res["error"] = "no hover window"
        print(json.dumps(res, indent=1))
        sys.exit(0)
    base = w[0]
    drift = [math.hypot(p[1] - base[1], p[2] - base[2]) for p in w]
    res["true_drift_final"] = round(drift[-1], 3)
    res["true_drift_max"] = round(max(drift), 3)
    res["true_drift_60s"] = round(math.hypot(w[-1][1] - w[0][1], w[-1][2] - w[0][2]), 3)
    # drift rate over the last 60 s (linear fit of the NE displacement)
    tail = [p for p in w if p[0] >= w[-1][0] - 60.0]
    if len(tail) > 5:
        t0t = tail[0][0]
        sn = fit_slope([p[0] - t0t for p in tail], [p[1] - tail[0][1] for p in tail])
        se = fit_slope([p[0] - t0t for p in tail], [p[2] - tail[0][2] for p in tail])
        res["drift_rate_ms"] = round(math.hypot(sn, se), 4)
        res["drift_dir_deg"] = round(math.degrees(math.atan2(se, sn)), 1)
    if we:
        edrift = [math.hypot(p[1] - we[0][1], p[2] - we[0][2]) for p in we]
        res["est_drift_final"] = round(edrift[-1], 3)
        res["est_drift_max"] = round(max(edrift), 3)
    err = []
    for p in w:
        e = at(ekf, p[0])
        err.append(math.hypot(e[1] - e0[1] - (p[1] - t0[1]), e[2] - e0[2] - (p[2] - t0[2])))
    res["pos_est_err_final"] = round(err[-1], 3)
    res["pos_est_err_max"] = round(max(err), 3)
    agl = [-(p[3] - t0[3]) for p in w]
    res["true_agl_mean"] = round(sum(agl) / len(agl), 3)
    res["true_agl_min"] = round(min(agl), 3)
    res["true_agl_max"] = round(max(agl), 3)
    res["alt_err_rms"] = rms([x - ALT for x in agl])
    hest = [-(at(ekf, p[0])[3] - e0[3]) - (-(p[3] - t0[3])) for p in w]
    res["hgt_est_rms"] = rms(hest)
    res["hgt_est_maxabs"] = round(max(abs(x) for x in hest), 3)
    res["true_radius_max"] = round(max(math.hypot(p[1] - t0[1], p[2] - t0[2]) for p in sim if s0 <= p[0] <= s1), 3)
else:
    auto = None
    for s0, s1 in segs:
        if any(s0 <= t <= s1 and msg.startswith("Mission: 1") for t, msg in msgs):
            auto = (s0, s1)
    if auto is None:
        res["error"] = "no AUTO segment"
        print(json.dumps(res, indent=1))
        sys.exit(0)
    s0, s1 = auto
    t0, e0 = at(sim, s0), at(ekf, s0)
    seg = [p for p in sim if s0 <= p[0] <= s1]
    tw = [t for t, msg in msgs if s0 <= t <= s1 and msg == "Mission: 2 WP"]
    tl = [t for t, msg in msgs if s0 <= t <= s1 and msg.startswith("Mission: ") and msg.endswith("Land")]
    reached = [(t, int(msg.split("#")[1])) for t, msg in msgs if s0 <= t <= s1 and msg.startswith("Reached command #")]
    res["mission_complete"] = bool(tl)
    res["reached"] = [k for _, k in reached]
    w0 = tw[0] if tw else s0
    w1 = tl[0] if tl else s1
    win = [p for p in seg if w0 <= p[0] <= w1]
    # the mission was uploaded with the true yaw at arming -> the planned square in the true
    # (north, east) frame is the axis-aligned square rotated by that yaw
    yaw = float(cmdtxt.split("yaw=")[1].split()[0]) if "yaw=" in cmdtxt else 0.0
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    pts = [(x * cy - y * sy, x * sy + y * cy) for (x, y) in
           [(0, 0), (SIDE, 0), (SIDE, SIDE), (0, SIDE), (0, 0)]]
    if win:
        cross = [min(seg_dist((p[1] - t0[1], p[2] - t0[2]), pts[i], pts[i + 1]) for i in range(4)) for p in win]
        res["track_rms"] = rms(cross)
        res["track_max"] = round(max(cross), 3)
        res["alt_rms"] = rms([-(p[3] - t0[3]) - ALT for p in win])
        res["alt_max"] = round(max(-(p[3] - t0[3]) for p in win), 3)
        res["alt_mean"] = round(sum(-(p[3] - t0[3]) for p in win) / len(win), 3)
        # waypoint errors: seq 2.. are the square corners, 4 per lap
        wperr = {}
        laperr = {}
        for t, k in reached:
            if k >= 2 and k <= 1 + 4 * LAPS:
                p = at(sim, t)
                idx = (k - 2) % 4
                cx, cy2 = pts[1 + idx]
                wperr[k] = round(math.hypot(p[1] - t0[1] - cx, p[2] - t0[2] - cy2), 3)
                if idx == 0:
                    laperr[k] = wperr[k]
        res["wp_err"] = wperr
        res["lap_err"] = laperr
    air = [p for p in seg if -(p[3] - t0[3]) > 0.2]
    if air:
        res["hgt_est_rms"] = rms([-(at(ekf, p[0])[3] - e0[3]) + (p[3] - t0[3]) for p in air])
        res["pos_est_err_max"] = round(max(math.hypot(at(ekf, p[0])[1] - e0[1] - (p[1] - t0[1]),
                                                      at(ekf, p[0])[2] - e0[2] - (p[2] - t0[2])) for p in air), 3)
    # use the touchdown point: the FC restarts the mission after landing, so the last log
    # sample is not the landing point
    end = at(sim, min((w1 + 6.0) if w1 else s1, s1))
    res["land_offset"] = round(math.hypot(end[1] - t0[1], end[2] - t0[2]), 3)
    ee = at(ekf, end[0])
    res["pos_est_err_end"] = round(math.hypot(ee[1] - e0[1] - (end[1] - t0[1]), ee[2] - e0[2] - (end[2] - t0[2])), 3)
    res["radius_max"] = round(max(math.hypot(p[1] - t0[1], p[2] - t0[2]) for p in seg), 3)
    res["auto_seconds"] = round(s1 - s0, 1)

# fault window: deviation of the true altitude from the pre-fault value
faultspec = None
for i, a_ in enumerate(args):
    if a_ == "--fault":
        faultspec = args[i + 1]
if faultspec:
    parts = faultspec.split(":")
    ft = float(parts[0])
    fname = parts[1].split("=")[0]
    fval = float(parts[1].split("=")[1])
    fdur = float(parts[2]) if len(parts) > 2 else 0.0
    # the fault clock starts at HOVER_START; map it to the dataflash time line
    ft0 = s0 + (markers.get("HOVER_START", 0.0) - markers.get("FC mode LOITER ARMED", 0.0)) + ft
    pre = [p for p in sim if ft0 - 3.0 <= p[0] < ft0]
    fw = [p for p in sim if ft0 <= p[0] <= ft0 + (fdur if fdur else 5.0)]
    if pre:
        pre_agl = sum(-(p[3] - t0[3]) for p in pre) / len(pre)
        res["pre_fault_agl"] = round(pre_agl, 3)
        if fw:
            devs = [(-(p[3] - t0[3])) - pre_agl for p in fw]
            res["fault_dev_max"] = round(max(devs), 3)
            res["fault_dev_min"] = round(min(devs), 3)
            res["fault_dev_absmax"] = round(max(abs(d) for d in devs), 3)
        res["fault_max_agl"] = round(max(-(p[3] - t0[3]) for p in fw), 3) if fw else None
        res["fault_min_agl"] = round(min(-(p[3] - t0[3]) for p in fw), 3) if fw else None
    res["fault_spec"] = faultspec
    res["final_modes"] = (res.get("script") or {}).get("modes")
    res["final_disarmed"] = (res.get("script") or {}).get("disarmed")
res["fc_msgs"] = [msg for t, msg in msgs if not msg.startswith(("ArduPilot Ready", "Param", "RC Prot", "New ", "Frame",
                                                              "Flight plan", "9fdc", "Mission: ", "Reached", "Arming",
                                                              "Disarming", "SIM Hit", "Barometer", "AHRS", "EKF3", "Field",
                                                              "RC7", "Compass", "GPS", "PreArm", "APM:"))][:25]
print(json.dumps(res, indent=1))
