#!/usr/bin/env python3
"""(gps_flow copy of ~/sim/mtf01/analyze.py) + planned square rotated by the TRUE yaw at arming (SIM.Yaw),
yaw error (ATT.Yaw - SIM.Yaw), RC8 source-switch events, hover window length from --hover-time.
Metrics of one run against SITL truth (dataflash SIM2 = true NED position, XKF1 core 0 = EKF estimate).

Per armed segment: the LOITER test (if any) and the AUTO mission (the segment that contains CMD 'Mission: 1 Takeoff').
  true AGL       = -(SIM2.PD - SIM2.PD at arming)
  EKF AGL        = -(XKF1.PD - XKF1.PD at arming)
  hgt_est_rms    = RMS(EKF AGL - true AGL) over the airborne part of the mission (true AGL > 0.2 m)
  alt_rms        = RMS(true AGL - mission altitude) from 'Mission: 2 WP' to 'Mission: <n> Land'
  alt_max        = max true AGL in the mission flight
  track_rms/max  = true horizontal distance to the planned square (polyline, true frame at arming), same window
  wp_err         = true distance to each waypoint at 'Reached command #k'
  land_offset    = true horizontal distance of the touchdown point from the arming point
  pos_est_err_end= |EKF NE - true NE| (both relative to arming) at disarm
  hover_*        = LOITER test: during the 15 s hover window (from run.log markers, sim time approx.)
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


ALT, SIDE = arg("--alt", 0.8), arg("--side", 1.0)
HOVER = arg("--hover-alt", 0.7)
logs = sorted(f for f in os.listdir(os.path.join(rd, "logs")) if f.endswith(".BIN")) if os.path.isdir(os.path.join(rd, "logs")) else []
res = {"tag": os.path.basename(rd), "bin": logs}
runlog = open(os.path.join(rd, "run.log")).read() if os.path.exists(os.path.join(rd, "run.log")) else ""
mj = re.search(r"RESULT_JSON (.*)", runlog)
res["script"] = json.loads(mj.group(1)) if mj else None
if not logs:
    res["error"] = "no dataflash log"
    print(json.dumps(res, indent=1))
    sys.exit(0)

sim, ekf, msgs, arms, rfnd = [], [], [], [], []
simyaw, attyaw, rc8, simv = [], [], [], []
for f in logs:
    m = mavutil.mavlink_connection(os.path.join(rd, "logs", f))
    while True:
        x = m.recv_match(type=["SIM2", "XKF1", "MSG", "ARM", "RFND", "SIM", "ATT", "RCIN"])
        if x is None:
            break
        k = x.get_type()
        if k == "SIM2":
            sim.append((x.TimeUS / 1e6, x.PN, x.PE, x.PD))
            simv.append((x.TimeUS / 1e6, math.hypot(x.VN, x.VE), x.VD))
        elif k == "SIM":
            simyaw.append((x.TimeUS / 1e6, x.Yaw))
        elif k == "ATT":
            attyaw.append((x.TimeUS / 1e6, x.Yaw))
        elif k == "RCIN":
            if not rc8 or rc8[-1][1] != x.C8:
                rc8.append((x.TimeUS / 1e6, x.C8))
        elif k == "XKF1" and x.C == 0:
            ekf.append((x.TimeUS / 1e6, x.PN, x.PE, x.PD))
        elif k == "MSG":
            msgs.append((x.TimeUS / 1e6, x.Message))
        elif k == "ARM":
            arms.append((x.TimeUS / 1e6, x.ArmState))
        elif k == "RFND":
            rfnd.append((x.TimeUS / 1e6, x.Dist, x.Stat))


def at(series, t):
    lo, hi = 0, len(series) - 1
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


# segments between arm and disarm
segs, t_arm = [], (sim[0][0] if arms and arms[0][1] == 0 else None)  # the log starts at arming: first ARM may be missing
for t, s in arms:
    if s == 1:
        t_arm = t
    elif t_arm is not None:
        segs.append((t_arm, t))
        t_arm = None
if t_arm is not None:
    segs.append((t_arm, sim[-1][0]))
if not arms and sim:  # log started at arming and the vehicle never disarmed
    segs.append((sim[0][0], sim[-1][0]))
res["armed_segments"] = len(segs)
auto = None
for s0, s1 in segs:
    if any(s0 <= t <= s1 and msg.startswith("Mission: 1") for t, msg in msgs):
        auto = (s0, s1)
loiter = segs[0] if segs and segs[0] != auto else None
res["fc_msgs"] = [msg for t, msg in msgs if not msg.startswith(("ArduCopter", "Param", "RC Prot", "New ", "Frame", "Flight plan", "9fdc", "Mission: ", "Reached"))][:30]

if loiter:
    s0, s1 = loiter
    t0 = at(sim, s0)
    e0 = at(ekf, s0)
    # hover window: last 15 s before the descent begins = find from run.log markers relative to arming
    mk = re.findall(r"\[\s*([\d.]+)\] (FC mode LOITER ARMED|LOITER_HOVER_START|LOITER_HOVER_END)", runlog)
    tt = {}
    for v, k in mk:
        tt.setdefault(k, float(v))
    HT = arg("--hover-time", 15.0)
    if "LOITER_HOVER_START" in tt and "FC mode LOITER ARMED" in tt:
        h0 = s0 + tt["LOITER_HOVER_START"] - tt["FC mode LOITER ARMED"]
        h1 = s0 + tt["LOITER_HOVER_END"] - tt["FC mode LOITER ARMED"]
        w = [p for p in sim if h0 <= p[0] <= h1]
        we = [p for p in ekf if h0 <= p[0] <= h1]
        if w:
            agl = [-(p[3] - t0[3]) for p in w]
            est = [-(at(ekf, p[0])[3] - e0[3]) - (-(p[3] - t0[3])) for p in w]
            res["hover"] = {"true_agl_mean": round(sum(agl) / len(agl), 3), "true_agl_min": round(min(agl), 3), "true_agl_max": round(max(agl), 3),
                            "hgt_est_rms": rms(est), "true_drift_max": round(max(math.hypot(p[1] - w[0][1], p[2] - w[0][2]) for p in w), 3),
                            "est_drift_max": round(max(math.hypot(p[1] - we[0][1], p[2] - we[0][2]) for p in we), 3) if we else None}
    seg = [p for p in sim if s0 <= p[0] <= s1]
    res["loiter_true_alt_max"] = round(max(-(p[3] - t0[3]) for p in seg), 3)
    res["loiter_true_radius_max"] = round(max(math.hypot(p[1] - t0[1], p[2] - t0[2]) for p in seg), 3)

if auto:
    s0, s1 = auto
    t0, e0 = at(sim, s0), at(ekf, s0)
    seg = [p for p in sim if s0 <= p[0] <= s1]
    tw = [t for t, msg in msgs if s0 <= t <= s1 and msg == "Mission: 2 WP"]
    tl = [t for t, msg in msgs if s0 <= t <= s1 and msg.startswith("Mission: ") and msg.endswith("Land")]
    reached = [(t, int(msg.split("#")[1])) for t, msg in msgs if s0 <= t <= s1 and msg.startswith("Reached command #")]
    w0 = tw[0] if tw else None
    w1 = tl[0] if tl else s1
    res["mission_complete"] = bool(tl)
    res["reached"] = [k for _, k in reached]
    res["alt_max"] = round(max(-(p[3] - t0[3]) for p in seg), 3)
    res["radius_max"] = round(max(math.hypot(p[1] - t0[1], p[2] - t0[2]) for p in seg), 3)
    yaw0 = math.radians(at(simyaw, s0)[1]) if simyaw else 0.0
    res["true_yaw_arm_deg"] = round(math.degrees(yaw0), 1)
    # mission placed with the EKF yaw; true yaw at arming is 0 in SITL (home heading 0). EKF yaw error -> error.
    # planned square in the true frame (x ahead = north for yaw 0)
    pts = [(x * math.cos(yaw0) - y * math.sin(yaw0), x * math.sin(yaw0) + y * math.cos(yaw0)) for x, y in [(0, 0), (SIDE, 0), (SIDE, SIDE), (0, SIDE), (0, 0)]]
    if w0:
        win = [p for p in seg if w0 <= p[0] <= w1]
        cross = [min(seg_dist((p[1] - t0[1], p[2] - t0[2]), pts[i], pts[i + 1]) for i in range(4)) for p in win]
        res["track_rms"] = rms(cross)
        res["track_max"] = round(max(cross), 3)
        res["alt_rms"] = rms([-(p[3] - t0[3]) - ALT for p in win])
        res["alt_err_max"] = round(max(abs(-(p[3] - t0[3]) - ALT) for p in win), 3)
        res["alt_mean"] = round(sum(-(p[3] - t0[3]) for p in win) / len(win), 3)
        wps = {2: pts[1], 3: pts[2], 4: pts[3], 5: pts[4]}
        hold_t, hold_w = [], []
        for t, k in reached:
            if k not in wps:
                continue
            # 'Reached command #k' is sent when the hold (--hold s) ends: hold window = last (hold - 1) s before it
            w = [p for p in seg if t - (arg("--hold", 2.0) - 1.0) <= p[0] <= t]
            if w:
                mn = sum(p[1] for p in w) / len(w), sum(p[2] for p in w) / len(w)
                hold_t += [math.hypot(p[1] - t0[1] - wps[k][0], p[2] - t0[2] - wps[k][1]) for p in w]
                hold_w.append(max(math.hypot(p[1] - mn[0], p[2] - mn[1]) for p in w))
        if hold_t:
            res["hold_rms_to_wp"] = rms(hold_t)
            res["hold_max_to_wp"] = round(max(hold_t), 3)
            res["hold_wander_max"] = round(max(hold_w), 3)
        res["wp_err"] = {k: round(math.hypot(at(sim, t)[1] - t0[1] - wps[k][0], at(sim, t)[2] - t0[2] - wps[k][1]), 3) for t, k in reached if k in wps}
    air = [p for p in seg if -(p[3] - t0[3]) > 0.2]
    if air:
        res["hgt_est_rms"] = rms([-(at(ekf, p[0])[3] - e0[3]) + (p[3] - t0[3]) for p in air])
        res["hgt_est_maxabs"] = round(max(abs(-(at(ekf, p[0])[3] - e0[3]) + (p[3] - t0[3])) for p in air), 3)
        res["pos_est_err_max"] = round(max(math.hypot(at(ekf, p[0])[1] - e0[1] - (p[1] - t0[1]), at(ekf, p[0])[2] - e0[2] - (p[2] - t0[2])) for p in air), 3)
    end = seg[-1]
    res["land_offset"] = round(math.hypot(end[1] - t0[1], end[2] - t0[2]), 3)
    ee = at(ekf, end[0])
    res["pos_est_err_end"] = round(math.hypot(ee[1] - e0[1] - (end[1] - t0[1]), ee[2] - e0[2] - (end[2] - t0[2])), 3)
    res["auto_seconds"] = round(s1 - s0, 1)
    res["disarmed_on_ground"] = abs(end[3] - t0[3]) < 0.15


def wrap(d):
    return (d + 180) % 360 - 180


def yaw_stats(s0, s1, key):
    ys = [(t, wrap(at(attyaw, t)[1] - y)) for t, y in simyaw if s0 <= t <= s1]
    if ys:
        e0 = ys[0][1]
        res[key] = {"at_arm": round(e0, 1), "mean": round(sum(e for _, e in ys) / len(ys), 1),
                    "maxabs": round(max(abs(e) for _, e in ys), 1), "max_change_from_arm": round(max(abs(wrap(e - e0)) for _, e in ys), 1)}


if attyaw and simyaw:
    if auto:
        yaw_stats(auto[0], auto[1], "yaw_err_auto")
    if loiter:
        yaw_stats(loiter[0], loiter[1], "yaw_err_loiter")
# source switches (RC8 changes while armed)
sw = []
for t, c in rc8[1:]:
    seg = next(((a0, a1) for a0, a1 in segs if a0 <= t <= a1), None)
    if seg is None:
        continue
    t0s, e0s = at(sim, t), at(ekf, t)

    def errv(tt):
        p, q = at(sim, tt), at(ekf, tt)
        return (q[1] - p[1], q[2] - p[2])
    eb = errv(t - 0.3)
    after = [p for p in sim if t <= p[0] <= min(t + 10, seg[1])]
    jump = max(math.hypot(errv(p[0])[0] - eb[0], errv(p[0])[1] - eb[1]) for p in after if p[0] <= t + 2.0) if after else None
    sv = [v for v in simv if t <= v[0] <= min(t + 10, seg[1])]
    svb = [v for v in simv if t - 5 <= v[0] < t]
    sw.append({"t_rel_arm": round(t - seg[0], 1), "ch8": c, "true_alt": round(-(t0s[3] - at(sim, seg[0])[3]), 2),
               "est_err_before": round(math.hypot(*eb), 3), "ekf_minus_true_jump_2s": round(jump, 3) if jump is not None else None,
               "true_move_10s": round(max(math.hypot(p[1] - t0s[1], p[2] - t0s[2]) for p in after), 3) if after else None,
               "true_hspd_max_10s": round(max(v[1] for v in sv), 2) if sv else None,
               "true_hspd_max_5s_before": round(max(v[1] for v in svb), 2) if svb else None,
               "true_alt_dev_10s": round(max(abs(p[3] - t0s[3]) for p in after), 3) if after else None,
               "msgs_10s": [mm for tm, mm in msgs if t <= tm <= t + 10 and not mm.startswith(("Mission", "Reached"))]})
res["switches"] = sw
print(json.dumps(res, indent=1))
