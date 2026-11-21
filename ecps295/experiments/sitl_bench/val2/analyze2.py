#!/usr/bin/env python3
"""Truth-based metrics for opt2_sim.py runs (HOV / SQ missions). Truth = dataflash SIM2 (NED pos) + SIM.Yaw,
EKF = XKF1 core 0. Everything relative to the arming instant of the AUTO flight.

Horizontal positions are rotated into the *initial true body frame* (x = nose at arming, y = right) with the true
SITL yaw at arming, so missions placed with the EKF yaw at arming are comparable for every home heading.

HOV (takeoff -> NAV_LOITER_TIME at the take-off point -> LAND):
  window          = 'Mission: 2' + SETTLE s  ..  min('Mission: 3' Land, first mode change away from AUTO)
  hov_alt_mean/min/max, hov_alt_rms = true AGL (and its RMS error vs --alt) in the window
  hov_hgt_est_rms = RMS(EKF height - true height) in the window
  hov_drift_end/max = true horizontal distance from the position at window start
  hov_rad_max     = max true horizontal distance from the arming point in the window
  hov_pos_est_end = |EKF horizontal displacement - true displacement| (since arming) at window end
SQ (takeoff -> laps x square, side --side -> LAND):
  track_rms/max   = true horizontal distance to the planned square, 'Mission: 2' .. Land
  alt_leg_rms / alt_cnr_rms = RMS(true AGL - --alt) for samples > / < CNR m from the nearest corner
  est_leg_rms / est_cnr_rms = RMS(EKF height - true height) for the same samples
  climb_est_rms   = RMS(EKF - true height), arming .. 'Mission: 2' while true AGL > 0.2
  climb_overshoot = max true AGL in 'Mission: 2' .. +8 s minus --alt
  desc_est_rms    = RMS(EKF - true height) during Land while true AGL > 0.2
Fault (any PARM change of SIM_SONAR_POS_Z after arming):
  f_t0/f_t1 (s after arming), f_true_max/min = true AGL from fault start until LAND/disarm (or end of log),
  f_dev_max = max |true AGL - --alt| in that span, f_est_err_max = max |EKF - true| in that span
Common: alt_max (whole flight), final_mode, mission_complete, disarmed, msgs (EKF/fence/failsafe texts).
"""
import json
import math
import os
import re
import sys

from pymavlink import mavutil

SETTLE = 10.0
CNR = 0.35

rd = sys.argv[1]
cmd = open(os.path.join(rd, "cmd.txt")).read()
args = cmd.split("args=")[1].split()


def arg(name, default):
    return float(args[args.index(name) + 1]) if name in args else default


ALT, SIDE, LAPS = arg("--alt", 0.8), arg("--side", 1.0), int(arg("--laps", 1))
MISSION = args[0]
res = {"tag": os.path.basename(rd), "mission": MISSION}
ld = os.path.join(rd, "logs")
logs = sorted(f for f in os.listdir(ld) if f.endswith(".BIN")) if os.path.isdir(ld) else []
runlog = open(os.path.join(rd, "run.log")).read() if os.path.exists(os.path.join(rd, "run.log")) else ""
mj = re.search(r"RESULT_JSON (.*)", runlog)
res["script"] = json.loads(mj.group(1)) if mj else None
if not logs:
    res["error"] = "no dataflash log"
    print(json.dumps(res, indent=1))
    sys.exit(0)

sim, yaw, ekf, msgs, arms, modes, parms = [], [], [], [], [], [], []
for f in logs:
    m = mavutil.mavlink_connection(os.path.join(ld, f))
    while True:
        x = m.recv_match(type=["SIM2", "SIM", "XKF1", "MSG", "ARM", "MODE", "PARM"])
        if x is None:
            break
        k = x.get_type()
        t = x.TimeUS / 1e6
        if k == "SIM2":
            sim.append((t, x.PN, x.PE, x.PD))
        elif k == "SIM":
            yaw.append((t, x.Yaw))
        elif k == "XKF1" and x.C == 0:
            ekf.append((t, x.PN, x.PE, x.PD, x.Yaw))
        elif k == "MSG":
            msgs.append((t, x.Message))
        elif k == "ARM":
            arms.append((t, x.ArmState))
        elif k == "MODE":
            modes.append((t, x.ModeNum))
        elif k == "PARM" and x.Name == "SIM_SONAR_POS_Z":
            parms.append((t, x.Value))


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
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = dx * dx + dy * dy
    u = 0 if L == 0 else max(0, min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L))
    return math.hypot(p[0] - a[0] - u * dx, p[1] - a[1] - u * dy)


# AUTO flight = armed segment containing 'Mission: 1'
segs, t_arm = [], (sim[0][0] if (not arms or arms[0][1] == 0) else None)  # log starts at arming: first ARM may be missing
for t, s in arms:
    if s == 1:
        t_arm = t
    elif t_arm is not None:
        segs.append((t_arm, t))
        t_arm = None
if t_arm is not None:
    segs.append((t_arm, sim[-1][0]))
seg = None
for s0, s1 in segs:
    if any(s0 <= t <= s1 and msg.startswith("Mission: 1") for t, msg in msgs) and "--no-auto" not in args:
        seg = (s0, s1)
if seg is None and "--no-auto" in args and segs:  # LOITER-only run (pilot RC3 1500, FC altitude hold + surface tracking)
    seg, MISSION, ALT = segs[0], "LOI", arg("--hover-alt", 0.7)
    res["mission"] = "LOI"
if seg is None:
    res["error"] = "no AUTO flight"
    res["msgs"] = [m_ for _, m_ in msgs][-20:]
    print(json.dumps(res, indent=1))
    sys.exit(0)
s0, s1 = seg
T0, E0 = at(sim, s0), at(ekf, s0)
psi = math.radians(at(yaw, s0)[1])
res["true_yaw0_deg"] = round(math.degrees(psi), 1)
c, s = math.cos(psi), math.sin(psi)


def body(p):  # true NE relative to arming -> initial true body frame
    n, e = p[1] - T0[1], p[2] - T0[2]
    return (c * n + s * e, -s * n + c * e)


def agl(p):
    return -(p[3] - T0[3])


def est_err(p):  # EKF height - true height
    return -(at(ekf, p[0])[3] - E0[3]) - agl(p)


psiE = math.radians(E0[4])
cE, sE = math.cos(psiE), math.sin(psiE)
res["ekf_yaw0_deg"] = round(E0[4], 1)


def pos_est_err(p):  # EKF displacement in the EKF initial body frame vs true displacement in the true initial body frame
    q = at(ekf, p[0])
    n, e = q[1] - E0[1], q[2] - E0[2]
    bx, by = body(p)
    return math.hypot(cE * n + sE * e - bx, -sE * n + cE * e - by)


fl = [p for p in sim if s0 <= p[0] <= s1]
res["alt_max"] = round(max(agl(p) for p in fl), 3)
mmsg = [(t, x) for t, x in msgs if s0 <= t <= s1 + 5]
t_m2 = next((t for t, x in mmsg if x.startswith("Mission: 2")), None)
t_land = next((t for t, x in mmsg if x.startswith("Mission: ") and x.endswith("Land")), None)
auto_on = next((t for t, mn in modes if mn == 3 and t >= s0), None)
t_leave = next((t for t, mn in modes if auto_on and t > auto_on and mn != 3), None)
res["mission_complete"] = t_land is not None
res["final_mode"] = [mn for t, mn in modes if t <= s1 + 1][-1] if modes else None
res["left_auto_at"] = round(t_leave - s0, 1) if t_leave else None
res["disarmed"] = any(t >= s0 and st == 0 for t, st in arms)
res["flight_s"] = round(s1 - s0, 1)
hits = [float(re.search(r"([\d.]+) m/s", x).group(1)) for t, x in msgs if s0 <= t <= s1 + 5 and x.startswith("SIM Hit ground")]
res["touchdown_mps"] = max(hits) if hits else None  # SITL 'SIM Hit ground at v m/s' (normal LAND ~0.5)
res["msgs"] = [x for t, x in mmsg if re.search(r"EKF|ence|ailsafe|Crash|lost|Land|variance|reset|switch|yaw", x, re.I)][:12]
w1 = min(x for x in (t_land, t_leave, s1) if x is not None)

if MISSION == "HOV" and t_m2:
    win = [p for p in fl if t_m2 + SETTLE <= p[0] <= w1]
    if win:
        a = [agl(p) for p in win]
        b0 = body(win[0])
        res.update(hov_alt_mean=round(sum(a) / len(a), 3), hov_alt_min=round(min(a), 3), hov_alt_max=round(max(a), 3),
                   hov_alt_rms=rms([v - ALT for v in a]), hov_hgt_est_rms=rms([est_err(p) for p in win]),
                   hov_drift_end=round(math.dist(body(win[-1]), b0), 3),
                   hov_drift_max=round(max(math.dist(body(p), b0) for p in win), 3),
                   hov_rad_max=round(max(math.hypot(*body(p)) for p in win), 3),
                   hov_pos_est_end=round(pos_est_err(win[-1]), 3), hov_window_s=round(win[-1][0] - win[0][0], 1))
if MISSION == "SQ" and t_m2:
    pts = [(0, 0), (SIDE, 0), (SIDE, SIDE), (0, SIDE), (0, 0)]
    win = [p for p in fl if t_m2 <= p[0] <= w1]
    if win:
        bp = [body(p) for p in win]
        cross = [min(seg_dist(q, pts[i], pts[i + 1]) for i in range(4)) for q in bp]
        cn = [min(math.dist(q, k) for k in pts[:4]) < CNR for q in bp]
        res.update(track_rms=rms(cross), track_max=round(max(cross), 3),
                   alt_leg_rms=rms([agl(p) - ALT for p, k in zip(win, cn) if not k]),
                   alt_cnr_rms=rms([agl(p) - ALT for p, k in zip(win, cn) if k]),
                   alt_err_max=round(max(abs(agl(p) - ALT) for p in win), 3),
                   est_leg_rms=rms([est_err(p) for p, k in zip(win, cn) if not k]),
                   est_cnr_rms=rms([est_err(p) for p, k in zip(win, cn) if k]),
                   pos_est_err_max=round(max(pos_est_err(p) for p in win), 3))
    res["climb_est_rms"] = rms([est_err(p) for p in fl if p[0] < t_m2 and agl(p) > 0.2])
    res["climb_overshoot"] = round(max(agl(p) for p in fl if t_m2 <= p[0] <= t_m2 + 8) - ALT, 3)
    if t_land:
        res["desc_est_rms"] = rms([est_err(p) for p in fl if p[0] >= t_land and agl(p) > 0.2])
end = fl[-1]
res["land_offset"] = round(math.hypot(*body(end)), 3)
res["pos_est_err_end"] = round(pos_est_err(end), 3)

fp = [(t, v) for t, v in parms if t > s0]
if fp:
    f0 = fp[0][0]
    f1 = next((t for t, v in fp if abs(v - fp[0][1]) > 1e-3), None)  # PARM can be logged twice for one change
    stop = min(x for x in (t_leave if t_leave and t_leave > f0 else None, t_land if t_land and t_land > f0 else None, s1) if x is not None)
    span = [p for p in fl if f0 <= p[0] <= stop]
    res.update(f_val=fp[0][1], f_t0=round(f0 - s0, 1), f_dur=round(f1 - f0, 1) if f1 else None, f_alt0=round(agl(at(sim, f0)), 3))
    if span:
        a = [agl(p) for p in span]
        res.update(f_true_max=round(max(a), 3), f_true_min=round(min(a), 3), f_dev_max=round(max(abs(v - ALT) for v in a), 3),
                   f_est_err_max=round(max(abs(est_err(p)) for p in span), 3), f_stop_after=round(stop - f0, 1))
    w15 = [agl(p) for p in fl if f0 <= p[0] <= f0 + 15]  # fixed 15 s window after fault start (used for LOITER runs)
    res.update(f_w15_min=round(min(w15), 3), f_w15_max=round(max(w15), 3))
    res["f_after_max"] = round(max(agl(p) for p in fl if p[0] >= f0), 3)
    res["f_after_min_air"] = round(min(agl(p) for p in fl if f0 <= p[0] <= (t_leave or s1)), 3)
print(json.dumps(res, indent=1))
