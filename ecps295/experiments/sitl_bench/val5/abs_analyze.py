#!/usr/bin/env python3
"""Truth-based metrics for abs_sim.py runs. Truth = dataflash SIM2 (NED) + SIM (Yaw/Roll/Pitch); EKF = XKF1 core 0.

Field frame (= mission frame): origin = true position at arming, x axis = true yaw at arming + --field-rot.
The EKF frame is used raw (EKF yaw is 0 at boot without compass, so EKF north = the field x axis when --field-rot 0);
with --ekf-frame ned (compass) the EKF NE is rotated into the field frame with the true field azimuth.

HOV/HOVW (target = field origin), window = 'Mission: 2' + 10 s .. min(Land, leave AUTO, disarm):
  dev_max / dev_rms / dev_end   true distance from the target point
  drift_max                     true distance from the window-start position (as opt_flow2 hov_drift_max)
SQ (laps x square side --side, target track in the field frame), window = 'Mission: 2' .. Land:
  track_rms / track_max         true distance to the planned square
  lap1/lap2/lap3                true distance from the take-off corner at 'Reached command #5/#9/#13'
Common:
  land_offset                   true distance from the field origin at the end of the log (on the ground)
  est_err_max / est_err_end     |EKF - truth| horizontal (window)
  est_jump_max                  largest EKF position step between XKF1 samples not explained by EKF velocity (m)
  spd_max / tilt_max            true horizontal speed (0.2 s differences) / true max(|roll|,|pitch|) deg, window
  corr_n, corr_spd_max, corr_move_max   correction events from comp.csv (EXT POS 'reset', or the first 'sent' after > 2 s
                                without fixes); true speed max and true displacement within 4 s after each event
  yaw_err_end / yaw_err_max     EKF yaw - true yaw in the field frame (deg), window
  alt_min / alt_max             true AGL in the window
"""
import csv
import json
import math
import os
import re
import sys

from pymavlink import mavutil

SETTLE = 10.0
rd = sys.argv[1].rstrip("/")
cmd = open(os.path.join(rd, "cmd.txt")).read()
args = cmd.split("args=")[1].split()


def arg(name, default):
    return float(args[args.index(name) + 1]) if name in args else default


ALT, SIDE, LAPS, ROT = arg("--alt", 2.0), arg("--side", 2.0), int(arg("--laps", 3)), arg("--field-rot", 0.0)
MISSION = args[0]
res = {"tag": os.path.basename(rd), "mission": MISSION}
ld = os.path.join(rd, "logs")
logs = sorted(f for f in os.listdir(ld) if f.endswith(".BIN")) if os.path.isdir(ld) else []
runlog = open(os.path.join(rd, "run.log")).read() if os.path.exists(os.path.join(rd, "run.log")) else ""
mj = re.search(r"RESULT_JSON (.*)", runlog)
res["script"] = json.loads(mj.group(1)) if mj else None
_sc = res["script"] or {}
res["fixes_sent"] = _sc.get("fixes_sent")
res["gate_innov_rej"] = _sc.get("gate_innov_rej")
res["inj"] = json.dumps(_sc.get("inj", {}))
res["auto_armed"] = _sc.get("auto_armed")
if not logs:
    res["error"] = "no dataflash log"
    print(json.dumps(res, indent=1))
    sys.exit(0)

sim, att, ekf, msgs, arms, modes = [], [], [], [], [], []
for f in logs:
    m = mavutil.mavlink_connection(os.path.join(ld, f))
    while True:
        x = m.recv_match(type=["SIM2", "SIM", "XKF1", "MSG", "ARM", "MODE"])
        if x is None:
            break
        k = x.get_type()
        t = x.TimeUS / 1e6
        if k == "SIM2":
            sim.append((t, x.PN, x.PE, x.PD))
        elif k == "SIM":
            att.append((t, x.Yaw, x.Roll, x.Pitch))
        elif k == "XKF1" and x.C == 0:
            ekf.append((t, x.PN, x.PE, x.PD, x.Yaw, x.VN, x.VE))
        elif k == "MSG":
            msgs.append((t, x.Message))
        elif k == "ARM":
            arms.append((t, x.ArmState))
        elif k == "MODE":
            modes.append((t, x.ModeNum))


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


def r3(v):
    return round(v, 3) if v is not None else None


def seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = dx * dx + dy * dy
    u = 0 if L == 0 else max(0, min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L))
    return math.hypot(p[0] - a[0] - u * dx, p[1] - a[1] - u * dy)


def wrap180(d):
    return (d + 180) % 360 - 180


segs, t_arm = [], (sim[0][0] if (not arms or arms[0][1] == 0) else None)
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
    if any(s0 <= t <= s1 and msg.startswith("Mission: 1") for t, msg in msgs):
        seg = (s0, s1)
if seg is None:
    res["error"] = "no AUTO flight"
    res["msgs"] = [m_ for _, m_ in msgs][-20:]
    print(json.dumps(res, indent=1))
    sys.exit(0)
s0, s1 = seg
T0 = at(sim, s0)
yaw0 = at(att, s0)[1]
psi = math.radians(yaw0 + ROT)
res["true_yaw0_deg"] = round(yaw0, 1)
c, s = math.cos(psi), math.sin(psi)


def fld(p):  # truth NE -> field frame
    n, e = p[1] - T0[1], p[2] - T0[2]
    return (c * n + s * e, -s * n + c * e)


def agl(p):
    return -(p[3] - T0[3])


NED = "--ekf-frame" in args and args[args.index("--ekf-frame") + 1] == "ned"


def ekf_ne(t):  # EKF NE -> field frame (identity unless the EKF frame is NED, i.e. compass yaw)
    q = at(ekf, t)
    if NED:
        return (c * q[1] + s * q[2], -s * q[1] + c * q[2])
    return (q[1], q[2])


def est_err(p):
    return math.dist(ekf_ne(p[0]), fld(p))


def yaw_err(t):
    if NED:
        return wrap180(at(ekf, t)[4] - at(att, t)[1])
    return wrap180(at(ekf, t)[4] - (at(att, t)[1] - yaw0 - ROT))


fl = [p for p in sim if s0 <= p[0] <= s1]
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
res["msgs"] = [x for t, x in mmsg if re.search(r"EKF|ence|ailsafe|Crash|lost|variance|reset|switch|yaw|VisOdom|aiding|external", x, re.I)][:14]
res["msgs_all"] = [x for t, x in mmsg if re.search(r"EKF|ence|ailsafe|Crash|lost|variance|reset|switch|yaw|VisOdom|aiding|external", x, re.I)]
w1 = min(x for x in (t_land, t_leave, s1) if x is not None)
win = [p for p in fl if t_m2 is not None and t_m2 + (SETTLE if MISSION.startswith("HOV") else 0) <= p[0] <= w1]
if not win:
    res["error"] = "empty window"
    print(json.dumps(res, indent=1))
    sys.exit(0)
fw = [fld(p) for p in win]
if MISSION.startswith("HOV"):
    d = [math.hypot(*q) for q in fw]
    res.update(dev_max=r3(max(d)), dev_rms=rms(d), dev_end=r3(d[-1]), drift_max=r3(max(math.dist(q, fw[0]) for q in fw)))
else:
    pts = [(0, 0), (SIDE, 0), (SIDE, SIDE), (0, SIDE), (0, 0)]
    cross = [min(seg_dist(q, pts[i], pts[i + 1]) for i in range(4)) for q in fw]
    res.update(track_rms=rms(cross), track_max=r3(max(cross)))
    for k in range(LAPS):
        idx = 2 + 4 * k + 3
        tr = next((t for t, x in mmsg if x.strip() == "Reached command #%d" % idx), None)
        res["lap%d" % (k + 1)] = r3(math.hypot(*fld(at(sim, tr)))) if tr else None
a_ = [agl(p) for p in win]
res.update(alt_min=r3(min(a_)), alt_max=r3(max(a_)))
ee = [est_err(p) for p in win]
res.update(est_err_max=r3(max(ee)), est_err_end=r3(ee[-1]))
ye = [yaw_err(p[0]) for p in win]
res.update(yaw_err_end=round(ye[-1], 1), yaw_err_max=round(max(ye, key=abs), 1))
end = fl[-1]
res["land_offset"] = r3(math.hypot(*fld(end)))
# EKF jumps (position step not explained by the EKF velocity)
ek = [q for q in ekf if win[0][0] <= q[0] <= win[-1][0]]
jm, tj = 0.0, None
for q0, q1 in zip(ek, ek[1:]):
    dt = q1[0] - q0[0]
    j = math.hypot(q1[1] - q0[1] - 0.5 * (q0[5] + q1[5]) * dt, q1[2] - q0[2] - 0.5 * (q0[6] + q1[6]) * dt)
    if j > jm:
        jm, tj = j, q1[0]
res["est_jump_max"] = r3(jm)
res["est_jump_t"] = round(tj - s0, 1) if tj else None


def spd_series(pts):
    out = []
    j = 0
    for i, p in enumerate(pts):
        while j < i and pts[j][0] < p[0] - 0.2:
            j += 1
        if j < i and p[0] - pts[j][0] > 0.1:
            a1, b1 = fld(pts[j]), fld(p)
            out.append((p[0], math.dist(a1, b1) / (p[0] - pts[j][0])))
    return out


sp = spd_series(win)
res["spd_max"] = r3(max(v for _, v in sp)) if sp else None
aw = [q for q in att if win[0][0] <= q[0] <= win[-1][0]]
res["tilt_max"] = round(max(max(abs(q[2]), abs(q[3])) for q in aw), 1) if aw else None
# correction events from the companion log
ev = []
cp = os.path.join(rd, "comp.csv")
if os.path.exists(cp):
    rows = list(csv.DictReader(open(cp)))
    last_sent = -1e9
    for r in rows:
        t = float(r["t"])
        if r["ev"] == "reset":
            ev.append(t)
        elif r["ev"] == "sent":
            if t - last_sent > 2.0:
                ev.append(t)
            last_sent = t
    res["n_obs"] = sum(1 for r in rows if r["ev"].startswith("obs"))
    res["n_outlier"] = sum(1 for r in rows if r["ev"] == "obs_outlier")
    res["n_obs_bad"] = sum(1 for r in rows if r["ev"] == "obs_bad")
    res["n_obs_badyaw"] = sum(1 for r in rows if r["ev"] == "obs_badyaw")
    res["n_block_rows"] = sum(1 for r in rows if r["ev"] == "block")
    res["n_block_vis"] = sum(1 for r in rows if r["ev"] == "block" and r["mx"])
    res["n_sent"] = sum(1 for r in rows if r["ev"] == "sent")
    res["n_gated"] = sum(1 for r in rows if r["ev"] == "gated")
res["corr_n"] = len(ev)
cs, cm = [], []
for t in ev:
    if not (s0 <= t <= s1):
        continue
    seg_ = [p for p in fl if t <= p[0] <= t + 4.0]
    if len(seg_) > 5:
        ss = spd_series(seg_)
        if ss:
            cs.append(max(v for _, v in ss))
        cm.append(max(math.dist(fld(seg_[0]), fld(p)) for p in seg_))
res["corr_spd_max"] = r3(max(cs)) if cs else None
res["corr_move_max"] = r3(max(cm)) if cm else None
print(json.dumps(res, indent=1))
