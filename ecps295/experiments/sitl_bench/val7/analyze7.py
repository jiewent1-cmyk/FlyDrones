#!/usr/bin/env python3
"""V6 metrics: analyze2.py + range-finder / arming / liftoff / landing-window details.
Adds:
  rfnd_stat_set, rfnd_stat_gnd, rfnd_stat_air, rfnd_dist_gnd_min/max, rfnd_dist_air_min/max
  liftoff_t, liftoff_err, liftoff_err_max
  desc_low_err_max  (true AGL in (0.05,0.55] during final descent)
  n_variance        (EKF 'variance' messages in the flight segment)
  arm_texts         (arming related STATUSTEXT from run.log)
  fault-window RFND distance/stat range (f_rfnd_min/max, f_rfnd_stats)
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

sim, yaw, ekf, msgs, arms, modes, parms, rfnd = [], [], [], [], [], [], [], []
for f in logs:
    m = mavutil.mavlink_connection(os.path.join(ld, f))
    while True:
        x = m.recv_match(type=["SIM2", "SIM", "XKF1", "MSG", "ARM", "MODE", "PARM", "RFND"])
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
        elif k == "RFND":
            rfnd.append((t, x.Dist, x.Stat))


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
    if any(s0 <= t <= s1 and msg.startswith("Mission: 1") for t, msg in msgs) and "--no-auto" not in args:
        seg = (s0, s1)
if seg is None and "--no-auto" in args and segs:
    seg, MISSION, ALT = segs[0], "LOI", arg("--hover-alt", 0.7)
    res["mission"] = "LOI"
if seg is None:
    res["error"] = "no AUTO flight"
    res["msgs"] = [m_ for _, m_ in msgs][-20:]
    print(json.dumps(res, indent=1))
    sys.exit(0)
s0, s1 = seg
res["s0"] = round(s0, 3)
res["s1"] = round(s1, 3)
_m = re.search(r"FB_TB0 ([\d.]+)", runlog)
res["fb_tb0"] = float(_m.group(1)) if _m else None
T0, E0 = at(sim, s0), at(ekf, s0)
psi = math.radians(at(yaw, s0)[1])
res["true_yaw0_deg"] = round(math.degrees(psi), 1)
c, s = math.cos(psi), math.sin(psi)


def body(p):
    n, e = p[1] - T0[1], p[2] - T0[2]
    return (c * n + s * e, -s * n + c * e)


def agl(p):
    return -(p[3] - T0[3])


def est_err(p):
    return -(at(ekf, p[0])[3] - E0[3]) - agl(p)


psiE = math.radians(E0[4])
cE, sE = math.cos(psiE), math.sin(psiE)
res["ekf_yaw0_deg"] = round(E0[4], 1)


def pos_est_err(p):
    q = at(ekf, p[0])
    n, e = q[1] - E0[1], q[2] - E0[2]
    bx, by = body(p)
    return math.hypot(cE * n + sE * e - bx, -sE * n + cE * e - by)


fl = [p for p in sim if s0 <= p[0] <= s1]
res["alt_max"] = round(max(agl(p) for p in fl), 3)
res["truth_max_all"] = round(max(agl(p) for p in fl), 3)
res["truth_min_all"] = round(min(agl(p) for p in fl), 3)
res["truth_dev_max"] = round(max(abs(agl(p) - ALT) for p in fl), 3)
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
res["touchdown_mps"] = max(hits) if hits else None
res["msgs"] = [x for t, x in mmsg if re.search(r"EKF|ence|ailsafe|Crash|lost|Land|variance|reset|switch|yaw", x, re.I)][:12]
res["n_variance"] = sum(1 for t, x in msgs if "variance" in (x or "").lower())
w1 = min(x for x in (t_land, t_leave, s1) if x is not None)

# --- arming texts (from run.log) ---
arm_texts = [t.split("FC text: ", 1)[1] for t in runlog.splitlines() if "FC text: " in t
             and re.search(r"[Aa]rm|Pre-arm|reject|denied|check", t)]
res["arm_texts"] = arm_texts[-8:]
res["armed_ok"] = bool(res.get("script") and res["script"].get("auto_armed")) if res.get("script") else None
if res.get("script"):
    res["loiter_armed"] = res["script"].get("loiter_armed")
    res["auto_armed"] = res["script"].get("auto_armed")

# --- range finder status / readings ---
rf_in = [r for r in rfnd if s0 <= r[0] <= s1 + 2]
res["rfnd_stat_set"] = sorted(set(r[2] for r in rf_in))
res["rfnd_n"] = len(rf_in)
gnd = [r for r in rf_in if agl(at(sim, r[0])) < 0.15]
air = [r for r in rf_in if 1.8 <= agl(at(sim, r[0])) <= 2.2]
res["rfnd_stat_gnd"] = sorted(set(r[2] for r in gnd))
res["rfnd_stat_air"] = sorted(set(r[2] for r in air))
res["rfnd_dist_gnd_min"] = round(min((r[1] for r in gnd), default=0), 3)
res["rfnd_dist_gnd_max"] = round(max((r[1] for r in gnd), default=0), 3)
res["rfnd_dist_air_min"] = round(min((r[1] for r in air), default=0), 3)
res["rfnd_dist_air_max"] = round(max((r[1] for r in air), default=0), 3)

# --- liftoff ---
lo = next((p for p in fl if agl(p) > 0.1), None)
if lo:
    res["liftoff_t"] = round(lo[0] - s0, 1)
    res["liftoff_err"] = round(est_err(lo), 3)
    win = [p for p in fl if lo[0] - 1.5 <= p[0] <= lo[0] + 1.5]
    res["liftoff_err_max"] = round(max(abs(est_err(p)) for p in win), 3) if win else None
# --- final descent below 0.55 m ---
t_above = [p[0] for p in fl if agl(p) > 0.55]
t_d = max(t_above) if t_above else s0
low = [p for p in fl if p[0] >= t_d and 0.05 < agl(p) <= 0.55]
res["desc_low_err_max"] = round(max((abs(est_err(p)) for p in low), default=0), 3)
res["desc_low_n"] = len(low)

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
    f1 = next((t for t, v in fp if abs(v - fp[0][1]) > 1e-3), None)
    stop = min(x for x in (t_leave if t_leave and t_leave > f0 else None, t_land if t_land and t_land > f0 else None, s1) if x is not None)
    span = [p for p in fl if f0 <= p[0] <= stop]
    res.update(f_val=fp[0][1], f_t0=round(f0 - s0, 1), f_dur=round(f1 - f0, 1) if f1 else None, f_alt0=round(agl(at(sim, f0)), 3))
    if span:
        a = [agl(p) for p in span]
        res.update(f_true_max=round(max(a), 3), f_true_min=round(min(a), 3), f_dev_max=round(max(abs(v - ALT) for v in a), 3),
                   f_est_err_max=round(max(abs(est_err(p)) for p in span), 3), f_stop_after=round(stop - f0, 1))
    w15 = [agl(p) for p in fl if f0 <= p[0] <= f0 + 15]
    res.update(f_w15_min=round(min(w15), 3), f_w15_max=round(max(w15), 3))
    res["f_after_max"] = round(max(agl(p) for p in fl if p[0] >= f0), 3)
    res["f_after_min_air"] = round(min(agl(p) for p in fl if f0 <= p[0] <= (t_leave or s1)), 3)
    # range finder readings inside the injection window
    fend = f1 if f1 else stop
    rw = [r for r in rfnd if f0 <= r[0] <= fend]
    if rw:
        res["f_rfnd_min"] = round(min(r[1] for r in rw), 3)
        res["f_rfnd_max"] = round(max(r[1] for r in rw), 3)
        res["f_rfnd_stats"] = sorted(set(r[2] for r in rw))
        res["f_rfnd_n"] = len(rw)

# --- 监视器（V7）指标 ---
import csv as _csv

mpath = os.path.join(rd, "monitor.csv")
if os.path.exists(mpath):
    mrows = list(_csv.DictReader(open(mpath)))
    res["mon_n_frames"] = len(mrows)
    if mrows:
        fl2 = lambda r, k: float(r[k]) if r.get(k) not in ("", None) else None
        trig = [r for r in mrows if r.get("trigger") == "1"]
        en = [r for r in mrows if r.get("enabled") == "1"]
        res["mon_reason"] = trig[0].get("reason") if trig else None
        res["mon_trigger_t"] = round(fl2(trig[0], "t"), 3) if trig else None
        res["mon_trigger_rel"] = round(fl2(trig[0], "t") - s0, 2) if trig else None
        res["mon_trig_A"] = bool(trig[0].get("trig_A") == "1") if trig else False
        res["mon_trig_B"] = bool(trig[0].get("trig_B") == "1") if trig else False
        if en:
            ev = [fl2(r, "e") for r in en]
            bv = [fl2(r, "b") for r in en]
            hv = [fl2(r, "h_b") for r in en]
            rv = [fl2(r, "h_r") for r in en]
            em = sum(ev) / len(ev)
            res["mon_e_mean"] = round(em, 4)
            res["mon_e_std"] = round(math.sqrt(sum((x - em) ** 2 for x in ev) / len(ev)), 4)
            res["mon_e_absmax"] = round(max(abs(x) for x in ev), 4)
            res["mon_b_min"] = round(min(bv), 4)
            res["mon_b_max"] = round(max(bv), 4)
            res["mon_b_end"] = round(bv[-1], 4)
            dif = []
            for r in en:
                dif.append(fl2(r, "h_b") - agl(at(sim, fl2(r, "t"))))
            res["mon_hb_truth_mean"] = round(sum(dif) / len(dif), 4)
            res["mon_hb_truth_absmax"] = round(max(abs(x) for x in dif), 4)
            res["mon_hb_minus_hr_mean"] = round(sum(hv[i] - rv[i] for i in range(len(hv))) / len(hv), 4)
            if trig:
                tt = fl2(trig[0], "t")
                res["mon_truth_at_trig"] = round(agl(at(sim, tt)), 3)
                res["mon_truth_dev_at_trig"] = round(agl(at(sim, tt)) - ALT, 3)
                res["mon_hb_at_trig"] = fl2(trig[0], "h_b")
                res["mon_hr_at_trig"] = fl2(trig[0], "h_r")
                res["mon_e_at_trig"] = fl2(trig[0], "e")
        if res.get("mon_trigger_rel") is not None and res.get("f_t0") is not None:
            res["mon_detect_delay"] = round(res["mon_trigger_rel"] - res["f_t0"], 2)

# --- 监视器动作（A1/A2/A3）---
ta = None
act = [t for t in runlog.splitlines() if "MONITOR ACTION" in t]
if act:
    res["mon_action_log"] = [a.strip() for a in act][-4:]
    m = re.search(r"MONITOR ACTION (A[123]) at ([\d.]+) s", act[0])
    if m:
        ta = float(m.group(2))
        res["mon_action"] = m.group(1)
        res["mon_action_t"] = ta
        res["mon_action_rel"] = round(ta - s0, 2)
        # 注入→动作延迟：动作时刻(FC 时钟) − 注入时刻(FC 时钟) = ta − (s0 + f_t0)
        res["mon_action_delay"] = round(ta - (s0 + res["f_t0"]), 2) if res.get("f_t0") is not None else None
        if m.group(1) in ("A2", "A3"):
            def est_hgt(t):
                return -(at(ekf, t)[3] - E0[3])
            res["mon_ekf_hgt_before"] = round(est_hgt(ta - 0.3), 3)
            res["mon_ekf_hgt_after"] = round(est_hgt(ta + 0.3), 3)
            res["mon_ekf_jump"] = round(est_hgt(ta + 0.3) - est_hgt(ta - 0.3), 3)
            res["mon_truth_change"] = round(agl(at(sim, ta + 0.3)) - agl(at(sim, ta - 0.3)), 3)
        land_t = next((t for t, x in msgs if x.startswith("SIM Hit ground") and t >= ta), None)
        if land_t:
            res["mon_action_to_touchdown"] = round(land_t - ta, 2)
        if fl and ta:
            res["mon_truth_max_after_action"] = round(max(agl(p) for p in fl if p[0] >= ta), 3)
            res["mon_truth_min_after_action"] = round(min(agl(p) for p in fl if p[0] >= ta), 3)

# --- 降落过程是否受控 ---
land_t = next((t for t, x in msgs if x.startswith("Mission: ") and x.endswith("Land")), None)
if land_t is None:
    land_t = next((t for t, x in msgs if x.startswith("SIM Hit ground") and (ta is None or t >= ta)), None)
if land_t:
    dseq = [p for p in fl if p[0] >= land_t]
    rates = []
    for i in range(1, len(dseq)):
        dt = dseq[i][0] - dseq[i - 1][0]
        if dt > 0:
            rates.append((dseq[i][0], (agl(dseq[i]) - agl(dseq[i - 1])) / dt))
    des = [r for t, r in rates if r < 0]
    res["land_desc_rate_max"] = round(min(des), 3) if des else None
    res["land_desc_rate_mean"] = round(sum(des) / len(des), 3) if des else None
    res["land_hang_s"] = round(max([p[0] for p in fl if 0.15 < agl(p) <= 0.6] or [land_t]) - land_t, 1)
    res["land_max_hgt_after"] = round(max((agl(p) for p in fl if p[0] >= land_t), default=0), 3)
print(json.dumps(res, indent=1))
