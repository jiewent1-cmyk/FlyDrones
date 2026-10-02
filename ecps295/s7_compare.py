"""Compare S7/S7c variant runs: truth (SIM) vs EKF from each DataFlash log.

usage: s7_compare.py RUNDIR [RUNDIR ...]    (each RUNDIR has logs/*.BIN and s7_events.json)
"""

import glob
import json
import math
import os
import sys

import numpy as np
from pymavlink import mavutil

SWITCH_M = 0.84  # EK3_RNG_USE_HGT 70% x RNGFND1_MAX 1.2


def load(path):
    mlog = mavutil.mavlink_connection(path)
    d = {k: [] for k in ("SIM", "POS", "CTUN", "MODE", "MSG")}
    while (x := mlog.recv_match(type=list(d))) is not None:
        d[x.get_type()].append(x)
    return d


def metrics(rundir):
    log = sorted(glob.glob(os.path.join(rundir, "logs", "*.BIN")), key=os.path.getmtime)[-1]
    d = load(log)
    sim = d["SIM"]
    lat0, lng0, alt0 = sim[0].Lat, sim[0].Lng, sim[0].Alt
    ts = np.array([s.TimeUS for s in sim]) / 1e6
    tn = np.array([(s.Lat - lat0) * 111320.0 for s in sim])
    te = np.array([(s.Lng - lng0) * 111320.0 * math.cos(math.radians(lat0)) for s in sim])
    tz = np.array([s.Alt - alt0 for s in sim])
    tc = np.array([c.TimeUS for c in d["CTUN"]]) / 1e6
    ez = np.array([c.Alt for c in d["CTUN"]])
    tp = np.array([p.TimeUS for p in d["POS"]]) / 1e6
    en = np.array([(p.Lat - lat0) * 111320.0 for p in d["POS"]])
    ee = np.array([(p.Lng - lng0) * 111320.0 * math.cos(math.radians(lat0)) for p in d["POS"]])
    modes = [(m.TimeUS / 1e6, m.Mode) for m in d["MODE"]]
    t_loiter = next((t for t, mo in modes if mo == 5), None)
    t_guided = next((t for t, mo in modes if mo == 4), None)
    t_land = next((t for t, mo in modes if mo == 9), tc[-1])

    tzc = np.interp(tc, ts, tz)
    fly = (tzc > 0.2) & (tc < t_land)
    err = ez - tzc
    crossed = tc >= (tc[fly & (tzc > SWITCH_M)][0] if np.any(fly & (tzc > SWITCH_M)) else np.inf)
    before, after = fly & ~crossed, fly & crossed
    out = {
        "run": os.path.basename(rundir.rstrip("/")),
        "alt_err_before_switch_med": float(np.median(np.abs(err[before]))) if before.any() else None,
        "alt_err_after_switch_med": float(np.median(np.abs(err[after]))) if after.any() else None,
        "alt_err_max": float(np.max(np.abs(err[fly]))),
        "true_alt_max_flight": float(tzc[fly].max()),
    }
    if t_loiter and t_guided:
        w = (ts >= t_loiter + 2) & (ts < t_guided)
        out["loiter_truth_xy_std"] = float(np.hypot(np.std(tn[w]), np.std(te[w])))
        out["loiter_truth_drift"] = float(math.hypot(tn[w][-1] - tn[w][0], te[w][-1] - te[w][0]))
    fp = (tp < t_land) & (np.interp(tp, ts, tz) > 0.2)
    herr = np.hypot(np.interp(tp, ts, tn) - en, np.interp(tp, ts, te) - ee)
    out["ekf_xy_err_med"] = float(np.median(herr[fp]))
    out["ekf_xy_err_end"] = float(herr[fp][-1])
    ev_path = os.path.join(rundir, "s7_events.json")
    if os.path.exists(ev_path):
        ev = json.load(open(ev_path))
        out["events_tail"] = [e[1] for e in ev if "P5" in e[1] and "ekf_pos" not in e[1]] + [
            e[1] for e in ev if "breach" in e[1] or "Failsafe" in e[1] or "ARM FAILED" in e[1]
        ]
    # P5a soft ceiling ran for 20 s right before P5b breached the hard fence
    t_breach = next((x.TimeUS / 1e6 for x in d["MSG"] if "fence breached" in x.Message), None)
    if t_breach:
        w = (ts > t_breach - 12) & (ts < t_breach - 3)
        out["true_alt_soft_ceiling"] = float(np.median(tz[w]))
        out["ekf_alt_soft_ceiling"] = float(np.median(ez[(tc > t_breach - 12) & (tc < t_breach - 3)]))
        out["true_alt_at_breach"] = float(np.interp(t_breach, ts, tz))
        out["ekf_alt_at_breach"] = float(np.interp(t_breach, tc, ez))
    return out


for r in sys.argv[1:]:
    try:
        print(json.dumps(metrics(r)))
    except Exception as e:  # noqa: BLE001
        print(json.dumps({"run": r, "error": repr(e)}))
