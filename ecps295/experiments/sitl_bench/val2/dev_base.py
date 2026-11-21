#!/usr/bin/env python3
"""Per fault run: dev_base = max |truth_fault(t) - truth_baseline(t)| for t in [fault start, +15 s],
both relative to AUTO mode-on; baseline = v2_tl_r<n> (same yaw / rep). Also writes fault start rel AUTO."""
import glob, json, os, re, sys

from pymavlink import mavutil
from concurrent.futures import ThreadPoolExecutor

D = "/home/jacky/sim/val2"


def load(rd):
    sim, mode, parm, msg = [], [], [], []
    for f in sorted(glob.glob(os.path.join(rd, "logs", "*.BIN"))):
        m = mavutil.mavlink_connection(f)
        while True:
            x = m.recv_match(type=["SIM2", "MODE", "PARM", "MSG"])
            if x is None:
                break
            t = x.TimeUS / 1e6
            if x.get_type() == "SIM2":
                sim.append((t, -(x.PD)))
            elif x.get_type() == "MODE":
                mode.append((t, x.ModeNum))
            elif x.get_type() == "PARM" and x.Name == "SIM_SONAR_POS_Z":
                parm.append((t, x.Value))
            elif x.get_type() == "MSG":
                msg.append((t, x.Message))
    sim.sort()
    t_land = next((t for t, x in msg if x.startswith("SIM Hit ground")), None)
    return sim, next(t for t, mn in mode if mn == 3), parm, t_land


def interp(sim, t):
    lo, hi = 0, len(sim) - 1
    if t <= sim[0][0]:
        return sim[0][1]
    if t >= sim[-1][0]:
        return sim[-1][1]
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if sim[mid][0] < t:
            lo = mid
        else:
            hi = mid
    a, b = sim[lo], sim[hi]
    return a[1] + (b[1] - a[1]) * (t - a[0]) / (b[0] - a[0])


def one(rd, base_rd):
    try:
        r = json.load(open(os.path.join(rd, "metrics.json")))
    except Exception:
        return None
    out = {"tag": os.path.basename(rd)}
    if "f_alt0" not in r:
        out["dev_base"] = None
        return out
    simF, t0F, parmF, landF = load(rd)
    simB, t0B, _, landB = load(base_rd)
    bads = [t for t, v in parmF if t > t0F and abs(v - (-0.05)) > 1e-3]
    if not bads:
        out["dev_base"] = None
        return out
    f0 = bads[0] - t0F  # fault start, s after AUTO on
    fend = bads[-1] - t0F
    cap = min(f0 + 15.0, (landF - t0F) if landF else 1e9, (landB - t0B) if landB else 1e9)
    n = max(2, int((cap - f0) / 0.05))
    devs = [(abs(interp(simF, t0F + t) - interp(simB, t0B + t)), t) for t in [f0 + 0.05 * i for i in range(n)]]
    out.update(f_start_auto=round(f0, 1), f_end_auto=round(fend, 1),
               dev_base=round(max(d for d, _ in devs), 3),
               dev_base_t=round(min(devs, key=lambda x: -x[0])[1], 1),
               dev_base_win=round(cap - f0, 1),
               base_max_w15=round(max(interp(simB, t0B + t) for t in [f0 + 0.05 * i for i in range(n)]), 3))
    return out


rows = []
faults = sorted(glob.glob(D + "/runs/v2_*_r[123]"))
jobs = []
for rd in faults:
    tag = os.path.basename(rd)
    m = re.match(r"v2_(cl|a2|cn|md|ds)_", tag)
    if not m:
        continue
    rep = re.search(r"_r(\d)$", tag).group(1)
    jobs.append((rd, D + "/runs/v2_tl_r" + rep))
with ThreadPoolExecutor(4) as ex:
    rows = [r for r in ex.map(lambda j: one(*j), jobs) if r]
with open(D + "/dev_base.json", "w") as fh:
    json.dump(rows, fh, indent=1)
print("wrote", len(rows))
