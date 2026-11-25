"""Time series around the guard trip for one run: truth vs EKF horizontal velocity/position, flow fusion state."""
import glob, json, math, re, sys, os
from pymavlink import mavutil
rd = sys.argv[1]; step = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
m = mavutil.mavlink_connection(sorted(glob.glob(rd + "/logs/*.BIN"))[-1])
last = {}; land = None; rows = []; tnext = None; msgs = []; names = set()
W = ["SIM2", "XKF1", "XKF5", "XKF4", "XKF3", "RFND", "OF", "ATT", "PSCN", "PSCE", "MODE", "MSG", "CTUN", "XKFS"]
while True:
    x = m.recv_match(type=W)
    if x is None: break
    ty = x.get_type(); t = x.TimeUS * 1e-6
    if ty == "MSG": msgs.append((round(t, 2), x.Message)); continue
    if ty == "MODE":
        msgs.append((round(t, 2), "MODE %d" % x.ModeNum))
        if x.ModeNum == 9 and land is None: land = t
        continue
    if ty.startswith("XKF") and getattr(x, "C", 0) != 0: continue
    last[ty] = x
    if ty == "SIM2":
        if tnext is None: tnext = t
        if t >= tnext:
            tnext += step; rows.append((t, dict(last)))
if len(sys.argv) > 3: print(sorted((k, last[k].get_fieldnames()) for k in last)); 
for t, s in msgs: print(t, s)
def g(d, k, f, sc=1.0):
    return getattr(d[k], f) * sc if k in d and hasattr(d[k], f) else float("nan")
print("land", land)
print("    t  alt  |tN   tE  tVN  tVE | eN   eE  eVN  eVE ePD | rfnd HAGL offs | FIX  FIY rngI | flowX flowY bodX bodY | roll dRol pit dPit | SV   SP   SH  SVT TS  SS")
n0 = e0 = d0 = None
for t, d in rows:
    if n0 is None: n0, e0, d0 = d["SIM2"].PN, d["SIM2"].PE, d["SIM2"].PD
    if land is None or not (land - 6 <= t <= land + 14): continue
    print("%6.1f %4.2f |%5.2f %5.2f %5.2f %5.2f |%5.2f %5.2f %5.2f %5.2f %5.2f | %4.2f %4.2f %5.2f |%5.0f %5.0f %5.2f | %5.2f %5.2f %5.2f %5.2f |%5.1f %5.1f %5.1f %5.1f |%4.2f %4.2f %4.2f %4.2f %3d %5d" % (
        t - land, -(d["SIM2"].PD - d0), d["SIM2"].PN - n0, d["SIM2"].PE - e0, d["SIM2"].VN, d["SIM2"].VE,
        g(d, "XKF1", "PN"), g(d, "XKF1", "PE"), g(d, "XKF1", "VN"), g(d, "XKF1", "VE"), g(d, "XKF1", "PD"),
        g(d, "RFND", "Dist"), g(d, "XKF5", "HAGL"), g(d, "XKF5", "offset"), g(d, "XKF5", "FIX"), g(d, "XKF5", "FIY"), g(d, "XKF5", "RI"),
        g(d, "OF", "flowX"), g(d, "OF", "flowY"), g(d, "OF", "bodyX"), g(d, "OF", "bodyY"),
        g(d, "ATT", "Roll"), g(d, "ATT", "DesRoll"), g(d, "ATT", "Pitch"), g(d, "ATT", "DesPitch"),
        g(d, "XKF4", "SV"), g(d, "XKF4", "SP"), g(d, "XKF4", "SH"), g(d, "XKF4", "SVT"), int(g(d, "XKF4", "TS")), int(g(d, "XKF4", "SS"))))
