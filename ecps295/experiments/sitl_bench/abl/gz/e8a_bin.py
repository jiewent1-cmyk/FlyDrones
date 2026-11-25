import glob, os, statistics as st
from multiprocessing import Pool
from pymavlink import mavutil
def one(d):
    m = mavutil.mavlink_connection(sorted(glob.glob(d + "/logs/*.BIN"))[-1])
    land = dis = None; pos = []; sim = []; modes = []; cleared = 0
    while True:
        x = m.recv_match(type=["MODE", "XKF1", "MSG", "SIM"])
        if x is None: break
        t = x.TimeUS * 1e-6; k = x.get_type()
        if k == "MODE":
            modes.append(x.ModeNum)
            if x.ModeNum == 9 and land is None: land = t
        elif k == "MSG":
            if "Disarming" in x.Message and land is not None and dis is None: dis = t
            if "Failsafe Cleared" in x.Message: cleared += 1
        elif k == "XKF1" and x.C == 0: pos.append((t, x.PN, x.PE, x.PD, x.VD))
        elif k == "SIM": sim.append((t, x.Alt))
    if land is None: return dict(tag=os.path.basename(d), land=None)
    seg = [p for p in pos if land <= p[0] <= (dis or 1e9)]
    n0, e0 = seg[0][1], seg[0][2]
    dr = max(((p[1] - n0) ** 2 + (p[2] - e0) ** 2) ** .5 for p in seg)
    low = [p[4] for p in seg if -p[3] < 0.15]
    return dict(tag=os.path.basename(d), land_s=None if dis is None else dis - land, drift=dr, h0=-seg[0][3], vd_max=max(p[4] for p in seg),
                modes_after=modes[modes.index(9):], cleared=cleared, has_sim=bool(sim))
if __name__ == "__main__":
    ds = sorted(glob.glob(os.path.expanduser("~/sim/runs/e8a__*")))
    with Pool(2) as p: R = p.map(one, ds)
    for r in R: print(r)
    ok = [r for r in R if r.get("land_s")]
    print("n", len(R), "disarmed", len(ok), "land_s min/med/max %.1f %.1f %.1f" % (min(r["land_s"] for r in ok), st.median(r["land_s"] for r in ok), max(r["land_s"] for r in ok)),
          "drift med/max %.2f %.2f" % (st.median(r["drift"] for r in ok), max(r["drift"] for r in ok)), "vd max %.2f" % max(r["vd_max"] for r in ok),
          "mode changed after LAND:", sum(r["modes_after"] != [9] for r in ok))
