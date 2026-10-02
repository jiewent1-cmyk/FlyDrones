"""Truth (SIM) vs EKF (POS/XKF) vs rangefinder (RFND) from a SITL DataFlash log, for S7 / S7c."""

import math
import sys

import matplotlib
import numpy as np
from pymavlink import mavutil

matplotlib.use("Agg")
import matplotlib.pyplot as plt

log, png = sys.argv[1], sys.argv[2]
mlog = mavutil.mavlink_connection(log)
d = {k: [] for k in ("SIM", "POS", "RFND", "MODE", "CTUN", "OF")}
while (msg := mlog.recv_match(type=list(d))) is not None:
    d[msg.get_type()].append(msg)
sim, pos, rf, ctun = d["SIM"], d["POS"], d["RFND"], d["CTUN"]
lat0, lng0, alt0 = sim[0].Lat, sim[0].Lng, sim[0].Alt


def ne(lat, lng):
    return (lat - lat0) * 111320.0, (lng - lng0) * 111320.0 * math.cos(math.radians(lat0))


ts = np.array([s.TimeUS for s in sim]) / 1e6
tn, te = np.array([ne(s.Lat, s.Lng) for s in sim]).T
tz = np.array([s.Alt - alt0 for s in sim])
tp = np.array([p.TimeUS for p in pos]) / 1e6
en, ee = np.array([ne(p.Lat, p.Lng) for p in pos]).T
ez = np.array([p.RelOriginAlt if hasattr(p, "RelOriginAlt") else p.Alt - pos[0].Alt for p in pos])
tr = np.array([r.TimeUS for r in rf]) / 1e6
rd = np.array([r.Dist for r in rf])
rs = np.array([getattr(r, "Stat", 0) for r in rf])
tc = np.array([c.TimeUS for c in ctun]) / 1e6
calt = np.array([c.Alt for c in ctun])

# EKF horizontal error vs truth (truth interpolated); position-only meaningful as drift since origin == home
herr = np.hypot(np.interp(tp, ts, tn) - en, np.interp(tp, ts, te) - ee)
airborne = np.interp(tp, ts, tz) > 0.2
print(f"truth alt max {tz.max():.2f} m | CTUN.Alt max {calt.max():.2f} m")
print(f"EKF horiz error while airborne: median {np.median(herr[airborne]):.2f} m, max {herr[airborne].max():.2f} m")
zerr = np.interp(tc, ts, tz) - calt
print(
    f"truth - CTUN.Alt while airborne: median {np.median(zerr[np.interp(tc, ts, tz) > 0.2]):+.2f} m, "
    f"max |err| {np.abs(zerr[np.interp(tc, ts, tz) > 0.2]).max():.2f} m"
)
for mm in d["MODE"]:
    print(f"  t={mm.TimeUS / 1e6:7.1f} MODE {mm.Mode} rsn {mm.Rsn}")

fig, ax = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
ax[0].plot(ts, tz, label="truth (SIM.Alt)")
ax[0].plot(tc, calt, label="EKF (CTUN.Alt)")
ax[0].plot(tr, rd, ".", ms=2, label="RFND.Dist")
ax[0].axhline(1.2, c="r", ls=":", label="RNGFND1_MAX")
ax[0].axhline(1.3, c="k", ls=":", label="FENCE_ALT_MAX")
ax[0].set_ylabel("m")
ax[0].legend(fontsize=8)
ax[0].grid(alpha=0.3)
ax[1].plot(tp, herr)
ax[1].set_ylabel("EKF horiz err m")
ax[1].grid(alpha=0.3)
ax[2].plot(ts, tn, label="truth N")
ax[2].plot(ts, te, label="truth E")
ax[2].plot(tp, en, "--", label="EKF N")
ax[2].plot(tp, ee, "--", label="EKF E")
ax[2].set_ylabel("m")
ax[2].legend(fontsize=8)
ax[2].grid(alpha=0.3)
ax[2].set_xlabel("t s")
for mm in d["MODE"]:
    for x in ax:
        x.axvline(mm.TimeUS / 1e6, c="gray", lw=0.5)
    ax[0].text(mm.TimeUS / 1e6, ax[0].get_ylim()[1] * 0.95, str(mm.Mode), fontsize=7)
fig.tight_layout()
fig.savefig(png, dpi=80)
