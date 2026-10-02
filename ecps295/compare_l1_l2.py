"""Compare L1 and L2 alt(t) on the same gesture timeline (TechRoute §5.5 step 5: RMS < 0.1 m)."""

import argparse
import csv

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument("l1")
ap.add_argument("l2")
ap.add_argument("--png", default="l1_vs_l2.png")
a = ap.parse_args()


def load(p):
    r = [x for x in csv.DictReader(open(p)) if x["alt"] not in ("", "None")]
    return (
        np.array([float(x["t"]) for x in r]),
        np.array([float(x["alt"]) for x in r]),
        np.array([float(x["cmd_throttle"]) for x in r]),
    )


t1, z1, c1 = load(a.l1)
t2, z2, c2 = load(a.l2)
tg = np.arange(0, min(t1[-1], t2[-1]), 0.05)
d = np.interp(tg, t2, z2) - np.interp(tg, t1, z1)
rms = float(np.sqrt(np.mean(d**2)))
print(f"alt RMS(L2-L1) = {rms:.3f} m  max|d| = {np.max(np.abs(d)):.3f} m  -> {'PASS' if rms < 0.1 else 'FAIL'} (< 0.1 m)")
fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
ax[0].plot(t1, z1, label="L1 TwinSimDrone")
ax[0].plot(t2, z2, label="L2 SITL")
ax[0].axhline(1.0, ls=":", c="r")
ax[0].axhline(0.4, ls=":", c="r")
ax[0].set_ylabel("alt m")
ax[0].legend()
ax[0].set_title(f"same MiniFly + gesture timeline, alt RMS = {rms:.3f} m")
ax[0].grid(alpha=0.3)
ax[1].plot(t1, c1, label="L1")
ax[1].plot(t2, c2, label="L2")
ax[1].set_ylabel("cmd throttle")
ax[1].set_xlabel("t s")
ax[1].legend()
ax[1].grid(alpha=0.3)
fig.tight_layout()
fig.savefig(a.png, dpi=80)
