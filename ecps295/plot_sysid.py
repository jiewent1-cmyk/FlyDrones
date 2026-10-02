"""Plot SITL step responses from sysid_tau.py CSV."""

import csv
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

rows = list(csv.DictReader(open(sys.argv[1])))
fig, axs = plt.subplots(4, 1, figsize=(11, 11))
for ax, (axis, col, unit) in zip(axs, [("x", "vbx", "m/s"), ("z", "vup", "m/s"), ("yaw", "yaw_rate_dps", "deg/s")]):
    r = [x for x in rows if x["axis"] == axis]
    t = [float(x["t"]) for x in r]
    ax.plot(t, [float(x["cmd"]) for x in r], "k--", label="cmd")
    ax.plot(t, [float(x[col]) for x in r], label=col)
    ax.set_title(f"{axis} step")
    ax.set_ylabel(unit)
    ax.grid(alpha=0.3)
    ax.legend()
t = [float(x["t"]) for x in rows]
axs[3].plot(t, [float(x["alt"]) for x in rows])
axs[3].set_ylabel("alt m")
axs[3].grid(alpha=0.3)
fig.tight_layout()
fig.savefig(sys.argv[2], dpi=80)
