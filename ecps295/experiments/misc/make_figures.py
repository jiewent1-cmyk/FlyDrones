"""Figure for README.md (the two Gazebo low-box job lists in this directory). Reads h1_lowbox30_49.txt and
h3_rog_lowbox.txt next to this file (one line per planned run: "<tag> <world> <config> <seed> <run_g4 args>") and
writes figures/fig1_planned_groups.png. It shows the plan, not results: the per-run results were not kept here.

    python make_figures.py
"""
from collections import Counter
from pathlib import Path
import re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OKABE = ["#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#56B4E9", "#F0E442", "#000000"]
plt.rcParams.update({"figure.dpi": 150, "savefig.dpi": 150, "savefig.bbox": "tight",
    "font.size": 10, "axes.titlesize": 11, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.alpha": 0.3, "axes.prop_cycle": matplotlib.cycler(color=OKABE),
    "figure.facecolor": "white"})
HERE = Path(__file__).parent
OUT = HERE / "figures"
OUT.mkdir(exist_ok=True)

LISTS = [("h1_lowbox30_49.txt", OKABE[0]), ("h3_rog_lowbox.txt", OKABE[1])]
rows = []  # (list, tag group, planned runs, lowest seed, highest seed)
for name, _ in LISTS:
    runs = {}
    for line in open(HERE / name):
        p = line.split(None, 4)
        if len(p) < 5 or line.startswith("#"):
            continue
        runs.setdefault(p[0].split("__")[0], []).append(int(p[3]))
    for g, seeds in runs.items():
        rows.append((name, g, len(seeds), min(seeds), max(seeds)))

fig, ax = plt.subplots(figsize=(7.6, 4.4))
ypos = list(range(len(rows)))[::-1]
colour = dict(LISTS)
for y, (name, g, n, lo, hi) in zip(ypos, rows):
    ax.barh(y, n, color=colour[name], edgecolor="k", linewidth=0.4)
    ax.text(n + 0.8, y, f"{n} runs, seeds {lo}-{hi}", va="center", fontsize=8)
ax.set_yticks(ypos, [r[1] for r in rows])
ax.set_xlabel("planned runs (lines in the job list)")
ax.set_xlim(0, 75)
ax.set_title("Planned runs per tag group (the plan, not results)")
ax.legend(loc="upper right", fontsize=8, handles=[plt.Rectangle((0, 0), 1, 1, color=c, label=n) for n, c in LISTS])
fig.savefig(OUT / "fig1_planned_groups.png")
plt.close(fig)
