"""Figure for README.md (planned Gazebo runs of the hover / hold / PIL work). Reads the job lists next to this file
(*_matrix.txt, n3a/*, hv/*): one line per planned run, "<tag> <world> <variant|vel|pos|parm> ...". It shows the plan
(number of planned runs), not results: the per-run results were not kept in this directory.

    python make_figures.py
"""
from pathlib import Path
import re
import numpy as np
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

# job lists in the order of the work (C: efference copy / C4 bypass arms, D: texture-free floors, S: sensing + shield,
# N: shield-in-the-loop follow-ups, hover: hover / hold lists)
LISTS = [
    "c1_matrix.txt", "c1b_matrix.txt", "c4_matrix.txt", "c4b_matrix.txt", "c4c_matrix.txt", "c4d_matrix.txt",
    "c4e_matrix.txt", "c4f_matrix.txt", "c4g_matrix.txt", "d1_matrix.txt", "d1b_matrix.txt", "d2_matrix.txt",
    "d3_matrix.txt", "d3r_matrix.txt", "sh_matrix.txt", "sh3d_matrix.txt", "s5_matrix.txt", "mb_matrix.txt",
    "mb5_matrix.txt", "s6_matrix.txt", "s7_matrix.txt", "s7b_matrix.txt", "s7c_matrix.txt",
    "n3a/q1_matrix.txt", "n3a/q2a_matrix.txt", "n3a/q2b_matrix.txt", "n3a/q3a_matrix.txt", "n3a/q3b_matrix.txt",
    "n3a/q4a_matrix.txt", "n3a/q4b_matrix.txt", "n3a/q5_matrix.txt", "n3a/q6rA.txt", "n3a/q6rB.txt",
    "n3a/q6tA.txt", "n3a/q6tB.txt", "n3a/wp0_matrix.txt", "n3a/wp1_matrix.txt", "n3a/wp2_matrix.txt",
    "n3a/pf1_matrix.txt",
    "hv/batch.txt", "hv/sweep.txt", "hv/hb.txt", "hv/hp.txt", "hv/hp5.txt",
]
FAMILIES = ["lowbox (textured)", "lowbox, no floor texture", "cage20", "cage20, ghost", "wall tests", "flat", "other"]


def family(w):
    if re.fullmatch(r"lowbox_s\d+", w):
        return 0
    if re.fullmatch(r"lowbox_s\d+_notex", w):
        return 1
    if w == "cage20":
        return 2
    if w == "cage20_ghost":
        return 3
    if w.startswith("wall"):
        return 4
    if w == "flat":
        return 5
    return 6


M = np.zeros((len(LISTS), len(FAMILIES)), dtype=int)
for i, name in enumerate(LISTS):
    for line in open(HERE / name):
        p = line.split()
        if len(p) >= 2 and not line.startswith("#"):
            M[i, family(p[1])] += 1

keep = M.sum(axis=0) > 0
fig, ax = plt.subplots(figsize=(7.4, 11.5))
im = ax.imshow(np.where(M[:, keep] > 0, M[:, keep], np.nan), cmap="Blues", aspect="auto", vmin=0, vmax=M.max() * 1.15)
ax.grid(False)
ax.set_xticks(range(keep.sum()), [f for f, k in zip(FAMILIES, keep) if k], rotation=25, ha="right")
ax.set_yticks(range(len(LISTS)), [f"{n}  ({M[i].sum()})" for i, n in enumerate(LISTS)], fontsize=8)
for i in range(len(LISTS)):
    for j in range(keep.sum()):
        v = M[i, keep][j]
        if v:
            ax.text(j, i, str(v), ha="center", va="center", fontsize=8, color="white" if v > M.max() * 0.55 else "black")
for s in ax.spines.values():
    s.set_visible(False)
ax.set_title("Planned Gazebo runs per job list and world family\n(the plan, not results; total runs in brackets)")
fig.colorbar(im, ax=ax, shrink=0.4, label="planned runs (lines in the list)")
fig.savefig(OUT / "fig1_planned_runs.png")
plt.close(fig)
