from pathlib import Path
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OKABE = ["#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#56B4E9", "#F0E442", "#000000"]
plt.rcParams.update({"figure.dpi": 150, "savefig.dpi": 150, "savefig.bbox": "tight", "font.size": 10, "axes.titlesize": 11, "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.alpha": 0.3, "axes.prop_cycle": matplotlib.cycler(color=OKABE), "figure.facecolor": "white"})
HERE = Path(__file__).resolve().parent; OUT = HERE / "figures"; OUT.mkdir(exist_ok=True)
df = pd.read_csv(HERE / "results.csv")

# Completion fraction across the parameter grid and six injected faults.
fault_order = ["a", "b", "c", "d", "e", "f"]
groups = list(dict.fromkeys(df.group))
mat = np.array([[df[(df.group == g) & (df.fault == f)].complete.astype(str).str.lower().eq("true").mean() for f in fault_order] for g in groups])
assert np.isin(mat, [0, 1]).all()  # every cell is 0/3 or 3/3, so a two-colour pass/fail map is exact
FAULT_LABEL = {"a": "a: climb\nDN 1.5 m, 1 s", "b": "b: climb\nDN 0.5 m, 3 s", "c": "c: at 2 m\nDN 1.5 m, 3 s",
               "d": "d: at 2 m\nUP 1.5 m, 3 s", "e": "e: corner\nDN 1.5 m, 3 s", "f": "f: descent\nUP 1.5 m, 1 s"}
GROUP_XLABEL = ("Parameter group h<G>n<N>: h = EK3_HGT_I_GATE G (unitless, 100 = 1 sigma),\n"
                "n = EK3_RNG_M_NSE N/10 (m); orig = G 500, 0.5 m, SURFTRAK_MODE 1")
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
fig, ax = plt.subplots(figsize=(9, 6.6))
ax.imshow(mat, aspect="auto", vmin=0, vmax=1, cmap=ListedColormap([OKABE[3], OKABE[2]]))
ax.grid(False)
for i, g in enumerate(groups):
    for j, f in enumerate(fault_order):
        c = df[(df.group == g) & (df.fault == f)].complete.astype(str).str.lower().eq("true")
        ax.text(j, i, f"{int(c.sum())}/{len(c)}", ha="center", va="center", color="white", fontsize=8, fontweight="bold")
ax.set(xticks=range(len(fault_order)), xticklabels=[FAULT_LABEL[f] for f in fault_order], yticks=range(len(groups)), yticklabels=groups, xlabel="Injected rangefinder fault (DN/UP = reading jumps down/up)", ylabel=GROUP_XLABEL, title="Mission completion across the fault matrix (completed runs / 3)")
ax.tick_params(axis="x", labelsize=8)
ax.legend(handles=[Patch(color=OKABE[2], label="3/3 completed"), Patch(color=OKABE[3], label="0/3 completed")], loc="upper center", bbox_to_anchor=(0.5, -0.17), ncol=2)
fig.tight_layout(); fig.savefig(OUT / "fig1_fault_completion_heatmap.png"); plt.close(fig)

# HOV height errors by parameter group, retaining individual repeat points.
hov = df[df.fault == "hov"].copy(); hov["hov_alt_rms"] = pd.to_numeric(hov.hov_alt_rms, errors="coerce")
fig, ax = plt.subplots(figsize=(8,4.5))
for i,g in enumerate(groups):
    vals=hov.loc[hov.group==g,"hov_alt_rms"].dropna(); ax.scatter([i]*len(vals), vals, s=18, alpha=.75, color=OKABE[0])
ax.set(xticks=range(len(groups)), xticklabels=groups, xlabel=GROUP_XLABEL, ylabel="True altitude RMS error (m)", title="HOV altitude error across parameter groups")
ax.tick_params(axis="x", rotation=55)
fig.tight_layout(); fig.savefig(OUT / "fig2_hov_altitude_error.png"); plt.close(fig)

# Distribution of maximum true altitude in the two reproduction configurations.
repro = df[(df.group.isin(["orig", "h200n01"])) & (df.fault == "a")].copy()
repro["alt_max"] = pd.to_numeric(repro.alt_max, errors="coerce")
fig, ax = plt.subplots(figsize=(7,4))
for i,g in enumerate(["orig", "h200n01"]):
    vals=repro.loc[repro.group==g,"alt_max"].dropna(); ax.scatter([i]*len(vals), vals, s=42, color=OKABE[i]);
    if len(vals): ax.plot([i-.16,i+.16],[vals.median()]*2,color="black",lw=2)
ax.axhline(2.5,color="#D55E00",ls="--",label="Fence altitude limit (2.5 m)")
ax.set(xticks=[0,1],xticklabels=["Original 500/0.5/1", "FINAL 200/0.1/0"],ylabel="Maximum true altitude (m)",title="Reproduction runs: altitude during climb fault"); ax.legend()
fig.tight_layout(); fig.savefig(OUT / "fig3_reproduction_altitude.png"); plt.close(fig)
