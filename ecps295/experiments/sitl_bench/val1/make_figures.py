from pathlib import Path
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OKABE = ["#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#56B4E9", "#F0E442", "#000000"]
plt.rcParams.update({"figure.dpi": 150, "savefig.dpi": 150, "savefig.bbox": "tight", "font.size": 10, "axes.titlesize": 11, "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.alpha": 0.3, "axes.prop_cycle": matplotlib.cycler(color=OKABE), "figure.facecolor": "white"})
HERE = Path(__file__).resolve().parent
OUT = HERE / "figures"
OUT.mkdir(exist_ok=True)
df = pd.read_csv(HERE / "results.csv")

# Use completed flight groups with directly reported per-run metrics.
groups = ["a_d3", "a_sq", "b_d3", "b_sq"]
labels = ["A1 D3", "A3 square", "B1 D3", "B3 square"]
fig, ax = plt.subplots(figsize=(7, 4))
for i, (g, label) in enumerate(zip(groups, labels)):
    vals = pd.to_numeric(df.loc[df.group == g, "track_rms"], errors="coerce").dropna()
    ax.scatter([i] * len(vals), vals, s=32, zorder=3, label=label)
    if len(vals): ax.plot([i-.16, i+.16], [vals.median()] * 2, color="black", lw=2)
ax.set(xticks=range(len(groups)), xticklabels=labels, ylabel="Track RMS error (m)", title="Tracking error across clean and harsh runs")
fig.tight_layout(); fig.savefig(OUT / "fig1_tracking_spread.png"); plt.close(fig)

# Outcome counts are derived from mission completion for the C unlock matrix.
c = df[df.group.isin(["c1_viso0_novis", "c2_viso1_novis", "c3_viso1_vis", "c4_viso0_noorigin"])].copy()
c["mission_complete"] = c.mission_complete.astype(str).str.lower().eq("true")
order = ["c1_viso0_novis", "c2_viso1_novis", "c3_viso1_vis", "c4_viso0_noorigin"]
rates = c.groupby("group").mission_complete.mean().reindex(order)
fig, ax = plt.subplots(figsize=(7, 4))
ax.bar(range(4), rates.values * 100, color=["#009E73" if v == 1 else "#D55E00" for v in rates.values])
for i, (k, v) in enumerate(zip(c.groupby("group").mission_complete.agg(["sum", "size"]).reindex(order).itertuples(), rates.values)):
    ax.text(i, v * 100 + 2, f"{int(k.sum)}/{int(k.size)}", ha="center", va="bottom", fontsize=11)
ax.set(xticks=range(4), xticklabels=["VISO 0\nno vision", "VISO 1\nno vision", "VISO 1\nvision", "VISO 0\nno origin"], ylabel="Mission completion (%, n=3)", ylim=(0, 110), title="Unlock matrix mission completion")
fig.tight_layout(); fig.savefig(OUT / "fig2_unlock_matrix.png"); plt.close(fig)

# Fence distance headroom from per-run maximum radius and configured radius.
f = df[df.group.isin(["e1_fence", "e2_r35", "e3_alt23"])].copy()
f["rad_max"] = pd.to_numeric(f.rad_max, errors="coerce")
radii = {"e1_fence": 3.7, "e2_r35": 3.2, "e3_alt23": 3.7}  # report table effective radius = configured radius - margin
fig, ax = plt.subplots(figsize=(7, 4))
for i, g in enumerate(["e1_fence", "e2_r35", "e3_alt23"]):
    vals = (radii[g] - f.loc[f.group == g, "rad_max"]).dropna()
    ax.scatter([i] * len(vals), vals, s=34)
    ax.plot([i-.15, i+.15], [vals.median()] * 2, color="black", lw=2)
ax.axhline(0, color="#D55E00", lw=1)
ax.set(xticks=range(3), xticklabels=["E1 R=4 m", "E2 R=3.5 m", "E3 ALT=2.3 m"], ylabel="Horizontal fence headroom (m)", title="Measured distance to effective fence radius")
fig.tight_layout(); fig.savefig(OUT / "fig3_fence_headroom.png"); plt.close(fig)
