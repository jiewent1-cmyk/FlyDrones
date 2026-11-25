from pathlib import Path
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OKABE = ["#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#56B4E9", "#F0E442", "#000000"]
plt.rcParams.update({"figure.dpi": 150, "savefig.dpi": 150, "savefig.bbox": "tight", "font.size": 10, "axes.titlesize": 11, "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.alpha": 0.3, "axes.prop_cycle": matplotlib.cycler(color=OKABE), "figure.facecolor": "white"})
HERE = Path(__file__).resolve().parent; OUT = HERE / "figures"; OUT.mkdir(exist_ok=True)
df = pd.read_csv(HERE / "results.csv")
df = df[df.phase != "tl"].copy()
df["dev_base"] = pd.to_numeric(df.dev_max, errors="coerce") if "dev_max" in df else pd.to_numeric(df.dev_base, errors="coerce")

# dev_max is the available raw maximum deviation metric per run.
fig, ax = plt.subplots(figsize=(8, 4.5))
phases = [p for p in ["cl", "a2", "cn", "md", "ds"] if p in set(df.phase)]
phase_labels = {"cl":"Climb (~1 m)", "a2":"At 2 m", "cn":"Corner", "md":"Mid-leg", "ds":"Descent (~1 m)"}
for i, p in enumerate(phases):
    for j, grp in enumerate(["D", "F"]):
        x = df[(df.phase == p) & (df.group == grp)]
        vals = pd.to_numeric(x.dev_max, errors="coerce").dropna()
        ax.scatter([i + (j-.5)*.22] * len(vals), vals, s=16, alpha=.75, color=OKABE[j])
ax.set(xticks=range(len(phases)), xticklabels=[phase_labels[p] for p in phases], ylabel="Maximum height deviation (m)", title="Height deviation by flight phase and parameter group")
ax.legend(handles=[plt.Line2D([], [], marker="o", ls="", color=OKABE[i], label=g) for i,g in enumerate(["D", "F"])], title="Group")
fig.tight_layout(); fig.savefig(OUT / "fig1_phase_deviation.png"); plt.close(fig)

# Compare completion rate by phase and configuration group.
df["complete_bool"] = df.mission_complete.astype(str).str.lower().eq("true")
rates = df.groupby(["phase", "group"]).complete_bool.mean().unstack().reindex(phases)
fig, ax = plt.subplots(figsize=(8, 4.5))
for i, grp in enumerate(["D", "F"]):
    if grp in rates: ax.plot(range(len(phases)), rates[grp].values * 100, marker="o", label=grp, color=OKABE[i])
ax.set(xticks=range(len(phases)), xticklabels=[phase_labels[p] for p in phases], ylabel="Mission completion (%)", ylim=(-5,105), title="Completion rate by phase")
ax.legend(title="Group")
fig.tight_layout(); fig.savefig(OUT / "fig2_completion_by_phase.png"); plt.close(fig)

# Fault-specific touchdown speed spread shows whether completed and aborted runs land differently.
df["touchdown_mps"] = pd.to_numeric(df.touchdown_mps, errors="coerce")
faults = [x for x in ["dn05", "dn15", "oor", "up15"] if x in set(df.ftype)]
fault_labels = {"dn05":"Down 0.5 m", "dn15":"Down 1.5 m", "oor":"Out of range", "up15":"Up 1.5 m"}
fig, ax = plt.subplots(figsize=(8, 4.5))
for i, fault in enumerate(faults):
    vals = df.loc[df.ftype == fault, "touchdown_mps"].dropna()
    ax.scatter([i] * len(vals), vals, s=16, alpha=.7)
ax.set(xticks=range(len(faults)), xticklabels=[fault_labels[f] for f in faults], ylabel="Touchdown speed (m/s)", title="Touchdown speed by injected range fault")
fig.tight_layout(); fig.savefig(OUT / "fig3_touchdown_speed.png"); plt.close(fig)
