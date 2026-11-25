from pathlib import Path
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OKABE = ["#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#56B4E9", "#F0E442", "#000000"]
plt.rcParams.update({"figure.dpi": 150, "savefig.dpi": 150, "savefig.bbox": "tight", "font.size": 10, "axes.titlesize": 11, "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.alpha": 0.3, "axes.prop_cycle": matplotlib.cycler(color=OKABE), "figure.facecolor": "white"})
HERE = Path(__file__).resolve().parent; OUT = HERE / "figures"; OUT.mkdir(exist_ok=True)
df = pd.read_csv(HERE / "results.csv")

# Data table in report.md §1.6 gives medians and ranges for uncorrected flight at three altitudes.
alt = [2.0, 1.5, 1.0]
hover = [0.96, 0.74, 0.36]
hover_lo = [0.56, 0.50, 0.27]; hover_hi = [0.98, 0.75, 0.46]
sq = [0.19, 0.12, 0.10]
sq_lo = [0.13, 0.09, 0.09]; sq_hi = [0.22, 0.35, 0.10]
fig, ax = plt.subplots(figsize=(7,4))
ax.errorbar(alt, hover, yerr=[[a-b for a,b in zip(hover,hover_lo)],[b-a for a,b in zip(hover,hover_hi)]], marker="o", capsize=3, label="Hover maximum deviation (marker = median)")
ax.errorbar(alt, sq, yerr=[[a-b for a,b in zip(sq,sq_lo)],[b-a for a,b in zip(sq,sq_hi)]], marker="s", capsize=3, label="Square tracking RMS (marker = median)")
ax.set(xlabel="Flight altitude (m)",ylabel="Position error (m)",title="Lower altitude reduces uncorrected drift\n(n=3 per altitude; bar ends = min and max of the 3 runs)"); ax.legend()
fig.tight_layout(); fig.savefig(OUT / "fig1_uncorrected_altitude.png"); plt.close(fig)

# Extract report table §1.2 comparison values, whose exact summary is not stored per run.
# Source: report.md §1.2, table “实测：两种方法把漂移从多少降到多少”.
labels=["No correction","ExtNav σ=.05","ExtNav σ=.15","ExtPos σ=.05","ExtPos σ=.15"]
values=[0.96,0.14,0.42,0.28,0.37]
fig, ax = plt.subplots(figsize=(8,4))
ax.bar(range(len(labels)),values,color=[OKABE[i%len(OKABE)] for i in range(len(labels))])
ax.set(xticks=range(len(labels)),xticklabels=labels,ylabel="Maximum hover deviation (m)",title="Position correction reduces 120 s hover drift")
ax.tick_params(axis="x",rotation=15)
fig.tight_layout(); fig.savefig(OUT / "fig2_hover_correction.png"); plt.close(fig)

# Direct per-run result distribution for square-route tracking RMS by representative methods.
groups=["b_SQ","en_SQ_sq4_s05_l100","en_SQ_sq4_s05_l300","ep_SQ_sq4_s05_l100","ep_SQ_sq4_s05_l300"]
present=[g for g in groups if g in set(df.group)]
fig, ax = plt.subplots(figsize=(8,4.5))
for i,g in enumerate(present):
    vals=pd.to_numeric(df.loc[df.group==g,"track_rms"],errors="coerce").dropna()
    ax.scatter([i]*len(vals),vals,s=30)
    if len(vals): ax.plot([i-.15,i+.15],[vals.median()]*2,color="black",lw=2)
ax.set(xticks=range(len(present)),xticklabels=present,ylabel="Tracking RMS error (m)",title="Per-run square tracking error by correction condition")
ax.tick_params(axis="x",rotation=20)
fig.tight_layout(); fig.savefig(OUT / "fig3_square_tracking.png"); plt.close(fig)
