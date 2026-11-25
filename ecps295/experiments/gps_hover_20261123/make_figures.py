#!/usr/bin/env python
"""Figures for the gps_hover_20261123 (GPS + compass hover drift) study.

Reads results_all.csv, results_h15_.csv, results_ft_.csv next to this file,
writes PNGs to figures/.
Run: python make_figures.py
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pathlib import Path

OKABE = ["#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#56B4E9", "#F0E442", "#000000"]
plt.rcParams.update({"figure.dpi": 150, "savefig.dpi": 150, "savefig.bbox": "tight",
    "font.size": 10, "axes.titlesize": 11, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.alpha": 0.3, "axes.prop_cycle": matplotlib.cycler(color=OKABE),
    "figure.facecolor": "white"})

HERE = Path(__file__).parent
OUT = HERE / "figures"
OUT.mkdir(exist_ok=True)
FLOWS = ["N05", "N15", "N25"]
GPSS = ["Gg", "Gm", "Gp"]
GPSLAB = {"Gg": "good GPS", "Gm": "medium GPS", "Gp": "poor GPS"}
a = pd.read_csv(HERE / "results_all.csv")
h = pd.read_csv(HERE / "results_h15_.csv")
ft = pd.read_csv(HERE / "results_ft_.csv")


def strip(ax, x, values, color, ms=5, alpha=0.7):
    values = [v for v in values if pd.notna(v)]
    if not values:
        return
    ax.plot(np.full(len(values), x), values, "o", ms=ms, alpha=alpha, color=color, zorder=3)
    ax.plot([x - 0.22, x + 0.22], [np.median(values)] * 2, "-", lw=2.5, color=color, zorder=2)


def grid(df, arms, flows=FLOWS, gps=GPSS, col="exc_max"):
    """Median exc_max [arm][flow][gps]; single-GPS arms broadcast across gps."""
    out = {}
    for arm in arms:
        d = df[df["arm"] == arm]
        seen = sorted(d["gps"].unique())
        for f in flows:
            for g in gps:
                rows = d[(d["flow"] == f) & (d["gps"] == g)]
                if rows.empty and len(seen) == 1:
                    rows = d[d["flow"] == f]  # GPS-independent arm: broadcast
                out[(arm, f, g)] = rows[col].median() if len(rows) else np.nan
    return out


# ---------------------------------------------------------------- fig1: 5-minute round 1 heatmap
# Report section 3.1 table: median exc_max, 5 reps per cell.
ARMS1 = ["F", "G", "GF", "FV", "A05", "A10", "A20", "A40"]
g1 = grid(a, ARMS1)
fig, axes = plt.subplots(1, 3, figsize=(10, 3.6), sharey=True)
for ax, f in zip(axes, FLOWS):
    M = np.array([[g1[(arm, f, g)] for g in GPSS] for arm in ARMS1])
    im = ax.imshow(M, cmap="viridis", aspect="auto", vmin=0.2, vmax=2.6)
    ax.set_xticks(range(3), ["good", "medium", "poor"], fontsize=9)
    ax.set_xlabel("GPS quality")
    for i in range(len(ARMS1)):
        for j in range(3):
            v = M[i, j]
            ax.text(j, i, "–" if np.isnan(v) else f"{v:.2f}", ha="center", va="center",
                    fontsize=7.5, color="white" if v > 1.6 else "black")
    ax.set_title(f)
axes[0].set_yticks(range(len(ARMS1)), ARMS1)
fig.colorbar(im, ax=axes, shrink=0.85, label="median max offset (m)")
fig.suptitle("5-minute hover, round 1 (n=5 per cell; F row: no GPS used, "
             "same runs in all columns)", y=1.04)
fig.savefig(OUT / "fig1_round1_heatmap.png")
plt.close(fig)

# ---------------------------------------------------------------- fig2: 15-minute heatmap
# Report section 3.2 table: median exc_max, 10 reps per cell.
ARMS2 = ["F", "G", "A20", "B120", "D05_030", "D10_030", "D10_060", "D10_120",
         "D15_030", "D15_060", "D20_030", "J10_030", "J10_060", "J10_120"]
g2 = grid(h, ARMS2)
M = np.array([[g2[(arm, f, g)] for f in FLOWS for g in GPSS] for arm in ARMS2])
fig, ax = plt.subplots(figsize=(10, 4.6))
im = ax.imshow(M, cmap="viridis", aspect="auto", vmin=0.3, vmax=4.2)
ax.set_xticks(range(9), [f"{f}\n{GPSLAB[g]}" for f in FLOWS for g in GPSS], fontsize=7.5)
ax.set_yticks(range(len(ARMS2)), ARMS2)
for i in range(len(ARMS2)):
    for j in range(9):
        ax.text(j, i, f"{M[i, j]:.2f}", ha="center", va="center", fontsize=7,
                color="white" if M[i, j] > 2.6 else "black")
fig.colorbar(im, ax=ax, shrink=0.85, label="median max offset (m)")
ax.set_title("15-minute hover: median max offset (n=10 per cell; F row: no GPS used)")
fig.savefig(OUT / "fig2_h15_heatmap.png")
plt.close(fig)

# ---------------------------------------------------------------- fig3: drift vs flow noise
# Report section 3.2 interpretation: G depends only on GPS quality; slow correction caps drift.
SHOW = ["F", "G", "B120", "D10_060", "J10_060"]
COLORS = dict(zip(SHOW, OKABE))
g3 = grid(h, SHOW)
fig, axes = plt.subplots(1, 3, figsize=(10, 3.4), sharey=True)
for ax, g in zip(axes, GPSS):
    for arm in SHOW:
        ax.plot(FLOWS, [g3[(arm, f, g)] for f in FLOWS], "-o", color=COLORS[arm],
                label=arm, ms=5)
    ax.set_title(GPSLAB[g], fontsize=10)
    ax.set_xticks(FLOWS)
    ax.set_xlabel("flow noise")
axes[0].set_ylabel("median max offset (m)")
axes[0].legend(loc="upper left", fontsize=8)
fig.suptitle("15-minute hover: drift vs flow noise, per GPS quality (n=10)", y=1.04)
fig.savefig(OUT / "fig3_drift_vs_noise.png")
plt.close(fig)

# ---------------------------------------------------------------- fig4: 15-min N15 distributions
# Spread behind the section 3.2 medians, for the N15 column.
fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
for ax, g in zip(axes, GPSS):
    for i, arm in enumerate(ARMS2):
        d = h[(h["arm"] == arm) & (h["flow"] == "N15")]
        if not (d["gps"] == g).any() and d["gps"].nunique() == 1:
            pass  # GPS-independent arm (F): same runs in every panel
        else:
            d = d[d["gps"] == g]
        strip(ax, i, d["exc_max"].tolist(), OKABE[3] if arm in ("F", "G") else OKABE[0], ms=6)
    ax.set_ylabel("max offset (m)")
    ax.set_title(GPSLAB[g])
    ax.tick_params(labelbottom=True)
    ax.set_xticks(range(len(ARMS2)), ARMS2, rotation=30, ha="right")
axes[-1].set_xlabel("scheme")
axes[0].set_xlim(-0.7, len(ARMS2) - 0.3)
axes[0].legend(handles=[plt.Line2D([], [], marker="o", ls="", color=OKABE[3], label="reference (F, G)"),
                        plt.Line2D([], [], marker="o", ls="", color=OKABE[0], label="other schemes")],
               loc="upper right")
fig.suptitle("15-minute hover, flow noise N15: per-run max offset (dots, n=10) and medians (bars)\n"
             "F shown in all panels is the same data")
fig.tight_layout()
fig.savefig(OUT / "fig4_h15_n15_dists.png")
plt.close(fig)

# ---------------------------------------------------------------- fig5: repeatability D vs J
# Report section 3.3 table: paired |D - J| differences by time constant.
fig, ax = plt.subplots(figsize=(7, 3.8))
rng = np.random.default_rng(0)
for i, tau in enumerate(["030", "060", "120"]):
    d = h[h["arm"] == "D10_" + tau]; j = h[h["arm"] == "J10_" + tau]
    m = d.merge(j, on=["flow", "gps", "rep"], suffixes=("_d", "_j"))
    diff = (m["exc_max_d"] - m["exc_max_j"]).abs()
    ax.plot(i + rng.uniform(-0.18, 0.18, len(diff)), diff, "o", ms=4, alpha=0.35,
            color=OKABE[0], zorder=3)
    ax.plot([i - 0.25, i + 0.25], [diff.median()] * 2, "-", lw=2.5, color=OKABE[3], zorder=4)
    ax.text(i + 0.3, diff.median(), f"{diff.median():.2f} m", va="center", fontsize=8.5)
ax.set_xticks(range(3), ["τ 30 s (90 pairs)", "τ 60 s (90 pairs)", "τ 120 s (90 pairs)"])
ax.set_ylabel("|paired difference| (m)")
ax.set_ylim(0, None)
ax.set_title("Repeatability: J equals D without jumps, same random seeds —\n"
             "repeated runs still differ (orange = median)")
fig.tight_layout()
fig.savefig(OUT / "fig5_repeatability.png")
plt.close(fig)

# ---------------------------------------------------------------- fig6: merged slow correction vs F
# Report section 3.3 table: D10_060/J10_060/D10_120/J10_120 averaged per rep, paired with F.
MERGED = ["D10_060", "J10_060", "D10_120", "J10_120"]
fdat = h[h["arm"] == "F"].set_index(["flow", "gps", "rep"])["exc_max"]
mdat = h[h["arm"].isin(MERGED)].groupby(["flow", "gps", "rep"])["exc_max"].mean()
fig, ax = plt.subplots(figsize=(10, 4))
conds = [(f, g) for f in FLOWS for g in GPSS]
reps = sorted(h["rep"].unique())  # 'r00'..'r09'
for i, (f, g) in enumerate(conds):
    for rep in reps:
        vf, vm = fdat.get((f, "Gm", rep)), mdat.get((f, g, rep))  # F runs once per rep, GPS-independent
        if vf is None or pd.isna(vf) or pd.isna(vm):
            continue
        ax.plot([i, i], [vf, vm], "-", color="#bbbbbb", lw=0.9, zorder=1)
        ax.plot(i, vf, "o", ms=5, color=OKABE[0], alpha=0.75, zorder=3)
        ax.plot(i, vm, "o", ms=5, color=OKABE[1], alpha=0.75, zorder=3)
ax.legend(handles=[plt.Line2D([], [], marker="o", ls="", color=OKABE[0], label="F"),
                   plt.Line2D([], [], marker="o", ls="", color=OKABE[1], label="merged slow corr.")],
          loc="upper left")
ax.set_xticks(range(len(conds)), [f"{f}\n{GPSLAB[g]}" for f, g in conds], fontsize=7.5)
ax.set_ylabel("max offset (m)")
ax.set_ylim(-0.3, None)
ax.set_title("Merged slow correction (mean of D10_060/J10_060/D10_120/J10_120) vs F,\n"
             "paired by repetition: no cell reaches significance (Wilcoxon, n=10)")
fig.tight_layout()
fig.savefig(OUT / "fig6_merged_vs_F.png")
plt.close(fig)

# ---------------------------------------------------------------- fig7: fault tests
# Report section 3.4 table: exc_max by fault x scheme, n=8 each (N15, Gm).
FAULTS = [("none", "none"), ("step3", "step 3 m\ncontinuous"), ("step3t", "step 3 m\nrecover 60 s"),
          ("loss", "GPS lost"), ("lossret", "lost, back\nafter 120 s"), ("mag6", "compass\ninterference")]
SCHEMES = [("F", OKABE[0]), ("G", OKABE[1]), ("D10_060", OKABE[2]), ("J10_060", OKABE[3])]
fig, ax = plt.subplots(figsize=(10, 4))
for i, (fk, flab) in enumerate(FAULTS):
    for j, (arm, col) in enumerate(SCHEMES):
        s = ft[ft["arm"].str.startswith(fk + "_") & (ft["arm"].str.split("_", n=1).str[1] == arm)]
        strip(ax, i + (j - 1.5) * 0.18, s["exc_max"].tolist(), col, ms=4.5)
ax.set_xticks(range(len(FAULTS)), [flab for _, flab in FAULTS], fontsize=8)
ax.set_ylabel("max offset (m)")
ax.legend(handles=[plt.Line2D([], [], marker="o", ls="", color=c, label=a) for a, c in SCHEMES],
          loc="upper left", fontsize=8)
ax.set_title("Fault tests (N15, medium GPS, fault at 120 s of 400 s hover, n=8 per cell):\n"
             "G fails all 16 GPS-loss flights; slow-correction schemes stay near F")
fig.tight_layout()
fig.savefig(OUT / "fig7_faults.png")
plt.close(fig)

# ---------------------------------------------------------------- fig8: compass
# Report section 3.5 table: yaw change and drift for F0/F/FM2/FM6.
CARMS = ["F0", "F", "FM2", "FM6"]
CCOL = dict(zip(CARMS, OKABE))
fig, axes = plt.subplots(1, 2, figsize=(10, 3.8), gridspec_kw={"width_ratios": [2, 3]})
ax = axes[0]
for i, arm in enumerate(CARMS):
    strip(ax, i, h[h["arm"] == arm]["yaw_chg_max"].tolist(), CCOL[arm], ms=4)
ax.set_xticks(range(4), ["F0\nno compass", "F\ngood compass", "FM2\ninterference", "FM6\nmore"], fontsize=8)
ax.set_ylabel("in-flight heading change (deg)")
ax.set_title("Heading change during 15 min hover")
ax = axes[1]
for i, f in enumerate(FLOWS):
    for j, arm in enumerate(CARMS):
        d = h[(h["arm"] == arm) & (h["flow"] == f)]
        strip(ax, i + (j - 1.5) * 0.18, d["exc_max"].tolist(), CCOL[arm], ms=4.5)
ax.set_xticks(range(3), [f"{f}\nflow noise" for f in FLOWS], fontsize=8)
ax.set_ylabel("max offset (m)")
ax.set_title("Max offset (GPS quality irrelevant to these arms)")
axes[0].legend(handles=[plt.Line2D([], [], marker="o", ls="", color=CCOL[a], label=a) for a in CARMS],
               loc="upper left", fontsize=8)
fig.suptitle("Compass: it bounds heading drift; position drift is unaffected (n=30 per arm)", y=1.02)
fig.tight_layout()
fig.savefig(OUT / "fig8_compass.png")
plt.close(fig)

print("wrote", sorted(p.name for p in OUT.glob("*.png")))
