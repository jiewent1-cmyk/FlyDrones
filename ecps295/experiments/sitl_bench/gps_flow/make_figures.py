#!/usr/bin/env python
"""Figures for the gps_flow (GPS + compass architecture) SITL study.

Reads results.csv next to this file, writes PNGs to figures/.
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
df = pd.read_csv(HERE / "results.csv")


def strip(ax, x, values, color, label=None, ms=5):
    values = [v for v in values if pd.notna(v)]
    if not values:
        return
    ax.plot(np.full(len(values), x), values, "o", ms=ms, alpha=0.7, color=color, label=label, zorder=3)
    ax.plot([x - 0.22, x + 0.22], [np.median(values)] * 2, "-", lw=2.5, color=color, zorder=2)


# ---------------------------------------------------------------- fig1: compass quality vs tracking
# Report section 2 table: optical-flow tracking vs in-flight heading error.
CFG = [("F0", OKABE[0]), ("FM", OKABE[2]), ("FMdec10", OKABE[5]),
       ("FMdec20", OKABE[1]), ("FMmot2", OKABE[3]), ("FMmot6", OKABE[4])]
fig, ax = plt.subplots(figsize=(7, 4))
for name, color in CFG:
    d = df[df["config"] == name]
    ax.scatter(d["yaw_change"], d["track_rms"], s=45, color=color, alpha=0.8,
               label=name, zorder=3)
    ax.scatter([d["yaw_change"].median()], [d["track_rms"].median()], s=130,
               facecolors="none", edgecolors=color, lw=1.8, zorder=4)
ax.set_xscale("log")
ax.set_xlabel("in-flight heading error change (deg, log scale)")
ax.set_ylabel("tracking RMS (m)")
ax.set_title("Compass errors vs optical-flow tracking (2 m square, n=3 each;\n"
             "open circles = medians)")
ax.legend(loc="upper left", fontsize=8)
fig.tight_layout()
fig.savefig(OUT / "fig1_compass.png")
plt.close(fig)

# ---------------------------------------------------------------- fig2: GPS quality and loss
# Report sections 1/3 tables: architecture and GPS-quality/loss configurations.
ORDER = [("IF_poor", "flow set, poor GPS"),
         ("IGfa_poor", "GPS+flow vel, poor"),
         ("II_poor", "arch ii, poor"),
         ("IG_good", "GPS set, good"),
         ("IG_poor", "GPS set, poor"),
         ("IG_step3", "GPS 3 m step"),
         ("IG_step3_gr25", "step+GLITCH 25"),
         ("IG_step3_pg300", "step+PIG 300"),
         ("IGfa_loss", "lost, flow fb"),
         ("II_loss", "arch ii, lost"),
         ("IG_loss", "lost, no fb"),
         ("Q5_in_org", "indoor + origin"),
         ("Q5_in_noorg", "indoor no org"),
         ("GSF_good", "GSF no compass")]
fig, ax = plt.subplots(figsize=(10, 4.2))
for i, (name, lab) in enumerate(ORDER):
    d = df[df["config"] == name]
    ok = d["mission_complete"].astype(str).str.lower() == "true"
    vals_ok = d.loc[ok, "track_rms"].dropna().tolist()
    strip(ax, i, vals_ok, OKABE[2])
    vals_bad = d.loc[~ok, "track_rms"].dropna().tolist()
    strip(ax, i, vals_bad, OKABE[3])
    if d["track_rms"].dropna().empty:
        ax.text(i, 0.15, "0/3 armed", ha="center", fontsize=8, color=OKABE[3], rotation=90)
ax.set_xticks(range(len(ORDER)), [lab for _, lab in ORDER], fontsize=8)
plt.setp(ax.get_xticklabels(), rotation=30, ha="right")
ax.set_ylabel("tracking RMS (m)")
ax.set_ylim(0, 5.2)
ax.set_title("Tracking RMS by GPS configuration (n=3; green = mission completed)")
from matplotlib.patches import Patch
ax.legend(handles=[Patch(color=OKABE[2], label="completed"),
                   Patch(color=OKABE[3], label="failed / aborted")], loc="upper left")
fig.tight_layout()
fig.savefig(OUT / "fig2_gps_quality.png")
plt.close(fig)

# ---------------------------------------------------------------- fig3: source-set switching
# Report section 4 table: EKF position jump and true displacement after switching.
SW = [("SW_G2F", "GPS to flow"), ("SW_F2G", "flow to GPS\n(natural offset)"),
      ("SW_F2G_d1", "flow to GPS\n(+1 m offset)"), ("SW_F2G_d3", "flow to GPS\n(+3 m offset)")]
fig, axes = plt.subplots(1, 2, figsize=(9, 3.8))
for ax, col, ttl, ylab in ((axes[0], "sw1_ekf_minus_true_jump_2s", "EKF position jump at switch",
                            "jump (m)"),
                           (axes[1], "sw1_true_move_10s", "True displacement within 10 s",
                            "displacement (m)")):
    for i, (name, lab) in enumerate(SW):
        vals = df.loc[df["config"] == name, col].dropna().tolist()
        strip(ax, i, vals, OKABE[i % 8])
    ax.set_xticks(range(len(SW)), [lab for _, lab in SW], fontsize=8)
    ax.set_ylabel(ylab)
    ax.set_title(ttl)
fig.suptitle("Switching EKF source set at WP2 hold (n=3 each)", y=1.02)
fig.tight_layout()
fig.savefig(OUT / "fig3_switching.png")
plt.close(fig)

print("wrote", sorted(p.name for p in OUT.glob("*.png")))
