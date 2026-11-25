#!/usr/bin/env python
"""Figures for the opt_flow (round 1) MTF-01 optical-flow + rangefinder study.

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


def strip(ax, x, values, color, label=None):
    """Dot strip + median tick for repeated runs at position x."""
    values = [v for v in values if pd.notna(v)]
    if not values:
        return
    ax.plot(np.full(len(values), x), values, "o", ms=5, alpha=0.75,
            color=color, label=label, zorder=3)
    ax.plot([x - 0.22, x + 0.22], [np.median(values)] * 2, "-", lw=2.5, color=color, zorder=2)


# ---------------------------------------------------------------- fig2: rangefinder faults
# Section 1 conclusion 4 / its table: fault cases (down-jump, up-jump, out-of-range) x config.
CASES = [("fd3", "fd3p", "DN\n3 s"), ("fd5", "fd5p", "DN\n5 s"),
         ("fu3", "fu3p", "UP\n3 s"), ("fu5", "fu5p", "UP\n5 s"),
         ("fo3", "fo3p", "OOR\n3 s"), ("fo5", "fo5p", "OOR\n5 s")]
base = df.set_index("tag")
x = np.arange(len(CASES))
dev_b = [abs(base.loc[c, "fault_dev_absmax"]) for c, _, _ in CASES]
dev_p = [abs(base.loc[c, "fault_dev_absmax"]) for _, c, _ in CASES]
hgt_b = [base.loc[c, "hgt_est_rms"] for c, _, _ in CASES]
hgt_p = [base.loc[c, "hgt_est_rms"] for _, c, _ in CASES]

fig, axes = plt.subplots(1, 2, figsize=(10, 3.8))
ax = axes[0]
ax.bar(x - 0.2, dev_b, 0.38, color=OKABE[0], label="base params")
ax.bar(x + 0.2, dev_p, 0.38, color=OKABE[1], label="tightened gates")
ax.set_yscale("log")
ax.set_xticks(x, [c[2] for c in CASES])
ax.set_ylabel("max |true-height deviation| (m)")
ax.set_title("True altitude deviation during fault")
ax.legend(loc="upper left")
for xi, vb, vp in zip(x, dev_b, dev_p):
    ax.text(xi - 0.2, vb * 1.15, f"{vb:.2f}", ha="center", fontsize=8)
    ax.text(xi + 0.2, vp * 1.15, f"{vp:.2f}", ha="center", fontsize=8)
ax.set_ylim(top=max(dev_b + dev_p) * 12)  # head room for value labels and legend

ax = axes[1]
ax.bar(x - 0.2, hgt_b, 0.38, color=OKABE[0], label="base params")
ax.bar(x + 0.2, hgt_p, 0.38, color=OKABE[1], label="tightened gates")
ax.set_xticks(x, [c[2] for c in CASES])
ax.set_ylabel("EKF height error RMS (m)")
ax.set_title("Height estimation error (EKF vs truth)")
ax.legend(loc="upper right")
fig.supxlabel("fault: DN = rangefinder reading jumps down 1.5 m, UP = up, OOR = out-of-range (12 m)")
fig.suptitle("Rangefinder fault response: base vs tightened gating (n=1 per bar)", y=1.0)
fig.tight_layout()
fig.savefig(OUT / "fig2_rng_fault.png")
plt.close(fig)

# ---------------------------------------------------------------- fig3: hover drift per config
# Section 2.1 table: single runs, one per configuration.
HOVER = [("h1a", "base, no wind, yaw 0"), ("h1b", "base, no wind, yaw 90"),
         ("h1d", "flow noise 0.2 rad/s"), ("hn_base", "flow noise 0.2, yaw 45"),
         ("hn_m01", "+ EK3_FLOW_M_NSE 0.1"), ("hn_m05", "+ EK3_FLOW_M_NSE 0.5"),
         ("hn_m10", "+ EK3_FLOW_M_NSE 1.0"), ("ofs_base", "wind 2 m/s"),
         ("ofs_raw", "wind + rng offset, FC unaware"), ("ofs_fixed", "wind + RNGFND1_POS_Z set"),
         ("fpos_raw", "wind + flow sensor 5 cm fwd"), ("hp_rngnse", "EK3_RNG_M_NSE 0.1"),
         ("hp_rnggate", "EK3_RNG_I_GATE 200"), ("hp_hgtgate", "EK3_HGT_I_GATE 200"),
         ("hp_hgtdelay", "EK3_HGT_DELAY 100")]
labels = [f"{t}\n({d})" for t, d in HOVER]
dm = [base.loc[t, "true_drift_max"] for t, _ in HOVER]
pe = [base.loc[t, "pos_est_err_max"] for t, _ in HOVER]
y = np.arange(len(HOVER))
order = np.argsort(dm)

fig, axes = plt.subplots(1, 2, figsize=(10, 5.6))
for ax, vals, ttl, xm in ((axes[0], dm, "True max hover drift", 1.9),
                          (axes[1], pe, "EKF-minus-true max position error", 2.3)):
    o = np.argsort(vals)
    ax.barh(y[o], np.array(vals)[o], color=OKABE[0], alpha=0.85)
    ax.set_yticks(y[o], [labels[i] for i in o], fontsize=7.5)
    ax.set_xlabel("distance (m)")
    ax.set_title(ttl)
    ax.set_xlim(0, xm)
    for yi, v in zip(y[o], np.array(vals)[o]):
        ax.text(v + 0.02, yi, f"{v:.2f}", va="center", fontsize=7.5)
fig.suptitle("Hover drift by configuration (single runs, LOITER 120 s)", y=1.0)
fig.tight_layout()
fig.savefig(OUT / "fig3_hover_drift_configs.png")
plt.close(fig)

# ---------------------------------------------------------------- fig1: A/B harsh-scenario repeats
# Section 1 conclusion 1 table: base / recA / recB repeats under the harsh scenario.
GROUPS = [("ab_base_r1", "ab_base_r2", "ab_base_r3"),
          ("ab_recA_r1", "ab_recA_r2", "ab_recA_r3"),
          ("ab_recB_r1", "ab_recB_r2")]
names = ["base (n=3)", "recA (n=3)", "recB (n=2)"]
fig, axes = plt.subplots(1, 2, figsize=(9, 3.8))
for ax, col, ttl in ((axes[0], "true_drift_final", "True end-point drift"),
                     (axes[1], "true_drift_max", "True max drift")):
    for i, tags in enumerate(GROUPS):
        vals = [base.loc[t, col] for t in tags]
        strip(ax, i, vals, OKABE[i])
    ax.set_xticks(range(len(GROUPS)), names)
    ax.set_ylabel("drift (m)")
    ax.set_title(ttl)
axes[0].set_xlim(-0.6, 2.6)
fig.suptitle("Harsh scenario (wind + scale + noise): repeats overlap, arms indistinguishable", y=1.02)
fig.tight_layout()
fig.savefig(OUT / "fig1_ab_harsh_repeats.png")
plt.close(fig)

# ---------------------------------------------------------------- fig4: mission tracking per scenario
# Section 2.2 table: AUTO square mission, tracking RMS, pass/fail.
mis = df[df["tag"].str.startswith("mis_")].copy()
mis["ok"] = mis["mission_complete"].astype(str).str.lower() == "true"
mis = mis.sort_values("track_rms")
fig, ax = plt.subplots(figsize=(8, 3.8))
colors = [OKABE[2] if ok else OKABE[3] for ok in mis["ok"]]
ax.bar(mis["tag"], mis["track_rms"], color=colors)
for t, v, ok in zip(mis["tag"], mis["track_rms"], mis["ok"]):
    ax.text(t, v + 0.01, f"{v:.3f}", ha="center", fontsize=8)
ax.set_ylabel("tracking RMS (m)")
ax.set_title("AUTO 2 m square, 3 laps: tracking RMS (green = mission completed)")
plt.setp(ax.get_xticklabels(), rotation=30, ha="right")
from matplotlib.patches import Patch
ax.legend(handles=[Patch(color=OKABE[2], label="completed"),
                   Patch(color=OKABE[3], label="fence / aborted")], loc="upper left")
fig.tight_layout()
fig.savefig(OUT / "fig4_mission_tracking.png")
plt.close(fig)

print("wrote", sorted(p.name for p in OUT.glob("*.png")))
