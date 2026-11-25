#!/usr/bin/env python
"""Figures for the opt_flow2 (round 2) optical-flow + rangefinder parameter study.

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
C = "#0072B2"


def strip(ax, x, values, color, label=None, ms=5, hw=0.25):
    """Dot strip + median tick for repeated runs at position x."""
    values = [v for v in values if pd.notna(v)]
    if not values:
        return
    ax.plot(np.full(len(values), x), values, "o", ms=ms, alpha=0.7,
            color=color, label=label, zorder=3)
    ax.plot([x - hw, x + hw], [np.median(values)] * 2, "-", lw=2.5, color=color, zorder=2)


# ---------------------------------------------------------------- fig1: hover drift vs disturbance
# Report section 2.1: 2 m hover 120 s, true max drift per disturbance group.
GROUPS = [("m_H2_C0", "C0\nnone"), ("m_H2_HDG", "HDG\nheading"), ("m_H2_WIND", "WIND\n1.5 m/s"),
          ("m_H2_FNSE", "FNSE\nflow noise"), ("m_H2_FSCP", "FSCP\nscale +5%"),
          ("m_H2_FSCM", "FSCM\nscale -5%"), ("m_H2_RNSE", "RNSE\nrng noise"),
          ("m_H2_ALL", "ALL\nall of\nabove"), ("x_H2_TURB10", "TURB 10\ngusts\n~10 m/s")]
fig, ax = plt.subplots(figsize=(8, 3.8))
for i, (g, lab) in enumerate(GROUPS):
    vals = df.loc[df["group"] == g, "hov_drift_max"].tolist()
    strip(ax, i, vals, OKABE[3] if g.endswith("TURB10") else C)
ax.set_xticks(range(len(GROUPS)), [lab for _, lab in GROUPS], fontsize=8)
ax.set_ylabel("true max drift (m)")
ax.set_ylim(top=ax.get_ylim()[1] + 0.15)  # room for the legend above the data
ax.legend(handles=[plt.Line2D([], [], marker="o", ls="", color=C, label="disturbance groups of the study"),
                   plt.Line2D([], [], marker="o", ls="", color=OKABE[3],
                              label="TURB 10: extreme-gust reference (round 1 setting)")],
          loc="upper left", fontsize=8)
ax.set_title("2 m hover, 120 s: max drift by disturbance (n=3, FNSE/ALL n=6)")
fig.tight_layout()
fig.savefig(OUT / "fig1_hover_disturbances.png")
plt.close(fig)

# ---------------------------------------------------------------- fig2: square mission vs disturbance
# Report section 2.2: 2 m square, tracking RMS per disturbance group.
GROUPS = [("m_SQ_C0", "C0\nnone"), ("m_SQ_HDG", "HDG\nheading"), ("m_SQ_WIND", "WIND\n1.5 m/s"),
          ("m_SQ_FNSE", "FNSE\nflow noise"), ("m_SQ_FSCP", "FSCP\nscale +5%"),
          ("m_SQ_FSCM", "FSCM\nscale -5%"), ("m_SQ_RNSE", "RNSE\nrng noise"),
          ("m_SQ_ALL", "ALL\nall of\nabove"), ("x_SQ_TURB10", "TURB 10\ngusts\n~10 m/s")]
fig, ax = plt.subplots(figsize=(8, 3.8))
for i, (g, _) in enumerate(GROUPS):
    vals = df.loc[df["group"] == g, "track_rms"].tolist()
    strip(ax, i, vals, OKABE[3] if g.endswith("TURB10") else C)
ax.set_xticks(range(len(GROUPS)), [lab for _, lab in GROUPS], fontsize=8)
ax.set_ylabel("tracking RMS (m)")
ax.legend(handles=[plt.Line2D([], [], marker="o", ls="", color=C, label="disturbance groups of the study"),
                   plt.Line2D([], [], marker="o", ls="", color=OKABE[3],
                              label="TURB 10: extreme-gust reference (round 1 setting)")],
          loc="upper left", fontsize=8)
ax.set_title("2 m square mission: tracking RMS by disturbance (n=3, ALL n=6)")
fig.tight_layout()
fig.savefig(OUT / "fig2_square_disturbances.png")
plt.close(fig)

# ---------------------------------------------------------------- fig3: A/B normal-flight cost
# Report section 2.4: baseline (m_*_ALL) vs recH vs recHF under ALL disturbances.
ARMS = [("m_H{h}_ALL", "original"), ("ab_H{h}_ALL_recH", "recH"), ("ab_H{h}_ALL_recHF", "recHF")]
fig, axes = plt.subplots(1, 2, figsize=(10, 3.8), gridspec_kw={"width_ratios": [3, 2]})
ax = axes[0]
pos, labs = [], []
i = 0
for h, hlab in (("1", "H1 1 m (n=3)"), ("2", "H2 2 m (n=6)")):
    for pat, lab in ARMS:
        vals = df.loc[df["group"] == pat.format(h=h), "hov_drift_max"].tolist()
        strip(ax, i, vals, OKABE[["original", "recH", "recHF"].index(lab)])
        pos.append(i)
        labs.append(f"{hlab}\n{lab}")
        i += 1
ax.set_xticks(range(len(labs)), labs, fontsize=8)
ax.set_ylabel("true max drift (m)")
ax.set_title("Hover: max drift")
ax = axes[1]
for i, (pat, lab) in enumerate([("m_SQ_ALL", "original"), ("ab_SQ_ALL_recH", "recH"), ("ab_SQ_ALL_recHF", "recHF")]):
    vals = df.loc[df["group"] == pat, "track_rms"].tolist()
    strip(ax, i, vals, OKABE[i])
ax.set_xticks(range(3), [f"SQ 2 m (n=6)\n{lab}" for _, lab in ARMS], fontsize=8)
ax.set_ylabel("tracking RMS (m)")
ax.set_title("Square mission: tracking RMS")
fig.suptitle("A/B under ALL disturbances: recommended params give no normal-flight gain", y=1.02)
fig.tight_layout()
fig.savefig(OUT / "fig3_ab_normal_flight.png")
plt.close(fig)

# ---------------------------------------------------------------- fig4: rangefinder faults in AUTO
# Report section 2.5: f_def_* (original) vs f_p1_* (tightened) vs f_p1b_* (no RNG_I_GATE change).
import re
faults = [("DN1", "DN\n1 s"), ("DN3", "DN\n3 s"), ("DN5", "DN\n5 s"), ("DN10", "DN\n10 s"),
          ("UP1", "UP\n1 s"), ("UP3", "UP\n3 s"), ("UP5", "UP\n5 s"), ("UP10", "UP\n10 s"),
          ("OOR1", "OOR\n1 s"), ("OOR3", "OOR\n3 s"), ("OOR5", "OOR\n5 s")]
COLORS = {"def": OKABE[0], "p1": OKABE[1], "p1b": OKABE[2]}
OFFS = {"def": -0.25, "p1": 0.0, "p1b": 0.25}
fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.0))
for ax, col, ttl, ylab in ((axes[0], "f_true_max", "Highest true altitude during/after fault",
                            "true altitude (m)"),
                           (axes[1], "touchdown_mps", "Touchdown vertical speed",
                            "vertical speed (m/s)")):
    for xi, (f, _) in enumerate(faults):
        for cfg in ("def", "p1", "p1b"):
            g = f"f_{cfg}_{f}"
            vals = df.loc[df["group"] == g, col].dropna().tolist()
            strip(ax, xi + OFFS[cfg], vals, COLORS[cfg], ms=4.5, hw=0.1)
    ax.set_xticks(range(len(faults)), [lab for _, lab in faults])
    ax.set_ylabel(ylab)
    ax.set_title(ttl)
axes[0].set_ylim(top=axes[0].get_ylim()[1] + 2.0)  # room for the legend above the data
axes[0].legend(handles=[plt.Line2D([], [], marker="o", ls="", color=COLORS[c], label=l) for c, l in
                          (("def", "def: original params"), ("p1", "p1: tightened gates"),
                           ("p1b", "p1b: tightened, RNG_I_GATE unchanged"))],
               loc="upper left", fontsize=8)
fig.supxlabel("fault: DN = reading jumps down 1.5 m, UP = up, OOR = out-of-range (12 m)")
fig.suptitle("AUTO 2 m hover with rangefinder faults: default vs tightened gates (n=3)", y=1.02)
fig.tight_layout()
fig.savefig(OUT / "fig4_rng_faults.png")
plt.close(fig)

print("wrote", sorted(p.name for p in OUT.glob("*.png")))

# sanity check against report table 2.1 / 2.4 medians
for g in ["m_H2_C0", "m_H2_FNSE", "m_H2_ALL", "x_H2_TURB10", "m_SQ_C0", "m_SQ_FNSE",
          "ab_H2_ALL_recH", "ab_H2_ALL_recHF"]:
    s = df.loc[df["group"] == g]
    col = "track_rms" if g.startswith(("m_SQ", "ab_SQ")) else "hov_drift_max"
    print(g, "n=", len(s), "median", round(s[col].median(), 3),
          "min", round(s[col].min(), 3), "max", round(s[col].max(), 3))
