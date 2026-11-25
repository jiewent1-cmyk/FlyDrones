#!/usr/bin/env python
"""Figures for the mtf01 (MTF-01 optical flow + rangefinder height) SITL study.

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


def sel(prefix):
    return df[df["tag"].str.startswith(prefix)]


def strip(ax, x, values, color, label=None, ms=5):
    values = [v for v in values if pd.notna(v)]
    if not values:
        return
    ax.plot(np.full(len(values), x), values, "o", ms=ms, alpha=0.7, color=color, label=label, zorder=3)
    ax.plot([x - 0.22, x + 0.22], [np.median(values)] * 2, "-", lw=2.5, color=color, zorder=2)


# ---------------------------------------------------------------- fig1: height source comparison
# Report section 2 table rows q2_B / q2_RNG / q2_BARO / q2_BRNG (n=3 each).
SRC = [("q2_B_", "a: companion\nvision hgt"), ("q2_RNG", "b: rangefinder\nhgt"),
       ("q2_BARO", "c: barometer\nhgt"), ("q2_BRNG", "c + RNG_USE\nHGT 70 mix")]
fig, axes = plt.subplots(1, 2, figsize=(9, 3.8))
for ax, col, ttl in ((axes[0], "alt_rms", "Mission altitude error RMS (truth vs task height)"),
                     (axes[1], "hgt_est_rms", "EKF height error RMS (truth, above 0.2 m)")):
    for i, (p, lab) in enumerate(SRC):
        strip(ax, i, sel(p)[col].tolist(), OKABE[i])
    ax.set_xticks(range(len(SRC)), [lab for _, lab in SRC], fontsize=8)
    ax.set_ylabel("RMS (m)")
    ax.set_title(ttl, fontsize=9.5)
fig.suptitle("Height source, 2 m square mission (n=3 each)", y=1.02)
fig.tight_layout()
fig.savefig(OUT / "fig1_height_source.png")
plt.close(fig)

# ---------------------------------------------------------------- fig2: flow scale sweep
# Report section 2 table rows q4scale_* with q2_RNG as the 0% baseline.
pts = [(-20, "q4scale_-200"), (-10, "q4scale_-100"), (0, "q2_RNG"), (10, "q4scale_100"), (20, "q4scale_200")]
fig, ax = plt.subplots(figsize=(7, 3.8))
for pct, p in pts:
    strip(ax, pct, sel(p)["track_rms"].tolist(), OKABE[3] if pct < 0 else OKABE[0], ms=6)
med = [sel(p)["track_rms"].median() for _, p in pts]
ax.plot([p for p, _ in pts], med, "-o", color="#000000", lw=1.5, ms=4, label="median", zorder=2)
ax.set_xlabel("flow scale error FLOW_FXSCALER/FYSCALER (%)")
ax.set_ylabel("tracking RMS (m)")
ax.set_xticks([-20, -10, 0, 10, 20])
ax.set_title("2 m square with optical-flow scale error (n=3 each; 0% = q2_RNG)")
ax.legend(loc="upper left")
fig.tight_layout()
fig.savefig(OUT / "fig2_flow_scale.png")
plt.close(fig)

# ---------------------------------------------------------------- fig3: rangefinder noise sweep
# Report section 2 table rows q4rnd_0.02 / 0.05 / 0.10.
NOISE = [("q4rnd_0.02", "0.02"), ("q4rnd_0.05", "0.05"), ("q4rnd_0.10", "0.10")]
fig, axes = plt.subplots(1, 2, figsize=(9, 3.8))
for ax, col, ttl in ((axes[0], "alt_rms", "Mission altitude error RMS"),
                     (axes[1], "hover_hgt_est_rms", "Hover EKF height error RMS (15 s)")):
    for i, (p, lab) in enumerate(NOISE):
        strip(ax, i, sel(p)[col].tolist(), OKABE[i])
    ax.set_xticks(range(len(NOISE)), [f"±{lab} m\n(SIM_SONAR_RND)" for _, lab in NOISE], fontsize=8)
    ax.set_ylabel("RMS (m)")
    ax.set_title(ttl, fontsize=9.5)
fig.suptitle("Rangefinder noise, uniform amplitude, 2 m square (n=3 each)", y=1.02)
fig.tight_layout()
fig.savefig(OUT / "fig3_rng_noise.png")
plt.close(fig)

# ---------------------------------------------------------------- fig4: rangefinder faults
# Report section 2 table rows q4oor1/oor5/jmp1/jmp5 x {B, RNG}.
FAULTS = [("q4oor1_", "OOR\n1 s"), ("q4oor5_", "OOR\n5 s"),
          ("q4jmp1_", "jump 1.5 m\n1 s"), ("q4jmp5_", "jump 1.5 m\n5 s")]
fig, ax = plt.subplots(figsize=(9, 4.4))
SRCMARK = (("a", "o"), ("b", "s"))
ticks, ticklabs = [], []
for i, (p, lab) in enumerate(FAULTS):
    for j, sfx in enumerate(("B", "RNG")):
        d = sel(p + sfx)
        ok = d["complete"].astype(str).str.lower() == "true"
        xs = i + (-0.18 if j == 0 else 0.18)
        ticks.append(xs)
        ticklabs.append(SRCMARK[j][0])
        for _, r in d.iterrows():
            col = OKABE[2] if str(r["complete"]).lower() == "true" else OKABE[3]
            ax.plot(xs, r["alt_max"], SRCMARK[j][1], ms=7, alpha=0.8, color=col, zorder=3)
        ax.plot([xs - 0.1, xs + 0.1], [d["alt_max"].median()] * 2, "-", lw=2.5,
                color="#000000", zorder=2)
        if not ok.all():
            ax.text(xs, 8.5,f"{(~ok).sum()}/3 fail", ha="center", fontsize=7.5, color=OKABE[3])
    ax.text(i, -0.1, lab.replace("\n", ", "), ha="center", va="top", fontsize=8.5,
            transform=ax.get_xaxis_transform())
    if i:
        ax.axvline(i - 0.5, color="#999999", lw=0.8)
ax.set_xticks(ticks, ticklabs, fontsize=9)
ax.set_xlabel("height source: a = companion vision height, b = rangefinder height", labelpad=30)
ax.set_ylabel("true max altitude (m)")
ax.set_ylim(1.8, 10.8)
ax.set_title("Rangefinder faults in AUTO, by height source (dots = runs n=3, bars = medians)")
from matplotlib.lines import Line2D
ax.legend(handles=[Line2D([], [], marker="o", ls="", color=OKABE[2], label="run completed"),
                   Line2D([], [], marker="o", ls="", color=OKABE[3], label="failsafe / fence LAND"),
                   Line2D([], [], marker="o", ls="", color="#555555", label="a: companion vision height"),
                   Line2D([], [], marker="s", ls="", color="#555555", label="b: rangefinder height"),
                   Line2D([], [], ls="-", lw=2.5, color="#000000", label="median")],
          loc="upper left", fontsize=8, ncol=3)
fig.tight_layout()
fig.savefig(OUT / "fig4_rng_faults.png")
plt.close(fig)

# ---------------------------------------------------------------- fig5: companion height stream cut
# Report section 2 table rows q2cut_* / q2hb_B vs their q2_* baselines.
PAIRS = [("q2_B_", "a: vision", "q2cut_B_", "q2hb_B"),
         ("q2_RNG", "b: rng", "q2cut_RNG", None),
         ("q2_BARO", "c: baro", "q2cut_BARO", None),
         ("q2_BRNG", "c + mix", "q2cut_BRNG", None)]
GROUPNAME = {"a: vision": "a: companion\nvision hgt", "b: rng": "b: rangefinder\nhgt",
             "c: baro": "c: barometer\nhgt", "c + mix": "c + RNG_USE\nHGT 70 mix"}
fig, ax = plt.subplots(figsize=(9, 4.2))
centers, names = [], []
i = 0
for base, lab, cut, hb in PAIRS:
    n = 3 if hb else 2
    strip(ax, i, sel(base)["alt_rms"].tolist(), OKABE[0])
    strip(ax, i + 1, sel(cut)["alt_rms"].tolist(), OKABE[1])
    if hb:
        strip(ax, i + 2, sel(hb)["alt_rms"].tolist(), OKABE[3])
    ax.text(i, 0.004, "normal", ha="center", fontsize=7, color=OKABE[0])
    ax.text(i + 1, 0.004, "cut", ha="center", fontsize=7, color=OKABE[1])
    if hb:
        ax.text(i + 2, 0.004, "cut+hb", ha="center", fontsize=7, color=OKABE[3])
    centers.append(i + (n - 1) / 2)
    names.append(GROUPNAME[lab])
    i += n + 1
    if base != PAIRS[-1][0]:
        ax.axvline(i - 1, color="#999999", lw=0.8)
ax.set_xticks(centers, names, fontsize=8.5)
ax.grid(axis="x", visible=False)
ax.set_xlabel("height source configuration (within group: normal flight / height stream cut / cut + heartbeat)")
ax.set_ylabel("mission altitude error RMS (m)")
ax.set_ylim(0, 0.095)
ax.set_title("Stopping the companion height stream after AUTO starts\n"
             "(n=3 each; blue = normal flight baseline)")
fig.tight_layout()
fig.savefig(OUT / "fig5_companion_cut.png")
plt.close(fig)

# ---------------------------------------------------------------- fig6: flow orientation fly-away
# Report section 2 table rows q4orient_9000 / q4orient_18000 (0/3 completed).
# The CSV records hover_drift (max true offset within the LOITER hover window, from the
# window start); the report table quotes larger radii: analyze.py's
# loiter_true_radius_max (whole LOITER segment, from the arming point), which is not in
# the CSV. diverge.py only reports threshold-crossing times, not a maximum radius.
ORIENT = [("q4orient_9000", "ORIENT_YAW\n9000 (90 deg)"), ("q4orient_18000", "ORIENT_YAW\n18000 (180 deg)")]
fig, ax = plt.subplots(figsize=(6, 3.8))
for i, (p, lab) in enumerate(ORIENT):
    strip(ax, i, sel(p)["hover_drift"].tolist(), OKABE[3], ms=7)
ax.set_xticks(range(len(ORIENT)), [lab for _, lab in ORIENT], fontsize=8)
ax.set_ylabel("true horizontal drift (m)")
ax.set_title("Flow rotated 90/180 deg: LOITER diverges, AUTO never arms (0/3)\n"
             "max true offset within the LOITER hover window (results.csv hover_drift)")
fig.tight_layout()
fig.savefig(OUT / "fig6_orient_flyaway.png")
plt.close(fig)

print("wrote", sorted(p.name for p in OUT.glob("*.png")))
for t in ["q2_B_", "q2_RNG", "q2_BARO", "q2_BRNG", "q4scale_-200", "q4rnd_0.10",
          "q4jmp5_B", "q4oor5_B", "q4orient_9000", "q4orient_18000"]:
    s = sel(t)
    print(t, "n=", len(s), "alt_max med", round(s["alt_max"].median(), 3) if s["alt_max"].notna().any() else "-",
          "trk med", round(s["track_rms"].median(), 3) if s["track_rms"].notna().any() else "-")
