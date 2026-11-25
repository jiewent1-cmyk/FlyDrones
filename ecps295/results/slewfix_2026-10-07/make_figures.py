"""Figures for REPORT_zh.md (slew-fix re-check). Reads A2_*.csv and B_*_vs_*.csv next to this file."""
from pathlib import Path
import numpy as np
import pandas as pd
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
rng = np.random.default_rng(0)


def boot(x, n=4000):
    x = np.asarray(x, float)
    return np.percentile(rng.choice(x, (n, len(x))).mean(axis=1), [2.5, 97.5])


def wilson(k, n, z=1.96):
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return c - h, c + h


def save(fig, name):
    fig.savefig(OUT / name)
    plt.close(fig)


def strip_ci(ax, df, conds, col, labels, hl=None):
    for i, c in enumerate(conds[::-1]):
        x = df[df.condition == c][col].values
        colr = OKABE[7] if c == hl else OKABE[0]
        ax.scatter(x, i + rng.uniform(-0.2, 0.2, len(x)), s=7, color="#9a9a9a", alpha=0.5, lw=0)
        lo, hi = boot(x)
        ax.plot([lo, hi], [i, i], color=colr, lw=2.5)
        ax.scatter([x.mean()], [i], color=colr, s=30, zorder=3)
    ax.set_yticks(range(len(conds)))
    ax.set_yticklabels([labels[c] for c in conds[::-1]])


LAB = {"full": "full (v4)", "no_lcb": "- LCb", "no_new_cells": "- all new cells", "no_saccade": "- saccade",
       "no_efference": "- efference copy", "no_lcv": "- LCv", "no_retreat": "- retreat", "lad_v3": "ladder v3",
       "no_routeB": "- route B"}

# fig1: same conditions before (S8 code, slew bug) and after the fix, paired dumbbells
e1b, e1f = pd.read_csv(HERE / "A2_e1_bug.csv"), pd.read_csv(HERE / "A2_e1_fixed.csv")
e2b, e2f = pd.read_csv(HERE / "A2_e2_bug.csv"), pd.read_csv(HERE / "A2_e2_fixed.csv")
e1f["condition"] = e1f.condition.str.removeprefix("fx_")
e2f["condition"] = e2f.condition.str.removeprefix("fx_")
conds1 = list(e1b.groupby("condition").contacts_per_100m.mean().sort_values().index)
conds2 = ["full", "no_lcb", "no_new_cells", "no_saccade", "no_efference"]
fig, axs = plt.subplots(1, 3, figsize=(10.5, 4.2), gridspec_kw={"wspace": 0.75})
for ax, (b, f, col, conds, xl, ttl) in zip(axs, [
        (e1b, e1f, "contacts_per_100m", conds1, "contacts per 100 m", "E1 contacts (n=20)"),
        (e1b, e1f, "success", conds1, "success rate", "E1 success (n=20)"),
        (e2b, e2f, "min_clearance_m", conds2, "minimum clearance to the wall (m)", "E2 min clearance (n=32)")]):
    for i, c in enumerate(conds):
        xb, xf = b[b.condition == c][col].mean(), f[f.condition == c][col].mean()
        ax.plot([xb, xf], [i, i], color="#aaaaaa", lw=1.5, zorder=1)
        ax.scatter([xb], [i], color=OKABE[1], s=40, zorder=3, label="before fix" if i == 0 else None)
        ax.scatter([xf], [i], color=OKABE[0], s=40, zorder=3, label="after fix" if i == 0 else None)
    ax.set_yticks(range(len(conds)))
    ax.set_yticklabels([LAB[c] for c in conds] if ax is not axs[1] else [""] * len(conds))
    ax.set_xlabel(xl)
    ax.set_title(ttl)
axs[0].legend(frameon=False, loc="lower right")
fig.suptitle("Slew fix: same conditions, same worlds and seeds, condition means", y=1.02)
save(fig, "fig1_before_after.png")

# fig2: lowbox, 8 models: contacts, success, time above boxes
low = pd.read_csv(HERE / "B_low_vs_v4.csv")
low["condition"] = low.condition.str.removeprefix("fx_").str.removeprefix("m_").replace({"full": "v4"})
M = ["v4", "v4_oc5", "v3_2", "es2", "es3", "wcov2", "wcov4", "wcov8"]
ML = {m: m for m in M}
fig, axs = plt.subplots(1, 3, figsize=(10, 3.9), sharey=True)
strip_ci(axs[0], low, M, "contacts_per_100m", ML, hl="v4")
axs[0].set_xlabel("contacts per 100 m")
axs[0].set_title("contacts")
ax = axs[1]
for i, m in enumerate(M[::-1]):
    s = low[low.condition == m].success
    lo, hi = wilson(s.sum(), len(s))
    ax.plot([lo * 100, hi * 100], [i, i], color=OKABE[7] if m == "v4" else OKABE[0], lw=2.5)
    ax.scatter([s.mean() * 100], [i], s=30, color=OKABE[7] if m == "v4" else OKABE[0], zorder=3)
ax.set_xlabel("success rate (%), 95% Wilson CI")
ax.set_title("success")
strip_ci(axs[2], low, M, "over_box_s", ML, hl="v4")
axs[2].set_xlabel("time above a low box (s)")
axs[2].set_title("hovering above boxes")
axs[0].set_yticks(range(len(M)))
axs[0].set_yticklabels(M[::-1])
fig.suptitle("lowbox (optical flow + route B), n=30 worlds per model: RL models do not beat v4", y=1.02)
save(fig, "fig2_lowbox_models.png")

# fig3: cage20 (GPS, RL training scene): contacts per 100 m, share of runs with contact, near misses
cg = pd.read_csv(HERE / "B_cage_vs_v4.csv")
cg["condition"] = cg.condition.str.removeprefix("fx_").str.removeprefix("m_")
fig, axs = plt.subplots(1, 3, figsize=(10, 3.9), sharey=True)
strip_ci(axs[0], cg, M, "contacts_per_100m", ML, hl="v4")
axs[0].set_xlabel("contacts per 100 m")
axs[0].set_title("contacts")
ax = axs[1]
for i, m in enumerate(M[::-1]):
    s = cg[cg.condition == m].contact_any
    lo, hi = wilson(s.sum(), len(s))
    ax.plot([lo * 100, hi * 100], [i, i], color=OKABE[7] if m == "v4" else OKABE[0], lw=2.5)
    ax.scatter([s.mean() * 100], [i], s=30, color=OKABE[7] if m == "v4" else OKABE[0], zorder=3)
ax.set_xlabel("runs with a contact (%), 95% Wilson CI")
ax.set_title("runs with contact")
strip_ci(axs[2], cg, M, "near_miss_episodes", ML, hl="v4")
axs[2].set_xlabel("near-miss episodes per run")
axs[2].set_title("near misses")
axs[0].set_yticks(range(len(M)))
axs[0].set_yticklabels(M[::-1])
fig.suptitle("cage20 (GPS, RL training scene), n=32 brain seeds per model", y=1.02)
save(fig, "fig3_cage20_models.png")

# fig4: wall approach per model: minimum and final clearance
wl = pd.read_csv(HERE / "B_wall_vs_v4.csv")
wl["condition"] = wl.condition.str.removeprefix("fx_").str.removeprefix("m_").replace({"full": "v4"})
fig, axs = plt.subplots(1, 2, figsize=(9, 3.9), sharey=True)
strip_ci(axs[0], wl, M, "min_clearance_m", ML, hl="v4")
axs[0].set_xlabel("minimum clearance to the wall (m)")
axs[0].set_title("minimum clearance")
strip_ci(axs[1], wl, M, "escape_onset_clearance_m", ML, hl="v4")
axs[1].set_xlabel("clearance when the escape starts (m)")
axs[1].set_title("escape onset")
axs[0].set_yticks(range(len(M)))
axs[0].set_yticklabels(M[::-1])
fig.suptitle("Wall approach (4 walls x 8 seeds, n=32 per model)", y=1.02)
save(fig, "fig4_wall_models.png")

# fig5: coverage vs contacts per model, two scenes (means)
fig, ax = plt.subplots(figsize=(7, 4.6))
for df, nm, mk, colr in [(low, "lowbox (flow + route B)", "o", OKABE[0]), (cg, "cage20 (GPS)", "s", OKABE[3])]:
    g = df.groupby("condition")[["coverage", "contacts_per_100m"]].mean()
    ax.scatter(g.coverage, g.contacts_per_100m, marker=mk, color=colr, s=40, label=nm, zorder=3)
    for m, r in g.iterrows():
        left = nm.startswith("cage20") and m == "wcov4"  # its right side collides with the wcov8 label
        ax.annotate(m, (r.coverage, r.contacts_per_100m), xytext=(-5, 4) if left else (4, 3),
                    textcoords="offset points", fontsize=7, color=colr, ha="right" if left else "left")
ax.set_xlabel("coverage of 0.5 m cells (fraction), mean per model")
ax.set_ylabel("contacts per 100 m, mean per model")
ax.set_title("Coverage vs contacts per model (lower right is better)")
ax.legend(frameon=False, loc="upper left")
save(fig, "fig5_coverage_vs_contacts.png")
