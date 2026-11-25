"""Extra figures for REPORT_zh.md (S8a ablation). Reads e1.csv, e2.csv, thermal.csv next to this file.
Headline forest / wall clearance / fault injection figures are in ../../reports/figures/ and not repeated here."""
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.dates
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

NAMES = {"full": "full system (v4)", "no_new_cells": "- all new cells", "no_saccade": "- saccade escape",
         "no_lcb": "- LCb", "no_lcb_side": "- LCb side split", "no_lcv": "- LCv", "no_retreat": "- retreat",
         "no_routeB": "- route B", "up": "upstream original", "no_lcn": "- LCn", "no_retreat_cap": "- retreat cap",
         "no_baro_rise": "- baro-rise check", "no_caution": "- caution period", "no_brake_mem": "- brake memory",
         "no_efference": "- efference copy", "no_yaw_decouple": "- yaw decoupling", "no_fence_turn": "- fence turn-back",
         "lad_v1": "ladder v1.3", "lad_v2": "ladder v2.1", "lad_v3": "ladder v3", "up_B": "upstream + route B",
         "gps_ref": "GPS reference (v4)"}


def boot(x, n=4000):
    x = np.asarray(x, float)
    m = rng.choice(x, (n, len(x))).mean(1)
    return np.percentile(m, [2.5, 97.5])


def save(fig, name):
    fig.savefig(OUT / name)
    plt.close(fig)


e1 = pd.read_csv(HERE / "e1.csv")
e2 = pd.read_csv(HERE / "e2.csv")
th = pd.read_csv(HERE / "thermal.csv")

# fig1: E1 outcome per condition (stacked share of runs), sorted by success
e1["outcome"] = np.where(e1.fc_land == 1, "FC forced landing",
                         np.where(e1.contact_any == 1, "contact", "success"))
print("success column == (no contact and no FC landing):", ((e1.outcome == "success") == (e1.success == 1)).mean())
order = e1.groupby("condition").success.mean().sort_values().index
share = e1.groupby(["condition", "outcome"]).size().unstack(fill_value=0).reindex(order)
share = share.div(share.sum(axis=1), axis=0) * 100
n = e1.groupby("condition").size().reindex(order)
fig, ax = plt.subplots(figsize=(8, 7))
left = np.zeros(len(order))
for col, c in [("success", "#009E73"), ("contact", "#D55E00"), ("FC forced landing", "#000000")]:
    v = share[col].values if col in share else np.zeros(len(order))
    ax.barh(range(len(order)), v, left=left, color=c, label=col, height=0.7)
    left += v
ax.set_yticks(range(len(order)))
ax.set_yticklabels([f"{NAMES[c]} (n={n[c]})" for c in order])
ax.set_xlim(0, 100)
ax.set_xlabel("share of runs (%)")
ax.set_title("E1 cruise: outcome of every run, per condition")
ax.grid(axis="y", visible=False)
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.09), ncol=3, frameon=False)
save(fig, "fig1_e1_outcomes.png")

# fig2: E1 path length and time hovering over boxes (mean + 95% bootstrap CI, runs as dots)
key = ["full", "no_new_cells", "no_saccade", "no_lcb", "no_lcb_side", "no_lcv", "no_retreat", "no_routeB", "up"]
fig, axs = plt.subplots(1, 2, figsize=(10, 4.8), sharey=True)
for ax, col, lab in [(axs[0], "path_m", "path length (m)"), (axs[1], "over_box_s", "time above a low box (s)")]:
    for i, c in enumerate(key[::-1]):
        x = e1[e1.condition == c][col].values
        colr = OKABE[7] if c == "full" else OKABE[0]
        ax.scatter(x, i + rng.uniform(-0.2, 0.2, len(x)), s=7, color="#9a9a9a", alpha=0.5, lw=0)
        lo, hi = boot(x)
        ax.plot([lo, hi], [i, i], color=colr, lw=2.5)
        ax.scatter([x.mean()], [i], color=colr, s=30, zorder=3)
    ax.set_xlabel(lab)
axs[0].set_yticks(range(len(key)))
axs[0].set_yticklabels([NAMES[c] for c in key[::-1]])
axs[0].set_title("distance flown in 120 s")
axs[1].set_title("hovering above a low box")
fig.suptitle("E1 cruise (n=50 per condition): mean, 95% bootstrap CI and single runs", y=1.02)
save(fig, "fig2_e1_path_overbox.png")

# fig3: highest true altitude per run, the altitude-related conditions
alt = ["full", "no_efference", "no_yaw_decouple", "no_routeB", "up", "up_B"]
fig, ax = plt.subplots(figsize=(7, 4))
data = [e1[e1.condition == c].alt_true_max_m.values for c in alt]
bp = ax.boxplot(data, widths=0.5, showfliers=False, patch_artist=True, medianprops={"color": "black"})
for p in bp["boxes"]:
    p.set_facecolor("#cfe3f1")
for i, x in enumerate(data):
    ax.scatter(i + 1 + rng.uniform(-0.15, 0.15, len(x)), x, s=8, color=OKABE[0], alpha=0.6, lw=0)
ax.axhline(1.3, color="#D55E00", ls="--")
ax.text(0.55, 1.31, "1.3 m hard fence", color="#D55E00", fontsize=9, va="bottom")
ax.set_xticks(range(1, len(alt) + 1))
short = {"full": "full system", "no_efference": "- efference\ncopy", "no_yaw_decouple": "- yaw\ndecoupling",
         "no_routeB": "- route B", "up": "upstream\noriginal", "up_B": "upstream\n+ route B"}
ax.set_xticklabels([f"{short[c]}\n(n={len(d)})" for c, d in zip(alt, data)], fontsize=8)
ax.set_ylabel("highest true altitude in run (m)")
ax.set_title("E1: altitude control (efference copy, route B)")
ax.grid(axis="x", visible=False)
save(fig, "fig3_e1_altitude.png")

# fig4: E2 escape-onset clearance vs minimum clearance, one dot per condition (mean over 32 runs)
g = e2.groupby("condition")[["escape_onset_clearance_m", "min_clearance_m"]].mean()
fig, ax = plt.subplots(figsize=(7, 5.2))
ax.axhline(0, color="#D55E00", lw=1)
ax.scatter(g.escape_onset_clearance_m, g.min_clearance_m, s=40, color=OKABE[0], zorder=3)
cluster = g[(g.min_clearance_m > 0.55) & (g.index != "no_yaw_decouple") & (g.index != "full")]
for c, r in g.iterrows():
    if c in cluster.index or c in ("up", "up_B"):
        continue
    # two neighbouring points get separated labels with leader lines
    off = {"ladder v1.3": (-8, -30), "- all new cells": (14, 22)}.get(NAMES[c], (4, 3))
    ax.annotate(NAMES[c], (r.escape_onset_clearance_m, r.min_clearance_m), xytext=off,
                textcoords="offset points", fontsize=7.5,
                arrowprops=dict(arrowstyle="-", color="#777777", lw=0.7) if off != (4, 3) else None)
ax.text(0.80, 0.50, "cluster at the top right:\n" + "\n".join(NAMES[c] for c in cluster.index),
        fontsize=7, ha="right", va="top")
ax.annotate("upstream original,\nupstream + route B", (0.205, -0.095), xytext=(8, 4),
            textcoords="offset points", fontsize=7.5)
ax.set_xlabel("clearance when the escape starts (m), mean of 32 runs")
ax.set_ylabel("minimum clearance to the wall (m), mean of 32 runs")
ax.set_title("E2: later escape start, smaller final clearance")
ax.set_xlim(0.15, 1.1)
save(fig, "fig4_e2_onset_vs_min.png")

# fig5: hardware monitoring during the campaign (thermal.csv) and RTF of all E1 runs
t = pd.to_datetime(th.time, format="%Y-%m-%d_%H:%M:%S")
fig, axs = plt.subplots(1, 2, figsize=(10, 3.8), gridspec_kw={"width_ratios": [1.6, 1]})
ax = axs[0]
ax.plot(t, th.cpu_tctl_c, lw=0.6, label="CPU Tctl")
ax.plot(t, th.gpu_c, lw=0.6, label="GPU")
ax.set_ylabel("temperature (deg C)")
ax.set_xlabel("time (day, hh:mm)")
ax.xaxis.set_major_formatter(matplotlib.dates.DateFormatter("%d %H:%M"))
ax.tick_params(axis="x", labelsize=8, rotation=30)
ax.set_title(f"ROG thermal log ({len(th)} samples)")
ax.legend(frameon=False, loc="lower left")
ax = axs[1]
ax.hist(e1.rtf_run, bins=30, color=OKABE[2])
ax.axvline(0.95, color="#D55E00", ls="--")
ax.set_xlabel("real-time factor of the run")
ax.set_ylabel("runs")
ax.set_title("E1 RTF per run (threshold 0.95)")
save(fig, "fig5_thermal_rtf.png")
