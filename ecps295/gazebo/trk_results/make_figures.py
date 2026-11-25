"""Figures for README.md (Gazebo target tracking, trk1/trk2/trk3). Reads matrix_trk1..3.log next to this file
(one line per run: "<tag> key=value ..."; the last line is DONE)."""
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
PASS, FAIL = "#009E73", "#D55E00"
VERS = {1: "trk1 (v1)", 2: "trk2 (v2)", 3: "trk3 (v2 + search relocation)"}


def load(v):
    rows = []
    for line in open(HERE / f"matrix_trk{v}.log"):
        p = line.split()
        if len(p) < 3 or "=" not in p[1]:
            continue
        d = dict(x.split("=", 1) for x in p[1:] if "=" in x)
        d["tag"] = p[0]
        rows.append(d)
    df = pd.DataFrame(rows)
    ep = df.tag.str.split("__").str[1]
    df["world"] = ep.str.replace(r"_k\d+$", "", regex=True)
    df["k"] = ep.str.extract(r"_k(\d+)$")[0].astype(int)
    df["version"] = v
    for c in ["trk_t_start", "trk_t_arrived", "trk_end_dist_m", "trk_range_err_med_m", "trk_bearing_err_med_deg",
              "min_clearance_m", "trk_end_facing_err_deg"]:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df["pass"] = df.trk_pass == "True"
    df["t_arrive"] = df.trk_t_arrived - df.trk_t_start
    return df


D = pd.concat([load(v) for v in (1, 2, 3)], ignore_index=True)
worlds = list(dict.fromkeys(D[D.version == 3].world))


def save(fig, name):
    fig.savefig(OUT / name)
    plt.close(fig)


# fig1: pass / fail per run (world x track_start variant k), one panel per tracker version
fig, axs = plt.subplots(1, 3, figsize=(10, 5.2), sharey=True)
for ax, v in zip(axs, (1, 2, 3)):
    d = D[D.version == v]
    for i, w in enumerate(worlds):
        for k in range(3):
            r = d[(d.world == w) & (d.k == k)]
            if r.empty:
                continue
            ok = bool(r["pass"].iloc[0])
            ax.add_patch(plt.Rectangle((k - 0.45, i - 0.45), 0.9, 0.9, color=PASS if ok else FAIL))
    ax.set_xlim(-0.6, 2.6)
    ax.set_ylim(len(worlds) - 0.4, -0.6)
    ax.set_xticks(range(3))
    ax.set_xticklabels([f"k{k}" for k in range(3)])
    ax.set_xlabel("start variant k")
    n = int(d["pass"].sum())
    ax.set_title(f"{VERS[v]}\n{n}/{len(d)} pass", fontsize=10)
    ax.grid(False)
axs[0].set_yticks(range(len(worlds)))
axs[0].set_yticklabels(worlds)
axs[0].set_ylabel("world")
fig.legend(handles=[plt.Rectangle((0, 0), 1, 1, color=PASS), plt.Rectangle((0, 0), 1, 1, color=FAIL)],
           labels=["pass", "fail"], loc="lower center", ncol=2, frameon=False, bbox_to_anchor=(0.5, -0.04))
fig.suptitle("Target tracking: pass / fail of every run", y=1.0)
save(fig, "fig1_pass_fail.png")


def strip(ax, col, ylabel, band=None, title=None, only_pass=False):
    for j, v in enumerate((1, 2, 3)):
        d = D[D.version == v]
        if only_pass:
            d = d[d["pass"]]
        d = d[d[col].notna()]
        x = j + rng.uniform(-0.18, 0.18, len(d))
        c10 = d.world.str.startswith("cage10").values
        ax.scatter(x[~c10], d[col][~c10], s=14, color=OKABE[0], alpha=0.7, lw=0)
        ax.scatter(x[c10], d[col][c10], s=26, color=OKABE[1], marker="D", alpha=0.9, lw=0)
        ax.hlines(d[col].median(), j - 0.3, j + 0.3, color="black", lw=2)
    if band:
        ax.axhspan(*band, color="#009E73", alpha=0.12, lw=0)
    ax.set_xticks(range(3))
    ax.set_xticklabels([VERS[v].replace(" (", "\n(") for v in (1, 2, 3)], fontsize=8)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(axis="x", visible=False)


# fig2: end distance to the marker
fig, ax = plt.subplots(figsize=(7, 4))
strip(ax, "trk_end_dist_m", "distance to the marker at the end (m)", band=(0.6, 1.5),
      title="End distance (green band: 0.6-1.5 m pass window; bar: median)")
ax.scatter([], [], s=26, color=OKABE[1], marker="D", label="cage10 runs")
ax.legend(frameon=False, loc="upper right")
save(fig, "fig2_end_distance.png")

# fig3: time from track start to arrival (runs that arrived)
fig, ax = plt.subplots(figsize=(7, 4))
strip(ax, "t_arrive", "time from track start to ARRIVED (s)", title="Time to arrival (runs that arrived; bar: median)")
ax.scatter([], [], s=26, color=OKABE[1], marker="D", label="cage10 runs")
ax.legend(frameon=False, loc="upper right")
save(fig, "fig3_arrival_time.png")

# fig4: range and bearing error medians per run
fig, axs = plt.subplots(1, 2, figsize=(10, 3.8))
strip(axs[0], "trk_range_err_med_m", "median range error per run (m)", title="Range error")
strip(axs[1], "trk_bearing_err_med_deg", "median bearing error per run (deg)", title="Bearing error")
fig.suptitle("Estimate error against simulator truth (one dot per run; bar: median)", y=1.02)
save(fig, "fig4_estimate_error.png")

# fig5: minimum clearance (safety; all runs had 0 contacts and 0 near misses)
fig, ax = plt.subplots(figsize=(7, 4))
strip(ax, "min_clearance_m", "minimum clearance during the run (m)",
      title=f"Minimum clearance ({int(D.contact_episodes.astype(int).sum())} contacts, "
            f"{int(D.near_miss_episodes.astype(int).sum())} near misses in {len(D)} runs)")
ax.axhline(0, color=FAIL, lw=1)
save(fig, "fig5_min_clearance.png")
