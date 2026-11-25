#!/usr/bin/env python3
"""make_figures.py -- figures for the V6 report (raising RNGFND1_MIN).

Data source (read next to this script):
  results.csv   one row per SITL run (288 runs, 8 configs x 10 cells)

Run:  python make_figures.py      (writes figures/fig*.png)
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

OKABE = ["#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#56B4E9", "#F0E442", "#000000"]
plt.rcParams.update({"figure.dpi": 150, "savefig.dpi": 150, "savefig.bbox": "tight",
    "font.size": 10, "axes.titlesize": 11, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.alpha": 0.3, "axes.prop_cycle": matplotlib.cycler(color=OKABE),
    "figure.facecolor": "white"})

HERE = Path(__file__).resolve().parent
FIG = HERE / "figures"
FIG.mkdir(exist_ok=True)

# configs in the order of the report total table; RNGFND1_MIN of each config
GROUPS = [("f01", 0.01), ("f10", 0.10), ("f20", 0.20), ("f30", 0.30),
          ("o01", 0.01), ("o10", 0.10), ("o20", 0.20), ("o30", 0.30)]

# normal-flight cells first, then the fault cells, in the order of report section 2.2
CELLS = [("d3", "D3"), ("sq", "SQ"), ("hov", "HOV"),
         ("a", "climb 15 m/s, 1 s"),
         ("c", "2 m, 15 m/s, 3 s"),
         ("g3", "2 m, reading 0.1 m, 3 s"),
         ("g5", "2 m, reading 0.1 m, 5 s"),
         ("g10", "2 m, reading 0.1 m, 10 s"),
         ("gN", "2 m, reading 0.1 m, no recovery"),
         ("h", "0.5 m, reading 0.1 m, no recovery")]
FAULT_CELLS = ["a", "c", "g3", "g5", "g10", "gN", "h"]
NORMAL_CELLS = ["d3", "sq", "hov"]


def load():
    return pd.read_csv(HERE / "results.csv")


def completion_figure(res):
    """Fig 1: completed runs per config and cell (heatmap)."""
    fig, ax = plt.subplots(figsize=(9.5, 4.6))
    mat = np.full((len(GROUPS), len(CELLS)), np.nan)
    txt = np.empty((len(GROUPS), len(CELLS)), dtype=object)
    for i, (g, _) in enumerate(GROUPS):
        for j, (c, _) in enumerate(CELLS):
            sub = res[(res.group == g) & (res.cell == c)]
            n_done = int(sub.complete.sum())
            mat[i, j] = 100.0 * n_done / len(sub) if len(sub) else np.nan
            txt[i, j] = f"{n_done}/{len(sub)}"
    im = ax.imshow(mat, cmap="viridis", vmin=0, vmax=100, aspect="auto")
    ax.set_xticks(range(len(CELLS)))
    ax.set_xticklabels([lab for _, lab in CELLS], rotation=25, ha="right", fontsize=9)
    ax.set_yticks(range(len(GROUPS)))
    ax.set_yticklabels([f"{g} (RNGFND1_MIN {m})" for g, m in GROUPS])
    for i in range(len(GROUPS)):
        for j in range(len(CELLS)):
            frac = mat[i, j]
            col = "white" if not np.isnan(frac) and frac < 55 else "black"
            ax.text(j, i, txt[i, j], ha="center", va="center", fontsize=9, color=col)
    ax.set_title("Completed missions per config and cell (share of runs, cell = runs completed)")
    ax.grid(False)
    cb = fig.colorbar(im, ax=ax, fraction=0.02, pad=0.02)
    cb.set_label("Completed runs (%)")
    fig.savefig(FIG / "fig1_completion_heatmap.png")
    plt.close(fig)


def ground_figure(res):
    """Fig 2: ground rangefinder readings per config against RNGFND1_MIN."""
    fig, ax = plt.subplots(figsize=(8.6, 4.6))
    for i, (g, mn) in enumerate(GROUPS):
        sub = res[(res.group == g) & res.rfnd_dist_gnd_max.notna()]
        lo = sub.rfnd_dist_gnd_min.min() if len(sub) else np.nan
        hi = sub.rfnd_dist_gnd_max.max() if len(sub) else np.nan
        stat = sorted({s.strip() for v in sub.rfnd_stat_gnd.dropna()
                       for s in str(v).strip("[]").split(",")})
        ax.plot([i, i], [lo, hi], color=OKABE[0], lw=6, solid_capstyle="butt", zorder=3)
        ax.plot(i, lo, "o", color=OKABE[0], ms=5, zorder=4)
        ax.plot(i, hi, "o", color=OKABE[0], ms=5, zorder=4)
        ax.plot([i - 0.28, i + 0.28], [mn, mn], color=OKABE[3], lw=2.5, zorder=5)
        ax.text(i, hi + 0.006, f"{lo:.2f}-{hi:.2f} m\nstat {'/'.join(stat) if stat else '-'}",
                ha="center", va="bottom", fontsize=8)
    ax.set_xticks(range(len(GROUPS)))
    ax.set_xticklabels([g for g, _ in GROUPS])
    ax.set_xlim(-0.6, len(GROUPS) - 0.4)
    ax.set_ylim(-0.005, 0.32)
    ax.set_ylabel("Ground rangefinder reading (m)")
    ax.set_title("Ground readings stay below every RNGFND1_MIN >= 0.10 m")
    handles = [plt.Line2D([], [], color=OKABE[0], lw=6, label="range of ground readings"),
               plt.Line2D([], [], color=OKABE[3], lw=2.5, label="RNGFND1_MIN of the config"),
               plt.Line2D([], [], color="0.4", marker="", ls="none",
                          label="RFND.Stat: 2 = OutOfRangeLow, 4 = Good")]
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.09),
              ncol=3, frameon=False, fontsize=8)
    fig.savefig(FIG / "fig2_ground_readings.png")
    plt.close(fig)


def normal_figure(res):
    """Fig 3: normal-flight metrics of the two min 0.01 configs."""
    keep = res[res.group.isin(["f01", "o01"]) & res.cell.isin(NORMAL_CELLS)]
    panels = [("alt_max", "Truth max height (m)", "Truth maximum height"),
              ("liftoff_err_max", "Height error at lift-off, max (m)", "EKF height error at lift-off"),
              ("touchdown_mps", "Touchdown speed (m/s)", "Touchdown speed")]
    fig, axes = plt.subplots(1, 3, figsize=(11.0, 4.2))
    fig.subplots_adjust(wspace=0.42)
    for ax, (col, ylab, title) in zip(axes, panels):
        x = np.arange(len(NORMAL_CELLS))
        w = 0.36
        ymax = max(keep[col].dropna()) * 1.45
        ax.set_ylim(0, ymax)
        for k, (g, glab, color) in enumerate([("f01", "FINAL gates", OKABE[0]),
                                              ("o01", "original gates", OKABE[1])]):
            meds, los, his = [], [], []
            for c in NORMAL_CELLS:
                v = keep[(keep.group == g) & (keep.cell == c)][col].dropna().values
                meds.append(np.median(v))
                los.append(v.min())
                his.append(v.max())
            ax.bar(x + (k - 0.5) * w, meds, width=w, color=color, label=glab, zorder=3)
            ax.errorbar(x + (k - 0.5) * w, meds,
                        yerr=[np.array(meds) - los, np.array(his) - np.array(meds)],
                        fmt="none", ecolor="black", capsize=3, lw=1.0, zorder=4)
            for xi, m, h in zip(x + (k - 0.5) * w, meds, his):
                ax.text(xi, h + 0.03 * ymax, f"{m:.3f}", ha="center",
                        va="bottom", fontsize=7.5, rotation=90)
        ax.set_xticks(x)
        ax.set_xticklabels([c.upper() for c in NORMAL_CELLS])
        ax.set_ylabel(ylab)
        ax.set_title(title)
    axes[0].legend(loc="upper left", frameon=False, fontsize=8)
    fig.suptitle("Normal flights with RNGFND1_MIN 0.01 (5 runs per task)", y=1.02)
    fig.savefig(FIG / "fig3_normal_flight.png")
    plt.close(fig)


def fault_figure(res):
    """Fig 4: truth height and baseline deviation per fault cell."""
    keep = res[res.group.isin(["f01", "o01"]) & res.cell.isin(FAULT_CELLS)]
    panels = [("f_true_max", "Truth max height in fault window (m)", "Truth maximum height"),
              ("dev_base", "Baseline deviation (m)", "Deviation from the route")]
    fig, axes = plt.subplots(1, 2, figsize=(10.6, 4.6))
    for ax, (col, ylab, title) in zip(axes, panels):
        x = np.arange(len(FAULT_CELLS))
        w = 0.36
        ymax = max(keep[col].dropna()) * 1.45
        ax.set_ylim(0, ymax)
        for k, (g, glab, color) in enumerate([("f01", "FINAL gates", OKABE[0]),
                                              ("o01", "original gates", OKABE[1])]):
            meds, los, his = [], [], []
            for c in FAULT_CELLS:
                v = keep[(keep.group == g) & (keep.cell == c)][col].dropna().values
                meds.append(np.median(v))
                los.append(v.min())
                his.append(v.max())
            ax.bar(x + (k - 0.5) * w, meds, width=w, color=color, label=glab, zorder=3)
            ax.errorbar(x + (k - 0.5) * w, meds,
                        yerr=[np.array(meds) - los, np.array(his) - np.array(meds)],
                        fmt="none", ecolor="black", capsize=3, lw=1.0, zorder=4)
            for xi, m, h in zip(x + (k - 0.5) * w, meds, his):
                ax.text(xi, h + 0.03 * ymax, f"{m:.2f}", ha="center",
                        va="bottom", fontsize=7.5, rotation=90)
        ax.set_xticks(x)
        ax.set_xticklabels([dict(CELLS)[c] for c in FAULT_CELLS], rotation=22, ha="right", fontsize=8)
        ax.set_ylabel(ylab)
        ax.set_title(title)
    axes[0].legend(loc="upper left", frameon=False, fontsize=8)
    fig.suptitle("Fault cells with RNGFND1_MIN 0.01 (3 runs per cell, median and min-max)", y=1.02)
    fig.savefig(FIG / "fig4_fault_deviation.png")
    plt.close(fig)


def ekf_figure(res):
    """Fig 5: EKF height error and injected rangefinder readings per fault cell."""
    keep = res[res.group.isin(["f01", "o01"]) & res.cell.isin(FAULT_CELLS)]
    fig, axes = plt.subplots(1, 2, figsize=(10.6, 4.6))
    ax = axes[0]
    x = np.arange(len(FAULT_CELLS))
    w = 0.36
    ymax = max(keep.f_est_err_max.dropna()) * 1.45
    ax.set_ylim(0, ymax)
    for k, (g, glab, color) in enumerate([("f01", "FINAL gates", OKABE[0]),
                                          ("o01", "original gates", OKABE[1])]):
        meds, los, his = [], [], []
        for c in FAULT_CELLS:
            v = keep[(keep.group == g) & (keep.cell == c)].f_est_err_max.dropna().values
            meds.append(np.median(v))
            los.append(v.min())
            his.append(v.max())
        ax.bar(x + (k - 0.5) * w, meds, width=w, color=color, label=glab, zorder=3)
        ax.errorbar(x + (k - 0.5) * w, meds,
                    yerr=[np.array(meds) - los, np.array(his) - np.array(meds)],
                    fmt="none", ecolor="black", capsize=3, lw=1.0, zorder=4)
        for xi, m, h in zip(x + (k - 0.5) * w, meds, his):
            ax.text(xi, h + 0.03 * ymax, f"{m:.2f}", ha="center", va="bottom",
                    fontsize=7.5, rotation=90)
    ax.set_xticks(x)
    ax.set_xticklabels([dict(CELLS)[c] for c in FAULT_CELLS], rotation=22, ha="right", fontsize=8)
    ax.set_ylabel("EKF height error, max in fault window (m)")
    ax.set_title("EKF height error during the fault")
    ax.legend(loc="upper left", frameon=False, fontsize=8)

    ax = axes[1]
    for k, (g, glab, color) in enumerate([("f01", "FINAL gates", OKABE[0]),
                                          ("o01", "original gates", OKABE[1])]):
        for i, c in enumerate(FAULT_CELLS):
            sub = keep[(keep.group == g) & (keep.cell == c)]
            if not len(sub):
                continue
            lo = sub.f_rfnd_min.min()
            hi = sub.f_rfnd_max.max()
            xi = i + (k - 0.5) * 0.36
            ax.plot([xi, xi], [lo, hi], color=color, lw=5, solid_capstyle="butt", zorder=3)
            ax.plot(xi, lo, "o", color=color, ms=4, zorder=4)
            ax.plot(xi, hi, "o", color=color, ms=4, zorder=4)
    ax.set_xticks(range(len(FAULT_CELLS)))
    ax.set_xticklabels([dict(CELLS)[c] for c in FAULT_CELLS], rotation=22, ha="right", fontsize=8)
    ax.set_ylabel("Rangefinder reading in fault window (m)")
    ax.set_title("Rangefinder reading during the fault")
    ax.set_ylim(0, 3.4)
    ax.legend(handles=[plt.Line2D([], [], color=OKABE[0], lw=5, label="FINAL gates"),
                       plt.Line2D([], [], color=OKABE[1], lw=5, label="original gates")],
              loc="upper left", frameon=False, fontsize=8)
    fig.suptitle("Fault cells with RNGFND1_MIN 0.01 (3 runs per cell)", y=1.02)
    fig.savefig(FIG / "fig5_fault_ekf_and_readings.png")
    plt.close(fig)


def main():
    res = load()
    completion_figure(res)
    ground_figure(res)
    normal_figure(res)
    fault_figure(res)
    ekf_figure(res)
    print("figures written to", FIG)
    for p in sorted(FIG.glob("*.png")):
        print(" ", p.name, p.stat().st_size // 1024, "kB")


if __name__ == "__main__":
    main()
