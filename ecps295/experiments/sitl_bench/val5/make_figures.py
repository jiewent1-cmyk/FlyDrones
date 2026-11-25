#!/usr/bin/env python3
"""make_figures.py -- figures for the V5 report (marker position correction under faults).

Data sources (read next to this script):
  results.csv       one row per SITL run, all metrics (96 runs)
  fault_rows.json   machine readable per-run rows of the fault-window tables
  table_fault_window.md / table_dev_window.md  the aggregated tables of the report

Run:  python make_figures.py      (writes figures/fig*.png)
"""
import json
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

# fault groups in the order of the report table, with the English label used on the axes
FAULTS = [
    ("bad1m_lap1", "1 m bad fix, 3 s (lap 1 straight)"),
    ("bad1m_lap2", "1 m bad fix, 3 s (lap 2 corner)"),
    ("bad03_10s", "0.3 m bad fix, 10 s"),
    ("yaw20_5s", "marker yaw wrong 20 deg, 5 s"),
    ("block30", "all markers blocked, 30 s"),
    ("stop10", "companion stops 10 s, recovers"),
    ("stop_norec", "companion stops, no recovery"),
    ("lat500_20s", "message latency 100->500 ms, 20 s"),
]
LAYOUTS = [("sq4", "four-corner markers", OKABE[0]),
           ("sq1", "single marker", OKABE[1])]


def load():
    res = pd.read_csv(HERE / "results.csv")
    rows = json.loads((HERE / "fault_rows.json").read_text())
    fr = pd.DataFrame(rows)
    return res, fr


def control_figure(res):
    """Fig 1: the three control layouts, tracking RMS and truth deviation."""
    groups = [("base_none", "no\ncorrection"), ("base_sq4", "four-corner\nmarkers"),
              ("base_sq1", "single\nmarker")]
    fig, axes = plt.subplots(1, 2, figsize=(9.5, 4.0))
    for ax, col, lab, unit in ((axes[0], "track_rms", "Tracking RMS", "m"),
                               (axes[1], "track_max", "Max truth deviation from route", "m")):
        med = [res[res.group == g][col].median() for g, _ in groups]
        lo = [res[res.group == g][col].min() for g, _ in groups]
        hi = [res[res.group == g][col].max() for g, _ in groups]
        x = np.arange(len(groups))
        ax.bar(x, med, width=0.55, color=OKABE[0], zorder=3)
        ax.errorbar(x, med, yerr=[np.array(med) - lo, np.array(hi) - np.array(med)],
                    fmt="none", ecolor="black", capsize=4, lw=1.2, zorder=4)
        for xi, m, h in zip(x, med, hi):
            ax.text(xi, h + 0.01 * max(hi), f"{m:.3f}", ha="center", va="bottom", fontsize=9)
        ax.set_xticks(x)
        ax.set_xticklabels([l for _, l in groups])
        ax.set_ylabel(f"{lab} ({unit})")
        ax.set_ylim(0, max(hi) * 1.3)
    axes[0].set_title("Tracking error of the control layouts (5 runs each)")
    axes[1].set_title("Worst truth deviation of the control layouts")
    fig.text(0.5, -0.04, "bars: median of 5 runs; whiskers: min-max of the 5 runs",
             ha="center", fontsize=8, color="0.3")
    fig.savefig(FIG / "fig2_control_layout.png")
    plt.close(fig)


def fault_figure(res, fr):
    """Fig 2: truth deviation per fault and layout (whole flight and fault window)."""
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 5.4), sharey=True)
    panels = [
        (axes[0], "Whole flight", "Max truth deviation from the 2 m x 2 m route (m)",
         lambda lay, flt: res[(res.group == f"{lay}_{flt}")].track_max.dropna().values),
        (axes[1], "Fault window only", "Max truth deviation inside the fault window (m)",
         lambda lay, flt: fr[(fr.lay == lay) & (fr.flt == flt)].dev_fault_window_max.dropna().values),
    ]
    h = 0.36
    for ax, title, xlabel, getter in panels:
        for k, (lay, lay_lab, col) in enumerate(LAYOUTS):
            ys, meds, los, his = [], [], [], []
            for i, (flt, _) in enumerate(FAULTS):
                v = getter(lay, flt)
                ys.append(i + (k - 0.5) * h)
                meds.append(np.median(v))
                los.append(v.min())
                his.append(v.max())
            ax.barh(ys, meds, height=h, color=col, label=lay_lab, zorder=3)
            ax.errorbar(meds, ys, xerr=[np.array(meds) - los, np.array(his) - np.array(meds)],
                        fmt="none", ecolor="black", capsize=3, lw=1.0, zorder=4)
            for y, m, hi in zip(ys, meds, his):
                ax.text(hi + 0.012, y, f"{m:.3f}", va="center", fontsize=7.5)
        ax.axvline(0.3, color=OKABE[3], ls="--", lw=1.2)
        ax.set_yticks(range(len(FAULTS)))
        ax.set_yticklabels([l for _, l in FAULTS])
        ax.invert_yaxis()
        ax.set_xlabel(xlabel)
        ax.set_title(title)
        ax.set_xlim(0, 0.82)
    axes[0].text(0.31, -0.62, "0.3 m acceptance limit", color=OKABE[3], fontsize=8, va="center")
    axes[0].legend(loc="upper center", bbox_to_anchor=(1.05, -0.11), ncol=2, frameon=False)
    fig.suptitle("Truth deviation per injected fault (5 runs each, median and min-max)", y=0.99)
    fig.savefig(FIG / "fig1_fault_deviation.png")
    plt.close(fig)


def speed_figure(fr):
    """Fig 3: truth speed peak inside the fault window against the 0.7 m/s limit."""
    fig, ax = plt.subplots(figsize=(8.5, 4.6))
    for k, (lay, lay_lab, col) in enumerate(LAYOUTS):
        for i, (flt, fl_lab) in enumerate(FAULTS):
            v = fr[(fr.lay == lay) & (fr.flt == flt)].spdm.dropna().values
            x = i + (k - 0.5) * 0.3
            ax.plot(np.full(len(v), x) + np.linspace(-0.07, 0.07, len(v)), v, "o",
                    ms=4.5, color=col, alpha=0.85, zorder=3,
                    label=lay_lab if i == 0 else None)
            ax.plot([x - 0.11, x + 0.11], [np.median(v)] * 2, color=col, lw=2, zorder=4)
    ax.axhline(0.7, color=OKABE[3], ls="--", lw=1.2)
    ax.text(-0.45, 0.715, "0.7 m/s limit of the task", color=OKABE[3], fontsize=8, va="bottom")
    ax.set_xticks(range(len(FAULTS)))
    ax.set_xticklabels([l for _, l in FAULTS], rotation=20, ha="right")
    ax.set_ylabel("Peak truth speed inside the fault window (m/s)")
    ax.set_title("Truth speed peak in the fault window: each dot is one run")
    ax.set_ylim(0.3, 0.85)
    ax.legend(loc="upper left", frameon=False)
    fig.savefig(FIG / "fig5_fault_window_speed.png")
    plt.close(fig)


def gate_figure(fr):
    """Fig 4: onboard gating of the bad observations."""
    fig, ax = plt.subplots(figsize=(8.5, 4.8))
    x = np.arange(len(FAULTS))
    w = 0.2
    for k, (lay, lay_lab, col) in enumerate(LAYOUTS):
        v1 = [np.median(fr[(fr.lay == lay) & (fr.flt == f)].gated.dropna().values) for f, _ in FAULTS]
        v2 = [np.median(fr[(fr.lay == lay) & (fr.flt == f)].innov.dropna().values) for f, _ in FAULTS]
        ax.bar(x + (2 * k - 1.5) * w, v1, width=w, color=col, label=f"{lay_lab}: consistency gate (v1)", zorder=3)
        ax.bar(x + (2 * k - 0.5) * w, v2, width=w, color=col, alpha=0.45,
               label=f"{lay_lab}: innovation gate (v2)", zorder=3)
    ax.set_xticks(x)
    ax.set_xticklabels([l for _, l in FAULTS], rotation=20, ha="right")
    ax.set_ylabel("Rejected observations per run (count)")
    ax.set_title("Observations rejected by the onboard gates (median of 5 runs)")
    ax.legend(loc="upper right", frameon=False, fontsize=8)
    fig.savefig(FIG / "fig3_gate_rejections.png")
    plt.close(fig)


def jump_figure(fr):
    """Fig 5: EKF position jump at the fault moment vs the largest jump of the flight."""
    fig, ax = plt.subplots(figsize=(8.5, 4.8))
    x = np.arange(len(FAULTS))
    w = 0.2
    for k, (lay, lay_lab, col) in enumerate(LAYOUTS):
        jf = [np.median(fr[(fr.lay == lay) & (fr.flt == f)].jump.dropna().values) for f, _ in FAULTS]
        jm = [np.median(fr[(fr.lay == lay) & (fr.flt == f)].jm.dropna().values) for f, _ in FAULTS]
        ax.bar(x + (2 * k - 1.5) * w, jf, width=w, color=col, label=f"{lay_lab}: jump at fault time", zorder=3)
        ax.bar(x + (2 * k - 0.5) * w, jm, width=w, color=col, alpha=0.45,
               label=f"{lay_lab}: largest jump of the flight", zorder=3)
    ax.set_xticks(x)
    ax.set_xticklabels([l for _, l in FAULTS], rotation=20, ha="right")
    ax.set_ylabel("EKF position jump (m)")
    ax.set_title("EKF position jump at the fault moment and over the whole flight (median of 5 runs)")
    ax.legend(loc="upper left", frameon=False, fontsize=8)
    fig.savefig(FIG / "fig4_ekf_jump.png")
    plt.close(fig)


def main():
    res, fr = load()
    control_figure(res)
    fault_figure(res, fr)
    speed_figure(fr)
    gate_figure(fr)
    jump_figure(fr)
    print("figures written to", FIG)
    for p in sorted(FIG.glob("*.png")):
        print(" ", p.name, p.stat().st_size // 1024, "kB")


if __name__ == "__main__":
    main()
