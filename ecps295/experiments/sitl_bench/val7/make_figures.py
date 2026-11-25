#!/usr/bin/env python3
"""make_figures.py -- figures for the V7 report (baro/range monitor validation).

Data sources (read next to this script):
  fp_grid.md     table of the 72 parameter groups of step 1 (false positives)
  fp_sel.csv     offline re-evaluation of the selected group on the step-1 logs
  fp_bh05.csv    same, with b_hold = 0.5 s
  fp_e05.csv     same, with E = 0.5 m
  det_s2.csv     offline re-evaluation of the 72 groups on the step-2 logs
  det_bh05.csv / det_bh1.csv / det_bh2.csv   same with b_hold = 0.5 / 1.0 / 2.0 s
  results.csv    one row per SITL run (all five steps)
  report.md      section 11.2 table (b_hold = 1/2/3 s false positives and detections)

Run:  python make_figures.py      (writes figures/fig*.png)
"""
import json
import re
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

SEL = {"E": 0.7, "R": 0.6, "T": 1.0, "fc": 0.5, "tau": 30.0}   # group selected in step 3

# fault cells, in the order of the report
CELLS = ["a", "c", "g3", "g5", "g10", "gN", "h", "i", "j1", "j2", "k"]
CELL_LABEL = {"a": "a: climb, 1 s", "c": "c: 3 s low reading", "g3": "g3: 3 s", "g5": "g5: 5 s",
              "g10": "g10: 10 s", "gN": "gN: no recovery", "h": "h: climbing ramp",
              "i": "i: 0.1 m/s slow", "j1": "j1: +1.5 m, 5 s", "j2": "j2: +1.5 m, no recovery",
              "k": "k: pinned at 0.2 m"}
ACTIONS = [("A1", OKABE[0]), ("A2", OKABE[1]), ("A3", OKABE[2]), ("none", OKABE[3])]
ACT_LABEL = {"A1": "A1: warn + LAND", "A2": "A2: switch source + LAND",
             "A3": "A3: switch source only", "none": "none: no monitor"}

# Baro conditions of step 1 (report section 5)
CONDS = ["base", "noise25", "dr03p", "dr03n", "dr12p", "dr12n",
         "st2p", "st2n", "st1p", "st1n", "wind3"]
COND_LABEL = {"base": "base", "noise25": "noise x2.5", "dr03p": "drift +0.3 m/min",
              "dr03n": "drift -0.3 m/min", "dr12p": "drift +1.2 m/min", "dr12n": "drift -1.2 m/min",
              "st2p": "step +0.5 m, 2 s", "st2n": "step -0.5 m, 2 s",
              "st1p": "step +0.5 m, 1 s", "st1n": "step -0.5 m, 1 s", "wind3": "wind 3 m/s"}


def load_grid():
    """fp_grid.md -> DataFrame with the 72 parameter groups."""
    rows = []
    for line in (HERE / "fp_grid.md").read_text().splitlines():
        parts = [p.strip() for p in line.strip().strip("|").split("|")]
        if len(parts) != 10 or not re.fullmatch(r"[0-9.]+", parts[0]):
            continue
        rows.append(dict(fc=float(parts[0]), tau=float(parts[1]), E=float(parts[2]),
                         T=float(parts[3]), R=float(parts[4]), n_trig=int(parts[5]),
                         trigA=int(parts[6]), trigB=int(parts[7]), flight_h=float(parts[8]),
                         per_h=float(parts[9])))
    return pd.DataFrame(rows)


def fp_by_cond(path):
    """per baro condition trigger counts from an offline_eval csv ('runs' column)."""
    d = pd.read_csv(HERE / path)
    runs = json.loads(d.iloc[0]["runs"])
    trig = {c: 0 for c in CONDS}
    tot = {c: 0 for c in CONDS}
    for r in runs:
        cond = r["run"].split("_")[2]
        if cond not in tot:
            continue
        tot[cond] += 1
        if r["t_trig"] is not None:
            trig[cond] += 1
    return trig, tot, float(d.iloc[0]["fp_per_h"]), int(d.iloc[0]["n_trigger"])


def det_row(path, params=None):
    d = pd.read_csv(HERE / path)
    if params is None:
        row = d.iloc[0]
    else:
        sel = d[d["params"].map(lambda s: json.loads(s) == params)]
        row = sel.iloc[0]
    return row


def fig1_grid(grid):
    """Fig 1: false positives per flight hour over the parameter grid (T = 1.0)."""
    fig, axes = plt.subplots(1, 2, figsize=(9.8, 4.2))
    mats = []
    for R in (0.6, 0.4):
        sub = grid[(grid["T"] == 1.0) & (grid["R"] == R)]
        mat = np.full((4, 3), np.nan)
        for i, (fc, tau) in enumerate([(0.5, 10.0), (0.5, 30.0), (1.0, 10.0), (1.0, 30.0)]):
            for j, E in enumerate((0.3, 0.5, 0.7)):
                v = sub[(sub["fc"] == fc) & (sub["tau"] == tau) & (sub["E"] == E)]["per_h"]
                mat[i, j] = v.iloc[0] if len(v) else np.nan
        mats.append(mat)
    vmax = max(np.nanmax(m) for m in mats)
    for ax, mat, R in zip(axes, mats, (0.6, 0.4)):
        im = ax.imshow(mat, cmap="viridis", vmin=0, vmax=vmax, aspect="auto")
        for i in range(4):
            for j in range(3):
                ax.text(j, i, f"{mat[i, j]:.2f}", ha="center", va="center", fontsize=9,
                        color="white" if mat[i, j] > 0.55 * vmax else "black")
        ax.set_xticks(range(3))
        ax.set_xticklabels(["E=0.3", "E=0.5", "E=0.7"])
        ax.set_yticks(range(4))
        ax.set_yticklabels(["fc 0.5, tau 10", "fc 0.5, tau 30", "fc 1.0, tau 10", "fc 1.0, tau 30"])
        ax.set_title(f"R = {R} m, T = 1.0 s")
        ax.grid(False)
    axes[1].set_yticklabels([])
    cb = fig.colorbar(im, ax=axes, fraction=0.03, pad=0.03)
    cb.set_label("False positives (per flight hour)")
    fig.suptitle("False positives per flight hour over the 72 parameter groups (T = 1.0 s)",
                 y=1.02)
    fig.savefig(FIG / "fig1_false_positive_grid.png")
    plt.close(fig)


def fig2_conditions():
    """Fig 2: false positives per baro condition, selected group and two variants."""
    sel_t, sel_n, sel_rate, sel_tot = fp_by_cond("fp_sel.csv")
    bh_t, bh_n, bh_rate, bh_tot = fp_by_cond("fp_bh05.csv")
    e5_t, e5_n, e5_rate, e5_tot = fp_by_cond("fp_e05.csv")
    fig, ax = plt.subplots(figsize=(9.8, 4.6))
    x = np.arange(len(CONDS))
    w = 0.26
    series = [("selected group", sel_t, OKABE[0], sel_rate),
              ("selected group, b_hold 0.5 s", bh_t, OKABE[1], bh_rate),
              ("E = 0.5 m", e5_t, OKABE[2], e5_rate)]
    for k, (lab, data, col, rate) in enumerate(series):
        vals = [data[c] for c in CONDS]
        ax.bar(x + (k - 1) * w, vals, width=w, color=col, label=f"{lab} ({rate:.2f}/h)", zorder=3)
        for xi, v in zip(x + (k - 1) * w, vals):
            if v:
                ax.text(xi, v + 0.15, str(v), ha="center", va="bottom", fontsize=7.5)
    ax.set_xticks(x)
    ax.set_xticklabels([COND_LABEL[c] for c in CONDS], rotation=25, ha="right", fontsize=9)
    ax.set_ylabel("Flights with a monitor trigger (count of 20)")
    ax.set_title("False positives per baro condition (offline replay of the 220 step-1 flights)")
    ax.set_ylim(0, 22)
    ax.legend(loc="upper left", frameon=False, fontsize=8)
    fig.savefig(FIG / "fig2_false_positive_conditions.png")
    plt.close(fig)


def fig3_detection():
    """Fig 3: detection delay per cell and truth deviation at the trigger."""
    row = det_row("det_s2.csv", SEL)
    per_cell = json.loads(row["per_cell"])
    delays = {c: [] for c in CELLS}
    for k, v in per_cell.items():
        cell = k.split("_")[2]
        if v is not None:
            delays[cell].append(v)
    # truth deviation at the trigger, values of the table in report.md section 6
    # (source: report.md, section 6 table "truth deviation at detection [m]")
    dev_lo = {"a": -0.45, "c": -0.003, "g3": 0.01, "g5": 0.01, "g10": -0.01, "gN": 0.03,
              "h": -1.00, "i": 0.79, "j1": 0.03, "j2": 0.02, "k": -1.00}
    dev_hi = {"a": -0.41, "c": 0.05, "g3": 0.06, "g5": 0.05, "g10": 0.05, "gN": 0.07,
              "h": -0.86, "i": 1.09, "j1": 0.06, "j2": 0.04, "k": -0.71}
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.6))
    ax = axes[0]
    x = np.arange(len(CELLS))
    meds = [np.median(delays[c]) for c in CELLS]
    los = [min(delays[c]) for c in CELLS]
    his = [max(delays[c]) for c in CELLS]
    ax.bar(x, meds, width=0.55, color=OKABE[0], zorder=3)
    ax.errorbar(x, meds, yerr=[np.array(meds) - los, np.array(his) - np.array(meds)],
                fmt="none", ecolor="black", capsize=3, lw=1.0, zorder=4)
    for xi, m, h in zip(x, meds, his):
        ax.text(xi, h + 0.25, f"{m:.2f}", ha="center", va="bottom", fontsize=7.5)
    ax.set_xticks(x)
    ax.set_xticklabels([CELL_LABEL[c] for c in CELLS], rotation=25, ha="right", fontsize=8)
    ax.set_ylabel("Detection delay (s)")
    ax.set_title("Detection delay per cell (selected group)")
    ax.set_ylim(0, 12)

    ax = axes[1]
    ax.axhspan(-0.5, 0.5, color=OKABE[2], alpha=0.12)
    meds = [(dev_lo[c] + dev_hi[c]) / 2 for c in CELLS]
    los = [dev_lo[c] for c in CELLS]
    his = [dev_hi[c] for c in CELLS]
    ax.bar(x, meds, width=0.55, color=OKABE[1], zorder=3)
    ax.errorbar(x, meds, yerr=[np.array(meds) - los, np.array(his) - np.array(meds)],
                fmt="none", ecolor="black", capsize=3, lw=1.0, zorder=4)
    ax.axhline(0, color="0.3", lw=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels([CELL_LABEL[c] for c in CELLS], rotation=25, ha="right", fontsize=8)
    ax.set_ylabel("Truth height deviation at the trigger (m)")
    ax.set_title("Truth deviation at the trigger (green band: +-0.5 m)")
    ax.set_ylim(-1.3, 1.3)
    fig.suptitle("Step 2: detection of the 11 fault cells (fc 0.5, tau 30, E 0.7, T 1.0, R 0.6)",
                 y=1.02)
    fig.savefig(FIG / "fig3_detection_delay.png")
    plt.close(fig)


def closed_loop(res):
    """rows of the closed-loop step (SQ mission), grouped by cell and action."""
    s4 = res[res.config.str.startswith("s4_")].copy()
    s4["mission"] = np.where(s4.config.str.startswith("s4_d3_"), "D3", "SQ")
    s4 = s4[s4.mission == "SQ"]
    s4["cell"] = s4.config.str.replace("^s4_", "", regex=True).str.replace(r"_(A1|A2|A3|none)$", "", regex=True)
    s4["act"] = s4.config.str.extract(r"_(A1|A2|A3|none)$")[0]
    return s4


def fig4_closed_loop(s4):
    """Fig 4: truth maximum height per cell and action."""
    fig, ax = plt.subplots(figsize=(9.2, 5.4))
    for k, (act, col) in enumerate(ACTIONS):
        for i, cell in enumerate(CELLS):
            v = s4[(s4.cell == cell) & (s4.act == act)].truth_max_all.dropna().values
            if not len(v):
                continue
            x = i + (k - 1.5) * 0.19
            ax.plot([x - 0.07, x + 0.07], [np.median(v)] * 2, color=col, lw=4, zorder=3)
            xs = x + np.linspace(-0.05, 0.05, len(v))
            ax.plot(xs, v, "o", color=col, ms=3.5, alpha=0.75, zorder=4)
    ax.axhline(2.070, color="0.3", ls="--", lw=1.0, zorder=2)
    ax.text(len(CELLS) - 0.45, 2.12, "no-fault baseline 2.070 m", fontsize=8, color="0.3", ha="right")
    ax.set_xticks(range(len(CELLS)))
    ax.set_xticklabels([CELL_LABEL[c] for c in CELLS], rotation=22, ha="right", fontsize=8)
    ax.set_ylabel("Truth maximum height of the flight (m)")
    ax.set_title("Closed loop: truth maximum height per fault cell and monitor action")
    ax.set_xlim(-0.6, len(CELLS) - 0.4)
    ax.set_ylim(0, 4.6)
    handles = [plt.Line2D([], [], color=col, lw=4, marker="o", ms=4, label=ACT_LABEL[act])
               for act, col in ACTIONS]
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=4,
              frameon=False, fontsize=8)
    fig.savefig(FIG / "fig4_closed_loop_height.png")
    plt.close(fig)


def fig5_latency(s4):
    """Fig 5: injection-to-action delay and task completion of A3."""
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 5.0))
    ax = axes[0]
    acts = [a for a in ACTIONS if a[0] != "none"]
    for k, (act, col) in enumerate(acts):
        for i, cell in enumerate(CELLS):
            v = s4[(s4.cell == cell) & (s4.act == act)].mon_action_delay.dropna().values
            if not len(v):
                continue
            x = i + (k - 1) * 0.22
            ax.plot([x - 0.08, x + 0.08], [np.median(v)] * 2, color=col, lw=4, zorder=3)
            xs = x + np.linspace(-0.06, 0.06, len(v))
            ax.plot(xs, v, "o", color=col, ms=3.5, alpha=0.75, zorder=4)
    ax.set_xticks(range(len(CELLS)))
    ax.set_xticklabels([CELL_LABEL[c] for c in CELLS], rotation=25, ha="right", fontsize=8)
    ax.set_ylabel("Injection to action delay (s)")
    ax.set_title("Injection to action delay (bar: median, dot: one run)")
    ax.set_ylim(0, 12.5)
    handles = [plt.Line2D([], [], color=col, lw=4, marker="o", ms=4, label=ACT_LABEL[act])
               for act, col in acts]
    ax.legend(handles=handles, loc="upper left", frameon=False, fontsize=8)

    ax = axes[1]
    a3 = s4[s4.act == "A3"]
    vals = []
    for cell in CELLS:
        sub = a3[a3.cell == cell]
        vals.append(int(sub.mission_complete.sum()))
    x = np.arange(len(CELLS))
    ax.bar(x, vals, width=0.55, color=OKABE[2], zorder=3)
    for xi, v in zip(x, vals):
        ax.text(xi, v + 0.08, f"{v}/5", ha="center", va="bottom", fontsize=8)
    ax.set_xticks(x)
    ax.set_xticklabels([CELL_LABEL[c] for c in CELLS], rotation=25, ha="right", fontsize=8)
    ax.set_ylabel("Missions completed (count of 5)")
    ax.set_title("Task completion of action A3 (switch source only)")
    ax.set_ylim(0, 6)
    fig.suptitle("Closed loop: monitor action latency and task completion", y=1.02)
    fig.savefig(FIG / "fig5_action_latency.png")
    plt.close(fig)


def fig6_source_switch(s4):
    """Fig 6: EKF height jump against the real height change at the source switch."""
    sub = s4[s4.act.isin(["A2", "A3"])]
    fig, ax = plt.subplots(figsize=(8.0, 6.4))
    for act, col, marker in (("A2", OKABE[0], "o"), ("A3", OKABE[1], "s")):
        for _, r in sub[sub.act == act].iterrows():
            if not (np.isfinite(r.mon_ekf_jump) and np.isfinite(r.mon_truth_change)):
                continue
            ax.plot(r.mon_truth_change, r.mon_ekf_jump, marker, color=col, ms=5, alpha=0.75,
                    zorder=3, label=ACT_LABEL[act] if False else None)
    ax.plot([], [], "o", color=OKABE[0], ms=6, label="A2: switch source + LAND")
    ax.plot([], [], "s", color=OKABE[1], ms=6, label="A3: switch source only")
    lim = [-0.7, 0.75]
    ax.plot(lim, lim, color="0.3", ls="--", lw=1.0)
    ax.text(lim[1] - 0.02, lim[1] - 0.03, "EKF jump = truth change", fontsize=8, color="0.3",
            ha="right", va="top")
    ax.set_xlim(lim)
    ax.set_ylim(lim)
    ax.set_xlabel("Real height change at the switch (m)")
    ax.set_ylabel("EKF height jump at the switch (m)")
    ax.set_title("Source switch: EKF height estimate jump against the real height change")
    ax.legend(loc="upper left", frameon=False, fontsize=8)
    fig.savefig(FIG / "fig6_source_switch_jump.png")
    plt.close(fig)


def fig7_step5(res):
    """Fig 7: step 5, the monitor failing itself."""
    s5 = res[res.config.str.startswith("s5_")]
    gnn = res[res.config == "s4_gN_none"].truth_max_all.dropna().values
    d3 = res[res.config == "s4_d3_gN_none"].truth_max_all.dropna().values
    groups = [("p0 bias 0.3 m", "s5_p0", OKABE[0]),
              ("baro stream stops", "s5_press", OKABE[1]),
              ("monitor process stops", "s5_stop", OKABE[3])]
    fig, ax = plt.subplots(figsize=(8.4, 4.8))
    for i, (lab, cfg, col) in enumerate(groups):
        v = s5[s5.config == cfg].truth_max_all.dropna().values
        x = i + np.linspace(-0.12, 0.12, len(v))
        ax.plot(x, v, "o", color=col, ms=8, zorder=4)
    ax.axhline(np.median(gnn), color=OKABE[2], ls="--", lw=1.2)
    ax.text(-0.45, np.median(gnn) + 0.06, f"gN without monitor (SQ): {np.median(gnn):.2f} m",
            fontsize=8, color=OKABE[2], ha="left")
    ax.axhline(np.median(d3), color=OKABE[4], ls=":", lw=1.2)
    ax.text(-0.45, np.median(d3) - 0.16, f"gN without monitor (D3): {np.median(d3):.2f} m",
            fontsize=8, color=OKABE[4], ha="left")
    ax.set_xticks(range(3))
    ax.set_xticklabels([g[0] for g in groups], fontsize=9)
    ax.set_xlim(-0.5, 2.5)
    ax.set_ylim(1.8, 4.3)
    ax.set_ylabel("Truth maximum height of the flight (m)")
    ax.set_title("Step 5: truth maximum height when the monitor itself fails (3 runs each)")
    handles = [plt.Line2D([], [], color=col, marker="o", ls="none", ms=7, label=lab)
               for lab, cfg, col in groups]
    ax.legend(handles=handles, loc="center left", frameon=False, fontsize=8)
    fig.savefig(FIG / "fig7_step5_self_failure.png")
    plt.close(fig)


def fig8_bhold():
    """Fig 8: b_hold trade-off (false positives vs detection)."""
    rows = []
    rows.append((0.0, "fp_sel.csv", "det_s2.csv", SEL))
    rows.append((0.5, "fp_bh05.csv", "det_bh05.csv", None))
    rows.append((1.0, None, "det_bh1.csv", None))
    rows.append((2.0, None, "det_bh2.csv", None))
    rows.append((3.0, None, None, None))
    # b_hold = 1/2/3 s false-positive counts come from the table in report.md section 11.2
    fp_extra = {1.0: (0, 0.00), 2.0: (0, 0.00), 3.0: (0, 0.00)}
    det_extra = {3.0: (50, 1.26, 10.2)}     # report.md section 11.2 table
    fp_rate, fp_n, det_n, det_med, det_max = [], [], [], [], []
    for bh, fpc, detc, params in rows:
        if fpc:
            _, _, rate, ntr = fp_by_cond(fpc)
        else:
            ntr, rate = fp_extra[bh]
        if bh in det_extra:
            n, med, mx = det_extra[bh]
        else:
            r = det_row(detc, params)
            n, med, mx = int(r["n_det"]), float(r["delay_med"]), float(r["delay_max"])
        fp_rate.append(rate)
        fp_n.append(ntr)
        det_n.append(n)
        det_med.append(med)
        det_max.append(mx)
    x = np.array([0.0, 0.5, 1.0, 2.0, 3.0])
    fig, axes = plt.subplots(1, 3, figsize=(10.6, 3.9))
    ax = axes[0]
    ax.bar(x, fp_rate, width=0.4, color=OKABE[0], zorder=3)
    for xi, v, n in zip(x, fp_rate, fp_n):
        ax.text(xi, v + 0.05, f"{v:.2f}\n({n}/220)", ha="center", va="bottom", fontsize=7.5)
    ax.set_xlabel("b_hold (s)")
    ax.set_ylabel("False positives (per flight hour)")
    ax.set_title("False positives vs b_hold")
    ax.set_ylim(0, 2.6)

    ax = axes[1]
    ax.bar(x, det_n, width=0.4, color=OKABE[1], zorder=3)
    for xi, v in zip(x, det_n):
        ax.text(xi, v + 0.6, f"{v}/55", ha="center", va="bottom", fontsize=7.5)
    ax.set_xlabel("b_hold (s)")
    ax.set_ylabel("Detections (count of 55)")
    ax.set_title("Detection completeness vs b_hold")
    ax.set_ylim(0, 62)

    ax = axes[2]
    ax.plot(x, det_med, "o-", color=OKABE[2], label="median delay")
    ax.plot(x, det_max, "s--", color=OKABE[3], label="max delay")
    for xi, m in zip(x, det_med):
        ax.text(xi, m + 0.25, f"{m:.2f}", ha="center", va="bottom", fontsize=7.5)
    ax.set_xlabel("b_hold (s)")
    ax.set_ylabel("Detection delay (s)")
    ax.set_title("Detection delay vs b_hold")
    ax.set_ylim(-0.4, 12)
    ax.legend(loc="upper left", bbox_to_anchor=(0.02, 0.72), frameon=False, fontsize=8)
    fig.suptitle("Suggestion 1: criterion B needs to hold b_hold seconds (offline replay)",
                 y=1.04)
    fig.savefig(FIG / "fig8_bhold_tradeoff.png")
    plt.close(fig)


def main():
    grid = load_grid()
    res = pd.read_csv(HERE / "results.csv")
    s4 = closed_loop(res)
    fig1_grid(grid)
    fig2_conditions()
    fig3_detection()
    fig4_closed_loop(s4)
    fig5_latency(s4)
    fig6_source_switch(s4)
    fig7_step5(res)
    fig8_bhold()
    print("figures written to", FIG)
    for p in sorted(FIG.glob("*.png")):
        print(" ", p.name, p.stat().st_size // 1024, "kB")


if __name__ == "__main__":
    main()
