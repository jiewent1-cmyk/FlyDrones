#!/usr/bin/env python3
"""make_figures.py -- figures for the V8 directory (real HeightGuard validation).

Data source (read next to this script):
  jobs_v8.txt     one planned SITL run per line: tag|parm|speedup|yaw|overlay|args

There is no measured result file in this directory yet, so every number below is
taken from that job list: how many runs each condition has, which range-finder
fault (SIM_SONAR_POS_Z) is injected when, and how yaw / wind direction are
covered.  The figures therefore describe the *plan*, not measured behaviour.

Run:  python make_figures.py      (writes figures/fig*.png)
"""
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OKABE = ["#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#56B4E9", "#F0E442", "#000000"]
plt.rcParams.update({"figure.dpi": 150, "savefig.dpi": 150, "savefig.bbox": "tight",
    "font.size": 10, "axes.titlesize": 11, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.alpha": 0.3, "axes.prop_cycle": matplotlib.cycler(color=OKABE),
    "figure.facecolor": "white"})

HERE = Path(__file__).resolve().parent
FIG = HERE / "figures"
FIG.mkdir(exist_ok=True)

# display name of every condition found in the job list, in the order used in the figures
LABELS = {
    "clean": "clean (no fault)",
    "a": "a", "c": "c", "g5": "g5", "gN": "gN", "h": "h", "i": "i", "j2": "j2", "k": "k",
    "d3_gN": "D3 gN", "d3_h": "D3 h", "d3_k": "D3 k",
}
SQ_ORDER = ["clean", "a", "c", "g5", "gN", "h", "i", "j2", "k"]
D3_ORDER = ["d3_gN", "d3_h", "d3_k"]


def parse_jobs(path):
    """Return {condition: [run dicts]} from a job list file."""
    runs = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        tag, parm, spd, yaw, ovl, args = line.split("|", 5)
        faults = [(float(t), float(v)) for t, v in
                  re.findall(r"--fault ([0-9.]+):SIM_SONAR_POS_Z=([0-9.\-]+)", args)]
        wind = re.search(r"SIM_WIND_DIR=(\d+)", ovl)
        runs.append({
            "tag": tag,
            "cond": re.sub(r"_hg$", "", re.sub(r"^v8_", "", re.sub(r"_r\d+$", "", tag))),
            "mission": args.split()[0],
            "yaw": int(yaw),
            "wind": int(wind.group(1)) if wind else None,
            "faults": faults,
        })
    out = {}
    for r in runs:
        out.setdefault(r["cond"], []).append(r)
    return out


def ladder(cond_runs):
    """Fault schedule of a condition: [(t, z)] (identical in every run of the condition)."""
    return cond_runs[0]["faults"]


def fig2_fault_schedule(conds):
    """Fig 2: injected rangefinder reading (SIM_SONAR_POS_Z) against mission time."""
    order = [c for c in SQ_ORDER if c in conds] + [c for c in D3_ORDER if c in conds]
    fig, ax = plt.subplots(figsize=(8.6, 5.0))
    color_of = {}
    for i, c in enumerate(order):
        f = ladder(conds[c])
        if not f:
            continue
        sq = c in SQ_ORDER
        key = tuple(f)
        if key not in color_of:
            color_of[key] = OKABE[len(color_of) % len(OKABE)]
        color = color_of[key]
        t = [p[0] for p in f] + [f[-1][0] + 3.0]
        z = [p[1] for p in f] + [f[-1][1]]
        ax.step(t, z, where="post", color=color, lw=0.9 if len(f) > 20 else 1.8,
                ls="-" if sq else "--",
                alpha=0.45 if len(f) > 20 else (1.0 if sq else 0.8),
                label=LABELS.get(c, c) if sq else None,
                zorder=1 if len(f) > 20 else (3 if sq else 2))
    ax.axhline(0.0, color="0.55", lw=1.0, ls="--", zorder=1)
    ax.set_xlim(0, 64)
    ax.set_xlabel("Mission time of the injection (s)")
    ax.set_ylabel("SIM_SONAR_POS_Z set by --fault (m)")
    ax.set_title("SIM_SONAR_POS_Z schedule per condition (from the job list, no measurements)")
    handles, labels = ax.get_legend_handles_labels()
    handles.append(plt.Line2D([], [], color="0.4", lw=1.8, ls="--"))
    labels.append("D3 conditions (same schedules as SQ)")
    ax.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=4,
              frameon=False, fontsize=8)
    fig.savefig(FIG / "fig1_fault_schedule.png")
    plt.close(fig)


def main():
    conds = parse_jobs(HERE / "jobs_v8.txt")
    fig2_fault_schedule(conds)
    print("figures written to", FIG)
    for p in sorted(FIG.glob("*.png")):
        print(" ", p.name, p.stat().st_size // 1024, "kB")


if __name__ == "__main__":
    main()
