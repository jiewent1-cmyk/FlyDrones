"""Figures for README.md (SITL rehearsal of the 2026-11-19 no-GPS AUTO mission, runs r3..r6).
Reads run_r3.log .. run_r6.log next to this file and writes PNG files to figures/.

    python make_figures.py
"""
from pathlib import Path
import json
import re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

OKABE = ["#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#56B4E9", "#F0E442", "#000000"]
plt.rcParams.update({"figure.dpi": 150, "savefig.dpi": 150, "savefig.bbox": "tight",
    "font.size": 10, "axes.titlesize": 11, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.alpha": 0.3, "axes.prop_cycle": matplotlib.cycler(color=OKABE),
    "figure.facecolor": "white"})
HERE = Path(__file__).parent
OUT = HERE / "figures"
OUT.mkdir(exist_ok=True)
PASS, FAIL = "#009E73", "#D55E00"
RUNS = ["r3", "r4", "r5", "r6"]
FENCE_ALT_MAX = 1.3  # field_sitl.parm


def parse(run):
    text = (HERE / f"run_{run}.log").read_text()
    d = {"run": run, "items": {}, "params": {}}
    for line in text.splitlines():
        m = re.search(r"mission item (\d+) REACHED  \(x ([-\d.]+) y ([-\d.]+) alt ([-\d.]+)\)", line)
        if m:
            d["items"][int(m[1])] = tuple(float(m[i]) for i in (2, 3, 4))
        m = re.search(r"uploading D3: .* alt ([\d.]+) m, side ([\d.]+) m, nose yaw (-?\d+) deg", line)
        if m:
            d["alt_cmd"], d["side"], d["yaw"] = float(m[1]), float(m[2]), int(m[3])
        m = re.search(r"\] param (\w+)\s+\S+ -> (\S+)", line)
        if m:
            d["params"][m[1]] = float(m[2])
        m = re.search(r"LOITER climb: alt ([\d.]+) m", line)
        if m:
            d["loiter_climb"] = float(m[1])
        m = re.search(r"LOITER stick centred: peak alt ([\d.]+) m", line)
        if m:
            d["loiter_peak"] = float(m[1])
    d["summary"] = json.loads(text[text.index("\n{") + 1:text.index("\n}\n") + 2])
    return d


D = [parse(r) for r in RUNS]
COL = dict(zip(RUNS, [OKABE[0], OKABE[1], OKABE[4], OKABE[5]]))
MK = dict(zip(RUNS, ["o", "s", "^", "D"]))


def label(d):
    return f"{d['run']}: alt {d['alt_cmd']:.1f} m, WP_SPD_UP {d['params']['WP_SPD_UP']:g}"


def save(fig, name):
    fig.savefig(OUT / name)
    plt.close(fig)


# fig1: plan view, where each waypoint item was reported REACHED
fig, ax = plt.subplots(figsize=(5.4, 5.2))
side = D[0]["side"]
ax.plot([0, side, side, 0, 0], [0, 0, side, side, 0], color="0.5", lw=1, ls="--", label="commanded square (1 m side)")
for d in D:
    pts = [d["items"][i] for i in (2, 3, 4, 5)]
    ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=45, marker=MK[d["run"]], color=COL[d["run"]],
               edgecolor="k", linewidth=0.5, label=label(d), zorder=3)
ax.scatter([0], [0], s=70, marker="*", color="k", label="start / land point", zorder=4)
ax.set_xlabel("x, ahead of the nose at start (m)")
ax.set_ylabel("y, right of the nose at start (m)")
ax.set_aspect("equal")
ax.set_title("Position when waypoints 2-5 were reported reached")
ax.legend(fontsize=7.5, loc="upper center", bbox_to_anchor=(0.5, -0.15))
save(fig, "fig1_waypoints_xy.png")

# fig2: altitude at each reported item against the commanded altitude
fig, ax = plt.subplots(figsize=(6.6, 4.2))
for j, d in enumerate(D):
    xs = [1, 2, 3, 4, 5]
    ys = [d["items"][i][2] for i in xs]
    ax.plot(xs, ys, marker=MK[d["run"]], color=COL[d["run"]], label=label(d))
ax.axhline(FENCE_ALT_MAX, color=FAIL, lw=1, ls="--", label=f"FENCE_ALT_MAX {FENCE_ALT_MAX} m")
for a_cmd in sorted({d["alt_cmd"] for d in D}):
    ax.axhline(a_cmd, color="0.4", ls=":", lw=1, label=f"commanded altitude {a_cmd:.1f} m")
ax.set_xticks([1, 2, 3, 4, 5], ["1 takeoff", "2 wp", "3 wp", "4 wp", "5 wp"])
ax.set_xlabel("mission item")
ax.set_ylabel("altitude when reported reached (m)")
ax.set_ylim(0.4, 1.4)
ax.set_title("Altitude at each item: takeoff item ~1.1 m, waypoints ~0.08 m above command")
ax.legend(fontsize=7.5, loc="upper right", ncol=1)
save(fig, "fig2_altitude_per_item.png")

# fig3: manual LOITER take-off test, before the AUTO mission
fig, ax = plt.subplots(figsize=(6.6, 4.0))
w = 0.26
for j, d in enumerate(D):
    vals = [d["loiter_climb"], d["loiter_peak"] if "loiter_peak" in d else None, d["summary"]["loiter"]["alt_m"]]
    armed = d["summary"]["loiter"]["armed"]
    for k, v in enumerate(vals):
        if v is None:
            continue
        ax.bar(j + (k - 1) * w, v, w, color=OKABE[k], edgecolor="k", linewidth=0.4)
    if not armed:
        ax.text(j, 0.14, "not armed:\nAccels\ninconsistent", ha="center", fontsize=7, color=FAIL)
handles = [Patch(color=OKABE[0], label="altitude after the climb phase"),
           Patch(color=OKABE[1], label="peak altitude, stick centred"),
           Patch(color=OKABE[2], label="altitude at end of 15 s hover")]
ax.set_xticks(range(4), [f"{d['run']}\n{d['params'].get('FENCE_MARGIN', 2.0):g}" for d in D])
ax.set_xlabel("run, and FENCE_MARGIN in m (2 = default, not overridden)")
ax.set_ylabel("altitude (m)")
ax.set_ylim(0, 1.2)
ax.set_title("Manual LOITER take-off test (r3: not armed, no peak logged)")
ax.legend(handles=handles, fontsize=7.5, loc="upper left")
save(fig, "fig3_loiter_test.png")
