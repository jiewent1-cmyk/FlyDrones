"""Figures for README.md (Gazebo console verification, planner on / off). Reads verify/pv_result.json (output of
verify/analyze.py --json) and verify/pv_truth.json (output of verify/analyze_truth.py --json) and writes PNG files to
figures/.

    python make_figures.py
"""
from collections import Counter, defaultdict
from pathlib import Path
import json
import sys
import numpy as np
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
PASS, FAIL = "#009E73", "#D55E00"
COND = {"noplan": "planner off", "plan": "planner on"}
CCOL = {"noplan": OKABE[1], "plan": OKABE[0]}
KINDS = ["on", "adj", "behind", "open", "tour6", "gap", "ops"]  # route kinds (tag prefix); "refuse" is not flown
KLAB = {"on": "goal on a box", "adj": "goal next to a box", "behind": "goal behind a box", "open": "open floor",
        "tour6": "6-point tour", "gap": "gap", "ops": "operator actions"}
rng = np.random.default_rng(0)

for name in ("pv_result.json", "pv_truth.json"):
    if not (HERE / "verify" / name).exists():
        sys.exit(f"verify/{name} is missing: copy it from the verification run directory first")
R = json.load(open(HERE / "verify" / "pv_result.json"))
T = json.load(open(HERE / "verify" / "pv_truth.json"))
S = [s for s in R["steps"] if s["kind"] != "refuse"]
kinds = [k for k in KINDS if any(s["kind"] == k for s in S)]


def save(fig, name):
    fig.savefig(OUT / name)
    plt.close(fig)


# fig1: routes completed, per kind and condition (counts: completed / not completed)
fig, ax = plt.subplots(figsize=(9, 4.3))
w = 0.38
for j, c in enumerate(("noplan", "plan")):
    for i, k in enumerate(kinds):
        g = [s for s in S if s["cond"] == c and s["kind"] == k]
        done = sum(bool(s["done"]) for s in g)
        x = i + (j - 0.5) * w
        ax.bar(x, done, w, color=PASS, edgecolor="k", linewidth=0.4, hatch="" if c == "plan" else "//")
        ax.bar(x, len(g) - done, w, bottom=done, color=FAIL, edgecolor="k", linewidth=0.4, hatch="" if c == "plan" else "//")
        ax.text(x, len(g) + 0.5, f"{done}/{len(g)}", ha="center", fontsize=7.5)
ax.set_xticks(range(len(kinds)), [KLAB[k] for k in kinds], rotation=15)
ax.set_ylabel("routes (count)")
ax.set_ylim(0, ax.get_ylim()[1] * 1.08)
ax.set_title("Routes completed per route kind: planner off (hatched) and on")
from matplotlib.patches import Patch
ax.legend(handles=[Patch(facecolor=PASS, edgecolor="k", label="completed"), Patch(facecolor=FAIL, edgecolor="k", label="not completed"),
                   Patch(facecolor="w", edgecolor="k", hatch="//", label="planner off (left bar)"),
                   Patch(facecolor="w", edgecolor="k", label="planner on (right bar)")], fontsize=8, loc="upper right")
save(fig, "fig1_routes_completed.png")

# fig2: minimum truth clearance per route, contacts marked
fig, ax = plt.subplots(figsize=(9, 4.5))
for j, c in enumerate(("noplan", "plan")):
    for i, k in enumerate(kinds):
        g = [s for s in S if s["cond"] == c and s["kind"] == k and s["min_clear"] is not None]
        x = i + (j - 0.5) * w + rng.uniform(-0.08, 0.08, len(g))
        y = np.array([s["min_clear"] for s in g])
        hit = np.array([s["contacts"] > 0 for s in g], dtype=bool)
        ax.scatter(x[~hit], y[~hit], s=14, color=CCOL[c], alpha=0.8, edgecolor="none")
        ax.scatter(x[hit], y[hit], s=40, color=CCOL[c], marker="X", edgecolor="k", linewidth=0.5)
ax.axhline(0, color=FAIL, lw=1)
ax.scatter([], [], s=14, color=CCOL["noplan"], label=f"{COND['noplan']}: one route")
ax.scatter([], [], s=14, color=CCOL["plan"], label=f"{COND['plan']}: one route")
ax.scatter([], [], s=40, color="0.5", marker="X", edgecolor="k", linewidth=0.5, label="route with a contact episode")
ax.plot([], [], color=FAIL, lw=1, label="clearance 0 (contact)")
ax.set_xticks(range(len(kinds)), [KLAB[k] for k in kinds], rotation=15)
ax.set_ylabel("minimum clearance to a box during the route (m)")
ax.set_title("Minimum Gazebo-truth clearance per route (left: planner off, right: on)")
ax.legend(fontsize=8, loc="upper right")
save(fig, "fig2_min_clearance.png")

# fig3: planner time per session (planner-on sessions only)
P = [s for s in R["sessions"] if s["cond"] == "plan" and s.get("plan_ms_p50") is not None]
if P:
    fig, ax = plt.subplots(figsize=(6.2, 4.3))
    for i, (key, lab) in enumerate((("plan_ms_p50", "median (p50)"), ("plan_ms_p99", "p99"), ("plan_ms_max", "maximum"))):
        y = np.array([s[key] for s in P], dtype=float)
        ax.scatter(i + rng.uniform(-0.12, 0.12, len(y)), y, s=16, color=OKABE[i], alpha=0.85, label=f"{lab}: one session")
        ax.plot([i - 0.25, i + 0.25], [np.median(y)] * 2, color="k", lw=1.5)
    ax.plot([], [], color="k", lw=1.5, label="median over sessions")
    ax.set_xticks(range(3), ["p50", "p99", "max"])
    ax.set_ylabel("time of one planner call (ms)")
    ax.set_yscale("log")
    ax.set_title(f"Planner call time, {len(P)} planner-on sessions")
    ax.legend(fontsize=8, loc="upper left")
    save(fig, "fig3_planning_time.png")

# fig4: truth-based waypoint reach rate per kind and condition (analyze_truth.py)
W = [x for x in T["waypoints"] if x["kind"] in kinds]
fig, ax = plt.subplots(figsize=(9, 4.3))
for j, c in enumerate(("noplan", "plan")):
    for i, k in enumerate(kinds):
        g = [x for x in W if x["cond"] == c and x["kind"] == k]
        if not g:
            continue
        r = 100 * np.mean([x["ok"] for x in g])
        ax.bar(i + (j - 0.5) * w, r, w, color=CCOL[c], edgecolor="k", linewidth=0.4, label=COND[c] if i == 0 else None)
        ax.text(i + (j - 0.5) * w, r + 1.5, f"{sum(x['ok'] for x in g)}/{len(g)}", ha="center", fontsize=7.5)
ax.set_xticks(range(len(kinds)), [KLAB[k] for k in kinds], rotation=15)
ax.set_ylabel("route waypoints reached (%)")
ax.set_ylim(0, 112)
ax.set_title("Waypoints reached by EKF position (within 0.5 m, or of the nearest safe point), per route kind")
ax.legend(fontsize=8, loc="lower right")
save(fig, "fig4_truth_waypoints.png")

# fig5: truth-based miss rate against the waypoint's distance to the fence
fig, ax = plt.subplots(figsize=(6.4, 4.2))
edges = [0, 0.7, 1.2, 9]
labs = ["< 0.7 m", "0.7 - 1.2 m", ">= 1.2 m"]
for j, c in enumerate(("noplan", "plan")):
    for i, (lo, hi) in enumerate(zip(edges[:-1], edges[1:])):
        g = [x for x in T["waypoints"] if x["cond"] == c and lo <= x["fence_gap"] < hi]
        if not g:
            continue
        r = 100 * np.mean([x["ok"] for x in g])
        ax.bar(i + (j - 0.5) * w, r, w, color=CCOL[c], edgecolor="k", linewidth=0.4, label=COND[c] if i == 0 else None)
        ax.text(i + (j - 0.5) * w, r + 1.5, f"n={len(g)}", ha="center", fontsize=7.5)
ax.set_xticks(range(3), labs)
ax.set_xlabel("distance of the waypoint to the fence")
ax.set_ylabel("waypoints reached (%)")
ax.set_ylim(0, 112)
ax.set_title("Waypoints reached against fence distance")
ax.legend(fontsize=8, loc="lower right")
save(fig, "fig5_reach_vs_fence.png")

# console summary, as printed by the analysis scripts, to check the figures against the README
print("routes:", Counter((s["cond"], s["kind"]) for s in S))
print("contacts (truth):", Counter(c["cond"] for c in T["contacts"]))
