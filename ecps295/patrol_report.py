"""Top-view map and near-miss list for a run_g4.py patrol (or any) run with a world layout.

python patrol_report.py RUN_DIR/T7_patrol.csv RUN_DIR/T7_patrol.json LAYOUT.json --png map.png
"""

import argparse
import csv
import json
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

PROP_TIP_R = 0.13

ap = argparse.ArgumentParser()
ap.add_argument("csv")
ap.add_argument("json")
ap.add_argument("layout")
ap.add_argument("--png", default="patrol_map.png")
ap.add_argument("--near", type=float, default=0.1)
a = ap.parse_args()

rows = list(csv.DictReader(open(a.csv)))
summ = json.load(open(a.json))
lay = json.load(open(a.layout))
boxes = []  # NED (north, east) AABBs
for b in lay["boxes"]:
    (e, n, z), (se, sn, sz) = b["center_enu"], b["size_enu"]
    boxes.append((b["name"], n - sn / 2, n + sn / 2, e - se / 2, e + se / 2, z + sz / 2))


def nearest(nn, ee, alt):
    best = (float("inf"), "-")
    for name, n0, n1, e0, e1, top in boxes:
        if top < alt - 0.03:
            continue
        dn, de = max(n0 - nn, 0, nn - n1), max(e0 - ee, 0, ee - e1)
        best = min(best, (math.hypot(dn, de) - PROP_TIP_R, name))
    return best


# near-miss episodes: clearance dips below a.near; report the minimum of each dip
episodes, cur = [], None
for r in rows:
    c = float(r["clearance"])
    if c < a.near:
        if cur is None or c < cur[0]:
            cur = (c, r)
    elif cur is not None:
        episodes.append(cur)
        cur = None
if cur is not None:
    episodes.append(cur)
print(f"{len(episodes)} episodes below {a.near} m")
for c, r in episodes:
    d, name = nearest(float(r["north"]), float(r["east"]), float(r["alt"]))
    print(
        f"  t={float(r['t']):6.1f}s clearance {c:.3f} m to {name:10s} at N={float(r['north']):+.2f} E={float(r['east']):+.2f} "
        f"alt={float(r['alt']):.2f} fwd={float(r['cmd_forward']):+.2f} yaw={float(r['cmd_yaw']):+.2f} escape={r['escape']}"
    )

fig, ax = plt.subplots(figsize=(7, 7))
for name, n0, n1, e0, e1, _top in boxes:  # plot east to the right, north up
    ax.add_patch(Rectangle((e0, n0), e1 - e0, n1 - n0, color="#8a6d3b" if not name.startswith("net") else "#555", alpha=0.8))
    if not name.startswith("net"):
        ax.text((e0 + e1) / 2, (n0 + n1) / 2, name, fontsize=7, ha="center", va="center", color="white")
fence = summ.get("geofence_radius_m", 2.0)
ax.add_patch(Circle((0, 0), fence, fill=False, ls="--", ec="tab:red", label=f"geofence {fence} m"))
E = [float(r["east"]) for r in rows]
N = [float(r["north"]) for r in rows]
T = [float(r["t"]) for r in rows]
sc = ax.scatter(E, N, c=T, s=3, cmap="viridis")
fig.colorbar(sc, ax=ax, label="t [s]", shrink=0.7)
for t_esc, _trig, d in summ.get("escape_log", []):
    r = min(rows, key=lambda r: abs(float(r["t"]) - t_esc))
    ax.plot(float(r["east"]), float(r["north"]), marker="<" if d < 0 else ">", ms=9, color="tab:orange")
for _c, r in episodes:
    ax.plot(float(r["east"]), float(r["north"]), "x", ms=10, mew=2, color="tab:red")
ax.plot(0, 0, "k^", ms=8, label="take-off")
ax.set_aspect("equal")
ax.set_xlabel("east [m]")
ax.set_ylabel("north [m]")
half = lay["floor_m"] / 2
ax.set_xlim(-half, half)
ax.set_ylim(-half, half)
ax.set_title(
    f"{summ.get('world', '')} {summ.get('variant', '')}: path {summ.get('path_m')} m, coverage {summ.get('coverage')}, "
    f"contacts {summ.get('contact_episodes')}\norange = saccade (< left, > right), red x = near miss < {a.near} m",
    fontsize=9,
)
ax.legend(loc="lower left", fontsize=8)
fig.tight_layout()
fig.savefig(a.png, dpi=110)
print("map ->", a.png)
