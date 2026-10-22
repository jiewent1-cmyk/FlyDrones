"""Plans + queue for the planner verification (2026-10-22).

usage: make_queue.py <layouts dir> <out dir> <plans dir on ROG>   -> <out dir>/plans/*.json, <out dir>/queue.txt

Wave 1: 33 worlds (cage20, wall_offset, cage10, lowbox_s0-29), route seed 1, planner on and off (paired, adjacent in
the queue so both run under similar load). Ops: 7 worlds, planner on: a route changed mid-flight, an emergency LAND
mid-route -> reset -> re-take-off -> a new route. Wave 2: lowbox_s0-29 with route seed 2, on and off.
"""

import json
import os
import subprocess
import sys

lay, out, rplans = sys.argv[1], sys.argv[2], sys.argv[3]
here = os.path.dirname(os.path.abspath(__file__))
os.makedirs(f"{out}/plans", exist_ok=True)
worlds = ["cage20", "wall_offset", "cage10"] + [f"lowbox_s{i}" for i in range(30)]
queue = []


def gen(w: str, seed: int) -> dict:
    r = subprocess.run([sys.executable, f"{here}/gen_routes.py", f"{lay}/ecps295_{w}.layout.json", str(seed)],
                       capture_output=True, text=True, check=True)
    return json.loads(r.stdout)


def put(name: str, plan: dict) -> str:
    json.dump(plan, open(f"{out}/plans/{name}.json", "w"), indent=1)
    return f"{rplans}/{name}.json"


for w in worlds:
    p = put(f"{w}_r1", gen(w, 1))
    for c in ("plan", "noplan"):
        queue.append(f"{w} {c} {p} pv1_{c}__{w}")
for w in ["cage20", "wall_offset"] + [f"lowbox_s{i}" for i in range(5)]:
    g = gen(w, 7)
    by = {s["tag"].split("_")[0]: s for s in g["steps"]}
    tour, opn = by.get("tour6"), by.get("open")
    other = by.get("behind") or by.get("adj") or by.get("on")
    steps = []
    if tour and opn:
        steps.append({"op": "route", "tag": "ops_change", "pts": tour["pts"], "change_after": 8.0, "change_pts": opn["pts"]})
    if other:
        after = (by.get("on") or by.get("adj") or opn)["pts"]
        steps.append({"op": "route", "tag": "ops_estop", "pts": other["pts"] + (opn["pts"] if opn else []), "estop_after": 6.0,
                      "after_pts": after})
    if tour:
        steps.append({"op": "route", "tag": "ops_tour_after", "pts": tour["pts"]})
    p = put(f"{w}_ops", {"world": g["world"], "fence_m": g["fence_m"], "steps": steps})
    queue.append(f"{w} plan {p} pvops_plan__{w}")
for w in [f"lowbox_s{i}" for i in range(30)]:
    p = put(f"{w}_r2", gen(w, 2))
    for c in ("plan", "noplan"):
        queue.append(f"{w} {c} {p} pv2_{c}__{w}")
open(f"{out}/queue.txt", "w").write("\n".join(queue) + "\n")
n_steps = sum(len(json.load(open(f"{out}/plans/{f}"))["steps"]) for f in os.listdir(f"{out}/plans"))
print(f"{len(queue)} sessions, {len(os.listdir(out + '/plans'))} plans, {n_steps} plan steps")
