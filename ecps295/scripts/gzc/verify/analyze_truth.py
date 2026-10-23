"""Truth-based waypoint accounting for the planner verification (2026-10-22), independent of the console event log.

usage: analyze_truth.py <runs dir> <tag glob> <layouts dir> [--json out.json]

Per route waypoint: the closest the drone came (Gazebo truth and EKF) during its step, and the closest a drone can
safely get (`reach_need`: distance from the waypoint to the nearest point whose prop tip clears every box by
SAFE_CLEAR_M and lies inside the go-to range). A waypoint counts as reached when the EKF came within ARRIVE_M of it,
or within reach_need + ARRIVE_M when it sits on / next to an obstacle. Contacts are listed with their context.
"""

from __future__ import annotations

import csv
import glob
import json
import math
import os
import sys
from collections import defaultdict

import numpy as np

ARRIVE_M = 0.5
SAFE_CLEAR_M = 0.3  # prop tip to box surface
PROP_TIP_R = 0.13


def boxes(layout):
    out = []
    for b in layout["boxes"]:
        (e, n, u), (se, sn, su) = b["center_enu"], b["size_enu"]
        out.append((b["name"], n - sn / 2, n + sn / 2, e - se / 2, e + se / 2, u + su / 2))
    return out


def surf(B, n, e):
    best, who = math.inf, ""
    for name, n0, n1, e0, e1, top in B:
        d = math.hypot(max(n0 - n, 0, n - n1), max(e0 - e, 0, e - e1))
        if d < best:
            best, who = d, name
    return best, who


def reach_need(B, p, lim):
    """Distance from p to the nearest safe point (grid search, 5 cm)."""
    if surf(B, *p)[0] - PROP_TIP_R >= SAFE_CLEAR_M:
        return 0.0
    best = math.inf
    for dn in np.arange(-1.5, 1.51, 0.05):
        for de in np.arange(-1.5, 1.51, 0.05):
            q = (p[0] + dn, p[1] + de)
            r = math.hypot(dn, de)
            if r < best and math.hypot(*q) <= lim and surf(B, *q)[0] - PROP_TIP_R >= SAFE_CLEAR_M:
                best = r
    return best


def main():
    runs, pat, lay = sys.argv[1:4]
    out = sys.argv[sys.argv.index("--json") + 1] if "--json" in sys.argv else None
    W, C, steps_all = [], [], []
    for d in sorted(glob.glob(os.path.join(runs, pat))):
        cj, rc = os.path.join(d, "client.json"), os.path.join(d, "run.csv")
        if not (os.path.exists(cj) and os.path.exists(rc)):
            continue
        c = json.load(open(cj))
        tag = os.path.basename(d)
        if c.get("companion_tag") != tag:
            continue
        cond = "plan" if "_plan__" in tag else "noplan"
        world = tag.split("__", 1)[1]
        layout = json.load(open(f"{lay}/ecps295_{world}.layout.json"))
        B = boxes(layout)
        lim = c["plan"]["fence_m"] - 0.4
        R = [(float(r["wall"]), float(r["north"]), float(r["east"]), float(r["alt"]), float(r["clearance"]) if r["clearance"] not in ("", "inf") else math.inf)
             for r in csv.DictReader(open(rc)) if r.get("wall")]
        T = c["ticks"]  # [wall, op, ctl, arm, fc, n, e, alt, es, ri]
        # contacts (truth clearance < 0), episodes over the whole session
        prev = False
        for w, n, e, a, cl in R:
            if cl < 0 and not prev:
                near = min(T, key=lambda t: abs(t[0] - w)) if T else None
                st = next((s["tag"] for s in c["steps"] if s["t0"] <= w <= s["t1"]), "between steps")
                C.append({"session": tag, "cond": cond, "step": st, "t": round(w - c["t0"], 1), "n": round(n, 2), "e": round(e, 2),
                          "alt": round(a, 2), "clear": round(cl, 3), "box": surf(B, n, e)[1], "op": near[1] if near else None,
                          "fc": near[4] if near else None})
            prev = cl < 0
        for s in c["steps"]:
            if s.get("expect_refuse") or not s.get("ack") or not s["ack"].get("ok"):
                continue
            pts = list(s["pts"])
            win_r = [r for r in R if s["t0"] <= r[0] <= s["t1"]]
            win_t = [t for t in T if s["t0"] <= t[0] <= s["t1"] and t[5] is not None]
            st = {"session": tag, "cond": cond, "world": world, "tag": s["tag"], "kind": s["tag"].split("_")[0], "n": len(pts), "reached": 0}
            for i, p in enumerate(pts):
                need = reach_need(B, p, lim)
                dt = min((math.hypot(r[1] - p[0], r[2] - p[1]) for r in win_r), default=math.inf)
                de = min((math.hypot(t[5] - p[0], t[6] - p[1]) for t in win_t), default=math.inf)
                ok = de <= ARRIVE_M + (need if math.isfinite(need) else 0.0)
                st["reached"] += ok
                W.append({"session": tag, "cond": cond, "world": world, "step": s["tag"], "kind": st["kind"], "i": i, "p": p,
                          "need": round(need, 2) if math.isfinite(need) else None, "r": round(math.hypot(*p), 2), "lim": lim,
                          "fence_gap": round(c["plan"]["fence_m"] - math.hypot(*p), 2),
                          "wp_clear": round(surf(B, *p)[0] - PROP_TIP_R, 2), "min_truth": round(dt, 2), "min_ekf": round(de, 2), "ok": ok})
            steps_all.append(st)
    # ---- report
    print(f"waypoints {len(W)} (steps {len(steps_all)}), contacts {len(C)}")
    by = defaultdict(list)
    for w in W:
        by[(w["cond"], w["kind"])].append(w)
    print("\nreached per condition / kind (EKF within 0.5 m, or within 0.5 m of the nearest safe point)")
    for k in sorted(by):
        g = by[k]
        print(f"  {k[0]:6s} {k[1]:7s} {sum(w['ok'] for w in g):4d}/{len(g):4d} = {100 * np.mean([w['ok'] for w in g]):5.1f}%  "
              f"miss median EKF gap {np.median([w['min_ekf'] for w in g if not w['ok']] or [0]):.2f} m")
    for cond in ("plan", "noplan"):
        g = [w for w in W if w["cond"] == cond]
        print(f"  {cond:6s} ALL     {sum(w['ok'] for w in g):4d}/{len(g):4d} = {100 * np.mean([w['ok'] for w in g]):5.1f}%")
    # pairs
    P = defaultdict(dict)
    for w in W:
        P[(w["session"].split("_")[0], w["world"], w["step"], w["i"])][w["cond"]] = w["ok"]
    pr = [(v["noplan"], v["plan"]) for v in P.values() if "plan" in v and "noplan" in v]
    if pr:
        a = sum(1 for x, y in pr if y and not x); b = sum(1 for x, y in pr if x and not y)
        from scipy.stats import binomtest
        print(f"\npaired waypoints n={len(pr)}: off {sum(x for x, _ in pr)} reached, on {sum(y for _, y in pr)} reached; "
              f"only-on {a}, only-off {b}, McNemar p={binomtest(a, a + b).pvalue if a + b else float('nan'):.3g}")
    # misses: where
    for cond in ("plan", "noplan"):
        m = [w for w in W if w["cond"] == cond and not w["ok"]]
        if not m:
            continue
        near_fence = sum(w["fence_gap"] < 0.7 for w in m)
        near_box = sum(w["wp_clear"] < 0.4 for w in m)
        print(f"\nmisses {cond}: {len(m)}; waypoint within 0.7 m of the fence {near_fence}; within 0.4 m of a box (prop tip) {near_box}; "
              f"neither {sum(w['fence_gap'] >= 0.7 and w['wp_clear'] >= 0.4 for w in m)}")
        allw = [w for w in W if w["cond"] == cond]
        for lo, hi in ((0, 0.7), (0.7, 1.2), (1.2, 9)):
            g = [w for w in allw if lo <= w["fence_gap"] < hi]
            if g:
                print(f"   fence gap {lo:.1f}-{hi:.1f} m: reached {100 * np.mean([w['ok'] for w in g]):5.1f}% of {len(g)}")
    print("\ncontacts:")
    for x in C:
        print("  ", x)
    if out:
        json.dump({"waypoints": W, "contacts": C, "steps": steps_all}, open(out, "w"))


if __name__ == "__main__":
    main()
