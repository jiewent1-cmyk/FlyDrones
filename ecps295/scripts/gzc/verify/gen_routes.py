"""Route scenarios for the console planner verification (2026-10-22), generated from a world's layout.

usage: gen_routes.py <world.layout.json> <seed> > plan.json

Frame: telemetry / console frame (x = north, y = east), north = ENU y, east = ENU x (world_layout.room_from_layout).
Every waypoint is inside the console's go-to range (fence - 0.4); the fence per world = min(2.7, floor/2 - 0.45).
Scenario kinds (tag prefix):
  behind   two points on either side of a box: the straight leg crosses it
  on       a waypoint on a box's centre (the planner must move it clear)
  adj      a waypoint 0.12 m from the face of the tallest boxes (the 10-21 failure: turning back and forth)
  gap      through the gap between two boxes that is narrower than 2 x clearance (must go around)
  tour     6 waypoints whose straight legs cross boxes as often as possible
  open     2 waypoints with clear straight legs (control)
  refuse   a waypoint beyond the go-to range (the console must refuse it, nothing flies)
"""

from __future__ import annotations

import json
import math
import random
import sys


def boxes_ne(layout: dict):
    out = []
    for b in layout["boxes"]:
        if b["name"].startswith("net"):
            continue
        (e, n, u), (se, sn, su) = b["center_enu"], b["size_enu"]
        out.append({"name": b["name"], "n": n, "e": e, "hn": sn / 2, "he": se / 2, "top": u + su / 2})
    return out


def surf(b, p) -> float:
    """Distance from p (n, e) to the box footprint (0 inside)."""
    return math.hypot(max(b["n"] - b["hn"] - p[0], 0, p[0] - b["n"] - b["hn"]), max(b["e"] - b["he"] - p[1], 0, p[1] - b["e"] - b["he"]))


def seg_hits(B, a, c, margin=0.0) -> int:
    n = 40
    hit = set()
    for i in range(n + 1):
        p = (a[0] + (c[0] - a[0]) * i / n, a[1] + (c[1] - a[1]) * i / n)
        for k, b in enumerate(B):
            if surf(b, p) <= margin:
                hit.add(k)
    return len(hit)


def main() -> None:
    layout = json.load(open(sys.argv[1]))
    rng = random.Random(int(sys.argv[2]) if len(sys.argv) > 2 else 0)
    B = boxes_ne(layout)
    fence = round(min(2.7, layout.get("floor_m", 6.5) / 2 - 0.45), 2)
    lim = fence - 0.4 - 0.05
    inside = lambda p: math.hypot(*p) <= lim  # noqa: E731
    free = lambda p, m=0.0: all(surf(b, p) > m for b in B)  # noqa: E731
    r2 = lambda p: [round(p[0], 2), round(p[1], 2)]  # noqa: E731
    steps = []

    # behind: the box nearest to mid range, crossing from the near side to the far side
    for b in sorted(B, key=lambda b: abs(math.hypot(b["n"], b["e"]) - 1.3))[:2]:
        d = math.hypot(b["n"], b["e"]) or 1.0
        u = (b["n"] / d, b["e"] / d)
        half = math.hypot(b["hn"], b["he"])
        for k in (0.35, 0.25, 0.15):
            A = (b["n"] - u[0] * (half + 0.55), b["e"] - u[1] * (half + 0.55))
            P = (b["n"] + u[0] * (half + k), b["e"] + u[1] * (half + k))
            if inside(A) and inside(P) and free(A, 0.1) and free(P, 0.05):
                steps.append({"op": "route", "tag": f"behind_{b['name']}", "pts": [r2(A), r2(P)]})
                break
    # on: the centres of two boxes in range
    for b in [b for b in B if inside((b["n"], b["e"]))][:2]:
        steps.append({"op": "route", "tag": f"on_{b['name']}", "pts": [r2((b["n"], b["e"]))]})
    # adj: 0.12 m from the face of the two tallest boxes, on the side toward the origin
    for b in sorted(B, key=lambda b: -b["top"])[:2]:
        d = math.hypot(b["n"], b["e"]) or 1.0
        u = (-b["n"] / d, -b["e"] / d)
        if abs(u[0]) > abs(u[1]):
            P = (b["n"] + math.copysign(b["hn"] + 0.12, u[0]), b["e"])
        else:
            P = (b["n"], b["e"] + math.copysign(b["he"] + 0.12, u[1]))
        if inside(P):
            steps.append({"op": "route", "tag": f"adj_{b['name']}", "pts": [r2(P)]})
    # gap: two boxes 0.2-0.9 m apart; cross the gap at right angles
    best = None
    for i in range(len(B)):
        for j in range(i + 1, len(B)):
            a, c = B[i], B[j]
            g = max(abs(a["n"] - c["n"]) - a["hn"] - c["hn"], abs(a["e"] - c["e"]) - a["he"] - c["he"])
            m = ((a["n"] + c["n"]) / 2, (a["e"] + c["e"]) / 2)
            if 0.2 <= g <= 0.9 and inside(m) and (best is None or g < best[0]):
                best = (g, a, c, m)
    if best:
        g, a, c, m = best
        dn, de = c["n"] - a["n"], c["e"] - a["e"]
        L = math.hypot(dn, de)
        v = (-de / L, dn / L)  # across the gap
        P1, P2 = (m[0] - v[0] * 0.8, m[1] - v[1] * 0.8), (m[0] + v[0] * 0.8, m[1] + v[1] * 0.8)
        if inside(P1) and inside(P2) and free(P1, 0.1) and free(P2, 0.1):
            steps.append({"op": "route", "tag": f"gap_{a['name']}_{c['name']}_{g:.2f}", "pts": [r2(P1), r2(P2)]})
    # tour: 6 points, greedy for legs that cross boxes
    pts, cur = [], (0.0, 0.0)
    for _ in range(6):
        cand = []
        for _ in range(300):
            r, th = lim * math.sqrt(rng.random()), rng.random() * 2 * math.pi
            p = (r * math.cos(th), r * math.sin(th))
            if free(p, 0.05) and math.hypot(p[0] - cur[0], p[1] - cur[1]) > 0.8:
                cand.append((seg_hits(B, cur, p), rng.random(), p))
        if not cand:
            break
        cand.sort(reverse=True)
        pts.append(cand[0][2])
        cur = cand[0][2]
    if len(pts) >= 3:
        steps.append({"op": "route", "tag": "tour6", "pts": [r2(p) for p in pts]})
    # open: two points whose legs stay 0.6 m clear
    for _ in range(500):
        p1 = (rng.uniform(-lim, lim), rng.uniform(-lim, lim))
        p2 = (rng.uniform(-lim, lim), rng.uniform(-lim, lim))
        if (inside(p1) and inside(p2) and free(p1, 0.6) and free(p2, 0.6) and math.hypot(p1[0] - p2[0], p1[1] - p2[1]) > 1.0
                and seg_hits(B, (0, 0), p1, 0.6) == 0 and seg_hits(B, p1, p2, 0.6) == 0):
            steps.append({"op": "route", "tag": "open", "pts": [r2(p1), r2(p2)]})
            break
    # refuse: beyond the go-to range
    steps.append({"op": "route", "tag": "refuse", "pts": [[0.0, 0.5], [lim + 0.5, 0.0]], "expect_refuse": True})
    json.dump({"world": layout.get("world"), "fence_m": fence, "steps": steps}, sys.stdout, indent=1)


if __name__ == "__main__":
    main()
