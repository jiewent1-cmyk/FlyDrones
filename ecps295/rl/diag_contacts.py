"""Contact taxonomy (route review 2026-10-13): at every contact onset, what the drone was doing and where the obstacle was.

Same jobs as rl.hw_arms (lowbox_s0-49 x brain seeds {k, k+100, k+200}, cage20 x 32), episodes re-run with keep_rows and
the true heading logged, so every record is bit-identical to the hw_arms / shield_screen one.
  phase       mean command over the 0.5 s before contact: back (forward < -0.05), turn (|yaw| > 0.3), forward, hover
  rel_deg     bearing of the contacted obstacle's closest point relative to the nose (+ = right); the camera sees +-60 deg
  v_fwd/v_lat true body-frame velocity over the last 0.2 s; v_app = component toward the obstacle
  seen_2s     the obstacle was inside the camera's +-60 deg and < 2 m at some tick in the 2 s before contact
  esc_dt      time since the last escape onset (and its trigger)
python -m rl.diag_contacts --conds v4 s3_rng --procs 8 --out rl/out/diag_contacts.jsonl
python -m rl.diag_contacts --report --out rl/out/diag_contacts.jsonl
"""
from __future__ import annotations

import argparse
import json
import math
from collections import Counter
from multiprocessing import get_context
from pathlib import Path

import numpy as np

from rl import hw_arms

E = Path(__file__).resolve().parent.parent
CAM_HALF = 60.0
_LOG: list[float] = []


def _patch() -> None:
    from rl import episode as ep
    from rl.twin import TwinDrone

    if getattr(TwinDrone, "_diag", False):
        return
    step = TwinDrone.step

    def logged(self, dt):
        step(self, dt)
        _LOG.append(self.yaw)

    TwinDrone.step, TwinDrone._diag = logged, True
    run = ep.run_episode
    ep.run_episode = lambda *a, **k: run(*a, **{**k, "keep_rows": True})


def _wrap(a: float) -> float:
    return math.degrees(math.atan2(math.sin(a), math.cos(a)))


def _run(job: dict) -> dict:
    _patch()
    from rl.batch import _job, world_path
    from rl.episode import DT, SUBSTEPS
    from rl.twin import World

    _LOG.clear()
    r = _job(job)
    if "error" in r:
        return r
    rows = r.pop("rows")
    n_t = len(rows)
    yaw = _LOG[-n_t * SUBSTEPS :][SUBSTEPS - 1 :: SUBSTEPS]
    world = World.from_layout(world_path(job["world"]))
    cl = [x[3] for x in rows]
    esc = [(e[0], e[3]) for e in r.get("escape_at", [])]
    ev = []
    for k in range(1, n_t):
        if not cl[k - 1] >= 0 > cl[k]:
            continue
        n, e, a = rows[k][:3]
        cand = [o for o in world.obstacles if o.hi[2] >= a - 0.03]
        o = min(cand, key=lambda o: math.hypot(max(o.lo[0] - n, 0, n - o.hi[0]), max(o.lo[1] - e, 0, e - o.hi[1])))

        def rel_dist(j):
            pn, pe = min(max(rows[j][0], o.lo[0]), o.hi[0]), min(max(rows[j][1], o.lo[1]), o.hi[1])
            return _wrap(math.atan2(pe - rows[j][1], pn - rows[j][0]) - yaw[j]), math.hypot(pn - rows[j][0], pe - rows[j][1])

        rel, _ = rel_dist(k)
        j0 = max(0, k - 4)
        vn, ve = (rows[k][0] - rows[j0][0]) / ((k - j0) * DT), (rows[k][1] - rows[j0][1]) / ((k - j0) * DT)
        h = yaw[k]
        v_fwd, v_lat = vn * math.cos(h) + ve * math.sin(h), -vn * math.sin(h) + ve * math.cos(h)
        b = math.radians(rel) + h
        v_app = vn * math.cos(b) + ve * math.sin(b)
        w = rows[max(0, k - 10) : k]
        fwd, yw = float(np.mean([x[4] for x in w])), float(np.mean([x[5] for x in w]))
        phase = "back" if fwd < -0.05 else "turn" if abs(yw) > 0.3 else "forward" if fwd > 0.05 else "hover"
        seen = {}
        for s in (2, 5):
            vis = [j for j in range(max(0, k - int(s / DT)), k) if (lambda q: abs(q[0]) < CAM_HALF and q[1] < 2.0)(rel_dist(j))]
            seen[s] = vis
        first = seen[5][0] if seen[5] else None
        last_esc = [x for x in esc if x[0] <= k * DT + 1e-6]
        ev.append({"t": round(k * DT, 2), "obst": o.name, "obst_h": round(o.hi[2], 2), "net": bool(getattr(o, "net", False)),
                   "rel_deg": round(rel, 1), "phase": phase, "cmd_fwd": round(fwd, 3), "cmd_yaw": round(yw, 3),
                   "v_fwd": round(v_fwd, 3), "v_lat": round(v_lat, 3), "v_app": round(v_app, 3),
                   "seen_2s": bool(seen[2]), "seen_5s": bool(seen[5]),
                   "seen_dist": round(rel_dist(first)[1], 2) if first is not None else None,
                   "unseen_s": round((k - seen[5][-1]) * DT, 2) if seen[5] else None,
                   "esc_dt": round(k * DT - last_esc[-1][0], 2) if last_esc else None, "esc_trig": last_esc[-1][1] if last_esc else None})
    keep = ("tag", "set", "key", "seed", "world", "contact_episodes", "near_miss_episodes", "coverage", "path_m", "min_clearance_m",
            "escapes", "wall_s")
    return {**{k: r[k] for k in keep if k in r}, "events": ev}


def _conds(names: list[str]) -> dict:
    from rl import shield_screen

    out = {}
    for c in names:
        if c in shield_screen.CONDS:
            out[c] = shield_screen.CONDS[c]
        elif c == "v4":
            out[c] = (hw_arms.V4, "intact", None)
        else:
            out[c] = (*hw_arms.exported(c), None)
    return out


def jobs(conds: dict, sets=("lowbox", "cage20")) -> list[dict]:
    from rl.twin import flow_dr

    J = []
    for c, (v, arm, ov) in conds.items():
        base = {"tag": c, "variant": v, "arm": arm, **({"overrides": ov} if ov else {})}
        if "lowbox" in sets:
            for k in range(50):
                for b in (0, 100, 200):
                    J.append({**base, "set": "lowbox", "key": f"{k}_{b}", "seed": k + b, "seconds": 120.0, "world": f"lowbox_s{k}",
                              "dyn_over": flow_dr(k + b)})
        if "cage20" in sets:
            for s in range(32):
                J.append({**base, "set": "cage20", "key": str(s), "seed": s, "seconds": 120.0, "world": "cage20", "dyn_over": flow_dr(1000 + s)})
    return J


def report(path: str) -> None:
    R = [json.loads(l) for l in open(path)]
    for st in ("lowbox", "cage20"):
        for tag in sorted({r["tag"] for r in R if r["set"] == st}, key=lambda t: (t != "v4", t)):
            X = [r for r in R if r["set"] == st and r["tag"] == tag and "error" not in r]
            ev = [e for r in X for e in r["events"]]
            if not X:
                continue
            n = max(1, len(ev))
            pct = lambda c: " ".join(f"{k} {100 * v / n:.0f}%" for k, v in c.most_common())  # noqa: E731
            side = Counter("front" if abs(e["rel_deg"]) < CAM_HALF else "side" if abs(e["rel_deg"]) < 120 else "rear" for e in ev)
            kind = Counter("net" if e["net"] else "stack" if e["obst_h"] >= 1.0 else "low" for e in ev)
            print(f"===== {st} {tag}: {len(X)} runs, {sum(r['contact_episodes'] > 0 for r in X)} with contact, {len(ev)} contact onsets")
            if not ev:
                continue
            print(f"  phase    {pct(Counter(e['phase'] for e in ev))}")
            print(f"  where    {pct(side)}   | obstacle {pct(kind)}")
            print(f"  seen in camera: 2 s before {100 * np.mean([e['seen_2s'] for e in ev]):.0f}%, 5 s before {100 * np.mean([e['seen_5s'] for e in ev]):.0f}%"
                  f"; first seen at median {np.median([e['seen_dist'] for e in ev if e['seen_dist'] is not None] or [np.nan]):.2f} m;"
                  f" out of view median {np.median([e['unseen_s'] for e in ev if e['unseen_s'] is not None] or [np.nan]):.2f} s before contact")
            print(f"  true motion at contact: approach {np.median([e['v_app'] for e in ev]):.2f} m/s, body fwd {np.median([e['v_fwd'] for e in ev]):+.2f}"
                  f" lat |{np.median([abs(e['v_lat']) for e in ev]):.2f}|; lateral-dominant {100 * np.mean([abs(e['v_lat']) > abs(e['v_fwd']) for e in ev]):.0f}%")
            ed = [e["esc_dt"] for e in ev if e["esc_dt"] is not None]
            print(f"  escape: within 1 s {100 * np.mean([d <= 1.0 for d in ed]) * len(ed) / n:.0f}%, within 3 s {100 * np.mean([d <= 3.0 for d in ed]) * len(ed) / n:.0f}%;"
                  f" triggers {pct(Counter(e['esc_trig'] for e in ev if e['esc_dt'] is not None and e['esc_dt'] <= 3.0))}")
            for ph in ("forward", "back", "turn", "hover"):
                P = [e for e in ev if e["phase"] == ph]
                if P:
                    s2 = Counter("front" if abs(e["rel_deg"]) < CAM_HALF else "side" if abs(e["rel_deg"]) < 120 else "rear" for e in P)
                    print(f"    {ph:8s} n={len(P):3d} where {dict(s2)} seen_2s {100 * np.mean([e['seen_2s'] for e in P]):.0f}%"
                          f" approach {np.median([e['v_app'] for e in P]):.2f} m/s lat-dominant {100 * np.mean([abs(e['v_lat']) > abs(e['v_fwd']) for e in P]):.0f}%")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--conds", nargs="+", default=["v4"])
    ap.add_argument("--sets", nargs="+", default=["lowbox", "cage20"])
    ap.add_argument("--procs", type=int, default=8)
    ap.add_argument("--out", default=str(E / "rl" / "out" / "diag_contacts.jsonl"))
    ap.add_argument("--report", action="store_true")
    a = ap.parse_args()
    if a.report:
        report(a.out); return
    J = jobs(_conds(a.conds), a.sets)
    if Path(a.out).exists():
        done = {(r["tag"], r["set"], r["key"]) for r in map(json.loads, open(a.out)) if "error" not in r}
        J = [j for j in J if (j["tag"], j["set"], j["key"]) not in done]
    print(len(J), "jobs", flush=True)
    with get_context("fork").Pool(a.procs) as pool, open(a.out, "a") as f:
        for r in pool.imap_unordered(_run, J):
            f.write(json.dumps(r) + "\n"); f.flush()
    report(a.out)


if __name__ == "__main__":
    main()
