"""Planner verification analysis (2026-10-22): per route step from the client log (console events, ticks) and the Gazebo
truth (run.csv, cut by the wall clock), then aggregated per condition and scenario kind, with planner-on/off pairs.

usage: analyze.py <runs dir> <tag glob, e.g. 'pv*'> [--json out.json]
"""

from __future__ import annotations

import csv
import glob
import json
import math
import os
import re
import sys
from collections import defaultdict

import numpy as np

RE_WP = re.compile(r"waypoint (\d+)(?: of (\d+))? (reached|skipped)")


def load_session(d: str):
    cj = os.path.join(d, "client.json")
    if not os.path.exists(cj):
        return None
    c = json.load(open(cj))
    rows = []
    rc = os.path.join(d, "run.csv")
    if os.path.exists(rc):
        for r in csv.DictReader(open(rc)):
            if r.get("wall"):
                rows.append((float(r["wall"]), float(r["clearance"]) if r["clearance"] not in ("", "inf") else math.inf,
                             float(r["north"]), float(r["east"]), float(r["alt"])))
    rj = os.path.join(d, "run.json")
    summ = json.load(open(rj)) if os.path.exists(rj) else {}
    return c, rows, summ.get("summary", summ)


def step_metrics(step: dict, events: list, rows: list, planned: bool) -> dict:
    t0, t1 = step["t0"], step["t1"]
    ev = [e for e in events if t0 <= e[0] <= t1 + 0.5]
    txt = [f"{e[1]}: {e[2]}" for e in ev]
    win = [r for r in rows if t0 <= r[0] <= t1]
    cl = [r[1] for r in win if math.isfinite(r[1])]
    contacts, prev = 0, False
    for r in win:
        c = r[1] < 0.0
        contacts += int(c and not prev)
        prev = c
    # planner on: the console's own "waypoint k of N ..." (the pilot's events are then about intermediate path points);
    # planner off: the pilot's waypoint layer events ("waypoint i reached / skipped")
    wp = [m for t in txt if (m := RE_WP.search(t)) and bool(m.group(2)) == planned]
    reached = sum(m.group(3) == "reached" for m in wp)
    skipped = sum(m.group(3) == "skipped" for m in wp)
    final = ""
    for t in txt:
        if t.startswith("operator: mode HOLD") or t.startswith("operator: mode PATROL"):
            final = t[len("operator: mode "):]
    n_wp = len(step["pts"]) + (len(step.get("change_pts") or []) if step.get("change_ack") else 0)
    done = bool(re.search(r"route done|goal reached|goal done", final)) and not step.get("timeout")
    return {
        "tag": step["tag"], "kind": step["tag"].split("_")[0], "n_wp": n_wp, "secs": round(t1 - t0, 1),
        "ack_ok": bool(step.get("ack") and step["ack"].get("ok")),
        "refused_ok": step.get("refused_ok"), "timeout": bool(step.get("timeout")), "final": final, "done": done,
        "reached": reached, "skipped": skipped,
        "moved": sum("clear of an obstacle" in t for t in txt), "replans": sum("new path" in t for t in txt),
        "low_cross": sum("crosses low boxes" in t for t in txt), "no_path": sum("no path yet" in t for t in txt),
        "escapes": sum(t.startswith("brain: escape") for t in txt), "fence_turns": sum("fence: turning back" in t for t in txt),
        "min_clear": round(min(cl), 3) if cl else None, "contacts": contacts, "rows": len(win),
        "estop": step.get("estop"), "change": bool(step.get("change_ack")),
    }


def main() -> None:
    runs, pat = sys.argv[1], sys.argv[2]
    out = sys.argv[sys.argv.index("--json") + 1] if "--json" in sys.argv else None
    S = []
    sess = []
    for d in sorted(glob.glob(os.path.join(runs, pat))):
        x = load_session(d)
        if x is None:
            continue
        c, rows, summ = x
        tag = os.path.basename(d)
        if c.get("companion_tag") != tag:  # 10-22: only sessions whose own companion answered (see run_session.sh)
            print(f"skip {tag}: companion tag {c.get('companion_tag')!r}")
            continue
        cond = "plan" if "_plan__" in tag else "noplan"
        world = tag.split("__", 1)[1]
        pl = (c.get("status") or {}).get("planner") or {}
        sess.append({"tag": tag, "cond": cond, "world": world, "control": c.get("control"), "error": c.get("error"),
                     "steps": len(c.get("steps", [])), "planned": len(c["plan"]["steps"]), "landed": c.get("landed"),
                     "contact_episodes": summ.get("contact_episodes"), "min_clear": summ.get("min_clearance_m"),
                     "rtf": summ.get("rtf_run"), "plans": pl.get("plans"), "plan_ms_p50": pl.get("ms_p50"),
                     "plan_ms_p99": pl.get("ms_p99"), "plan_ms_max": pl.get("ms_max")})
        for st in c.get("steps", []):
            m = step_metrics(st, c.get("events", []), rows, cond == "plan")
            m.update(cond=cond, world=world, wave=tag.split("_")[0], session=tag)
            S.append(m)
    print(f"sessions {len(sess)}: control {sum(bool(s['control']) for s in sess)}, errors {[s['tag'] for s in sess if s['error']]}, "
          f"incomplete {[s['tag'] for s in sess if s['steps'] < s['planned']]}")
    fly = [s for s in S if s["kind"] != "refuse"]
    print("\nper condition and kind (routes; waypoints reached / total; min truth clearance median / min; contacts)")
    groups = defaultdict(list)
    for s in fly:
        groups[(s["cond"], s["kind"])].append(s)
    for (cond, kind), g in sorted(groups.items()):
        mc = [s["min_clear"] for s in g if s["min_clear"] is not None]
        print(f"  {cond:6s} {kind:7s} n={len(g):3d} done {sum(s['done'] for s in g):3d}  wp {sum(s['reached'] for s in g):3d}/{sum(s['n_wp'] for s in g):3d}"
              f"  skipped {sum(s['skipped'] for s in g):3d}  timeouts {sum(s['timeout'] for s in g):2d}"
              f"  secs med {np.median([s['secs'] for s in g]):5.1f}  escapes/route {np.mean([s['escapes'] for s in g]):4.1f}"
              f"  clear {np.median(mc) if mc else float('nan'):.2f}/{min(mc) if mc else float('nan'):.2f}  contacts {sum(s['contacts'] for s in g)}"
              f"  moved {sum(s['moved'] for s in g)} replans {sum(s['replans'] for s in g)} low {sum(s['low_cross'] for s in g)}")
    for cond in ("plan", "noplan"):
        g = [s for s in fly if s["cond"] == cond]
        if g:
            mc = [s["min_clear"] for s in g if s["min_clear"] is not None]
            print(f"  {cond:6s} ALL     n={len(g):3d} done {sum(s['done'] for s in g)} ({100 * np.mean([s['done'] for s in g]):.0f}%)"
                  f"  wp {sum(s['reached'] for s in g)}/{sum(s['n_wp'] for s in g)}  skipped {sum(s['skipped'] for s in g)}"
                  f"  contacts {sum(s['contacts'] for s in g)}  min clear {min(mc) if mc else float('nan'):.2f}")
    ref = [s for s in S if s["kind"] == "refuse"]
    print(f"\nrefuse: {sum(bool(s['refused_ok']) for s in ref)}/{len(ref)} refused as expected")
    ops = [s for s in S if s["kind"] == "ops"]
    if ops:
        print("ops:", [(s["world"], s["tag"], s["done"], s["contacts"], (s["estop"] or {}).get("ok")) for s in ops])
    # pairs: the same world, plan file and step tag, planner on vs off
    P = {(s["wave"], s["world"], s["tag"], s["cond"]): s for s in fly}
    pairs = [(P[(w, wo, t, "plan")], P[(w, wo, t, "noplan")]) for (w, wo, t, c) in P if c == "plan" and (w, wo, t, "noplan") in P]
    if pairs:
        from scipy.stats import wilcoxon

        print(f"\npaired routes (planner on vs off): n={len(pairs)}")
        for k, lab in (("done", "completed"), ("reached", "waypoints reached"), ("skipped", "waypoints skipped"),
                       ("escapes", "brain escapes"), ("secs", "time s"), ("min_clear", "min clearance m"), ("contacts", "contacts")):
            x = np.array([float(b[k] or 0) for a, b in pairs]); y = np.array([float(a[k] or 0) for a, b in pairs])
            p = wilcoxon(y - x).pvalue if np.any(y != x) else float("nan")
            print(f"  {lab:18s} off {x.mean():6.2f} (sum {x.sum():5.0f})  on {y.mean():6.2f} (sum {y.sum():5.0f})  p={p:.3g}")
    pl = [s for s in sess if s["cond"] == "plan" and s["plan_ms_p50"] is not None]
    if pl:
        print(f"\nplanner: plans {sum(s['plans'] or 0 for s in pl)}, ms p50 median {np.median([s['plan_ms_p50'] for s in pl]):.1f}, "
              f"p99 max {max(s['plan_ms_p99'] for s in pl):.1f}, max {max(s['plan_ms_max'] for s in pl):.1f}")
    print(f"RTF median {np.median([s['rtf'] for s in sess if s['rtf']]) if any(s['rtf'] for s in sess) else float('nan'):.3f}")
    if out:
        json.dump({"sessions": sess, "steps": S}, open(out, "w"))


if __name__ == "__main__":
    main()
