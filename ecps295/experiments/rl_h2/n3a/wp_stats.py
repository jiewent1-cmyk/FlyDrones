"""Waypoint batch (argv[1], default wp1) vs patrol baseline g7_sc2_tf10 (same worlds, same brain seeds). Run on ROG."""
import json, glob, os, sys, statistics as st
R = os.path.expanduser("~/sim/runs")
def load(pfx):
    out = {}
    for p in glob.glob(f"{R}/{pfx}__*/run.json"):
        try: out[p.split("/")[-2].split("__", 1)[1]] = json.load(open(p))
        except Exception: pass
    return out
PFX = sys.argv[1] if len(sys.argv) > 1 else "wp1"
wp, base = load(PFX), load("g7_sc2_tf10")
print(f"{PFX} runs: {len(wp)}")
for grp in ("lowbox", "cage20", "flat"):
    W = {k: v for k, v in wp.items() if k.startswith(grp)}
    if not W: continue
    tot = sum(v["wp_total"] for v in W.values()); rea = sum(v["wp_reached"] for v in W.values())
    blk = sum(v["wp_blocked"] for v in W.values()); skp = sum(v["wp_skipped"] for v in W.values())
    skp_free = sum(1 for v in W.values() for w in v["waypoints"] if w["result"] == "skipped" and not w["blocked"])
    errs = [w["true_err_at_m"] for v in W.values() for w in v["waypoints"] if w["result"] == "reached"]
    full = sum(1 for v in W.values() if v["wp_reached"] == v["wp_total"])
    done = [v["wp_done_t"] for v in W.values() if v["wp_done_t"] is not None]
    print(f"\n== {grp}: {len(W)} runs; points reached {rea}/{tot} ({rea/tot:.0%}), skipped {skp} (on free points {skp_free}), "
          f"blocked by a tall box {blk}; all 5 reached in {full}/{len(W)} runs")
    if errs: print(f"   truth error at the declared arrival: median {st.median(errs):.2f} m, max {max(errs):.2f} m")
    if done: print(f"   mission time: median {st.median(done):.0f} s, max {max(done):.0f} s; not finished {len(W)-len(done)}")
    c = sum(v.get("contact_episodes") or 0 for v in W.values()); n = sum(v.get("near_miss_episodes") or 0 for v in W.values())
    path = sum(v.get("path_m") or 0 for v in W.values())
    cl = [v["min_clearance_m"] for v in W.values() if v.get("min_clearance_m") is not None]
    fl = sum(bool(v.get("fc_land")) for v in W.values())
    line = f"   waypoints: contacts {c} in {sum((v.get('contact_episodes') or 0) > 0 for v in W.values())} runs, near {n}, path {path:.0f} m"
    if cl: line += f", min clearance median {st.median(cl):.2f} / min {min(cl):.2f} m"
    print(line + f", fc LAND {fl}")
    B = {k: base[k] for k in W if k in base}
    if B:
        c = sum(v.get("contact_episodes") or 0 for v in B.values()); n = sum(v.get("near_miss_episodes") or 0 for v in B.values())
        cl = [v["min_clearance_m"] for v in B.values()]
        print(f"   patrol (same {len(B)} worlds/seeds): contacts {c}, near {n}, path {sum(v['path_m'] for v in B.values()):.0f} m, "
              f"min clearance median {st.median(cl):.2f} / min {min(cl):.2f} m, fc LAND {sum(bool(v.get('fc_land')) for v in B.values())}")
bad = [(k, w["point"], w["true_min_dist_m"]) for k, v in sorted(wp.items()) for w in v["waypoints"] if w["result"] != "reached"]
print("\nnot reached:", bad)
