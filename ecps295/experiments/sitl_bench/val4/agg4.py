#!/usr/bin/env python3
"""V4 aggregate: rows = 17 threshold groups, cols = faults a..f (complete / dev_base median [min..max] / outcome)
+ harsh-disturbance HOV (hov_alt_rms, hov_hgt_est_rms, EKF variance count). Also writes results.csv (one row per run)."""
import csv, glob, json, os, re, statistics as stt

D = "/home/jacky/sim/val4"
FLN = {"a": "climb dn15 1s", "b": "climb dn05 3s", "c": "2m dn15 3s", "d": "2m up15 3s",
       "e": "corner dn15 3s", "f": "desc up15 1s"}
FLKEY = {"a": 0, "c": 1, "b": 2, "d": 3, "e": 4, "f": 5}
GATES = [200, 300, 400, 500]
NSES = [0.1, 0.2, 0.3, 0.5]
GORDER = [f"h{g}n{round(n * 10):02d}" for g in GATES for n in NSES] + ["orig"]

db = {r["tag"]: r for r in json.load(open(D + "/dev_base4.json"))}
runs, byg = {}, {}
csvrows = []
for f in sorted(glob.glob(D + "/runs/v4*/metrics.json")):
    tag = os.path.basename(os.path.dirname(f))
    r = json.load(open(f))
    m = re.match(r"v4h?_(\w+)_(\w+)_r(\d)$", tag)
    if not m:
        continue
    g, key, rep = m.group(1), m.group(2), m.group(3)
    if "f_t0" in r:
        b = db.get(tag)
        if b and b.get("dev_base") is not None:
            r["dev_base"] = b["dev_base"]
    runs[tag] = r
    if g not in GORDER:
        continue
    if tag.startswith("v4h_"):
        byg.setdefault((g, "hov"), []).append(r)
    else:
        byg.setdefault((g, key), []).append(r)
    d = r.get("dev_base")
    csvrows.append({"tag": tag, "group": g, "fault": key, "rep": rep,
                    "complete": r.get("mission_complete"), "dev_base": d,
                    "alt_max": r.get("alt_max"), "f_true_max": r.get("f_true_max"),
                    "f_alt0": r.get("f_alt0"), "touchdown_mps": r.get("touchdown_mps"),
                    "land_offset": r.get("land_offset"),
                    "hov_alt_rms": r.get("hov_alt_rms"), "hov_hgt_est_rms": r.get("hov_hgt_est_rms")})


def med(rs, k, nd=2):
    v = [r[k] for r in rs if isinstance(r.get(k), (int, float))]
    if not v:
        return "-"
    f = f"{{:.{nd}f}}"
    return f.format(stt.median(v)) if len(v) < 3 else f"{f.format(stt.median(v))} [{f.format(min(v))}..{f.format(max(v))}]"


def outcome(rs):
    outs = {}
    for r in rs:
        why = []
        for t in (r.get("msgs") or []):
            if "fence breached" in t:
                why.append("fence")
            if "variance" in t.lower():
                why.append("EKFvar")
            if "Failsafe" in t and "Cleared" not in t:
                why.append("EKFfail")
        o = "OK" if r.get("mission_complete") else "LAND:" + "+".join(dict.fromkeys(why))
        outs[o] = outs.get(o, 0) + 1
    return ";".join(f"{k}x{v}" for k, v in sorted(outs.items()))


print("| group (GATE/NSE/ST) | " + " | ".join(f"{k}<br>{FLN[k]}" for k in ["a", "c", "b", "d", "e", "f"]) + " | HOV alt RMS m | HOV est RMS m | EKFvar |")
print("|---" * 10 + "|")
summary = {}
for g in GORDER:
    label = {"h200n01": "200/0.1/0", "h200n02": "200/0.2/0", "h200n03": "200/0.3/0", "h200n05": "200/0.5/0",
             "h300n01": "300/0.1/0", "h300n02": "300/0.2/0", "h300n03": "300/0.3/0", "h300n05": "300/0.5/0",
             "h400n01": "400/0.1/0", "h400n02": "400/0.2/0", "h400n03": "400/0.3/0", "h400n05": "400/0.5/0",
             "h500n01": "500/0.1/0", "h500n02": "500/0.2/0", "h500n03": "500/0.3/0", "h500n05": "500/0.5/0",
             "orig": "500/0.5/1"}.get(g, g)
    cells, allpass = [], True
    for k in ["a", "c", "b", "d", "e", "f"]:
        rs = byg.get((g, k), [])
        ok = sum(1 for r in rs if r.get("mission_complete"))
        if len(rs) < 3 or ok < 3:
            allpass = False
        if len(rs) == 3 and ok == 3:
            dbv = [r["dev_base"] for r in rs if isinstance(r.get("dev_base"), (int, float))]
            if len(dbv) == 3 and stt.median(dbv) >= 0.5:
                allpass = False
            cells.append(f"{ok}/3, {med(rs, 'dev_base')} m")
        else:
            cells.append(f"{ok}/{len(rs)}")
        if not rs:
            allpass = None
    hov = byg.get((g, "hov"), [])
    nvar = sum(1 for r in hov if any("variance" in (t or "").lower() for t in (r.get("msgs") or [])))
    cells += [med(hov, "hov_alt_rms", 3), med(hov, "hov_hgt_est_rms", 3), f"{nvar}/{len(hov)}"]
    summary[g] = (allpass, cells)
    print(f"| {label} ({g}) | " + " | ".join(cells) + " |")
print()
print("ALL-PASS groups (6/6 faults complete x3 with dev_base median < 0.5 m):",
      [g for g in GORDER if summary[g][0]] or "none")

with open(D + "/results.csv", "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=list(csvrows[0].keys()))
    w.writeheader()
    w.writerows(csvrows)
print("results.csv:", len(csvrows), "rows")
