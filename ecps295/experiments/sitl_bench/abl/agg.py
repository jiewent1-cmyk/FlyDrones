#!/usr/bin/env python3
"""Aggregate the ablation: runs/*/safety.json + metrics.json -> all_runs.csv, tables.md, heat_*.png"""
import csv, glob, json, os, re, statistics as st, collections
D = os.path.dirname(os.path.abspath(__file__))
rows = []
for sj in sorted(glob.glob(D + "/runs/ab_*/safety.json")):
    try:
        s = json.load(open(sj))
    except Exception:
        continue
    m = re.match(r"ab_(.+?)__(.+?)__r(\d+)$", s["tag"])
    if not m:
        continue
    arm, fault, rep = m.group(1), m.group(2), int(m.group(3))
    mission = "D3" if fault.startswith("D3") or "_D3" in s["tag"] else "SQ"
    try:
        met = json.load(open(os.path.dirname(sj) + "/metrics.json"))
    except Exception:
        met = {}
    s.update(arm=arm, fault=fault, rep=rep, disarmed=met.get("disarmed"), final_mode=met.get("final_mode"),
             flight_s=met.get("flight_s"), left_auto_at=met.get("left_auto_at"),
             fence=any("fence" in str(t).lower() and "breach" in str(t).lower() for t in (met.get("script") or {}).get("texts", [])),
             ekf_fs=any("EKF" in str(t) and ("variance" in str(t) or "failsafe" in str(t).lower()) for t in (met.get("script") or {}).get("texts", [])))
    c = s.get("cls", "?")
    # landed but motors still armed at the end of the run: its own class unless already worse
    if c in ("OK", "ABORT") and s.get("disarmed") is False and s.get("touchdown") is not None:
        c = "ARMED_GND"
    s["cls2"] = c
    rows.append(s)
keys = sorted({k for r in rows for k in r})
with open(D + "/all_runs.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(rows)
CL = ["OK", "ABORT", "ARMED_GND", "DEGRADED", "UNSAFE", "NOFLY", "?"]
cell = collections.defaultdict(list)
for r in rows:
    cell[(r["arm"], r["fault"])].append(r)
arms = sorted({a for a, _ in cell}, key=lambda a: ["FULL","NOGUARD","BARE","STOCKGATE","SURF1","FULL_NOALTF","BARE_NOALTF","G_LANDONLY","G_RATE","G_E05","G_E10","BAROPRI"].index(a) if a in ["FULL","NOGUARD","BARE","STOCKGATE","SURF1","FULL_NOALTF","BARE_NOALTF","G_LANDONLY","G_RATE","G_E05","G_E10","BAROPRI"] else 99)
faults = sorted({f for _, f in cell})
out = ["# ablation tables", "", "runs: %d" % len(rows), ""]
out += ["## per arm (all faults)", "", "| arm | n | " + " | ".join(CL) + " | max alt | min clr | max td |", "|-" * (len(CL) + 5) + "|"]
for a in arms:
    rs = [r for r in rows if r["arm"] == a]
    cnt = collections.Counter(r["cls2"] for r in rs)
    fl = [r for r in rs if r.get("alt_max") is not None]
    out.append("| %s | %d | " % (a, len(rs)) + " | ".join(str(cnt.get(c, 0)) for c in CL) + " | %.2f | %.2f | %.2f |" % (
        max(r["alt_max"] for r in fl), min(r["clr"] for r in fl), max((r["touchdown"] or 0) for r in fl)))
def cellstr(rs):
    cnt = collections.Counter(r["cls2"] for r in rs)
    n = len(rs)
    bad = "/".join("%s%d" % (k[0] if k != "ARMED_GND" else "G", cnt[k]) for k in ("UNSAFE", "DEGRADED", "ARMED_GND", "ABORT", "NOFLY") if cnt.get(k))
    return "%d %s" % (n, bad or "ok")
out += ["", "## arm x fault: n, then U=unsafe D=degraded G=armed on ground A=abort N=nofly", "", "| fault | " + " | ".join(arms) + " |", "|-" * (len(arms) + 1) + "|"]
for f in faults:
    out.append("| %s | " % f + " | ".join(cellstr(cell[(a, f)]) if (a, f) in cell else "-" for a in arms) + " |")
out += ["", "## arm x fault: true max height median / max (m)", "", "| fault | " + " | ".join(arms) + " |", "|-" * (len(arms) + 1) + "|"]
for f in faults:
    def hs(rs):
        z = [r["alt_max"] for r in rs if r.get("alt_max") is not None]
        return "%.1f/%.1f" % (st.median(z), max(z)) if z else "-"
    out.append("| %s | " % f + " | ".join(hs(cell[(a, f)]) if (a, f) in cell else "-" for a in arms) + " |")
out += ["", "## guard: trips and trip time after arm (s) median/max, blind, rtf<0.9", "", "| fault | " + " | ".join(a for a in arms if any(r.get("guard") not in (None, "off") for r in rows if r["arm"] == a)) + " |"]
garms = [a for a in arms if any(r.get("guard") not in (None, "off") for r in rows if r["arm"] == a)]
out.append("|-" * (len(garms) + 1) + "|")
for f in faults:
    def gs(rs):
        tr = [r["guard_trip_after_arm_s"] for r in rs if r.get("guard") == "tripped" and r.get("guard_trip_after_arm_s") is not None]
        s = "%d/%d" % (sum(r.get("guard") == "tripped" for r in rs), len(rs))
        if tr: s += " %.1f/%.1f" % (st.median(tr), max(tr))
        return s
    out.append("| %s | " % f + " | ".join(gs(cell[(a, f)]) if (a, f) in cell else "-" for a in garms) + " |")
slow = [r["tag"] for r in rows if (r.get("rtf_min") or 1) < 0.9]
blind = [r["tag"] for r in rows if r.get("guard_blind")]
nolink = [r["tag"] for r in rows if r.get("guard") == "no link"]
out += ["", "rtf_min < 0.9: %d  %s" % (len(slow), slow[:10]), "guard blind: %d  %s" % (len(blind), blind[:10]), "guard no link: %d %s" % (len(nolink), nolink[:10])]
open(D + "/tables.md", "w").write("\n".join(out) + "\n")
print("\n".join(out))
