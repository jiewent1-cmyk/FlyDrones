#!/usr/bin/env python3
"""Merge dev_base.json into agg table: dev_base median/range per cell + per-phase verdict columns."""
import glob, json, os, re, statistics as stt

D = "/home/jacky/sim/val2"
db = {r["tag"]: r for r in json.load(open(D + "/dev_base.json"))}
rows = []
for f in sorted(glob.glob(D + "/runs/v2_*/metrics.json")):
    tag = os.path.basename(os.path.dirname(f))
    r = json.load(open(f))
    m = re.match(r"v2_(cl|a2|cn|md|ds)_([FD])_(dn15|up15|oor|dn05)_(\d+)_r(\d+)$", tag)
    if not m:
        continue
    r.update(phase=m.group(1), group=m.group(2), ftype=m.group(3), dur=m.group(4))
    b = db.get(tag)
    if b and b.get("dev_base") is not None:
        r["dev_base"] = b["dev_base"]
        r["f_start_auto"] = b["f_start_auto"]
    rows.append(r)


def med(rs, k, nd=2):
    v = [r[k] for r in rs if isinstance(r.get(k), (int, float))]
    if not v:
        return "-"
    f = f"{{:.{nd}f}}"
    return f.format(stt.median(v)) if len(v) == 1 else f.format(stt.median(v)) + " [" + f.format(min(v)) + ".." + f.format(max(v)) + "]"


groups = {}
for r in rows:
    groups.setdefault((r["phase"], r["ftype"], r["dur"], r["group"]), []).append(r)
PHN = {"cl": "climb~1m", "a2": "just-2m", "cn": "corner", "md": "midleg", "ds": "desc~1m"}
print("| phase | type | dur | grp | n | compl | dev_max m | dev_base m | f_alt0 m | td m/s | off m | outcome | clip |")
print("|---" * 13 + "|")
for g in sorted(groups, key=lambda k: (["cl", "a2", "cn", "md", "ds"].index(k[0]), k[1], int(k[2]), k[3])):
    rs = groups[g]
    ok = sum(1 for r in rs if r.get("mission_complete"))
    outs = {}
    for r in rs:
        why = []
        for t in (r.get("msgs") or []):
            if "fence breached" in t:
                why.append("fence")
            if "variance" in t:
                why.append("EKFvar")
            if "Failsafe" in t and "Cleared" not in t:
                why.append("EKFfail")
        o = "complete" if r.get("mission_complete") else "LAND:" + "+".join(dict.fromkeys(why))
        outs[o] = outs.get(o, 0) + 1
    alt0 = stt.median([r["f_alt0"] for r in rs])
    clip = ""
    if g[1] == "dn15" and alt0 < 1.55:
        clip = "yes->0(OORlow)"
    elif g[1] == "dn15":
        clip = "no"
    if g[1] == "dn05":
        clip = "no" if alt0 > 0.55 else "yes->0"
    outs_s = ";".join(f"{k}x{v}" for k, v in sorted(outs.items()))
    print(f"| {PHN[g[0]]} | {g[1]} | {g[2]} | {g[3]} | {len(rs)} | {ok}/{len(rs)} | {med(rs,'dev_max')} | {med(rs,'dev_base')} | {med(rs,'f_alt0')} | "
          f"{med(rs,'touchdown_mps')} | {med(rs,'land_offset')} | {outs_s} | {clip} |")
