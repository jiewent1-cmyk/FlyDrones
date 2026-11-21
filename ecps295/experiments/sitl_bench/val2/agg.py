#!/usr/bin/env python3
"""Aggregate v2_* metrics.json -> results.csv + markdown tables (phase x type x dur x group).
dev_up = f_w15_max - f_alt0, dev_dn = f_alt0 - f_w15_min (15 s window after fault start; if the run
landed inside the window, dev_dn is dominated by the protective landing)."""
import csv, glob, json, os, re, statistics as stt, sys

D = "/home/jacky/sim/val2"
rows = []
for f in sorted(glob.glob(D + "/runs/*/metrics.json")):
    tag = os.path.basename(os.path.dirname(f))
    if not tag.startswith("v2_"):
        continue
    r = json.load(open(f))
    r["tag"] = tag
    m = re.match(r"v2_(cl|a2|cn|md|ds)_([FD])_(dn15|up15|oor|dn05|none)_(\d+)_r(\d+)$", tag)
    if m:
        r["phase"], r["group"], r["ftype"], r["dur"], r["rep"] = m.groups()
    else:
        r["phase"], r["group"], r["ftype"], r["dur"] = "tl", "-", "none", "-"
    r["yaw"] = re.search(r"yaw=(\S+)", open(os.path.join(os.path.dirname(f), "cmd.txt")).read()).group(1)
    if "f_alt0" in r:
        r["dev_up"] = round(r["f_w15_max"] - r["f_alt0"], 3)
        r["dev_dn"] = round(r["f_alt0"] - r["f_w15_min"], 3)
        r["dev_max"] = round(max(r["dev_up"], r["dev_dn"]), 3)
    why = []
    for t in (r.get("msgs") or []):
        if "fence breached" in t:
            why.append("fence")
        if "variance" in t:
            why.append("EKF variance")
        if re.search("ailsafe", t):
            why.append("failsafe:" + t.strip()[:40])
    r["why"] = ";".join(dict.fromkeys(why))
    if r.get("mission_complete"):
        r["outcome"] = "complete"
    else:
        r["outcome"] = f"{r.get('final_mode')}:{r['why'] or 'no-complete'}"
    rows.append(r)
keys = ["tag", "phase", "group", "ftype", "dur", "yaw"] + sorted({k for r in rows for k in r} - {"tag", "phase", "group", "ftype", "dur", "yaw", "script", "msgs", "texts"}) + ["msgs", "texts"] if False else None
keys = ["tag", "phase", "group", "ftype", "dur", "rep", "yaw"] + sorted(
    {k for r in rows for k in r} - {"tag", "phase", "group", "ftype", "dur", "rep", "yaw", "script", "msgs"})
with open(D + "/results.csv", "w", newline="") as fh:
    w = csv.DictWriter(fh, keys, extrasaction="ignore")
    w.writeheader()
    for r in rows:
        r2 = dict(r)
        r2["msgs"] = " / ".join(r.get("msgs") or [])
        r2["script"] = json.dumps(r.get("script"))[:2000] if isinstance(r.get("script"), dict) else ""
        w.writerow(r2)

def med(rs, k):
    v = [r[k] for r in rs if isinstance(r.get(k), (int, float)) and not isinstance(r.get(k), bool)]
    if not v:
        return "-"
    return f"{stt.median(v):.2f}" if len(v) == 1 else f"{stt.median(v):.2f} [{min(v):.2f}..{max(v):.2f}]"

pat = sys.argv[1] if len(sys.argv) > 1 else ""
groups = {}
for r in rows:
    if r["phase"] == "tl":
        continue
    groups.setdefault((r["phase"], r["ftype"], r["dur"], r["group"]), []).append(r)
PHN = {"cl": "climb~1m", "a2": "just-2m", "cn": "corner", "md": "midleg", "ds": "desc~1m"}
print("| phase | type | dur | grp | n | compl | dev_max m | dev_up m | dev_dn m | f_alt0 m | td m/s | off m | outcome |")
print("|---" * 13 + "|")
for g in sorted(groups, key=lambda k: (["cl", "a2", "cn", "md", "ds"].index(k[0]), k[1], int(k[2]), k[3])):
    rs = groups[g]
    if pat and not pat in "|".join(g):
        continue
    ok = sum(1 for r in rs if r.get("mission_complete"))
    outs = {}
    for r in rs:
        outs[r["outcome"]] = outs.get(r["outcome"], 0) + 1
    print(f"| {PHN[g[0]]} | {g[1]} | {g[2]} | {g[3]} | {len(rs)} | {ok}/{len(rs)} | {med(rs,'dev_max')} | {med(rs,'dev_up')} | {med(rs,'dev_dn')} | "
          f"{med(rs,'f_alt0')} | {med(rs,'touchdown_mps')} | {med(rs,'land_offset')} | {outs} |")
