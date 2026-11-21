#!/usr/bin/env python3
"""V6 aggregate: results.csv (one row per run) + two markdown tables.
Table 1: normal flight, 8 configs x {D3, SQ, HOV}.
Table 2: anomalies, 8 configs x {a, c, g3, g5, g10, gN, h}."""
import csv
import glob
import json
import os
import re
import statistics as stt

D = "/home/jacky/sim/val6"
GROUPS = ["f01", "f10", "f20", "f30", "o01", "o10", "o20", "o30"]
GLABEL = {"f01": "FINAL/0.01", "f10": "FINAL/0.10", "f20": "FINAL/0.20", "f30": "FINAL/0.30",
          "o01": "orig/0.01", "o10": "orig/0.10", "o20": "orig/0.20", "o30": "orig/0.30"}
CELLS = ["a", "c", "g3", "g5", "g10", "gN", "h"]
CLABEL = {"a": "爬升dn15 1s", "c": "2m dn15 3s", "g3": "2m塌0.1m 3s", "g5": "2m塌0.1m 5s",
          "g10": "2m塌0.1m 10s", "gN": "2m塌0.1m 不恢复", "h": "0.5m塌0.1m 不恢复"}

db = {}
if os.path.exists(D + "/dev_base6.json"):
    db = {r["tag"]: r for r in json.load(open(D + "/dev_base6.json"))}

rows = []
byg = {}
for f in sorted(glob.glob(D + "/runs/v6*/metrics.json")):
    tag = os.path.basename(os.path.dirname(f))
    try:
        r = json.load(open(f))
    except Exception:
        continue
    m = re.match(r"v6_(d3|hov|sq)_(?:(\w+))_r(\d)$", tag)
    if m:
        grp, cell, rep = m.group(2), m.group(1), m.group(3)
    elif re.match(r"v6_sq_r\d+$", tag):
        grp, cell, rep = "f01", "sq", re.search(r"r(\d)$", tag).group(1)
    else:
        m = re.match(r"v6_(\w+?)_(a|c|g3|g5|g10|gN|h)_r(\d)$", tag)
        if not m:
            continue
        grp, cell, rep = m.group(1), m.group(2), m.group(3)
    if grp in GROUPS:
        key = (grp, cell)
    elif tag.startswith("v6_sq_r"):
        grp, key = "f01", ("f01", "sq")
    else:
        continue
    if "dev_base" in db.get(tag, {}):
        r["dev_base"] = db[tag]["dev_base"]
    byg.setdefault(key, []).append((tag, r))
    outcome = []
    for t in (r.get("msgs") or []):
        if "fence breached" in t:
            outcome.append("fence")
        if "variance" in t.lower():
            outcome.append("EKFvar")
        if "Failsafe" in t and "Cleared" not in t:
            outcome.append("EKFfail")
    if not r.get("mission_complete"):
        oc = "未完成:" + ("+".join(dict.fromkeys(outcome)) if outcome else ("未解锁" if r.get("auto_armed") is False else "其他"))
    else:
        oc = "完成" + ("+" + "+".join(dict.fromkeys(outcome)) if outcome else "")
    rows.append({"tag": tag, "group": grp, "cell": cell, "rep": rep,
                 "armed": r.get("auto_armed"), "complete": r.get("mission_complete"), "outcome": oc,
                 "alt_max": r.get("alt_max"), "touchdown_mps": r.get("touchdown_mps"),
                 "disarmed": r.get("disarmed"), "n_variance": r.get("n_variance"),
                 "rfnd_stat_gnd": r.get("rfnd_stat_gnd"), "rfnd_stat_air": r.get("rfnd_stat_air"),
                 "rfnd_dist_gnd_min": r.get("rfnd_dist_gnd_min"), "rfnd_dist_gnd_max": r.get("rfnd_dist_gnd_max"),
                 "rfnd_dist_air_min": r.get("rfnd_dist_air_min"), "rfnd_dist_air_max": r.get("rfnd_dist_air_max"),
                 "liftoff_err_max": r.get("liftoff_err_max"), "desc_low_err_max": r.get("desc_low_err_max"),
                 "hov_alt_rms": r.get("hov_alt_rms"), "hov_hgt_est_rms": r.get("hov_hgt_est_rms"),
                 "f_t0": r.get("f_t0"), "f_dur": r.get("f_dur"), "f_rfnd_min": r.get("f_rfnd_min"),
                 "f_rfnd_max": r.get("f_rfnd_max"), "f_rfnd_stats": r.get("f_rfnd_stats"),
                 "f_true_max": r.get("f_true_max"), "f_dev_max": r.get("f_dev_max"),
                 "f_est_err_max": r.get("f_est_err_max"), "f_stop_after": r.get("f_stop_after"),
                 "dev_base": r.get("dev_base"), "land_offset": r.get("land_offset"),
                 "climb_overshoot": r.get("climb_overshoot"), "alt_err_max": r.get("alt_err_max"),
                 "msgs": "; ".join((r.get("msgs") or [])[:3])})

if rows:
    with open(D + "/results.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print("results.csv:", len(rows), "rows")


def med(rs, k, nd=2):
    v = [r[k] for _, r in rs if isinstance(r.get(k), (int, float))]
    if not v:
        return "-"
    f = "{:." + str(nd) + "f}"
    m = f.format(stt.median(v))
    if len(v) > 1:
        return f"{m} [{f.format(min(v))}..{f.format(max(v))}]"
    return m


def outcome(rs):
    outs = {}
    for _, r in rs:
        o = "完成" if r.get("mission_complete") else ("未解锁" if r.get("auto_armed") is False else "未完成")
        outs[o] = outs.get(o, 0) + 1
    return ";".join(f"{k}x{v}" for k, v in sorted(outs.items()))


print("\n## 表1 正常飞行（每配置每任务 5 次，航向 30/150/270/90/210）")
print("| 配置 | 任务 | 完成/5 | 结局 | 真值最高 m | 离地误差max m | 落地前0.5m误差max m | 触地 m/s | 上锁 | EKFvar | RFND.Stat 地面/空中 |")
print("|---|---|---|---|---|---|---|---|---|---|---|")
for g in GROUPS:
    for task, tname in (("d3", "D3"), ("sq", "SQ"), ("hov", "HOV")):
        rs = byg.get((g, task), [])
        if not rs:
            continue
        ok = sum(1 for _, r in rs if r.get("mission_complete"))
        stat = f"{rs[0][1].get('rfnd_stat_gnd')}/{rs[0][1].get('rfnd_stat_air')}"
        print(f"| {GLABEL[g]} | {tname} | {ok}/{len(rs)} | {outcome(rs)} | {med(rs,'alt_max',2)} | "
              f"{med(rs,'liftoff_err_max',3)} | {med(rs,'desc_low_err_max',3)} | {med(rs,'touchdown_mps',2)} | "
              f"{sum(1 for _,r in rs if r.get('disarmed'))}/{len(rs)} | "
              f"{sum(r.get('n_variance') or 0 for _,r in rs)}/{len(rs)} | {stat} |")

print("\n## 表2 异常（每格 3 次，航向 30/150/270；任务 SQ）")
print("| 配置 | 格 | 完成/3 | 结局 | dev_base m | 真值最高 m | EKF高度误差max m | 注入期读数 m | RFND.Stat | 注入到切LAND s | 触地 m/s |")
print("|---|---|---|---|---|---|---|---|---|---|---|")
for g in GROUPS:
    for c in CELLS:
        rs = byg.get((g, c), [])
        if not rs:
            continue
        ok = sum(1 for _, r in rs if r.get("mission_complete"))
        rr = ",".join(f"{r.get('f_rfnd_min')}-{r.get('f_rfnd_max')}" for _, r in rs)
        stt_ = ",".join(str(r.get("f_rfnd_stats")) for _, r in rs)
        print(f"| {GLABEL[g]} | {CLABEL[c]} | {ok}/{len(rs)} | {outcome(rs)} | {med(rs,'dev_base')} | "
              f"{med(rs,'f_true_max')} | {med(rs,'f_est_err_max')} | {rr} | {stt_} | {med(rs,'f_stop_after',1)} | "
              f"{med(rs,'touchdown_mps',2)} |")
