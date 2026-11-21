#!/usr/bin/env python3
"""runs/*/metrics.json -> summary_tables.md (median [min-max] per group, group = tag without _rN). Also refreshes results.csv."""
import glob
import json
import re
import statistics as stt
import subprocess

D = "/home/jacky/sim/abs_fix"
subprocess.run(["python3", D + "/collect.py"], capture_output=True)  # writes results.csv
groups = {}
for f in sorted(glob.glob(D + "/runs/*/metrics.json")):
    r = json.load(open(f))
    groups.setdefault(re.sub(r"_r\d+$", "", r["tag"]), []).append(r)


def cell(rs, c):
    v = [r[c] for r in rs if isinstance(r.get(c), (int, float)) and not isinstance(r.get(c), bool)]
    if not v:
        return "-"
    return f"{stt.median(v):.2f}" if len(v) == 1 else f"{stt.median(v):.2f} [{min(v):.2f}–{max(v):.2f}]"


def table(title, pat, cols):
    out = [f"### {title}", "", "| 组 | n | 完成 | " + " | ".join(cols) + " |", "|---" * (len(cols) + 3) + "|"]
    for g in sorted(groups):
        if re.fullmatch(pat, g):
            rs = groups[g]
            ok = sum(1 for r in rs if r.get("mission_complete"))
            out.append(f"| {g} | {len(rs)} | {ok}/{len(rs)} | " + " | ".join(cell(rs, c) for c in cols) + " |")
    return "\n".join(out) + "\n"


HC = ["dev_max", "dev_rms", "dev_end", "land_offset", "est_jump_max", "corr_spd_max", "corr_move_max", "yaw_err_end"]
SC = ["track_rms", "track_max", "lap1", "lap2", "lap3", "land_offset", "est_jump_max", "spd_max", "yaw_err_end"]
md = ["# abs_fix 汇总表（自动生成，tables.py）", "",
      "中位数 [最小–最大]，单位 m / m/s / deg。列定义见 abs_analyze.py 文件头。", ""]
md.append(table("悬停 2 m 120 s：基线与两种方法", r"(b_H2|en_H2_.*|ep_H2_.*)", HC))
md.append(table("正方形 2 m × 2 m 3 圈：基线与两种方法", r"(b_SQ|en_SQ_.*|ep_SQ_.*)", SC))
md.append(table("降低高度（无修正）", r"b_(H2|H15|H1)", HC))
md.append(table("降低高度（无修正）正方形", r"b_(SQ|SQ15|SQ1)", SC))
md.append(table("信标对照（SITL 无噪声 UWB，罗盘航向）", r"bcn4?_.*", HC + ["track_rms", "lap3"]))
md.append(table("航向：场地 x 轴与开机机头差 10°（无罗盘）", r"(b|en|ep)_SQ.*rot10.*", SC))
md.append(table("航向：有罗盘（EKF 坐标系 NED）；m2 = SIM_MAG_MOT 2/1，mot = 10/5 mGauss/A", r"(b|en|ep)_SQ.*cmp.*", SC))
md.append(table("失效：错检 / 机载电脑停止 / 无门限 20% 错检（悬停 2 m）", r"(rec_)?(en|ep)_H2_(bad|stop|nogate|p30_bad).*", HC + ["est_err_max", "spd_max"]))
md.append(table("其他：光流比例 +10%、5 Hz、reset_counter、VISO_DELAY_MS 失配、LOITER_TIME、claim 0.3", r"((b|en|ep)_SQ.*(fs10|5hz|rst|dl40|c30))|((en|ep)_H2.*(rst|c30))|((en|ep)_H2L_.*)", HC[:5] + SC[:1] + ["lap3", "spd_max"]))
md.append(table("推荐配置复核（gate v2 + EK3_GLITCH_RAD 5 + claim max(0.1, 2σ)）", r"rec_.*", HC[:5] + ["track_rms", "lap3", "spd_max", "yaw_err_end"]))
open(D + "/summary_tables.md", "w").write("\n".join(md))
print("\n".join(md))
