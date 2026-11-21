#!/usr/bin/env python3
"""V5 summary: runs/*/metrics.json -> results.csv + summary_tables.md (one row per group)."""
import glob
import json
import re
import statistics as stt
import subprocess

D = "/home/jacky/sim/val5"
subprocess.run(["python3", D + "/collect.py"], capture_output=True)  # results.csv

runs = []
for f in sorted(glob.glob(D + "/runs/*/metrics.json")):
    r = json.load(open(f))
    r["group"] = re.sub(r"_r\d+$", "", r["tag"])
    runs.append(r)
groups = {}
for r in runs:
    groups.setdefault(r["group"], []).append(r)


def num(rs, c):
    v = [r[c] for r in rs if isinstance(r.get(c), (int, float)) and not isinstance(r.get(c), bool)]
    return v


def cell(rs, c):
    v = num(rs, c)
    if not v:
        return "-"
    if len(v) == 1:
        return f"{v[0]:.3f}"
    return f"{stt.median(v):.3f} [{min(v):.3f}–{max(v):.3f}]"


def outcome(rs):
    """short outcome string per group"""
    bad = []
    n_var = sum(1 for r in rs if any("variance" in (m or "") for m in (r.get("msgs") or [])))
    n_fs = sum(1 for r in rs if any(re.search("failsafe|FS:", m or "", re.I) for m in (r.get("msgs") or [])))
    n_fence = sum(1 for r in rs if any(re.search("breach|Fence action|FENCE_ACTION", m or "", re.I) for m in (r.get("msgs") or [])))
    n_mode = sum(1 for r in rs if r.get("left_auto_at") is not None)
    n_land = sum(1 for r in rs if r.get("final_mode") not in (3, None))
    if n_var:
        bad.append(f"EKF variance {n_var}")
    if n_fs:
        bad.append(f"failsafe {n_fs}")
    if n_fence:
        bad.append(f"fence {n_fence}")
    if n_mode:
        bad.append(f"left AUTO {n_mode}")
    if n_land:
        bad.append(f"final mode !=AUTO {n_land}")
    return ", ".join(bad) if bad else "no mode change / no failsafe"


COLS = ["track_rms", "track_max", "lap3", "land_offset", "est_jump_max", "spd_max", "yaw_err_max",
        "n_gated", "gate_innov_rej", "n_obs_bad", "n_obs_badyaw", "n_block_rows", "n_block_vis"]
HEAD = ["track RMS", "truth max off track", "lap3 err", "land offset", "EKF jump max", "truth spd max",
        "yaw err max", "gated obs", "innov rej", "bad pos obs", "bad yaw obs", "block ticks", "block ticks w/ marker"]

ORDER = ["base_none", "base_sq4", "base_sq1"] + \
        [f"sq4_{n}" for n in ["bad1m_lap1", "bad1m_lap2", "bad03_10s", "yaw20_5s", "block30", "stop10",
                             "stop_norec", "lat500_20s"]] + \
        [f"sq1_{n}" for n in ["bad1m_lap1", "bad1m_lap2", "bad03_10s", "yaw20_5s", "block30", "stop10",
                             "stop_norec", "lat500_20s"]]
md = ["# V5 汇总表（自动生成，tables_v5.py）", "",
      "中位数 [最小–最大]。每格 5 次。列：track RMS / 真值离航线最大距离 / 第 3 圈回角误差 / 落点偏差 /",
      "EKF 位置单步跳变 / 真值速度峰值 / EKF 航向误差峰值 / 机载门限拒掉条数 / 新息门限拒掉条数 /",
      "注入的错检位置条数 / 注入的错检航向条数 / 遮挡计时条数 / 其中标记本应可见的条数。", "",
      "| 布置×故障 | n | 完成 | " + " | ".join(HEAD) + " | 结局 |",
      "|---" * (len(HEAD) + 4) + "|"]
for g in ORDER:
    rs = groups.get(g, [])
    if not rs:
        continue
    ok = sum(1 for r in rs if r.get("mission_complete"))
    md.append(f"| {g} | {len(rs)} | {ok}/{len(rs)} | " + " | ".join(cell(rs, c) for c in COLS) + f" | {outcome(rs)} |")
other = [g for g in sorted(groups) if g not in ORDER]
for g in other:
    rs = groups[g]
    ok = sum(1 for r in rs if r.get("mission_complete"))
    md.append(f"| {g} | {len(rs)} | {ok}/{len(rs)} | " + " | ".join(cell(rs, c) for c in COLS) + f" | {outcome(rs)} |")
md.append("")
md.append("## 每格逐次明细")
for g in ORDER:
    rs = groups.get(g, [])
    if not rs:
        continue
    md.append(f"### {g}")
    md.append("")
    md.append("| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |")
    md.append("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for r in sorted(rs, key=lambda x: x["tag"]):
        jt = f"{r.get('est_jump_max')} ({r.get('est_jump_t')})"
        msg = "; ".join((r.get("msgs") or [])[:3])
        md.append("| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |".format(
            r["tag"], r.get("mission_complete"), r.get("track_rms"), r.get("track_max"), r.get("lap3"),
            r.get("land_offset"), jt, r.get("spd_max"), r.get("n_gated"), r.get("gate_innov_rej"),
            (r.get("n_obs_bad", 0) or 0) + (r.get("n_obs_badyaw", 0) or 0), msg))
    md.append("")
open(D + "/summary_tables.md", "w").write("\n".join(md))
print("\n".join(md))
