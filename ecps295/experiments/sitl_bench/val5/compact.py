#!/usr/bin/env python3
"""Compact per-group table for report.md: layout x fault -> n, completed, key metrics, outcome."""
import glob
import json
import re
import statistics as stt

D = "/home/jacky/sim/val5"
groups = {}
for f in sorted(glob.glob(D + "/runs/*/metrics.json")):
    if not f or open(f).read(1) == "":
        continue
    r = json.load(open(f))
    groups.setdefault(re.sub(r"_r\d+$", "", r["tag"]), []).append(r)


def cell(rs, c):
    v = [r[c] for r in rs if isinstance(r.get(c), (int, float)) and not isinstance(r.get(c), bool)]
    if not v:
        return "-"
    if len(v) == 1:
        return f"{v[0]:.3f}"
    return f"{stt.median(v):.3f} [{min(v):.3f}–{max(v):.3f}]"


def outcome(rs):
    bad = []
    for pat, lbl in (("variance", "EKF variance"), ("failsafe|FS:", "failsafe"),
                      ("breach|Fence action|FENCE_ACTION", "fence")):
        n = sum(1 for r in rs if any(re.search(pat, m or "", re.I) for m in (r.get("msgs_all") or r.get("msgs") or [])))
        if n:
            bad.append(f"{lbl} {n}")
    n = sum(1 for r in rs if r.get("left_auto_at") is not None)
    if n:
        bad.append(f"left AUTO {n}")
    n = sum(1 for r in rs if r.get("final_mode") not in (3, None))
    if n:
        bad.append(f"final mode!=AUTO {n}")
    return ", ".join(bad) if bad else "无切模式/无告警"


ORDER = ["base_none", "base_sq4", "base_sq1"] + \
        [f"sq4_{n}" for n in ["bad1m_lap1", "bad1m_lap2", "bad03_10s", "yaw20_5s", "block30", "stop10",
                             "stop_norec", "lat500_20s"]] + \
        [f"sq1_{n}" for n in ["bad1m_lap1", "bad1m_lap2", "bad03_10s", "yaw20_5s", "block30", "stop10",
                             "stop_norec", "lat500_20s"]]
out = ["| 布置 × 故障 | n | 完成 | 跟踪 RMS | 真值离航线最大 | 第 3 圈回角 | 落点偏差 | EKF 跳变 | 真值速度峰值 | 门限拒掉 | 注入条数 | 结局 |",
       "|---|---|---|---|---|---|---|---|---|---|---|---|"]
for g in ORDER:
    rs = groups.get(g, [])
    if not rs:
        continue
    ok = sum(1 for r in rs if r.get("mission_complete"))
    inj = []
    for c in ("n_obs_bad", "n_obs_badyaw", "n_block_rows"):
        v = [r.get(c) or 0 for r in rs]
        if max(v):
            inj.append(f"{c.replace('n_', '')}={sum(v)}")
    out.append("| {} | {} | {}/{} | {} | {} | {} | {} | {} | {} | {} | {} | {} |".format(
        g, len(rs), ok, len(rs), cell(rs, "track_rms"), cell(rs, "track_max"), cell(rs, "lap3"),
        cell(rs, "land_offset"), cell(rs, "est_jump_max"), cell(rs, "spd_max"),
        cell(rs, "n_gated"), ", ".join(inj) or "-", outcome(rs)))
open(D + "/table_compact.md", "w").write("\n".join(out) + "\n")
print("\n".join(out))
