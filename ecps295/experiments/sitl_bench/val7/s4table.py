#!/usr/bin/env python3
"""s4table.py -- 第 4 步闭环结果汇总表（markdown）。

读 runs/*/metrics.json，按 异常格 × 动作 分组，输出：
  真值最高/最低高度、相对无异常基线的偏差、注入->动作时间、动作->落地时间、
  落地控制性（下降率、着地速度、悬停）、落点偏差、A3 任务是否完成、切源 EKF 跳变。
"""
import json
import os
import re
import statistics as st
import sys

D = "/home/jacky/sim/val7"
RUNS = os.path.join(D, "runs")


def load(tag):
    p = os.path.join(RUNS, tag, "metrics.json")
    if not os.path.exists(p):
        return None
    try:
        return json.load(open(p))
    except Exception:
        return None


def med(v):
    v = [x for x in v if x is not None]
    return round(st.median(v), 3) if v else None


def rng(v):
    v = [x for x in v if x is not None]
    if not v:
        return "-"
    return "%s..%s" % (round(min(v), 3), round(max(v), 3))


def medstr(v):
    v = [x for x in v if x is not None]
    if not v:
        return "-"
    return "%s [%s]" % (med(v), rng(v))


def groups(prefix):
    g = {}
    for tag in sorted(os.listdir(RUNS)):
        m = re.match(re.escape(prefix) + r"_(.+?)_r(\d+)$", tag)
        if not m:
            continue
        d = load(tag)
        if d:
            g.setdefault(m.group(1), []).append(d)
    return g


def main():
    base = [d for d in [load("v7_s4_clean_r%d" % i) for i in range(1, 11)] if d]
    bmax = med([d.get("truth_max_all") for d in base])
    bmin = med([d.get("truth_min_all") for d in base])
    print("无异常基线(%d 次闭环): 真值最高 %.3f m  真值最低 %.3f m" % (len(base), bmax or 0, bmin or 0))
    hdr = ["| 组 | 次数 | 触发 | 真值最高 | 真值最低 | 偏离基线 | 注入->动作 | 动作->落地 | 着地速度 | 下降率最大 | 落点偏差 | 任务完成 | EKF 跳变 |",
           "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    lines = []
    prefixes = sys.argv[1:] or ["v7_s4", "v7_s4_d3", "v7_s5"]
    for prefix in prefixes:
        g = groups(prefix)
        for name in sorted(g):
            ds = g[name]
            trig = sum(1 for d in ds if d.get("mon_trigger_rel") is not None)
            tmax = med([d.get("truth_max_all") for d in ds])
            tmin = med([d.get("truth_min_all") for d in ds])
            dev = med([(d.get("truth_max_all") or 0) - (bmax or 0) for d in ds if d.get("truth_max_all") is not None])
            ad = med([d.get("mon_action_delay") for d in ds])
            td = med([d.get("mon_action_to_touchdown") for d in ds])
            tdm = med([d.get("touchdown_mps") for d in ds])
            drm = med([d.get("land_desc_rate_max") for d in ds])
            off = med([d.get("land_offset") for d in ds])
            mc = sum(1 for d in ds if d.get("mission_complete"))
            jmp = med([d.get("mon_ekf_jump") for d in ds])
            lines.append("| %s_%s | %d | %d/%d | %s | %s | %s | %s | %s | %s | %s | %s | %d/%d | %s |" % (
                prefix.replace("v7_s4", "s4"), name, len(ds), trig, len(ds),
                medstr([d.get("truth_max_all") for d in ds]), medstr([d.get("truth_min_all") for d in ds]),
                ("%+.3f" % dev) if dev is not None else "-",
                medstr([d.get("mon_action_delay") for d in ds]), medstr([d.get("mon_action_to_touchdown") for d in ds]),
                medstr([d.get("touchdown_mps") for d in ds]), medstr([d.get("land_desc_rate_max") for d in ds]),
                medstr([d.get("land_offset") for d in ds]), mc, len(ds), medstr([d.get("mon_ekf_jump") for d in ds])))
    open(os.path.join(D, "s4_table.md"), "w").write("\n".join(hdr + lines) + "\n")
    print("\n".join(hdr + lines))


if __name__ == "__main__":
    main()
