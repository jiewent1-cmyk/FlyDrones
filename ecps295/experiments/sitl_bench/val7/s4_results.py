#!/usr/bin/env python3
# s4_results.py -- 从 results.csv 生成 s4_results.md（第 4 步闭环结果）
import csv
import re
import statistics as st
from collections import Counter

D = "/home/jacky/sim/val7"
rows = list(csv.DictReader(open(D + "/results.csv")))


def grp(g):
    return [r for r in rows if r["config"] == "s4_" + g]


def num(rs, k):
    v = []
    for r in rs:
        x = r.get(k, "")
        if x not in ("", "None", None):
            try:
                v.append(float(x))
            except ValueError:
                pass
    return v


def f(x, n=3):
    return ("%." + str(n) + "f") % x


def med(rs, k):
    v = num(rs, k)
    return f(st.median(v)) if v else "-"


def rng(rs, k):
    v = num(rs, k)
    if not v:
        return "-"
    return "[%s..%s]" % (f(min(v)), f(max(v)))


def cnt(rs, k, val):
    return sum(1 for r in rs if r.get(k) == val)


base = grp("clean")
bmax = max(num(base, "truth_max_all")) if num(base, "truth_max_all") else None
cells = ["a", "c", "g3", "g5", "g10", "gN", "h", "i", "j1", "j2", "k"]
acts = ["A1", "A2", "A3", "none"]

out = []
w = out.append
w("### 8.1 无异常基线（10 次闭环，监视器在线但无异常注入）\n")
w("| 运行数 | 完成数 | 监视器触发 | 真值最高 [m] | 真值最低 [m] | 着地速度 [m/s] | 落点偏差 [m] | 任务完成 |")
w("|---|---|---|---|---|---|---|---|")
w("| %d | %d | %d/%d | %s %s | %s %s | %s %s | %s %s | %d/%d |" % (
    len(base), cnt(base, "ok", "1"),
    sum(1 for r in base if r.get("mon_trigger_rel") not in ("", "None")), len(base),
    med(base, "truth_max_all"), rng(base, "truth_max_all"),
    med(base, "truth_min_all"), rng(base, "truth_min_all"),
    med(base, "touchdown_mps"), rng(base, "touchdown_mps"),
    med(base, "land_offset"), rng(base, "land_offset"),
    cnt(base, "mission_complete", "True"), len(base)))
w("")
w("基线取真值最高的中位数 **%.3f m**（范围 %s m）。\n" % (
    bmax, rng(base, "truth_max_all")))
w("10 次无异常闭环飞行监视器 **一次都没有触发**（0/10），飞控没有出现任何监视器相关的动作。\n")
w("")

w("### 8.2 每个异常格 × 动作：触发、动作延迟、落地\n")
w("| 异常格 | 动作 | 触发 | 注入→动作 [s] | 动作→触地 [s] | 真值最高 [m] | 相对基线 [m] | 着地速度 [m/s] | 下降率最陡 [m/s] | 空中悬停 [s] | 落点偏差 [m] | 任务完成 |")
w("|---|---|---|---|---|---|---|---|---|---|---|---|")
for c in cells:
    for a in acts:
        rs = grp("%s_%s" % (c, a))
        if not rs:
            continue
        tmax = num(rs, "truth_max_all")
        w("| %s | %s | %d/%d | %s %s | %s %s | %s %s | %s | %s %s | %s | %s | %s %s | %d/%d |" % (
            c, a, sum(1 for r in rs if r.get("mon_trigger_rel") not in ("", "None")), len(rs),
            med(rs, "mon_action_delay"), rng(rs, "mon_action_delay"),
            med(rs, "mon_action_to_touchdown"), rng(rs, "mon_action_to_touchdown"),
            med(rs, "truth_max_all"), rng(rs, "truth_max_all"),
            ("%+.3f" % (st.median(tmax) - bmax)) if tmax else "-",
            med(rs, "touchdown_mps"), rng(rs, "touchdown_mps"),
            med(rs, "land_desc_rate_max"),
            med(rs, "land_hang_s"),
            med(rs, "land_offset"), rng(rs, "land_offset"),
            cnt(rs, "mission_complete", "True"), len(rs)))
w("")

w("### 8.3 `D3`（LOITER 手动起飞）复测：`gN` / `h` / `k`\n")
w("| 异常格 | 动作 | 触发 | 注入→动作 [s] | 动作→触地 [s] | 真值最高 [m] | 相对基线 [m] | 着地速度 [m/s] | 落点偏差 [m] | 任务完成 |")
w("|---|---|---|---|---|---|---|---|---|---|")
for c in ["gN", "h", "k"]:
    for a in acts:
        rs = grp("d3_%s_%s" % (c, a))
        if not rs:
            continue
        tmax = num(rs, "truth_max_all")
        w("| %s | %s | %d/%d | %s %s | %s %s | %s %s | %s | %s %s | %s %s | %d/%d |" % (
            c, a, sum(1 for r in rs if r.get("mon_trigger_rel") not in ("", "None")), len(rs),
            med(rs, "mon_action_delay"), rng(rs, "mon_action_delay"),
            med(rs, "mon_action_to_touchdown"), rng(rs, "mon_action_to_touchdown"),
            med(rs, "truth_max_all"), rng(rs, "truth_max_all"),
            ("%+.3f" % (st.median(tmax) - bmax)) if tmax else "-",
            med(rs, "touchdown_mps"), rng(rs, "touchdown_mps"),
            med(rs, "land_offset"), rng(rs, "land_offset"),
            cnt(rs, "mission_complete", "True"), len(rs)))
w("")

w("### 8.4 A3（只切源组、不切模式）的任务完成情况与动作后高度误差\n")
w("| 组 | 任务完成 | 动作后高度误差 `alt_err_max` 中位 [m] | 动作后真值最低 [m] | EKF 高度跳变 [m] |")
w("|---|---|---|---|---|")
for g in ["a_A3", "c_A3", "g3_A3", "g5_A3", "g10_A3", "gN_A3", "h_A3", "i_A3", "j1_A3", "j2_A3", "k_A3",
          "d3_gN_A3", "d3_h_A3", "d3_k_A3", "clean"]:
    rs = grp(g)
    if not rs:
        continue
    w("| %s | %d/%d | %s %s | %s | %s %s |" % (
        g, cnt(rs, "mission_complete", "True"), len(rs),
        med(rs, "alt_err_max"), rng(rs, "alt_err_max"),
        med(rs, "mon_truth_min_after_action"),
        med(rs, "mon_ekf_jump"), rng(rs, "mon_ekf_jump")))
w("")

w("### 8.5 源切换（EKF 高度估计跳变）与实际高度变化\n")
w("`mon_ekf_jump` = 切源组前后 XKF1 记录的 EKF 高度差；`mon_truth_change` = 同一时刻真值高度的变化。\n")
w("| 组 | EKF 跳变 [m] | 真值变化 [m] | 真值最高 [m] |")
w("|---|---|---|---|")
for g in ["a_A2", "a_A3", "c_A2", "c_A3", "gN_A2", "gN_A3", "h_A2", "h_A3", "k_A2", "k_A3",
          "d3_gN_A2", "d3_gN_A3", "d3_h_A2", "d3_h_A3", "d3_k_A2", "d3_k_A3", "i_A2", "i_A3"]:
    rs = grp(g)
    if not rs:
        continue
    w("| %s | %s %s | %s %s | %s |" % (g, med(rs, "mon_ekf_jump"), rng(rs, "mon_ekf_jump"),
                                        med(rs, "mon_truth_change"), rng(rs, "mon_truth_change"),
                                        med(rs, "truth_max_all")))
w("")

w("### 8.6 各组出现过的飞控 `STATUSTEXT`（次数）\n")
msgs = {}
try:
    for line in open(D + "/msgs_sum.txt"):
        m = re.match(r"(\S+)\s+(\d+)\s+(.*)", line.rstrip())
        if m:
            msgs[m.group(1)] = (int(m.group(2)), m.group(3))
except IOError:
    pass
w("| 组 | n | STATUSTEXT |")
w("|---|---|---|")
for g in sorted(msgs):
    n, s = msgs[g]
    w("| %s | %d | %s |" % (g, n, s))
w("")

open(D + "/s4_results.md", "w").write("\n".join(out) + "\n")
print("s4_results.md written", len(out))

try:
    open(D + "/s4_results.md", "a").write("\n" + open(D + "/s4_prose.md").read())
except IOError:
    pass
