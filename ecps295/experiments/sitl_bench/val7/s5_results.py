#!/usr/bin/env python3
# s5_results.py -- 从 results.csv 生成 s5_results.md（第 5 步：监视器自身失效）
import csv
import re
import statistics as st

D = "/home/jacky/sim/val7"
rows = list(csv.DictReader(open(D + "/results.csv")))


def grp(g):
    return [r for r in rows if r["config"] == "s5_" + g]


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
    return "[%s..%s]" % (f(min(v)), f(max(v))) if v else "-"


def cnt(rs, k, val):
    return sum(1 for r in rs if r.get(k) == val)


out = []
w = out.append
w("| 组 | n | 监视器动作 | 注入→动作 [s] | 动作→触地 [s] | 真值最高 [m] | 着地速度 [m/s] | 落点偏差 [m] | 任务完成 | 飞控最终模式 |")
w("|---|---|---|---|---|---|---|---|---|---|")
for g in ["stop", "press", "p0"]:
    rs = grp(g)
    if not rs:
        continue
    acted = sum(1 for r in rs if r.get("mon_action") not in ("", "None"))
    w("| %s | %d | %d/%d | %s %s | %s %s | %s %s | %s %s | %s %s | %d/%d | %s |" % (
        g, len(rs), acted, len(rs),
        med(rs, "mon_action_delay"), rng(rs, "mon_action_delay"),
        med(rs, "mon_action_to_touchdown"), rng(rs, "mon_action_to_touchdown"),
        med(rs, "truth_max_all"), rng(rs, "truth_max_all"),
        med(rs, "touchdown_mps"), rng(rs, "touchdown_mps"),
        med(rs, "land_offset"), rng(rs, "land_offset"),
        cnt(rs, "mission_complete", "True"), len(rs),
        ",".join(sorted(set(r.get("final_mode", "") for r in rs))) or "-"))
w("")

w("逐次运行明细：\n")
w("| 运行 | 监视器动作 | 注入→动作 [s] | 真值最高 [m] | 真值最低 [m] | 着地速度 [m/s] | 任务完成 | 最终模式 |")
w("|---|---|---|---|---|---|---|---|")
for g in ["stop", "press", "p0"]:
    for r in sorted(grp(g), key=lambda x: x["tag"]):
        w("| %s | %s | %s | %s | %s | %s | %s | %s |" % (
            r["tag"], r.get("mon_action") or "-", r.get("mon_action_delay") or "-",
            r.get("truth_max_all") or "-", r.get("truth_min_all") or "-",
            r.get("touchdown_mps") or "-", r.get("mission_complete") or "-",
            r.get("final_mode") or "-"))
w("")

open(D + "/s5_results.md", "w").write("\n".join(out) + "\n")
print("s5_results.md written")

try:
    open(D + "/s5_results.md", "a").write("\n" + open(D + "/s5_prose.md").read())
except IOError:
    pass
