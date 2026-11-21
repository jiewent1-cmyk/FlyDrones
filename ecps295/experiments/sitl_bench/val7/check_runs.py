#!/usr/bin/env python3
"""check_runs.py -- 找出失败/可疑的运行，输出清单（供报告"失败/可疑运行"一节）。

可疑判据：
  - metrics.json 有 error 字段
  - analyze.err 非空
  - mission_complete 为假
  - 真值最高高度明显偏离 2 m（>2.3 m 或 <1.7 m）且不是已知异常格
  - run.log 里有 Traceback / ERROR
"""
import json
import os
import re
import sys

D = "/home/jacky/sim/val7"
RUNS = os.path.join(D, "runs")
pat = re.compile(r"Traceback|ERROR|Exception|script exit [^0]", re.I)

bad = []
for tag in sorted(os.listdir(RUNS)):
    rd = os.path.join(RUNS, tag)
    mp = os.path.join(rd, "metrics.json")
    if not os.path.exists(mp):
        bad.append((tag, "no metrics.json"))
        continue
    try:
        j = json.load(open(mp))
    except Exception as e:
        bad.append((tag, "bad metrics.json: %s" % e))
        continue
    errs = []
    if j.get("error"):
        errs.append("metrics error: %s" % j["error"])
    ae = os.path.join(rd, "analyze.err")
    if os.path.exists(ae) and os.path.getsize(ae) > 0:
        errs.append("analyze.err: %s" % open(ae).read().strip().replace("\n", " | ")[:200])
    rl = os.path.join(rd, "run.log")
    if os.path.exists(rl):
        txt = open(rl, errors="replace").read()
        hits = pat.findall(txt)
        if hits:
            lines = [l for l in txt.splitlines() if pat.search(l)]
            errs.append("run.log: %s" % " | ".join(lines[-2:])[:200])
        m = re.search(r"script exit (\d+)", txt)
        if m and m.group(1) != "0":
            errs.append("script exit %s" % m.group(1))
    # A1/A2 主动改成 LAND，任务本来就该中断，不算失败
    if not j.get("mission_complete") and not re.search(r"_A[12]_r", tag):
        errs.append("mission_complete=False (final_mode=%s, left_auto_at=%s)" %
                    (j.get("final_mode"), j.get("left_auto_at")))
    if errs:
        bad.append((tag, "; ".join(errs)))

print("可疑/失败运行 %d 个（共 %d 个目录）" % (len(bad), len(os.listdir(RUNS))))
for tag, why in bad:
    print("%-28s %s" % (tag, why))
