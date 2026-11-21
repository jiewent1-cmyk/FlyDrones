#!/usr/bin/env python3
"""select7.py -- 第 3 步：参数选择。

读入 offline_eval.py 产出的两个表：
  fp_s1.csv   第 1 步的误报表（每个参数组一行）
  det_s2.csv  第 2 步的检出表（每个参数组一行）
规则（按任务书）：
  - 先取"第 1 步零误报"的参数组；
  - 在其中按"第 2 步检出延迟最短"取前三；
  - 若一个零误报的组都没有，则按每小时误报数升序取前三，并写明没有零误报的组。
"""
import csv
import json
import sys

fp_file = sys.argv[1] if len(sys.argv) > 1 else "fp_s1.csv"
det_file = sys.argv[2] if len(sys.argv) > 2 else "det_s2.csv"


def load(fn, key="params"):
    out = {}
    try:
        for r in csv.DictReader(open(fn)):
            out[r[key]] = r
    except FileNotFoundError:
        pass
    return out


fp = load(fp_file)
det = load(det_file)
rows = []
for p, f in fp.items():
    d = det.get(p, {})
    rows.append({
        "params": p,
        "fp_n": int(f.get("n_trigger", 0)),
        "fp_per_h": float(f.get("fp_per_h", 0) or 0),
        "fp_runs": int(f.get("n_runs", 0)),
        "det_n": int(d.get("n_det", 0)),
        "det_runs": int(d.get("n_runs", 0)),
        "delay_med": float(d["delay_med"]) if d.get("delay_med") else None,
        "delay_max": float(d["delay_max"]) if d.get("delay_max") else None,
    })

zero = [r for r in rows if r["fp_n"] == 0]
print("参数组总数 %d，零误报 %d 个" % (len(rows), len(zero)))
if zero:
    # 任务书：按第 2 步检出延迟最短取前三；延迟相同则取检出数多的
    zero.sort(key=lambda r: (r["delay_med"] if r["delay_med"] is not None else 99,
                             r["delay_max"] if r["delay_max"] is not None else 99,
                             -r["det_n"]))
    print("\n零误报组按第 2 步检出延迟排序（前 5）：")
    for r in zero[:5]:
        print("  %s  检出 %d/%d  延迟中位 %s s 最大 %s s" %
              (r["params"], r["det_n"], r["det_runs"], r["delay_med"], r["delay_max"]))
    print("\n第 3 步选中：")
    for r in zero[:3]:
        print("  %s" % r["params"])
    json.dump(zero[:3], open("step3_selected.json", "w"), indent=1)
else:
    # 没有零误报的组：按每小时误报数升序；误报数相同时用第 2 步检出完整度破平，再按延迟
    rows.sort(key=lambda r: (r["fp_per_h"], r["fp_n"], -r["det_n"],
                              r["delay_med"] if r["delay_med"] is not None else 99,
                              r["delay_max"] if r["delay_max"] is not None else 99))
    print("\n没有零误报的参数组。按每小时误报数升序取前三"
          "（误报数相同时先比第 2 步检出完整度，再比延迟中位）：")
    for r in rows[:8]:
        print("  %s  误报 %d/%d  %.2f 次/小时  检出 %d/%d 延迟中位 %s" %
              (r["params"], r["fp_n"], r["fp_runs"], r["fp_per_h"], r["det_n"], r["det_runs"], r["delay_med"]))
    print("\n第 3 步选中：")
    for r in rows[:3]:
        print("  %s" % r["params"])
    json.dump(rows[:3], open("step3_selected.json", "w"), indent=1)
