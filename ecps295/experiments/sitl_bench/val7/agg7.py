#!/usr/bin/env python3
"""agg7.py -- 汇总 runs/*/metrics.json -> results.csv，并按配置分组打印汇总表。"""
import csv
import json
import os
import re
import statistics as st
import sys

D = "/home/jacky/sim/val7"
RUNS = os.path.join(D, "runs")

COLS = ["tag", "config", "mission", "ok", "mon_trigger", "mon_reason", "mon_trigger_rel", "mon_detect_delay",
        "mon_truth_dev_at_trig", "mon_truth_at_trig", "mon_e_mean", "mon_e_std", "mon_e_absmax",
        "mon_action", "mon_action_rel", "mon_action_delay", "mon_action_to_touchdown", "mon_ekf_jump",
        "mon_truth_change", "mon_truth_max_after_action",
        "alt_max", "truth_max_all", "truth_min_all", "truth_dev_max", "f_after_max",
        "mission_complete", "left_auto_at", "flight_s", "touchdown_mps", "land_desc_rate_max",
        "land_desc_rate_mean", "land_hang_s", "land_offset", "f_t0", "f_true_max", "n_variance", "error"]


COLS += ["mon_trigger_t", "mon_e_at_trig", "mon_hr_at_trig", "mon_hb_at_trig", "mon_trig_A", "mon_trig_B",
         "mon_ekf_hgt_before", "mon_ekf_hgt_after", "mon_truth_min_after_action", "alt_err_max",
         "final_mode", "disarmed"]


def cfg(tag):
    """从 tag 解析配置名：v7_s1_<cond>_<mn>_r<i> / v7_s2_<cell>_r<i> / v7_s4_<cell>_<act>_r<i>"""
    m = re.match(r"v7_(s1|s2|s4|s5)_(.+?)_r(\d+)$", tag)
    if not m:
        return tag
    return "%s_%s" % (m.group(1), m.group(2))


def med(v):
    v = [x for x in v if x is not None]
    return round(st.median(v), 3) if v else None


def rng(v):
    v = [x for x in v if x is not None]
    if not v:
        return None
    return "%s..%s" % (round(min(v), 3), round(max(v), 3))


def main():
    rows = []
    for tag in sorted(os.listdir(RUNS)):
        p = os.path.join(RUNS, tag, "metrics.json")
        if not os.path.exists(p):
            continue
        try:
            d = json.load(open(p))
        except Exception as e:
            rows.append({"tag": tag, "config": cfg(tag), "ok": 0, "error": "bad json %s" % e})
            continue
        r = {"tag": tag, "config": cfg(tag), "mission": d.get("mission"),
             "ok": 0 if d.get("error") else 1, "error": d.get("error")}
        for c in COLS:
            if c in d:
                r[c] = d[c]
        rows.append(r)
    with open(os.path.join(D, "results.csv"), "w") as fh:
        w = csv.DictWriter(fh, fieldnames=COLS, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, "") for c in COLS})
    print("runs:", len(rows), "-> results.csv")

    groups = {}
    for r in rows:
        groups.setdefault(r["config"], []).append(r)
    keys = sys.argv[1:] if len(sys.argv) > 1 else ["mon_trigger_rel", "mon_detect_delay", "mon_truth_dev_at_trig",
                                                  "alt_max", "touchdown_mps", "mon_ekf_jump", "land_offset"]
    print("%-28s %4s %4s  %s" % ("config", "n", "ok", "  ".join("%s(med[range])" % k for k in keys)))
    for c in sorted(groups):
        g = groups[c]
        line = "%-28s %4d %4d  " % (c, len(g), sum(1 for x in g if x.get("ok")))
        for k in keys:
            v = [x.get(k) for x in g]
            if k == "mon_trigger_rel":
                line += "%d/%d        " % (sum(1 for x in g if x.get(k) is not None), len(g))
            else:
                line += "%s[%s]  " % (med(v), rng(v))
        print(line)

    # markdown 表（写 summary.md，便于贴进报告）
    mk = ["mon_trigger_rel", "mon_detect_delay", "mon_truth_dev_at_trig", "truth_max_all", "touchdown_mps",
          "land_desc_rate_max", "mon_ekf_jump", "land_offset", "flight_s"]
    with open(os.path.join(D, "summary.md"), "w") as fh:
        fh.write("| 配置 | 运行数 | 完成数 | %s |\n" % " | ".join(mk))
        fh.write("|" + "|".join(["---"] * (len(mk) + 3)) + "|\n")
        for c in sorted(groups):
            g = groups[c]
            cells = []
            for k in mk:
                v = [x.get(k) for x in g]
                if k == "mon_trigger_rel":
                    cells.append("%d/%d" % (sum(1 for x in g if x.get(k) is not None), len(g)))
                else:
                    cells.append("%s [%s]" % (med(v), rng(v)))
            fh.write("| %s | %d | %d | %s |\n" % (c, len(g), sum(1 for x in g if x.get("ok")), " | ".join(cells)))
    print("summary.md written")


if __name__ == "__main__":
    main()
