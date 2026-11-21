#!/usr/bin/env python3
# mk_tables.py -- 生成报告里的总表
import csv
import json
import os

D = "/home/jacky/sim/val7"
OUT = open(os.path.join(D, "total_table.md"), "w")


def w(s=""):
    OUT.write(s + "\n")


def fp_rows():
    rows = list(csv.DictReader(open(os.path.join(D, "fp_s1.csv"))))
    out = []
    for r in rows:
        p = json.loads(r["params"])
        out.append((p["fc"], p["tau"], p["E"], p["T"], p["R"], int(r["n_trigger"]), int(r["n_runs"]),
                    int(r["n_trig_A"]), int(r["n_trig_B"]), float(r["fp_per_h"])))
    out.sort(key=lambda x: (x[9], x[5]))
    return out


def det_rows():
    rows = list(csv.DictReader(open(os.path.join(D, "det_s2.csv"))))
    out = []
    for r in rows:
        p = json.loads(r["params"])
        out.append((p["fc"], p["tau"], p["E"], p["T"], p["R"], int(r["n_det"]), int(r["n_runs"]),
                    r["delay_med"] or "-", r["delay_max"] or "-"))
    out.sort(key=lambda x: (-x[5], float(x[7]) if x[7] != "-" else 99))
    return out


def main():
    w("## 总表")
    w()
    w("### A. 第 0 步：工具链自检（无异常 2 m 悬停）")
    w()
    w("| 配置 | 运行数 | 完成数 | 真值高度 | 关键指标 | 是否与预期一致 |")
    w("|---|---|---|---|---|---|")
    w("| 无异常悬停 2 m（`runs/v7_step0`） | 1 | 1 | 见第 4 节 | 真值 2.0 m 附近 | 是 |")
    w()
    w("### B. 第 1 步：误报（每组参数 220 次飞行，全部完成，9.844 飞行小时）")
    w()
    w("| fc | tau | E | T | R | 触发飞行数 | 判据A | 判据B | 次/小时 | 零误报 |")
    w("|---|---|---|---|---|---|---|---|---|---|")
    for r in fp_rows():
        w("| %s | %s | %s | %s | %s | %d/%d | %d | %d | %.2f | %s |" %
          (r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], "是" if r[5] == 0 else "否"))
    w()
    w("### C. 第 2 步：检出（每组参数 55 次飞行，全部完成）")
    w()
    w("| fc | tau | E | T | R | 检出数 | 延迟中位 [s] | 延迟最大 [s] |")
    w("|---|---|---|---|---|---|---|---|")
    for r in det_rows():
        w("| %s | %s | %s | %s | %s | %d/%d | %s | %s |" % (r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]))


def s4_block(prefix, title, w):
    import s4table
    g = s4table.groups(prefix)
    base = [s4table.load("v7_s4_clean_r%d" % i) for i in range(1, 11)]
    base = [d for d in base if d]
    bmax = s4table.med([d.get("truth_max_all") for d in base]) or 0
    w(title)
    w()
    w("| 组 | 运行数 | 完成数 | 触发 | 真值最高 [m] | 真值最低 [m] | 相对基线 | 注入→动作 [s] | 动作→落地 [s] | 着地速度 [m/s] | 落点偏差 [m] | 任务完成 | EKF 跳变 [m] | 是否与预期一致 |")
    w("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for name in sorted(g):
        ds = g[name]
        trig = sum(1 for d in ds if d.get("mon_trigger_rel") is not None)
        mc = sum(1 for d in ds if d.get("mission_complete"))
        dev = s4table.med([(d.get("truth_max_all") or 0) - bmax for d in ds if d.get("truth_max_all") is not None])
        w("| %s | %d | %d | %d/%d | %s | %s | %s | %s | %s | %s | %s | %d/%d | %s | %s |" % (
            name, len(ds), sum(1 for d in ds if not d.get("error")), trig, len(ds),
            s4table.medstr([d.get("truth_max_all") for d in ds]),
            s4table.medstr([d.get("truth_min_all") for d in ds]),
            ("%+.3f" % dev) if dev is not None else "-",
            s4table.medstr([d.get("mon_action_delay") for d in ds]),
            s4table.medstr([d.get("mon_action_to_touchdown") for d in ds]),
            s4table.medstr([d.get("touchdown_mps") for d in ds]),
            s4table.medstr([d.get("land_offset") for d in ds]),
            mc, len(ds), s4table.medstr([d.get("mon_ekf_jump") for d in ds]), verdict(name, ds, trig)))
    w()


def verdict(name, ds, trig):
    """按任务书的预期判定：与预期一致 / 部分一致 / 不一致。"""
    import s4table
    act = name.split("_")[-1]
    tmax = s4table.med([d.get("truth_max_all") for d in ds])
    tdm = s4table.med([d.get("touchdown_mps") for d in ds])
    hang = s4table.med([d.get("land_hang_s") for d in ds])
    desc = s4table.med([d.get("land_desc_rate_max") for d in ds])
    if name == "clean":
        return "是（无误动作）" if trig == 0 else "否（无误动作）"
    if act == "none":
        if tmax is None:
            return "-"
        return "是（对照更高）" if tmax >= 3.5 else "否（对照不够高）"
    if act in ("A1", "A2", "A3"):
        if tmax is None:
            return "-"
        ok_h = tmax <= 3.0
        ok_l = (tdm is None or tdm <= 0.7) and (hang is None or hang <= 6) and (desc is None or desc >= -1.0)
        if ok_h and ok_l:
            return "是"
        return "部分（高度达标，降落不理想）" if ok_h else "否（真值最高 > 3 m）"
    return "-"


def main3(w):
    w("### D. 第 3 步：参数选择")
    w()
    sel = json.load(open(os.path.join(D, "step3_selected.json")))
    w("第 1 步**没有零误报的组**，按任务书改用“按每小时误报数升序”取前三：")
    w()
    w("| 名次 | fc | tau | E | T | R | 误报 | 次/小时 | 检出 | 延迟中位 [s] |")
    w("|---|---|---|---|---|---|---|---|---|---|")
    for i, r in enumerate(sel, 1):
        p = json.loads(r["params"])
        w("| %d | %s | %s | %s | %s | %s | %s/%s | %s | %s/%s | %s |" % (
            i, p["fc"], p["tau"], p["E"], p["T"], p["R"], r["fp_n"], r["fp_runs"],
            r["fp_per_h"], r["det_n"], r["det_runs"], r["delay_med"]))
    w()
    w("第 4 步用第 1 名：`fc=0.5, tau=30.0, E=0.7, T=1.0, R=0.6`。")
    w()
    w("### E. 第 4 步：闭环（监视器真的动作）")
    w()
    s4_block("v7_s4", "#### E-1 `SQ --alt 2.0 --side 2.0 --laps 1`", w)
    s4_block("v7_s4_d3", "#### E-2 `D3`（LOITER 手动起飞）", w)
    w("### F. 第 5 步：监视器自身失效")
    w()
    s4_block("v7_s5", "#### F-1 `gN` 异常格 + 三种自身失效（动作 A1）", w)


if __name__ == "__main__":
    main()
    main3(w)
    OUT.close()
    print("total_table.md written")
