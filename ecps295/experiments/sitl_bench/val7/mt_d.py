

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
