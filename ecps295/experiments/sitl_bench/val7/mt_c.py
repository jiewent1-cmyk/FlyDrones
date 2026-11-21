

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
