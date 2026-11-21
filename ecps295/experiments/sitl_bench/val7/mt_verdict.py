

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
