#!/usr/bin/env python3
"""gen7.py -- V7 任务文件生成。

任务文件每行: tag|parm|speedup|yaw|overlay|args
"""
import os

D = "/home/jacky/sim/val7"
PARM = "mtf01_sitl_rng.parm"
FINAL = ("EK3_RNG_M_NSE=0.1,EK3_HGT_I_GATE=200,SURFTRAK_MODE=0,FENCE_ALT_MAX=2.5,"
         "FENCE_RADIUS=4,FENCE_MARGIN=0.3,SIM_SONAR_RND=0.02,SIM_WIND_T=1")
HARSH = ("SIM_WIND_SPD=1.5,SIM_WIND_TURB=1.0,SIM_FLOW_RND=0.15,FLOW_FXSCALER=50,"
         "FLOW_FYSCALER=50,SIM_SONAR_RND=0.05")
BASE = FINAL + "," + HARSH + ",EK3_SRC2_VELXY=5"      # V7 的基础覆盖（FINAL + 苛刻扰动 + EK3_SRC2_VELXY=5）
YAW = [30, 150, 270, 90, 210]
WDIR = [0, 120, 240, 60, 180]

HOV = "HOV --alt 2.0 --hold 180 --no-loiter-test --thr-auto 1500 --settle 20 --seconds 260"
SQ3 = "SQ --alt 2.0 --side 2.0 --laps 3 --hold 2.0 --no-loiter-test --thr-auto 1500 --settle 20 --seconds 220"
SQ1 = "SQ --alt 2.0 --side 2.0 --laps 1 --hold 2.0 --no-loiter-test --thr-auto 1500 --settle 20 --seconds 120"
D3 = "D3 --alt 2.0 --side 2.0 --hold 2.0 --thr-auto 1500 --settle 20 --seconds 180"

MON = "--mon"

# ---------------------------------------------------------------- 第 1 步：误报
# 气压条件：名称 -> (额外覆盖, 额外 fault)
S1 = {
    "base":    ("", ""),
    "noise25": ("SIM_BARO_RND=0.5", ""),                      # 噪声 x2.5 (0.2 -> 0.5 m)
    "dr03p":   ("SIM_BARO_DRIFT=0.005", ""),                   # 慢漂 +0.3 m/min
    "dr03n":   ("SIM_BARO_DRIFT=-0.005", ""),                  # 慢漂 -0.3 m/min
    "dr12p":   ("SIM_BARO_DRIFT=0.02", ""),                    # 慢漂 +1.2 m/min
    "dr12n":   ("SIM_BARO_DRIFT=-0.02", ""),                   # 慢漂 -1.2 m/min
    "st2p":    ("", " --fault 3.5:SIM_BARO_GLITCH=0.5 --fault 5.5:SIM_BARO_GLITCH=0"),   # 起飞后 +0.5 m 持续 2 s
    "st2n":    ("", " --fault 3.5:SIM_BARO_GLITCH=-0.5 --fault 5.5:SIM_BARO_GLITCH=0"),  # 起飞后 -0.5 m 持续 2 s
    "st1p":    ("", " --fault 10:SIM_BARO_GLITCH=0.5 --fault 11:SIM_BARO_GLITCH=0"),     # 飞行中 +0.5 m 持续 1 s
    "st1n":    ("", " --fault 10:SIM_BARO_GLITCH=-0.5 --fault 11:SIM_BARO_GLITCH=0"),    # 飞行中 -0.5 m 持续 1 s
    "wind3":   ("SIM_WIND_SPD=3,SIM_WIND_TURB=1.5", ""),        # 风 3 m/s 加强湍流
}


def gen_s1():
    rows = []
    for cond, (extra, faults) in S1.items():
        for mn, margs in (("hov", HOV), ("sq", SQ3)):
            for i in range(1, 11):
                yaw = YAW[(i - 1) % 5]
                ovl = BASE + ("," + extra if extra else "") + ",SIM_WIND_DIR=%d" % WDIR[(i - 1) % 5]
                tag = "v7_s1_%s_%s_r%d" % (cond, mn, i)
                rows.append("%s|%s|10|%d|%s|%s %s%s" % (tag, PARM, yaw, ovl, margs, MON, faults))
    return rows


# ---------------------------------------------------------------- 第 2 步：检出
HFAULTS = "".join(" --fault %s:SIM_SONAR_POS_Z=%s" % (t, o) for t, o in
                  [(2.5, 0.53), (3.0, 0.81), (3.5, 1.04), (4.0, 1.29), (4.5, 1.53), (5.0, 1.77), (5.5, 1.93)])
# i: 读数缓慢变小 0.1 m/s 持续 10 s（台阶式偏移，每秒 +0.1 m）
IFAULTS = "".join(" --fault %s:SIM_SONAR_POS_Z=%s" % (6 + k, round(0.05 + 0.1 * k, 2)) for k in range(11))
S2 = {
    "a":  " --fault 3.5:SIM_SONAR_POS_Z=1.5 --fault 4.5:SIM_SONAR_POS_Z=-0.05",
    "c":  " --fault 6.0:SIM_SONAR_POS_Z=1.5 --fault 9.0:SIM_SONAR_POS_Z=-0.05",
    "g3": " --fault 6.0:SIM_SONAR_POS_Z=1.9 --fault 9.0:SIM_SONAR_POS_Z=-0.05",
    "g5": " --fault 6.0:SIM_SONAR_POS_Z=1.9 --fault 11.0:SIM_SONAR_POS_Z=-0.05",
    "g10": " --fault 6.0:SIM_SONAR_POS_Z=1.9 --fault 16.0:SIM_SONAR_POS_Z=-0.05",
    "gN": " --fault 6.0:SIM_SONAR_POS_Z=1.9",
    "h":  HFAULTS,
    "i":  IFAULTS,
    "j1": " --fault 6.0:SIM_SONAR_POS_Z=-1.55 --fault 11.0:SIM_SONAR_POS_Z=-0.05",
    "j2": " --fault 6.0:SIM_SONAR_POS_Z=-1.55",
}


def k_faults():
    """k 格：读数钉在 0.2 m（跟随爬升的 SIM_SONAR_POS_Z 台阶），由 mk_k_fault.py 生成。"""
    p = os.path.join(D, "k_faults.txt")
    return open(p).read().strip() if os.path.exists(p) else ""


def gen_s2():
    rows = []
    cells = dict(S2)
    cells["k"] = " " + k_faults() if k_faults() else ""
    HOV60 = "HOV --alt 2.0 --hold 60 --no-loiter-test --thr-auto 1500 --settle 20 --seconds 120"
    for cell, faults in cells.items():
        margs = HOV60 if cell == "i" else SQ1     # i 格定义为"2 m 悬停时读数缓慢变小"
        for i, yaw in enumerate(YAW, 1):
            ovl = BASE + ",SIM_WIND_DIR=%d" % WDIR[(i - 1) % 5]
            tag = "v7_s2_%s_r%d" % (cell, i)
            rows.append("%s|%s|10|%d|%s|%s %s%s" % (tag, PARM, yaw, ovl, margs, MON, faults))
    return rows


# ---------------------------------------------------------------- 第 4 步：闭环
def gen_s4(params, cells=None, actions=("A1", "A2", "A3")):
    """params: 第 3 步选出的监视器参数 dict。
    顺序按任务书：先 g-不恢复(gN)、h、k、无异常，再做其余。"""
    cells = cells or list(S2.keys()) + ["k"]
    p = ("--mon-fc %s --mon-tau %s --mon-E %s --mon-T %s --mon-R %s" %
         (params["fc"], params["tau"], params["E"], params["T"], params["R"]))
    rows = []

    def cell_rows(cell, prefix, mission):
        faults = S2.get(cell, "")
        if cell == "k":
            faults = " " + k_faults() if k_faults() else ""
        out = []
        for act in actions:
            for i, yaw in enumerate(YAW, 1):
                ovl = BASE + ",SIM_WIND_DIR=%d" % WDIR[(i - 1) % 5]
                tag = "%s_%s_%s_r%d" % (prefix, cell, act, i)
                out.append("%s|%s|10|%d|%s|%s %s --mon-action %s %s%s" %
                           (tag, PARM, yaw, ovl, mission, MON, act, p, (" " + faults if faults else "")))
        for i, yaw in enumerate(YAW, 1):   # 对照：无监视器
            ovl = BASE + ",SIM_WIND_DIR=%d" % WDIR[(i - 1) % 5]
            tag = "%s_%s_none_r%d" % (prefix, cell, i)
            out.append("%s|%s|10|%d|%s|%s%s" % (tag, PARM, yaw, ovl, mission, faults))
        return out

    # 第一批：g-不恢复、h、k
    for cell in ("gN", "h", "k"):
        rows += cell_rows(cell, "v7_s4", SQ1)
    # 无异常闭环 10 次
    for i in range(1, 11):
        yaw = YAW[(i - 1) % 5]
        ovl = BASE + ",SIM_WIND_DIR=%d" % WDIR[(i - 1) % 5]
        tag = "v7_s4_clean_r%d" % i
        rows.append("%s|%s|10|%d|%s|%s %s --mon-action A1%s" % (tag, PARM, yaw, ovl, SQ1, MON, p))
    # D3（LOITER 手动起飞）：gN / h / k
    for cell in ("gN", "h", "k"):
        rows += cell_rows(cell, "v7_s4_d3", D3)
    # 其余异常格
    for cell in cells:
        if cell in ("gN", "h", "k"):
            continue
        rows += cell_rows(cell, "v7_s4", SQ1)
    return rows


# ---------------------------------------------------------------- 第 5 步：监视器自身失效
def gen_s5(params):
    """在 gN 异常格上加监视器自身的三种失效，各 3 次。"""
    rows = []
    p = ("--mon-fc %s --mon-tau %s --mon-E %s --mon-T %s --mon-R %s" %
         (params["fc"], params["tau"], params["E"], params["T"], params["R"]))
    modes = {
        "stop": "--mon-stop-at 40",            # 监视器进程中途停掉（FC 秒）
        "press": "--mon-press-stop-at 40",     # 气压消息流中断（FC 秒）
        "p0": "--mon-p0-bias 0.3",             # p0 取错（解锁前飞机被拿起来 0.3 m）
    }
    for name, extra in modes.items():
        for i, yaw in enumerate(YAW[:3], 1):
            ovl = BASE + ",SIM_WIND_DIR=%d" % WDIR[(i - 1) % 5]
            tag = "v7_s5_%s_r%d" % (name, i)
            rows.append("%s|%s|10|%d|%s|%s %s --mon-action A1 %s %s%s" %
                        (tag, PARM, yaw, ovl, SQ1, MON, p, extra, S2["gN"]))
    return rows


if __name__ == "__main__":
    import json
    import sys
    which = sys.argv[1] if len(sys.argv) > 1 else "s1"
    if which == "s1":
        rows = gen_s1()
        out = "jobs_v7_s1.txt"
    elif which == "s2":
        rows = gen_s2()
        out = "jobs_v7_s2.txt"
    elif which == "s4":
        params = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {"fc": 1.0, "tau": 30.0, "E": 0.5, "T": 1.0, "R": 0.6}
        rows = gen_s4(params)
        out = "jobs_v7_s4.txt"
    elif which == "s5":
        params = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {"fc": 1.0, "tau": 30.0, "E": 0.5, "T": 1.0, "R": 0.6}
        rows = gen_s5(params)
        out = "jobs_v7_s5.txt"
    else:
        raise SystemExit("unknown job set")
    with open(os.path.join(D, out), "w") as fh:
        fh.write("\n".join(rows) + "\n")
    print(out, len(rows))
