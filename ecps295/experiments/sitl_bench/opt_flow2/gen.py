#!/usr/bin/env python3
"""Generate job files. Rep r uses home yaw YAWS[r] and wind direction WDIR[r] (to make SITL repeats differ)."""
import sys
P = "mtf01_sitl_rng.parm"
SPD = 10
BASE = "SIM_SONAR_RND=0.02,FENCE_RADIUS=4,SIM_WIND_T=1"
YAWS, WDIR = [30, 150, 270, 60, 200, 330], [0, 120, 240, 60, 180, 300]  # reps 4-6 only for the extra repeats
WIND = "SIM_WIND_SPD=1.5,SIM_WIND_TURB=1.0,SIM_WIND_DIR={wd}"
PERT = {
    "C0": "", "HDG": "", "WIND": WIND, "FNSE": "SIM_FLOW_RND=0.15",
    "FSCP": "FLOW_FXSCALER=50,FLOW_FYSCALER=50", "FSCM": "FLOW_FXSCALER=-50,FLOW_FYSCALER=-50",
    "RNSE": "SIM_SONAR_RND=0.05",
    "ALL": WIND + ",SIM_FLOW_RND=0.15,FLOW_FXSCALER=50,FLOW_FYSCALER=50,SIM_SONAR_RND=0.05",
}
SCEN = {"H1": "HOV --alt 1.0 --hold 120", "H2": "HOV --alt 2.0 --hold 120",
        "SQ": "SQ --alt 2.0 --side 2.0 --laps 3 --hold 0"}
COMMON = " --no-loiter-test --thr-auto 1500 --settle 20 --seconds 260"


def job(tag, pert, scen, r, extra="", fault=""):
    yaw = 0 if pert == "C0" else YAWS[r]
    ovl = ",".join(x for x in (BASE, PERT[pert].format(wd=WDIR[r]), extra) if x)
    return f"{tag}_r{r+1}|{P}|{SPD}|{yaw}|{ovl}|{SCEN[scen]}{COMMON}{fault}"


if __name__ == "__main__":
    which = sys.argv[1]
    out = []
    if which == "main":
        for scen in ("H2", "SQ", "H1"):
            for pert in PERT:
                if scen == "H1" and pert not in ("C0", "WIND", "ALL"):
                    continue
                for r in range(3):
                    out.append(job(f"m_{scen}_{pert}", pert, scen, r))
    if which == "fault":
        SCEN["HF"] = "HOV --alt 2.0 --hold 60"
        FT = {"DN": 1.5, "UP": -1.5, "OOR": -10}  # SIM_SONAR_POS_Z: +1.5 = reading 1.5 m smaller, -1.5 = larger, -10 = ~12 m > RNGFND1_MAX
        CFG = {"def": "", "p1": "EK3_RNG_M_NSE=0.1,EK3_RNG_I_GATE=200,EK3_HGT_I_GATE=200"}
        for cfg in sys.argv[2:]:
            for ft, v in FT.items():
                for dur in (1, 3, 5):
                    for r in range(3):
                        out.append(job(f"f_{cfg}_{ft}{dur}", "HDG", "HF", r, CFG[cfg],
                                       f" --fault 25:SIM_SONAR_POS_Z={v} --fault {25 + dur}:SIM_SONAR_POS_Z=-0.05"))
    if which == "sweep":
        SW = [("H2", "FNSE", "fmn01", "EK3_FLOW_M_NSE=0.1"), ("H2", "FNSE", "fmn05", "EK3_FLOW_M_NSE=0.5"),
              ("H2", "FNSE", "fig150", "EK3_FLOW_I_GATE=150"), ("H2", "FNSE", "fig500", "EK3_FLOW_I_GATE=500"),
              ("H2", "FNSE", "fdl0", "EK3_FLOW_DELAY=0"), ("H2", "FNSE", "fdl50", "EK3_FLOW_DELAY=50"),
              ("SQ", "WIND", "fposU", "SIM_FLOW_POS_X=0.05,SIM_FLOW_POS_Z=0.03"),
              ("SQ", "WIND", "fposS", "SIM_FLOW_POS_X=0.05,SIM_FLOW_POS_Z=0.03,FLOW_POS_X=0.05,FLOW_POS_Z=0.03"),
              ("SQ", "ALL", "rmn01", "EK3_RNG_M_NSE=0.1"), ("SQ", "ALL", "rmn025", "EK3_RNG_M_NSE=0.25"),
              ("SQ", "ALL", "hig200", "EK3_HGT_I_GATE=200"), ("SQ", "ALL", "rig200", "EK3_RNG_I_GATE=200"),
              ("SQ", "ALL", "rposS", "RNGFND1_POS_Z=-0.05")]
        for scen, pert, name, extra in SW:
            for r in range(3):
                out.append(job(f"s_{scen}_{pert}_{name}", pert, scen, r, extra))
    if which == "loiter":
        B = BASE
        CFG = {"def": "", "p1": "EK3_RNG_M_NSE=0.1,EK3_RNG_I_GATE=200,EK3_HGT_I_GATE=200",
               "p1st0": "EK3_RNG_M_NSE=0.1,EK3_RNG_I_GATE=200,EK3_HGT_I_GATE=200,SURFTRAK_MODE=0"}
        for cfg, extra in CFG.items():
            for r in range(3):
                ovl = ",".join(x for x in (B, extra) if x)
                out.append(f"l_{cfg}_UP3_r{r+1}|{P}|{SPD}|{YAWS[r]}|{ovl}|HOV --no-auto --hover-alt 1.6 --loiter-hold 40 --loiter-fault"
                           f" --fault 10:SIM_SONAR_POS_Z=-1.5 --fault 13:SIM_SONAR_POS_Z=-0.05 --settle 20")
    if which == "fault2":
        SCEN["HF"] = "HOV --alt 2.0 --hold 60"
        CFG = {"def": "", "p1": "EK3_RNG_M_NSE=0.1,EK3_RNG_I_GATE=200,EK3_HGT_I_GATE=200",
               "p1b": "EK3_RNG_M_NSE=0.1,EK3_HGT_I_GATE=200"}
        sets = [("def", "DN", 1.5, 10), ("def", "UP", -1.5, 10), ("p1", "DN", 1.5, 10), ("p1", "UP", -1.5, 10),
                ("p1b", "DN", 1.5, 3), ("p1b", "UP", -1.5, 3)]
        for cfg, ft, v, dur in sets:
            for r in range(3):
                out.append(job(f"f_{cfg}_{ft}{dur}", "HDG", "HF", r, CFG[cfg],
                               f" --fault 25:SIM_SONAR_POS_Z={v} --fault {25 + dur}:SIM_SONAR_POS_Z=-0.05"))
    if which == "more":
        for tag, scen, pert, extra in [("m_H2_FNSE", "H2", "FNSE", ""), ("s_H2_FNSE_fmn01", "H2", "FNSE", "EK3_FLOW_M_NSE=0.1"),
                                       ("s_H2_FNSE_fdl0", "H2", "FNSE", "EK3_FLOW_DELAY=0")]:
            for r in range(3, 6):
                out.append(job(tag, pert, scen, r, extra))
    if which == "ab":
        REC = {"recH": "EK3_RNG_M_NSE=0.1,EK3_HGT_I_GATE=200,SURFTRAK_MODE=0",
               "recHF": "EK3_RNG_M_NSE=0.1,EK3_HGT_I_GATE=200,SURFTRAK_MODE=0,EK3_FLOW_M_NSE=0.1"}
        for scen in ("H2", "SQ", "H1"):
            reps = range(6) if scen != "H1" else range(3)
            for r in reps:
                if r >= 3:
                    out.append(job(f"m_{scen}_ALL", "ALL", scen, r))
                for name, extra in REC.items():
                    out.append(job(f"ab_{scen}_ALL_{name}", "ALL", scen, r, extra))
    print("\n".join(out))
