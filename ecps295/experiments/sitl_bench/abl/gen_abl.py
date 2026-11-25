#!/usr/bin/env python3
"""Safety ablation matrix (2026-11-21). Writes jobs_abl.txt (tag|parm|speedup|yaw|overlay|args) and hg/<tag>.json
(the height-guard config for that run, or "off"). One protection layer is removed per arm; every arm meets the same
faults. Runs at speedup 1 because the real guard code (companion/height_guard.py, b3a6691) runs on wall time."""
import json
import os

D = os.path.dirname(os.path.abspath(__file__))
os.makedirs(D + "/hg", exist_ok=True)
SRC3 = "EK3_SRC3_POSXY=0,EK3_SRC3_VELXY=5,EK3_SRC3_POSZ=1,EK3_SRC3_VELZ=0,EK3_SRC3_YAW=0"
HARSH = ("FENCE_RADIUS=4,FENCE_MARGIN=0.3,FENCE_ALT_MAX=2.5,SIM_SONAR_RND=0.05,SIM_WIND_T=1,SIM_WIND_SPD=1.5,"
         "SIM_WIND_TURB=1.0,SIM_FLOW_RND=0.15,FLOW_FXSCALER=50,FLOW_FYSCALER=50," + SRC3)
TIGHT, STOCK = "EK3_RNG_M_NSE=0.1,EK3_HGT_I_GATE=200", "EK3_RNG_M_NSE=0.5,EK3_HGT_I_GATE=500"
ON = {}
# arm: (overlay, guard cfg or "off", tier)
ARMS = {
    "FULL":       (TIGHT + ",SURFTRAK_MODE=0", ON, 1),
    "NOGUARD":    (TIGHT + ",SURFTRAK_MODE=0", "off", 1),
    "BARE":       (STOCK + ",SURFTRAK_MODE=1", "off", 1),
    "STOCKGATE":  (STOCK + ",SURFTRAK_MODE=0", ON, 2),
    "SURF1":      (TIGHT + ",SURFTRAK_MODE=1", ON, 2),
    "FULL_NOALTF": (TIGHT + ",SURFTRAK_MODE=0,FENCE_TYPE=2", ON, 2),
    "BARE_NOALTF": (STOCK + ",SURFTRAK_MODE=1,FENCE_TYPE=2", "off", 2),
    "G_LANDONLY": (TIGHT + ",SURFTRAK_MODE=0", {"source_set": 1}, 2),
    "G_RATE":     (TIGHT + ",SURFTRAK_MODE=0", {"rate_m": 0.6}, 2),
    "G_E05":      (TIGHT + ",SURFTRAK_MODE=0", {"e_m": 0.5}, 2),
    "G_E10":      (TIGHT + ",SURFTRAK_MODE=0", {"e_m": 1.0}, 2),
    "BAROPRI":    (TIGHT + ",SURFTRAK_MODE=0,EK3_SRC1_POSZ=1", "off", 2),
}
Z = "SIM_SONAR_POS_Z="
H = " ".join(f"--fault {t}:{Z}{v}" for t, v in ((2.5, .53), (3.0, .81), (3.5, 1.04), (4.0, 1.29), (4.5, 1.53), (5.0, 1.77), (5.5, 1.93)))
K_SRC = open("~/sim/val7/jobs_v7_s4.txt").read().split("v7_s4_k_none_r1|")[1].split("\n")[0]
K = K_SRC[K_SRC.index("--fault"):].strip()
I = " ".join(f"--fault {6 + n}:{Z}{0.05 + 0.1 * n:.2f}" for n in range(11))
# fault: (extra overlay, extra args)
FAULTS = {
    "none":     ("", ""),
    "wind3":    ("SIM_WIND_SPD=3,SIM_WIND_TURB=1.5", ""),
    "r_a":      ("", f"--fault 3.5:{Z}1.5 --fault 4.5:{Z}-0.05"),        # climb, reads 1.5 m low for 1 s
    "r_c":      ("", f"--fault 6.0:{Z}1.5 --fault 9.0:{Z}-0.05"),        # at 2 m, 1.5 m low for 3 s
    "r_g5":     ("", f"--fault 6.0:{Z}1.9 --fault 11.0:{Z}-0.05"),       # at 2 m, collapses to ~0.1 m for 5 s
    "r_gN":     ("", f"--fault 6.0:{Z}1.9"),                             # ... and never recovers
    "r_h":      ("", H),                                                 # under-reads from the climb on, never recovers
    "r_k":      ("", K),                                                 # stuck near 0.2 m from lift-off
    "r_i":      ("", I),                                                 # slow drift, 0.1 m/s for 10 s
    "r_j2":     ("", f"--fault 6.0:{Z}-1.55"),                           # reads 1.55 m high, never recovers
    "r_oor3":   ("", f"--fault 6.0:{Z}-10 --fault 9.0:{Z}-0.05"),        # out of range for 3 s
    "b_dr12p":  ("SIM_BARO_DRIFT=0.02", ""),                             # baro drift +1.2 m/min
    "b_dr12n":  ("SIM_BARO_DRIFT=-0.02", ""),
    "b_st1p":   ("", "--fault 10:SIM_BARO_GLITCH=0.5 --fault 11:SIM_BARO_GLITCH=0"),     # baro step in cruise
    "b_st2p":   ("", "--fault 3.5:SIM_BARO_GLITCH=0.5 --fault 5.5:SIM_BARO_GLITCH=0"),   # baro step at take-off
    "b_st2n":   ("", "--fault 3.5:SIM_BARO_GLITCH=-0.5 --fault 5.5:SIM_BARO_GLITCH=0"),
    "b_noise":  ("SIM_BARO_RND=0.5", ""),
    "x_h_drp":  ("SIM_BARO_DRIFT=0.02", H),                              # rangefinder fault + baro drift either way
    "x_h_drn":  ("SIM_BARO_DRIFT=-0.02", H),
    "x_k_st2n": ("", K + " --fault 3.5:SIM_BARO_GLITCH=-0.5 --fault 5.5:SIM_BARO_GLITCH=0"),
    "x_gN_noise": ("SIM_BARO_RND=0.5", f"--fault 6.0:{Z}1.9"),
    "f_scale":  ("FLOW_FXSCALER=200,FLOW_FYSCALER=200", ""),             # flow scale 20 % off
    "f_noise":  ("SIM_FLOW_RND=0.3", ""),
}
SQ = "SQ --alt 2.0 --side 2.0 --laps 1 --hold 2.0 --no-loiter-test --thr-auto 1500 --settle 20 --seconds 120"
D3 = "D3 --alt 2.0 --side 2.0 --hold 2.0 --thr-auto 1500 --settle 20 --seconds 180"
YAWS = [(18 * i + 9) % 360 for i in range(20)]
import sys
R2 = len(sys.argv) > 1 and sys.argv[1] == "r2"
if R2:  # round 2: mitigations for the lateral run-away during the LAND started by the guard (set 2 = baro height, no flow)
    T0 = TIGHT + ",SURFTRAK_MODE=0"
    FAST = ",LAND_ALT_LOW_M=1,LAND_SPD_HIGH_MS=1.5"
    ARMS = {"X_NOFLOW": (T0, {"source_set": 2}, 3), "X_FAST": (T0 + FAST, ON, 3), "X_OOR": (T0, {"oor_s": 0.3}, 3),
            "X_OOR_NOFLOW": (T0, {"oor_s": 0.3, "source_set": 2}, 3),
            "X_ALL": (T0 + FAST, {"oor_s": 0.3, "source_set": 2}, 3), "X_NOFLOW_FAST": (T0 + FAST, {"source_set": 2}, 3)}
    FAULTS = {k: FAULTS[k] for k in ("none", "wind3", "r_a", "r_c", "r_g5", "r_gN", "r_h", "r_k", "r_i", "r_j2", "r_oor3",
                                     "x_gN_noise", "x_h_drn", "x_h_drp", "x_k_st2n")}
jobs = []
for rep in range(1, 21):
    for arm, (ovl, hg, tier) in ARMS.items():
        if rep > {1: 20, 2: 8, 3: 10}[tier]:
            continue
        for fault, (fo, fa) in FAULTS.items():
            yaw = YAWS[rep - 1]
            wdir = (yaw * 7 + 40 * rep) % 360
            tag = f"ab_{arm}__{fault}__r{rep:02d}"
            o = ",".join(x for x in (HARSH, ovl, fo, f"SIM_WIND_DIR={wdir}") if x)
            jobs.append((rep, "|".join([tag, "mtf01_sitl_rng.parm", "1", str(yaw), o, (SQ + " " + fa).strip()])))
            json.dump(hg, open(f"{D}/hg/{tag}.json", "w"))
    if rep <= 10 and not R2:  # manual LOITER take-off first (D3), key arms and faults only
        for arm in ("FULL", "NOGUARD", "BARE"):
            ovl, hg, _ = ARMS[arm]
            for fault in ("none", "r_h", "r_k", "r_gN", "b_st2n"):
                fo, fa = FAULTS[fault]
                yaw = YAWS[rep - 1]
                wdir = (yaw * 7 + 40 * rep) % 360
                tag = f"ab_{arm}__d3_{fault}__r{rep:02d}"
                o = ",".join(x for x in (HARSH, ovl, fo, f"SIM_WIND_DIR={wdir}") if x)
                jobs.append((rep, "|".join([tag, "mtf01_sitl_rng.parm", "1", str(yaw), o, (D3 + " " + fa).strip()])))
                json.dump(hg, open(f"{D}/hg/{tag}.json", "w"))
open(D + ("/jobs_r2.txt" if R2 else "/jobs_abl_regen.txt"), "w").write("\n".join(j for _, j in jobs) + "\n")   # rep 1 of every cell first: partial results stay balanced
print(len(jobs), "jobs;", len(ARMS), "arms;", len(FAULTS), "faults")
