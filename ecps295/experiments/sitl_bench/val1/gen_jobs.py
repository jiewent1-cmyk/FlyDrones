#!/usr/bin/env python3
"""val1 job generator: A (FINAL), B (FINAL+harsh), C (unlock matrix), D (failsafes), E (fence margin)."""
import os

FINAL = ("EK3_RNG_M_NSE=0.1,EK3_HGT_I_GATE=200,SURFTRAK_MODE=0,FENCE_ALT_MAX=2.5,"
         "FENCE_RADIUS=4,FENCE_MARGIN=0.3,SIM_SONAR_RND=0.02,SIM_WIND_T=1")
HARSH = ("SIM_WIND_SPD=1.5,SIM_WIND_TURB=1.0,SIM_FLOW_RND=0.15,FLOW_FXSCALER=50,"
         "FLOW_FYSCALER=50,SIM_SONAR_RND=0.05")
YAW = [30, 150, 270, 90, 210]
WDIR = [0, 120, 240, 60, 180]
P = "mtf01_sitl_rng.parm"
S = 10

D = "/home/jacky/sim/val1"
lines = {}


def add(group, tag, yaw, ovl, args):
    lines.setdefault(group, []).append("|".join([tag, P, str(S), str(yaw), ovl, args]))


# ---------------- A : FINAL, no disturbance ----------------
for i, y in enumerate(YAW, 1):
    add("A", "a_d3_r%d" % i, y, FINAL, "D3 --alt 2.0 --side 2.0 --settle 25 --seconds 200")
    add("A", "a_hov2_r%d" % i, y, FINAL, "HOV --alt 2.0 --hold 120 --no-loiter-test --settle 25 --seconds 260")
    add("A", "a_sq_r%d" % i, y, FINAL, "SQ --alt 2.0 --side 2.0 --laps 3 --no-loiter-test --settle 25 --seconds 260")
    add("A", "a_hov1_r%d" % i, y, FINAL, "HOV --alt 1.0 --hold 120 --no-loiter-test --settle 25 --seconds 260")

# ---------------- B : FINAL + harsh disturbance ----------------
for i, y in enumerate(YAW, 1):
    ovl = FINAL + "," + HARSH + ",SIM_WIND_DIR=%d" % WDIR[i - 1]
    add("B", "b_d3_r%d" % i, y, ovl, "D3 --alt 2.0 --side 2.0 --settle 25 --seconds 200")
    add("B", "b_hov2_r%d" % i, y, ovl, "HOV --alt 2.0 --hold 120 --no-loiter-test --settle 25 --seconds 260")
    add("B", "b_sq_r%d" % i, y, ovl, "SQ --alt 2.0 --side 2.0 --laps 3 --no-loiter-test --settle 25 --seconds 260")
    add("B", "b_hov1_r%d" % i, y, ovl, "HOV --alt 1.0 --hold 120 --no-loiter-test --settle 25 --seconds 260")

# ---------------- C : unlock matrix (task = A1) ----------------
C1 = "D3 --alt 2.0 --side 2.0 --settle 25 --seconds 200"
for i, y in enumerate([30, 150, 270], 1):
    add("C", "c1_viso0_novis_r%d" % i, y, FINAL + ",VISO_TYPE=0", C1 + " --no-vis")
    add("C", "c2_viso1_novis_r%d" % i, y, FINAL + ",VISO_TYPE=1", C1 + " --no-vis")
    add("C", "c3_viso1_vis_r%d" % i, y, FINAL + ",VISO_TYPE=1", C1)
    add("C", "c4_viso0_noorigin_r%d" % i, y, FINAL + ",VISO_TYPE=0", C1 + " --no-origin")

# ---------------- D : failsafe protection ----------------
HOV2 = "HOV --alt 2.0 --hold 120 --no-loiter-test --settle 25 --seconds 260"
for i, y in enumerate(YAW, 1):
    add("D", "d1_hb_r%d" % i, y, FINAL, HOV2 + " --vis-cut 40 --hb-cut")
    add("D", "d2_rcfail_r%d" % i, y, FINAL, HOV2 + " --fault 40:SIM_RC_FAIL=1")
    add("D", "d3_batt_r%d" % i, y, FINAL + ",BATT_FS_LOW_ACT=1,BATT_LOW_VOLT=11.0",
        HOV2 + " --fault 40:SIM_BATT_VOLTAGE=10.5")
    add("D", "d4_son_dn_r%d" % i, y, FINAL,
        HOV2 + " --fault 40:SIM_SONAR_POS_Z=1.5 --fault 43:SIM_SONAR_POS_Z=-0.05")
    add("D", "d5_son_up_r%d" % i, y, FINAL,
        HOV2 + " --fault 40:SIM_SONAR_POS_Z=-1.5 --fault 43:SIM_SONAR_POS_Z=-0.05")

# ---------------- E : fence margin ----------------
SQ3 = "SQ --alt 2.0 --side 2.0 --laps 3 --no-loiter-test --settle 25 --seconds 260"
for i, y in enumerate(YAW, 1):
    ovl = FINAL + "," + HARSH + ",SIM_WIND_DIR=%d" % WDIR[i - 1]
    add("E", "e1_fence_r%d" % i, y, ovl, SQ3)
    add("E", "e2_r35_r%d" % i, y, ovl + ",FENCE_RADIUS=3.5", SQ3)
    add("E", "e3_alt23_r%d" % i, y, ovl + ",FENCE_ALT_MAX=2.3", SQ3)

for g, ls in lines.items():
    with open(os.path.join(D, "jobs_%s.txt" % g), "w") as fh:
        fh.write("\n".join(ls) + "\n")
    print(g, len(ls))
