#!/usr/bin/env python3
"""V6 job files. 8 configs = {FINAL gate, orig gate} x RNGFND1_MIN {0.01,0.10,0.20,0.30}.
Normal flight: D3 / SQ / HOV, 5 reps, yaw 30/150/270/90/210.
Anomalies (SQ): a (val4 a cell), c (val4 c cell), g3/g5/g10/gN (collapse to 0.1 m), h (collapse at 0.5 m, never recovers).
Fault times are on the opt2_sim fault clock (= AUTO + T + ~0.95 s)."""
import os

D = "/home/jacky/sim/val6"
REST = "FENCE_ALT_MAX=2.5,FENCE_RADIUS=4,FENCE_MARGIN=0.3,SIM_SONAR_RND=0.02,SIM_WIND_T=1"
GATE_F = "EK3_RNG_M_NSE=0.1,EK3_HGT_I_GATE=200,SURFTRAK_MODE=0"
GATE_O = "EK3_RNG_M_NSE=0.5,EK3_HGT_I_GATE=500,SURFTRAK_MODE=1"
GROUPS = {}
for pre, gate in (("f", GATE_F), ("o", GATE_O)):
    for mn in (0.01, 0.10, 0.20, 0.30):
        GROUPS[f"{pre}{int(round(mn * 100)):02d}"] = f"{gate},{REST},RNGFND1_MIN={mn}"
GORDER = ["f01", "f10", "f20", "f30", "o01", "o10", "o20", "o30"]
YAW5 = {1: 30, 2: 150, 3: 270, 4: 90, 5: 210}
YAW3 = {1: 30, 2: 150, 3: 270}
D3 = "D3 --alt 2.0 --side 2.0 --hold 2.0 --thr-auto 1500 --settle 20 --seconds 150"
SQ = "SQ --alt 2.0 --side 2.0 --laps 1 --hold 2.0 --no-loiter-test --thr-auto 1500 --settle 20 --seconds 120"
HOV = "HOV --alt 0.5 --hold 60 --no-loiter-test --thr-auto 1500 --settle 20 --seconds 150"
PARM = "mtf01_sitl_rng.parm"

# h cell: offsets chosen from the f01 baseline truth so the reading stays ~0.1 m while climbing.
HFAULTS = "".join(f" --fault {t}:SIM_SONAR_POS_Z={o}" for t, o in
                  [(2.5, 0.53), (3.0, 0.81), (3.5, 1.04), (4.0, 1.29), (4.5, 1.53), (5.0, 1.77), (5.5, 1.93)])
FAULTS = {
    "a": f" --fault 3.5:SIM_SONAR_POS_Z=1.5 --fault 4.5:SIM_SONAR_POS_Z=-0.05",
    "c": f" --fault 6.0:SIM_SONAR_POS_Z=1.5 --fault 9.0:SIM_SONAR_POS_Z=-0.05",
    "g3": f" --fault 6.0:SIM_SONAR_POS_Z=1.9 --fault 9.0:SIM_SONAR_POS_Z=-0.05",
    "g5": f" --fault 6.0:SIM_SONAR_POS_Z=1.9 --fault 11.0:SIM_SONAR_POS_Z=-0.05",
    "g10": f" --fault 6.0:SIM_SONAR_POS_Z=1.9 --fault 16.0:SIM_SONAR_POS_Z=-0.05",
    "gN": f" --fault 6.0:SIM_SONAR_POS_Z=1.9",
    "h": HFAULTS,
}

normal, gcell, hcell, accell = [], [], [], []
for g in GORDER:
    ovl = GROUPS[g]
    for r, yaw in YAW5.items():
        normal.append(f"v6_d3_{g}_r{r}|{PARM}|10|{yaw}|{ovl}|{D3}")
        if g == "f01":  # baseline runs (no fault) used for dev_base
            normal.append(f"v6_sq_r{r}|{PARM}|10|{yaw}|{ovl}|{SQ}")
        else:
            normal.append(f"v6_sq_{g}_r{r}|{PARM}|10|{yaw}|{ovl}|{SQ}")
        normal.append(f"v6_hov_{g}_r{r}|{PARM}|10|{yaw}|{ovl}|{HOV}")
for g in GORDER:
    for key, target in (("g3", gcell), ("g5", gcell), ("g10", gcell), ("gN", gcell), ("h", hcell),
                        ("a", accell), ("c", accell)):
        for r, yaw in YAW3.items():
            target.append(f"v6_{g}_{key}_r{r}|{PARM}|10|{yaw}|{GROUPS[g]}|{SQ}{FAULTS[key]}")
for name, rows in (("normal", normal), ("g", gcell), ("h", hcell), ("ac", accell)):
    with open(os.path.join(D, f"jobs_v6_{name}.txt"), "w") as fh:
        fh.write("\n".join(rows) + "\n")
    print(name, len(rows))
print("groups", GORDER)
