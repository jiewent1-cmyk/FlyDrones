#!/usr/bin/env python3
"""GPS hover study, job generator. usage: gen.py <round>   -> jobs_<round>.txt, parms/<tag>.parm
tag = <arm>__<flow>_<gps>__r<rep>. Same rep = same heading, wind direction and GPS error realisation (paired arms)."""
import os, sys
D = os.path.dirname(os.path.abspath(__file__))
BASE = open("/home/jacky/sim/gps_flow/base_rng.parm").read()
COMMON = {"FENCE_RADIUS": 15, "FENCE_ALT_MAX": 4, "VISO_TYPE": 0, "GPS1_TYPE": 1, "SIM_GPS1_ENABLE": 1, "SIM_GPS1_TYPE": 1,
          "SIM_WIND_SPD": 1.5, "SIM_WIND_TURB": 1, "SIM_WIND_T": 1, "SIM_FLOW_POS_X": 0, "LOG_DISARMED": 0}
MAG = {"COMPASS_ENABLE": 1, "COMPASS_USE": 1, "EK3_SRC1_YAW": 1, "EK3_SRC2_YAW": 1, "EK3_SRC3_YAW": 1}
FLOWSRC = {"EK3_SRC1_POSXY": 0, "EK3_SRC1_VELXY": 5, "EK3_SRC_OPTIONS": 0}
ARMS = {  # overlay, gh_sim args, depends on the GPS condition
    "F0": (dict(FLOWSRC, COMPASS_ENABLE=0), "", False),                      # flow, no compass (today's cage set-up)
    "F": (dict(MAG, **FLOWSRC), "", False),                                  # flow + compass
    "G": (dict(MAG, EK3_SRC1_POSXY=3, EK3_SRC1_VELXY=3, EK3_SRC_OPTIONS=0), "", True),   # GPS position + velocity
    "GF": (dict(MAG, EK3_SRC1_POSXY=3, EK3_SRC1_VELXY=3, EK3_SRC2_VELXY=5, EK3_SRC_OPTIONS=1), "", True),  # + flow velocity
    "FV": (dict(MAG, EK3_SRC1_POSXY=0, EK3_SRC1_VELXY=5, EK3_SRC2_VELXY=3, EK3_SRC_OPTIONS=1), "", True),  # flow + GPS velocity
}
for n in (0.5, 1, 2, 4):                                                     # A: flow velocity + GPS position, weak anchor
    ARMS["A%02d" % (n * 10)] = (dict(MAG, EK3_SRC1_POSXY=3, EK3_SRC1_VELXY=5, EK3_SRC_OPTIONS=0, EK3_POSNE_M_NSE=n), "", True)
for tau in (20, 60, 120):                                                    # B: flow EKF + companion drift fix
    ARMS["B%03d" % tau] = (dict(MAG, **FLOWSRC), "--fix %d" % tau, True)
for k, tau in ((1.0, 30), (0.5, 30), (1.0, 120)):                          # D: drift fix with a dead band of k x reported accuracy
    ARMS["D%02d_%03d" % (k * 10, tau)] = (dict(MAG, **FLOWSRC), "--fix %d --fix-dead-k %g" % (tau, k), True)
for k, tau in ((1.5, 30), (2.0, 30), (1.0, 60), (1.5, 60)):               # round 3: wider dead band / other time constant
    ARMS["D%02d_%03d" % (k * 10, tau)] = (dict(MAG, **FLOWSRC), "--fix %d --fix-dead-k %g" % (tau, k), True)
for k, tau in ((1.0, 30), (1.0, 120), (1.5, 30), (1.5, 60), (1.0, 60)):    # J: dead band + GPS jump detector (1.5 m)
    ARMS["J%02d_%03d" % (k * 10, tau)] = (dict(MAG, **FLOWSRC), "--fix %d --fix-dead-k %g --fix-jump 1.5" % (tau, k), True)
for y in (2, 6):                                                             # compass disturbed by motor current
    ARMS["FM%d" % y] = (dict(MAG, **FLOWSRC, SIM_MAG_MOT_Y=y), "", False)
FLOW = {"N05": 0.05, "N15": 0.15, "N25": 0.25}
GPSQ = {  # slow walk sigma:tau + fast sigma:tau, reported accuracy, velocity noise
    "Gg": ("0.3:300:0.08:5", {"SIM_GPS1_ACC": 0.6, "SIM_GPS1_NUMSATS": 16, "SIM_GPS1_VERR_X": 0.05, "SIM_GPS1_VERR_Y": 0.05}),
    "Gm": ("0.8:300:0.15:5", {"SIM_GPS1_ACC": 1.2, "SIM_GPS1_NUMSATS": 12, "SIM_GPS1_VERR_X": 0.10, "SIM_GPS1_VERR_Y": 0.10}),
    "Gp": ("1.5:300:0.30:5", {"SIM_GPS1_ACC": 2.0, "SIM_GPS1_NUMSATS": 8, "SIM_GPS1_VERR_X": 0.30, "SIM_GPS1_VERR_Y": 0.30}),
}
HDG = [10, 130, 250, 70, 190, 310, 40, 160, 280, 100]
jobs = []


def add(arm, fl, gq, rep, hover=300, extra="", pre="", ov2=None):
    ov, args, _ = ARMS[arm]
    walk, gov = GPSQ[gq]
    tag = "%s%s__%s_%s__r%02d" % (pre, arm, fl, gq, rep)
    p = dict(COMMON, **gov, **ov, SIM_FLOW_RND=FLOW[fl], SIM_WIND_DIR=(HDG[rep] * 7 + 40 * rep) % 360, **(ov2 or {}))
    pf = "%s/parms/%s.parm" % (D, tag)
    open(pf, "w").write(BASE + "\n# --- gps_hover %s\n" % tag + "".join("%s %s\n" % kv for kv in p.items()))
    home = "%.6f,-117.842000,20,%d" % (33.643, HDG[rep])
    jobs.append("|".join([tag, pf, "5", home, ("--hover %d --gps-walk %s --seed %d %s %s" % (hover, walk, rep + 1, args, extra)).strip()]))


os.makedirs(D + "/parms", exist_ok=True)
R = sys.argv[1]
if R == "smoke":
    for arm in ("F0", "F", "G", "GF", "FV", "A20", "B060"):
        add(arm, "N15", "Gm", 0, hover=60, pre="sm_")
elif R == "r1":
    for rep in range(5):
        for fl in FLOW:
            for arm, (_, _, dep) in ARMS.items():
                for gq in (GPSQ if dep else ["Gm"]):
                    add(arm, fl, gq, rep)
elif R == "r2":                                                              # 15 min hold, 10 repeats
    for rep in range(10):
        for fl in FLOW:
            for arm in ("F", "F0", "FM2", "FM6", "B120", "D10_030", "D05_030", "D10_120", "A20", "G"):
                for gq in (GPSQ if ARMS[arm][2] else ["Gm"]):
                    add(arm, fl, gq, rep, hover=900, pre="h15_")
elif R == "r3b":                                                             # jump detector: faults, then no-regression at 15 min
    FAULT = {"none": "", "step3": "--gps-step 120:3,0:0", "step3t": "--gps-step 120:3,0:60",
             "loss": "--fault 120:SIM_GPS1_ENABLE=0", "lossret": "--fault 120:SIM_GPS1_ENABLE=0 --fault 240:SIM_GPS1_ENABLE=1",
             "mag6": "--fault 120:SIM_MAG_MOT_Y=6"}
    for rep in range(8):
        for fn, fa in FAULT.items():
            for arm in ("J10_030", "J10_120", "J15_030", "J15_060"):
                add(arm, "N15", "Gm", rep, hover=400, extra=fa, pre="ft_%s_" % fn)
    for rep in range(10):
        for fl in FLOW:
            for arm in ("J10_030", "J10_120"):
                for gq in GPSQ:
                    add(arm, fl, gq, rep, hover=900, pre="h15_")
elif R == "r3c":                                                             # chosen setting (dead band 1 x accuracy, tau 60 s) + jump detector
    FAULT = {"none": "", "step3": "--gps-step 120:3,0:0", "step3t": "--gps-step 120:3,0:60",
             "loss": "--fault 120:SIM_GPS1_ENABLE=0", "lossret": "--fault 120:SIM_GPS1_ENABLE=0 --fault 240:SIM_GPS1_ENABLE=1",
             "mag6": "--fault 120:SIM_MAG_MOT_Y=6"}
    for rep in range(8):
        for fn, fa in FAULT.items():
            for arm in ("J10_060", "D10_060"):
                add(arm, "N15", "Gm", rep, hover=400, extra=fa, pre="ft_%s_" % fn)
    for rep in range(10):
        for fl in FLOW:
            for gq in GPSQ:
                add("J10_060", fl, gq, rep, hover=900, pre="h15_")
elif R == "r3":                                                              # faults first (short), then dead band tuning
    FAULT = {"none": "", "step3": "--gps-step 120:3,0:0", "step3t": "--gps-step 120:3,0:60",
             "loss": "--fault 120:SIM_GPS1_ENABLE=0", "lossret": "--fault 120:SIM_GPS1_ENABLE=0 --fault 240:SIM_GPS1_ENABLE=1",
             "mag6": "--fault 120:SIM_MAG_MOT_Y=6"}
    for rep in range(8):
        for fn, fa in FAULT.items():
            for arm in ("F", "G", "D10_030", "D10_120"):
                add(arm, "N15", "Gm", rep, hover=400, extra=fa, pre="ft_%s_" % fn)
    for rep in range(10):
        for fl in FLOW:
            for arm in ("D15_030", "D20_030", "D10_060", "D15_060"):
                for gq in GPSQ:
                    add(arm, fl, gq, rep, hover=900, pre="h15_")
open("%s/jobs_%s.txt" % (D, R), "w").write("\n".join(jobs) + "\n")
print(len(jobs), "jobs")
