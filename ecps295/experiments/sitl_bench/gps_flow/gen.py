#!/usr/bin/env python3
"""Build per-run parm files (base_rng.parm + config overlay + per-repeat disturbance) and a job list.
usage: gen.py <jobs.txt> <config> [<config> ...]     (reps 0..2 each)
job line: tag|parm|speedup|home|gf_sim args
Per-repeat disturbance r (so repeats differ, not only timing jitter):
  start heading 10/130/250 deg, start point shifted r*2.2 m north, wind 1.5/3.0/2.0 m/s from 45/200/300 deg,
  SIM_WIND_TURB 0.5, FENCE_RADIUS 4 (mtf01 recommendation),, SIM_WIND_T 1 (no height profile), GPS error walk seed r+1.
"""
import os
import sys

D = "/home/jacky/sim/gps_flow"
BASE = open(D + "/base_rng.parm").read() + "# --- gps_flow: FENCE_RADIUS 4 as recommended in mtf01 (start point near cage centre)\nFENCE_RADIUS 4\n"
MIS = "D3 --alt 2 --side 2 --hold 8 --no-loiter-test --settle 25"
IND = MIS  # indoor: companion sets origin + VISION as before (VISO_TYPE 1, ignored for height by EK3_SRC1_POSZ 2)
OUT = MIS + " --no-origin --no-vis"  # outdoor GPS: no companion origin / vision

MAG = {"COMPASS_ENABLE": 1, "COMPASS_USE": 1, "EK3_SRC1_YAW": 1, "EK3_SRC2_YAW": 1, "EK3_SRC3_YAW": 1}
GPS = {"GPS1_TYPE": 1, "SIM_GPS1_ENABLE": 1, "SIM_GPS1_TYPE": 1, "VISO_TYPE": 0}
# architecture (i): set1 GPS pos/vel + compass + rangefinder height, set2 = set3 = flow + compass + rangefinder
ARCH_I = dict(MAG, **GPS, **{"EK3_SRC1_POSXY": 3, "EK3_SRC1_VELXY": 3, "EK3_SRC1_VELZ": 0, "EK3_SRC1_POSZ": 2,
                             "EK3_SRC2_POSXY": 0, "EK3_SRC2_VELXY": 5, "EK3_SRC2_VELZ": 0, "EK3_SRC2_POSZ": 2,
                             "EK3_SRC3_POSXY": 0, "EK3_SRC3_VELXY": 5, "EK3_SRC3_VELZ": 0, "EK3_SRC3_POSZ": 2,
                             "EK3_SRC_OPTIONS": 0, "RC8_OPTION": 90})
# architecture (ii): one set, GPS position + flow velocity (+ compass, rangefinder); set2/3 flow only
ARCH_II = dict(ARCH_I, **{"EK3_SRC1_VELXY": 5})
GOODGPS = {"SIM_GPS1_ACC": 0.5}
POORGPS = {"SIM_GPS1_ACC": 2.0, "SIM_GPS1_NUMSATS": 6, "SIM_GPS1_VERR_X": 0.3, "SIM_GPS1_VERR_Y": 0.3}
GW_GOOD, GW_POOR = " --gps-walk 0.5:20", " --gps-walk 1.5:20"
DEC = 0.197293  # COMPASS_DEC on the real FC (auto-declination at the field = SITL home)

C = {
    # Q2: flow + rangefinder, no GPS (indoor), with / without compass yaw, compass errors
    "F0": ({}, IND),
    "FM": (MAG, IND),
    "FMdec10": (dict(MAG, EK3_MAG_EF_LIM=0, COMPASS_AUTODEC=0, COMPASS_DEC=DEC + 0.174533), IND),
    "FMdec20": (dict(MAG, EK3_MAG_EF_LIM=0, COMPASS_AUTODEC=0, COMPASS_DEC=DEC + 0.349066), IND),
    "FMmot2": (dict(MAG, SIM_MAG_MOT_Y=2), IND),
    "FMmot6": (dict(MAG, SIM_MAG_MOT_Y=6), IND),
    # architectures outdoors
    "IG_good": (dict(ARCH_I, **GOODGPS), OUT + GW_GOOD),
    "IG_poor": (dict(ARCH_I, **POORGPS), OUT + GW_POOR),
    "IF_poor": (dict(ARCH_I, **POORGPS), OUT + GW_POOR + " --ch8 1500"),
    "IFfa_poor": (dict(ARCH_I, **POORGPS, EK3_SRC_OPTIONS=1), OUT + GW_POOR + " --ch8 1500"),
    "II_poor": (dict(ARCH_II, **POORGPS), OUT + GW_POOR),
    "IGfa_poor": (dict(ARCH_I, **POORGPS, EK3_SRC_OPTIONS=1), OUT + GW_POOR),
    "GSF_good": (dict(ARCH_I, **GOODGPS, COMPASS_ENABLE=0, EK3_SRC1_YAW=8, EK3_SRC2_YAW=0, EK3_SRC3_YAW=0), OUT + GW_GOOD),
    # Q3 GPS faults in GPS mode, during AUTO (t=15 s after AUTO = WP2 hold)
    "IG_step3": (dict(ARCH_I, **GOODGPS), OUT + GW_GOOD + " --gps-step 15:3,0:10"),
    "IG_loss": (dict(ARCH_I, **GOODGPS), OUT + GW_GOOD + " --fault 15:SIM_GPS1_ENABLE=0"),
    "IGfa_loss": (dict(ARCH_I, **GOODGPS, EK3_SRC_OPTIONS=1), OUT + GW_GOOD + " --fault 15:SIM_GPS1_ENABLE=0"),
    "II_loss": (dict(ARCH_II, **GOODGPS), OUT + GW_GOOD + " --fault 15:SIM_GPS1_ENABLE=0"),
    # Q4 switching in AUTO (arch i, good GPS); offset between GPS and flow-integrated position via GPS step before the switch
    "SW_F2G": (dict(ARCH_I, **GOODGPS), OUT + GW_GOOD + " --ch8 1500 --switch 15:1000"),
    "SW_F2G_d1": (dict(ARCH_I, **GOODGPS), OUT + GW_GOOD + " --ch8 1500 --gps-step 5:0.7,0.7 --switch 15:1000"),
    "SW_F2G_d3": (dict(ARCH_I, **GOODGPS), OUT + GW_GOOD + " --ch8 1500 --gps-step 5:2.1,2.1 --switch 15:1000"),
    "SW_G2F": (dict(ARCH_I, **GOODGPS), OUT + GW_GOOD + " --switch 15:1500"),
    "SW_hover": (dict(ARCH_I, **GOODGPS), OUT.replace("--no-loiter-test", "--no-auto --hover-alt 2") + GW_GOOD + " --ch8 1500 --hover-time 25 --hover-switch 6:1000 --hover-switch 16:1500"),
    "IG_step3_gr25": (dict(ARCH_I, **GOODGPS, EK3_GLITCH_RAD=25), OUT + GW_GOOD + " --gps-step 15:3,0:10"),
    "IG_step3_pg300": (dict(ARCH_I, **GOODGPS, EK3_POS_I_GATE=300), OUT + GW_GOOD + " --gps-step 15:3,0:10"),
    # companion keeps sending SET_GPS_GLOBAL_ORIGIN (500 m away from the true field) + VISION height, GPS fitted
    "IG_comp500": (dict(ARCH_I, **GOODGPS, VISO_TYPE=1), MIS + " --origin-ofs 500" + GW_GOOD),
    "IG_comp0": (dict(ARCH_I, **GOODGPS, VISO_TYPE=1), MIS + GW_GOOD),
    # Q5 indoor with the outdoor parameter set (arch i, GPS fitted but no fix), switch on flow
    "Q5_in_noorg": (dict(ARCH_I, SIM_GPS1_ENABLE=0), MIS + " --no-origin --no-vis --ch8 1500"),
    "Q5_in_org": (dict(ARCH_I, SIM_GPS1_ENABLE=0), MIS + " --no-vis --ch8 1500"),
}
HDG, WSPD, WDIR = [10, 130, 250], [1.5, 3.0, 2.0], [45, 200, 300]

out = open(sys.argv[1], "w")
os.makedirs(D + "/parms", exist_ok=True)
for name in sys.argv[2:]:
    ov, args = C[name]
    for r in range(3):
        tag = "%s_r%d" % (name, r)
        dist = {"SIM_WIND_SPD": WSPD[r], "SIM_WIND_DIR": WDIR[r], "SIM_WIND_TURB": 0.5, "SIM_WIND_T": 1}
        txt = BASE + "\n# --- gps_flow overlay %s\n" % name + "".join("%s %s\n" % kv for kv in ov.items())
        txt += "# --- disturbance r%d\n" % r + "".join("%s %s\n" % kv for kv in dist.items())
        pf = "%s/parms/%s.parm" % (D, tag)
        open(pf, "w").write(txt)
        home = "%.6f,-117.842000,20,%d" % (33.643 + r * 2e-5, HDG[r])
        out.write("%s|%s|5|%s|%s --seed %d\n" % (tag, pf, home, args, r + 1))
