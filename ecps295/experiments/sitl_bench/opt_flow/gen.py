#!/usr/bin/env python3
"""gen.py - generate parm variants and job lists for the opt_flow study.

base = copy of /home/jacky/sim/mtf01/mtf01_sitl_rng.parm (MTF-01 + EK3_SRC1_POSZ 2) with
       FENCE_RADIUS 3 -> 4 (the value recommended in mtf01_recommended.parm, so the A/B arms
       differ only in the parameters under test).
"""
import os

D = "/home/jacky/sim/opt_flow"
SRC = "/home/jacky/sim/mtf01/mtf01_sitl_rng.parm"
os.makedirs(D + "/parm", exist_ok=True)

base = {}
for line in open(SRC):
    line = line.strip()
    if not line or line.startswith("#"):
        continue
    k, v = line.split()[0], line.split()[1]
    base[k] = v
base["FENCE_RADIUS"] = "4"

# perturbations (truth side and FC side), used to compose the harsher scenarios
WIND = {"SIM_WIND_SPD": "2.0", "SIM_WIND_DIR": "90", "SIM_WIND_TURB": "10"}
WIND45 = {"SIM_WIND_SPD": "1.5", "SIM_WIND_DIR": "45", "SIM_WIND_TURB": "10"}
HARSH = dict(WIND45)
HARSH.update({"FLOW_FXSCALER": "50", "FLOW_FYSCALER": "50", "SIM_FLOW_RND": "0.10", "SIM_SONAR_RND": "0.05"})

V = {
    "base": {},
    # --- sensor imperfections (truth side = SIM_*) ---
    "flownoise": {"SIM_FLOW_RND": "0.20"},
    "rngnoise": {"SIM_SONAR_RND": "0.05"},
    "flowscale_p5": {"FLOW_FXSCALER": "50", "FLOW_FYSCALER": "50"},
    "flowscale_m5": {"FLOW_FXSCALER": "-50", "FLOW_FYSCALER": "-50"},
    "wind": WIND,
    "ofs": {"SIM_SONAR_POS_Z": "-0.10"},                     # sensor 10 cm below the FC, FC does not know
    "ofs_fix": {"SIM_SONAR_POS_Z": "-0.10", "RNGFND1_POS_Z": "-0.10"},
    "flowpos": {"SIM_FLOW_POS_X": "0.05"},                   # flow sensor 5 cm ahead, FC does not know
    "flowpos_fix": {"SIM_FLOW_POS_X": "0.05", "FLOW_POS_X": "0.05"},
    # --- EKF flow side ---
    "flownse": {"EK3_FLOW_M_NSE": "0.1"},
    "flowgate": {"EK3_FLOW_I_GATE": "200"},
    "flowdelay": {"EK3_FLOW_DELAY": "50"},
    "terrgrad": {"EK3_TERR_GRAD": "0.05"},
    "rngspd": {"EK3_RNG_USE_SPD": "1.0"},
    "accnse": {"EK3_ACC_P_NSE": "0.2"},
    # --- EKF height side ---
    "rngnse": {"EK3_RNG_M_NSE": "0.1"},
    "rnggate": {"EK3_RNG_I_GATE": "200"},
    "altnse": {"EK3_ALT_M_NSE": "1.0"},
    "hgtgate": {"EK3_HGT_I_GATE": "200"},
    "hgtdelay": {"EK3_HGT_DELAY": "100"},
    # --- protection combinations ---
    "prot": {"EK3_RNG_M_NSE": "0.1", "EK3_RNG_I_GATE": "200", "EK3_HGT_I_GATE": "200"},
    "prot_max4": {"EK3_RNG_M_NSE": "0.1", "EK3_RNG_I_GATE": "200", "EK3_HGT_I_GATE": "200", "RNGFND1_MAX": "4"},
    "prot_fence": {"EK3_RNG_M_NSE": "0.1", "EK3_RNG_I_GATE": "200", "EK3_HGT_I_GATE": "200",
                   "FS_EKF_ACTION": "3"},
    "rngmax4": {"RNGFND1_MAX": "4"},
    # --- composed harsh scenarios ---
    "harsh": HARSH,
    "ofs_wind": dict(WIND, **{"SIM_SONAR_POS_Z": "-0.10"}),
    "ofs_fix_wind": dict(WIND, **{"SIM_SONAR_POS_Z": "-0.10", "RNGFND1_POS_Z": "-0.10"}),
    "flowpos_wind": dict(WIND, **{"SIM_FLOW_POS_X": "0.05"}),
    "flowpos_fix_wind": dict(WIND, **{"SIM_FLOW_POS_X": "0.05", "FLOW_POS_X": "0.05"}),
}
for k, extra in (("flownse", "EK3_FLOW_M_NSE 0.1"), ("flowgate", "EK3_FLOW_I_GATE 200"),
                 ("flowdelay", "EK3_FLOW_DELAY 50"), ("terrgrad", "EK3_TERR_GRAD 0.05"),
                 ("rngspd", "EK3_RNG_USE_SPD 1.0"), ("accnse", "EK3_ACC_P_NSE 0.2"),
                 ("rngnse", "EK3_RNG_M_NSE 0.1"), ("rnggate", "EK3_RNG_I_GATE 200"),
                 ("altnse", "EK3_ALT_M_NSE 1.0"), ("hgtgate", "EK3_HGT_I_GATE 200"),
                 ("hgtdelay", "EK3_HGT_DELAY 100")):
    kk, vv = extra.split()
    V["harsh_" + k] = dict(HARSH, **{kk: vv})
V["harsh_ofs"] = dict(HARSH, **{"SIM_SONAR_POS_Z": "-0.10"})
V["harsh_ofs_fix"] = dict(HARSH, **{"SIM_SONAR_POS_Z": "-0.10", "RNGFND1_POS_Z": "-0.10"})


V["flownoise_mnse05"] = dict({"SIM_FLOW_RND": "0.20"}, **{"EK3_FLOW_M_NSE": "0.5"})
V["flownoise_mnse10"] = dict({"SIM_FLOW_RND": "0.20"}, **{"EK3_FLOW_M_NSE": "1.0"})
V["flownoise_mnse01"] = dict({"SIM_FLOW_RND": "0.20"}, **{"EK3_FLOW_M_NSE": "0.1"})
V["flownoise_mnse05_wind"] = dict({"SIM_FLOW_RND": "0.20"}, **WIND, **{"EK3_FLOW_M_NSE": "0.5"})
V["flownoise_wind"] = dict({"SIM_FLOW_RND": "0.20"}, **WIND)


V["recA"] = dict(HARSH, **{"EK3_FLOW_M_NSE": "0.1", "EK3_RNG_M_NSE": "0.1", "EK3_RNG_I_GATE": "200",
                           "EK3_HGT_I_GATE": "200"})
V["recB"] = dict(HARSH, **{"EK3_RNG_M_NSE": "0.1", "EK3_RNG_I_GATE": "200", "EK3_HGT_I_GATE": "200"})


V["mis_clean"] = {"SIM_SONAR_RND": "0.05"}
V["mis_windonly"] = dict({"SIM_SONAR_RND": "0.05"}, **WIND45)
V["mis_scaleonly"] = dict({"SIM_SONAR_RND": "0.05"}, **{"FLOW_FXSCALER": "50", "FLOW_FYSCALER": "50"})
V["mis_harsh_f4"] = dict(HARSH, **{"FENCE_ALT_MAX": "4", "FENCE_MARGIN": "0.3"})

for name, ov in V.items():
    p = dict(base)
    p.update(ov)
    with open("%s/parm/%s.parm" % (D, name), "w") as f:
        f.write("# variant %s : %s\n" % (name, ", ".join("%s=%s" % kv for kv in sorted(ov.items()))))
        for k in sorted(p):
            f.write("%s %s\n" % (k, p[k]))
print("wrote %d parm files" % len(V))
