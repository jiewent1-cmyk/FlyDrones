#!/usr/bin/env python3
"""Job files for batch.py: 'tag|parm|speedup|home yaw|overlay|abs_sim args'.
Rep r: home yaw YAWS[r], wind direction WDIR[r], observation RNG seed r+1 (SITL random() is unseeded -> same each run)."""
import sys

P = "mtf01_sitl_rng.parm"
SPD = 4
YAWS, WDIR = [30, 150, 270, 60, 200, 330], [0, 120, 240, 60, 180, 300]
# opt_flow2 base overlay + its recommended range-finder params + the harsh setting (flow noise 0.15 rad/s, wind 1.5 m/s + turb 1)
BASE = ("SIM_SONAR_RND=0.02,FENCE_RADIUS=4,SIM_WIND_T=1,EK3_RNG_M_NSE=0.1,EK3_HGT_I_GATE=200,SURFTRAK_MODE=0,"
        "SIM_FLOW_RND=0.15,SIM_WIND_SPD=1.5,SIM_WIND_TURB=1.0,SIM_WIND_DIR={wd}")
METH = {
    "none": "VISO_TYPE=0",
    "extpos": "VISO_TYPE=0",
    "extnav": "VISO_TYPE=1,EK3_SRC1_POSXY=6,EK3_SRC1_YAW=6,VISO_QUAL_MIN=1,VISO_POS_M_NSE=0.1,VISO_YAW_M_NSE=0.05,VISO_DELAY_MS={dl}",
}
SCEN = {"H2": "HOVW --alt 2.0 --hold 120 --markers hov --set WP_YAW_BEHAVIOR=0", "H2L": "HOV --alt 2.0 --hold 120 --markers hov",
        "H15": "HOVW --alt 1.5 --hold 120 --markers hov --set WP_YAW_BEHAVIOR=0", "H1": "HOVW --alt 1.0 --hold 120 --markers hov --set WP_YAW_BEHAVIOR=0",
        "SQ": "SQ --alt 2.0 --side 2.0 --laps 3 --hold 0", "SQ15": "SQ --alt 1.5 --side 2.0 --laps 3 --hold 0",
        "SQ1": "SQ --alt 1.0 --side 2.0 --laps 3 --hold 0"}
COMMON = " --settle 20 --seconds 300"


def job(tag, meth, scen, r, args="", extra="", lat=0.1):
    dl = min(250, int(round(lat * 1000)))
    ovl = ",".join(x for x in (BASE.format(wd=WDIR[r]), METH[meth].format(dl=dl), extra) if x)
    return f"{tag}_r{r+1}|{P}|{SPD}|{YAWS[r]}|{ovl}|{SCEN[scen]} --method {meth} --lat {lat} --seed {r+1} {args}{COMMON}"


def main(which):
    out = []
    R = range(3)
    if which == "smoke":
        out += [job("smk_base_H2", "none", "H2", 0)]
        out += [job("smk_en_H2_all", "extnav", "H2", 0, "--vis all")]
        out += [job("smk_ep_H2_p30", "extpos", "H2", 0, "--vis p30_5")]
        out += [job("smk_en_SQ_sq1", "extnav", "SQ", 0, "--markers sq1")]
    if which == "main":
        for r in R:
            out.append(job("b_H2", "none", "H2", r))
            out.append(job("b_SQ", "none", "SQ", r))
        for meth, mt in (("extnav", "en"), ("extpos", "ep")):
            for sg in (0.05, 0.15):
                for lat in (0.1, 0.3):
                    cfg = f"s{int(sg*100):02d}_l{int(lat*1000)}"
                    for r in R:
                        for vis in ("all", "first10", "p30_5"):
                            out.append(job(f"{mt}_H2_{vis}_{cfg}", meth, "H2", r, f"--vis {vis} --sigma {sg}", lat=lat))
                        for mk in ("sq1", "sq4"):
                            out.append(job(f"{mt}_SQ_{mk}_{cfg}", meth, "SQ", r, f"--markers {mk} --sigma {sg}", lat=lat))
    if which == "extra":
        CMP = "COMPASS_ENABLE=1,COMPASS_USE=1,EK3_SRC1_YAW=1"
        MOT = ",SIM_MAG_MOT_X=10,SIM_MAG_MOT_Y=5"
        GR5 = "EK3_GLITCH_RAD=5"
        FS10 = "FLOW_FXSCALER=100,FLOW_FYSCALER=100"
        BCN = CMP + ",BCN_TYPE=10,BCN_LATITUDE=33.6430,BCN_LONGITUDE=-117.8420,BCN_ALT=20,EK3_SRC1_POSXY=4"
        H = "--vis all --sigma 0.05"
        X = [  # (tag, method, scen, args, extra overlay, latency)
            # Q5: altitude only
            ("b_H15", "none", "H15", "", "", 0.1), ("b_H1", "none", "H1", "", "", 0.1),
            ("b_SQ15", "none", "SQ15", "", "", 0.1), ("b_SQ1", "none", "SQ1", "", "", 0.1),
            # Q3: field frame rotated 10 deg against the nose at boot (no compass)
            ("b_SQ_rot10", "none", "SQ", "--field-rot 10", "", 0.1),
            ("en_SQ_sq4_rot10", "extnav", "SQ", "--markers sq4 --field-rot 10", "", 0.1),
            ("en_SQ_sq1_rot10", "extnav", "SQ", "--markers sq1 --field-rot 10", "", 0.1),
            ("en_SQ_sq4_rot10_echo", "extnav", "SQ", "--markers sq4 --field-rot 10 --yaw-out echo", "", 0.1),
            ("ep_SQ_sq4_rot10", "extpos", "SQ", "--markers sq4 --field-rot 10", "", 0.1),
            ("ep_SQ_sq4_rot10_fcyaw", "extpos", "SQ", "--markers sq4 --field-rot 10 --yaw-mode fc", "", 0.1),
            # Q3: compass (EKF frame NED), with and without motor interference
            ("b_SQ_cmp", "none", "SQ", "--ekf-frame ned", CMP, 0.1),
            ("b_SQ_cmpmot", "none", "SQ", "--ekf-frame ned", CMP + MOT, 0.1),
            ("en_SQ_sq4_cmpmot", "extnav", "SQ", "--markers sq4 --ekf-frame ned", CMP + MOT, 0.1),
            ("en_SQ_sq4_cmpmotY6", "extnav", "SQ", "--markers sq4 --ekf-frame ned", CMP + MOT + ",EK3_SRC1_YAW=6", 0.1),
            ("ep_SQ_sq4_cmpmot", "extpos", "SQ", "--markers sq4 --ekf-frame ned", CMP + MOT, 0.1),
            # Q4: failures
            ("en_H2_bad3", "extnav", "H2", H + " --badfix 60:3:1:0", "", 0.1),
            ("en_H2_bad20", "extnav", "H2", H + " --badfix 60:20:1:0", "", 0.1),
            ("en_H2_bad20_gr5", "extnav", "H2", H + " --badfix 60:20:1:0", GR5, 0.1),
            ("ep_H2_bad3", "extpos", "H2", H + " --badfix 60:3:1:0", "", 0.1),
            ("ep_H2_bad20", "extpos", "H2", H + " --badfix 60:20:1:0", "", 0.1),
            ("en_H2_stop", "extnav", "H2", H + " --comp-stop 60", "", 0.1),
            ("ep_H2_stop", "extpos", "H2", H + " --comp-stop 60", "", 0.1),
            ("en_H2_nogate_o20", "extnav", "H2", H + " --no-gate --outlier-p 0.2", "", 0.1),
            ("en_H2_nogate_o20_gr5", "extnav", "H2", H + " --no-gate --outlier-p 0.2", GR5, 0.1),
            ("ep_H2_nogate_o20", "extpos", "H2", H + " --no-gate --outlier-p 0.2", "", 0.1),
            ("b_SQ_fs10", "none", "SQ", "", FS10, 0.1),
            ("en_SQ_sq1_fs10", "extnav", "SQ", "--markers sq1", FS10, 0.1),
            ("ep_SQ_sq1_fs10", "extpos", "SQ", "--markers sq1", FS10, 0.1),
            # misc
            ("en_H2L_p30", "extnav", "H2L", "--vis p30_5", "", 0.1),
            ("ep_H2L_p30", "extpos", "H2L", "--vis p30_5", "", 0.1),
            ("en_SQ_sq1_s15_l300_5hz", "extnav", "SQ", "--markers sq1 --sigma 0.15 --rate 5", "", 0.3),
            ("ep_SQ_sq1_s15_l300_5hz", "extpos", "SQ", "--markers sq1 --sigma 0.15 --rate 5", "", 0.3),
            ("en_SQ_sq1_s15_l300_rst", "extnav", "SQ", "--markers sq1 --sigma 0.15 --reset-gap 2", "", 0.3),
            ("en_H2_p30_5_s15_l300_rst", "extnav", "H2", "--vis p30_5 --sigma 0.15 --reset-gap 2", "", 0.3),
            ("en_SQ_sq1_s15_l300_dl40", "extnav", "SQ", "--markers sq1 --sigma 0.15", "VISO_DELAY_MS=40", 0.3),
            # (f) control: SITL UWB beacons (noise-free ranges, needs compass yaw)
            ("bcn_H2", "none", "H2", "--ekf-frame ned", BCN, 0.1),
            ("bcn_SQ", "none", "SQ", "--ekf-frame ned", BCN, 0.1),
        ]
        for r in R:
            for tag, meth, scen, args, extra, lat in X:
                out.append(job(tag, meth, scen, r, args, extra, lat))
    print("\n".join(out))


if __name__ == "__main__":
    main(sys.argv[1])
