#!/usr/bin/env python3
"""V5 job list: ArUco marker position-fix failures during a square mission.

Job line: tag|parm|speedup|home yaw|overlay|abs_sim args
Recommended config copied from abs_fix (rec_en_SQ_sq4_s05_l100_*).
"""
import sys

P = "mtf01_sitl_rng.parm"
SPD = 4
YAWS = [30, 150, 270, 90, 210]
WDIR = [0, 120, 240, 60, 180]
BASE = ("SIM_SONAR_RND=0.02,FENCE_RADIUS=4,SIM_WIND_T=1,EK3_RNG_M_NSE=0.1,EK3_HGT_I_GATE=200,SURFTRAK_MODE=0,"
        "SIM_FLOW_RND=0.15,SIM_WIND_SPD=1.5,SIM_WIND_TURB=1.0,SIM_WIND_DIR={wd}")
EN = ("VISO_TYPE=1,EK3_SRC1_POSXY=6,EK3_SRC1_YAW=6,VISO_QUAL_MIN=1,VISO_POS_M_NSE=0.1,VISO_YAW_M_NSE=0.05,"
      "VISO_DELAY_MS=100,EK3_GLITCH_RAD=5")
NONE = "VISO_TYPE=0"
SQ = "SQ --alt 2.0 --side 2.0 --laps 3 --hold 0 --settle 20 --seconds 300"

# fault cell -> (args template with {t}, t for sq4, t for sq1)
# times are relative to the AUTO start; from the mission timeline of rec_en_SQ_sq4_s05_l100_r1:
# waypoint reached at rel 12.5 (2,0) 17.7 (2,2) 22.9 (0,2) 28.2 (0,0) 33.4 38.8 44.0 49.3 54.6 59.8 65.0 71.1 s
# sq4: a marker 1 m beyond every corner -> marker in view during the whole square flight
# sq1: single marker 1 m beyond the take-off corner -> in view only on the last leg of each lap
FAULTS = [
    ("bad1m_lap1", "--badfix {t}:3:1:0", 15.0, 25.0),
    ("bad1m_lap2", "--badfix {t}:3:1:0", 37.5, 48.0),
    ("bad03_10s", "--badfix {t}:10:0.3:0", 30.0, 44.0),
    ("yaw20_5s", "--bad-yaw {t}:5:20", 44.0, 46.0),
    ("block30", "--block {t}:30", 33.0, 33.0),
    ("stop10", "--comp-stop {t}:10", 50.0, 45.0),
    ("stop_norec", "--comp-stop {t}", 45.0, 45.0),
    ("lat500_20s", "--lat-switch {t}:20:0.5", 40.0, 44.0),
]


def job(tag, meth, layout, r, fault="", t=0.0, nrep=5):
    """one job line; r = 0..4 -> yaw/wind/seed of that repetition"""
    ovl = BASE.format(wd=WDIR[r])
    if meth == "extnav":
        ovl += "," + EN
    else:
        ovl += "," + NONE
    args = SQ
    if meth == "extnav":
        args += " --method extnav --lat 0.1 --sigma 0.05 --claim 0.1 --gate2"
        args += " --markers %s" % layout
    else:
        args += " --method none --lat 0.1"
    args += " --seed %d" % (r + 1)
    if fault:
        args += " " + fault.format(t=t)
    return f"{tag}_r{r+1}|{P}|{SPD}|{YAWS[r]}|{ovl}|{args}"


def main(which):
    out = []
    if which == "p1":  # reproduce the two baselines from the previous round, 3 runs each
        for r in range(3):
            out.append(job("base_none", "none", "none", r))
            out.append(job("base_sq4", "extnav", "sq4", r))
    elif which == "p2":  # controls: no fix / 4 markers / 1 marker, 5 runs each
        for r in (3, 4):
            out.append(job("base_none", "none", "none", r))
            out.append(job("base_sq4", "extnav", "sq4", r))
        for r in range(5):
            out.append(job("base_sq1", "extnav", "sq1", r))
    elif which in ("p3", "p4"):
        layout = "sq4" if which == "p3" else "sq1"
        for name, tmpl, t4, t1 in FAULTS:
            t = t4 if layout == "sq4" else t1
            for r in range(5):
                out.append(job(f"{layout}_{name}", "extnav", layout, r, tmpl, t))
    print("\n".join(out))


if __name__ == "__main__":
    main(sys.argv[1])
