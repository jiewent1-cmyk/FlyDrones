#!/usr/bin/env python3
"""V4 threshold-matrix job list. Order: tl baselines, fault a (repro cells orig/h200n01 first), a, c, b, d, e, f, HOV harsh.
Faults (sim s, --fault base = fly start ~= AUTO + ~1 s; restore value -0.05):
  a climb~1m  dn15 1s (3.5..4.5)   b climb~1m  dn05 3s (3.5..6.5)   c just-2m dn15 3s (6.0..9.0)
  d just-2m  up15 3s (6.0..9.0)   e corner   dn15 3s (16.0..19.0)  f desc~1m  up15 1s (45.0..46.0)
Groups: h{GATE}n{NSE*10} with SURFTRAK_MODE=0 (16) + orig (500/0.5/SURFTRAK_MODE=1)."""
REST = "FENCE_ALT_MAX=2.5,FENCE_RADIUS=4,FENCE_MARGIN=0.3,SIM_SONAR_RND=0.02,SIM_WIND_T=1"
GATES = [200, 300, 400, 500]
NSES = [0.1, 0.2, 0.3, 0.5]
GROUPS = {f"h{g}n{round(n * 10):02d}": f"EK3_RNG_M_NSE={n},EK3_HGT_I_GATE={g},SURFTRAK_MODE=0,{REST}"
          for g in GATES for n in NSES}
GROUPS["orig"] = f"EK3_HGT_I_GATE=500,EK3_RNG_M_NSE=0.5,SURFTRAK_MODE=1,{REST}"
HARSH = "SIM_WIND_SPD=1.5,SIM_WIND_TURB=1.0,SIM_FLOW_RND=0.15,FLOW_FXSCALER=50,FLOW_FYSCALER=50,SIM_SONAR_RND=0.05"
FL = {"a": ("1.5", 3.5, 4.5), "b": ("0.5", 3.5, 6.5), "c": ("1.5", 6.0, 9.0),
      "d": ("-1.5", 6.0, 9.0), "e": ("1.5", 16.0, 19.0), "f": ("-1.5", 45.0, 46.0)}
YAW = {1: 30, 2: 150, 3: 270}
WDIR = {1: 0, 2: 120, 3: 240}
SQ = "SQ --alt 2.0 --side 2.0 --laps 1 --hold 2.0 --no-loiter-test --thr-auto 1500 --settle 20 --seconds 120"
HOV = "HOV --alt 2.0 --hold 120 --no-loiter-test --thr-auto 1500 --settle 20 --seconds 260"
lines = []
for r, yaw in YAW.items():
    lines.append(f"v4_tl_r{r}|mtf01_sitl_rng.parm|10|{yaw}|{GROUPS['h200n01']}|{SQ}")
repro = ["orig", "h200n01"]
rest = [g for g in GROUPS if g not in repro]
blocks = [(["a"], repro), (["a"], rest), (["c"], list(GROUPS)), (["b", "d", "e", "f"], list(GROUPS))]
for keys, gs in blocks:
    for key in keys:
        val, t0, t1 = FL[key]
        args = f"{SQ} --fault {t0}:SIM_SONAR_POS_Z={val} --fault {t1}:SIM_SONAR_POS_Z=-0.05"
        for g in gs:
            for r, yaw in YAW.items():
                lines.append(f"v4_{g}_{key}_r{r}|mtf01_sitl_rng.parm|10|{yaw}|{GROUPS[g]}|{args}")
for g in GROUPS:
    for r, yaw in YAW.items():
        ovl = GROUPS[g] + "," + HARSH + f",SIM_WIND_DIR={WDIR[r]}"
        lines.append(f"v4h_{g}_hov_r{r}|mtf01_sitl_rng.parm|10|{yaw}|{ovl}|{HOV}")
open("/home/jacky/sim/val4/jobs_v4.txt", "w").write("\n".join(lines) + "\n")
print(len(lines), "jobs")
