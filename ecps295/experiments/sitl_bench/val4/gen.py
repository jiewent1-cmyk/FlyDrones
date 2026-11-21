#!/usr/bin/env python3
"""V2 fault-injection job list. Phases in order cl(a~1m), a2(just 2m), cn(first corner), md(mid leg), ds(descent ~1m).
Fault times (sim s, --fault base = fly_report start ~= AUTO switch + ~1 s): cl 3.5, a2 6.0, cn 16.0, md 29.5, ds 45.0.
DN15=SIM_SONAR_POS_Z 1.5 (read smaller), UP15=-1.5 (read bigger), OOR=-10 (out of range), DN05=0.5 (climb+descent only)."""
PH = {"cl": 3.5, "a2": 6.0, "cn": 16.0, "md": 29.5, "ds": 45.0}
TY = {"dn15": "1.5", "up15": "-1.5", "oor": "-10", "dn05": "0.5"}
GROUPS = {"F": "EK3_RNG_M_NSE=0.1,EK3_HGT_I_GATE=200,SURFTRAK_MODE=0,FENCE_ALT_MAX=2.5,FENCE_RADIUS=4,FENCE_MARGIN=0.3,SIM_SONAR_RND=0.02,SIM_WIND_T=1",
          "D": "FENCE_ALT_MAX=2.5,FENCE_RADIUS=4,FENCE_MARGIN=0.3,SIM_SONAR_RND=0.02,SIM_WIND_T=1"}
YAW = [30, 150, 270]
lines = []
for ph in ["cl", "a2", "cn", "md", "ds"]:
    tys = ["dn05", "dn15", "up15", "oor"] if ph in ("cl", "ds") else ["dn15", "up15", "oor"]
    for ty in tys:
        for dur in [1, 3]:
            for g in ["F", "D"]:
                for r, yaw in enumerate(YAW, 1):
                    t = PH[ph]
                    args = (f"SQ --alt 2.0 --side 2.0 --laps 1 --hold 2.0 --no-loiter-test --thr-auto 1500 --settle 20 "
                            f"--seconds 120 --fault {t}:SIM_SONAR_POS_Z={TY[ty]} --fault {t + dur}:SIM_SONAR_POS_Z=-0.05")
                    lines.append(f"v2_{ph}_{g}_{ty}_{dur}_r{r}|mtf01_sitl_rng.parm|10|{yaw}|{GROUPS[g]}|{args}")
open("/home/jacky/sim/val2/jobs_v2.txt", "w").write("\n".join(lines) + "\n")
print(len(lines), "jobs")
