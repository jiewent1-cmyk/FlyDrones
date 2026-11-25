#!/usr/bin/env python3
"""Round 6 (2026-11-22): the guard as ported to the FlyDrones-mtf01 worktree (companion_p = its height_guard.py and
baro_range_monitor.py, unchanged), closed loop. PORT = its defaults + synth; PORT_T05 = the same with hold time 0.5 s."""
import json, os
D = os.path.dirname(os.path.abspath(__file__))
G = {"__file__": D + "/gen_r5.py"}
exec(open(D + "/gen_r5.py").read().split("CELLS = [")[0], G)
HARSH, SYNP, FAULTS, YAWS, SQ = (G[k] for k in ("HARSH", "SYNP", "FAULTS", "YAWS", "SQ"))
ARMS = {"PORT": {"_dir": "companion_p", "synth": True}, "PORT_T05": {"_dir": "companion_p", "synth": True, "t_s": 0.5}}
F6 = ["none", "wind3", "b_dr12p", "b_dr12n", "b_st2p", "b_st2n", "b_noise", "f_scale", "f_noise",
      "m_blip", "m_oo03", "m_oo10", "m_ooN", "r_gN", "r_h", "r_k", "r_a", "r_c", "r_i", "m_dr005", "m_dr020", "x_gN_noise"]
jobs = []
for rep in range(1, 11):
    for fault in F6:
        for arm, hg in ARMS.items():
            fo, fa = FAULTS[fault]
            yaw = YAWS[rep - 1]
            wdir = (yaw * 7 + 40 * rep) % 360
            tag = f"ab_{arm}__{fault}__r{rep:02d}"
            o = ",".join(x for x in (HARSH, SYNP, fo, f"SIM_WIND_DIR={wdir}") if x)
            jobs.append("|".join([tag, "mtf01_sitl_rng.parm", "1", str(yaw), o, (SQ + " " + fa).strip()]))
            json.dump(hg, open(f"{D}/hg/{tag}.json", "w"))
open(D + "/jobs_r6.txt", "w").write("\n".join(jobs) + "\n")
pick = ["ab_PORT__none__r01", "ab_PORT__r_gN__r01", "ab_PORT__m_blip__r01", "ab_PORT__m_dr005__r01", "ab_PORT__m_ooN__r01", "ab_PORT_T05__r_k__r01"]
open(D + "/jobs_r6_smoke.txt", "w").write("\n".join(j for j in jobs if j.split("|")[0] in pick) + "\n")
print(len(jobs), "jobs")
