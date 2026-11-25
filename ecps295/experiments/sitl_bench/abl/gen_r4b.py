#!/usr/bin/env python3
"""Round 4 (synthetic rangefinder after a trip; generated from gen_r3.py by r4_patch.py). Original header: Round 3 of the safety ablation (2026-11-21, evening). Writes jobs_r3.txt, jobs_r3_smoke.txt and hg/<tag>.json.

Blocks, in queue order (tags that already have results are skipped by batch.py):
  E1  X_OOR (range invalid for 0.3 s -> trip): no-false-trip check on every fault without a rangefinder fault, 20 reps,
      plus the rangefinder faults to 20 reps, plus out-of-range duration / blip faults.
  E3  dose-response: fault size (bias low / high, drift rate, collapse duration, out-of-range duration), FULL vs NOGUARD.
  E2  guard threshold e_m x hold time t_s, closed loop (detection faults and false-trip conditions).
  E5  the small arms of round 1 topped up from 8 to 20 reps.
The definitions of round 1 (HARSH, TIGHT, FAULTS, missions, yaw / wind per rep) are taken from gen_abl.py unchanged.
"""
import json
import os

D = os.path.dirname(os.path.abspath(__file__))
G = {"__file__": D + "/gen_abl.py"}
exec(open(D + "/gen_abl.py").read().split("import sys\nR2")[0], G)   # definitions only, writes nothing
HARSH, TIGHT, FAULTS, ARMS1, SQ, D3, YAWS, Z, ON = (G[k] for k in ("HARSH", "TIGHT", "FAULTS", "ARMS", "SQ", "D3", "YAWS", "Z", "ON"))
T0 = TIGHT + ",SURFTRAK_MODE=0"
OOR = {"oor_s": 0.3}
RNG = ["r_a", "r_c", "r_g5", "r_gN", "r_h", "r_k", "r_i", "r_j2", "r_oor3", "x_gN_noise", "x_h_drn", "x_h_drp", "x_k_st2n"]
NORNG = ["none", "wind3", "b_dr12p", "b_dr12n", "b_st1p", "b_st2p", "b_st2n", "b_noise", "f_scale", "f_noise"]


def f(*steps):
    return " ".join(f"--fault {t:g}:{Z}{v:g}" for t, v in steps)


def drift(rate, total=1.9, t0=6.0, tmax=40.0, dt=0.5):
    s, t, v = [], t0, 0.0
    while v < total - 1e-9 and t <= tmax:
        v = min(total, v + rate * dt)
        s.append((t, round(v, 3)))
        t += dt
    return f(*s)


DOSE = {}
for v in (0.2, 0.4, 0.6, 0.8, 1.0, 1.3, 1.6):                     # reads low by v from 6 s on (1.9 = r_gN)
    DOSE["m_lo%03d" % (v * 100)] = ("", f((6.0, v)))
for v in (0.3, 0.6, 1.0):                                         # reads high by v from 6 s on (1.55 = r_j2)
    DOSE["m_hi%03d" % (v * 100)] = ("", f((6.0, -v)))
for r in (0.05, 0.1, 0.2, 0.4):                                   # grows at r m/s up to 1.9 m low
    DOSE["m_dr%03d" % (r * 100)] = ("", drift(r))
for d in (0.5, 1.0, 2.0, 3.0):                                    # collapses to ~0.1 m for d seconds (5 = r_g5)
    DOSE["m_du%02d" % (d * 10)] = ("", f((6.0, 1.9), (6.0 + d, -0.05)))
OORF = {
    "m_oo03": ("", f((6.0, -10), (6.3, -0.05))),                  # out of range for 0.3 s / 1 s / for good (3 s = r_oor3)
    "m_oo10": ("", f((6.0, -10), (7.0, -0.05))),
    "m_ooN":  ("", f((6.0, -10))),
    "m_blip": ("", f(*[(t + k, v) for t in (6.0, 8.0, 10.0, 12.0) for k, v in ((0, -10), (0.2, -0.05))])),  # 4 blips of 0.2 s
}
FAULTS = {**FAULTS, **DOSE, **OORF}
ARMS = {**ARMS1, "X_OOR": (T0, OOR, 3)}
SYNP = T0 + ",RNGFND2_TYPE=10,RNGFND2_ORIENT=25,RNGFND2_MAX=8,RNGFND2_MIN=0.01,RNGFND2_GNDCLR=0.03"
R4 = {"_dir": "companion_r4", "synth": 1}
ARMS4 = {"X_SYN": (SYNP, R4, 4), "X_SYN_OOR": (SYNP, {**R4, "oor_s": 0.3}, 4), "X_SYN_L1": (SYNP, {**R4, "oor_s": 0.3, "source_set": 1}, 4)}
ARMS.update(ARMS4)
for e in (0.3, 0.5, 0.7, 1.0):
    for t in (0.2, 0.5, 1.0):
        if t != 0.5 or e == 0.3:                                  # t 0.5 with e 0.5 / 0.7 / 1.0 = G_E05 / FULL / G_E10
            ARMS["S_E%02d_T%02d" % (e * 10, t * 10)] = (T0, {"e_m": e, "t_s": t}, 3)
SWEEP = [a for a in ARMS if a.startswith("S_")]
SWEEP_F = ["r_h", "r_gN", "r_k", "r_i", "r_a", "r_c", "none", "wind3", "b_st2n", "b_st2p", "b_noise", "b_dr12p"]

ARMS["X_SYN_FIX"] = (SYNP, {**R4, "oor_s": 0.3, "skip_inv": 1}, 4)
CELLS = [("X_SYN_FIX", f) for f in ("m_blip", "m_oo03", "m_oo10", "m_ooN", "r_oor3", "none", "wind3", "b_st2n", "r_gN", "r_h", "r_k", "r_i")] \
    + [("X_SYN_OOR", f) for f in ("m_oo03", "m_oo10")]
jobs, seen = [], set()
for rep in range(1, 11):
    for arm, fault in CELLS:
        if 1:
            ovl, hg, _ = ARMS[arm]
            fo, fa = FAULTS[fault]
            yaw = YAWS[rep - 1]
            wdir = (yaw * 7 + 40 * rep) % 360
            tag = f"ab_{arm}__{fault}__r{rep:02d}"
            o = ",".join(x for x in (HARSH, ovl, fo, f"SIM_WIND_DIR={wdir}") if x)
            jobs.append("|".join([tag, "mtf01_sitl_rng.parm", "1", str(yaw), o, (SQ + " " + fa).strip()]))
            json.dump(hg, open(f"{D}/hg/{tag}.json", "w"))
open(D + "/jobs_r4b.txt", "w").write("\n".join(jobs) + "\n")
pick = [f"ab_{a}__{f}__r01" for a, f in (("X_SYN", "none"), ("X_SYN", "r_gN"), ("X_SYN", "r_h"), ("X_SYN_OOR", "r_oor3"), ("X_SYN_OOR", "m_ooN"), ("X_SYN_L1", "r_gN"), ("X_SYN", "r_g5"), ("X_SYN_OOR", "r_k"))]
open(D + "/jobs_r4b_smoke.txt", "w").write("\n".join(j for j in jobs if j.split("|")[0] in pick) + "\n")
print(len(jobs), "jobs")
