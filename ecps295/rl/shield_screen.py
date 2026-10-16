"""Route review 2026-10-13: S4 range-shield and speed screen on the rl.hw_arms test set, paired with v4.

Conds are v4 + config overrides (no YAML needed). v4 / s3_rng records from rl/out/struct.jsonl are bit-identical and can
be copied into --out first (--seed-from), so only the new conds run.
  sh3       right / rear / left single-zone rangers only (shield counterpart of the rule-based s3_rng)
  sh4       + a front single-zone ranger (27 deg cone, 10 Hz, 1 cm noise, d_safe 0.25 m, alpha 1/s); smoke test: it
            stalls in front of boxes (the brain keeps asking forward, a 0-deg beam only cancels forward, no slide)
  sh4u      sh4 + unstick (yaw toward the open side after 1 s of clamped forward)
  sh5u      front pair at +-25 deg instead of one front ranger (off-axis beams let the projection slide) + unstick
  shfu      multi-zone front array (8 zones over 45 deg, 15 Hz, VL53L5CX-like) + right / rear / left + unstick
  shou      oracle: 32-beam ring, exact, 20 Hz, + unstick (upper bound of what a range shield can do)
  v4_cXXX   v4 with cruise = max_forward = 0.XX (speed baseline: the safety you get just by flying slower)
  shfu_c065 shfu at cruise 0.65 (does the shield buy speed / coverage?)
S5 (ecps_retina vision.ecps_tof): the same ToF beams also feed the brain's `near` (LCn) grids, so the brain turns away
by itself and the shield is only the backstop (same beam spec -> the same readings):
  s5n       ToF -> near only (sh5u beams), no shield: what the brain alone does with the extra sense
  s5        ToF -> near + the sh5u shield
  s5f       ToF -> near + the shfu shield (multi-zone front)
  s5f_g2    s5f with a more eager near cue (gain 2, d0 1.0 m)
  s5f_c065  s5f at cruise 0.65
S6 sensor-layout screen (2026-10-14): cheaper / simpler hardware than five single-zone boards. Every layout feeds the
brain (S5, beams within +-60 deg) and the shield with unstick; the "m" variants add the range memory and backing corridor
(ecps_struct RangeShield._memory) to cover blind sectors:
  s6_f1m    one front single-zone ToF (0 deg) + memory
  s6_f2     front pair only (+-25 deg, two VL53L1X on two I2C buses)        s6_f2m  + memory
  s6_f3b    front pair + rear (3 boards)
  s6_mr4    front / right / rear / left single-zone (Bitcraze Multi-ranger-like single board)
  s6_m1     one forward 8x8 multi-zone board (VL53L5CX, 45 deg, 15 Hz)        s6_m1m  + memory
  s6_w1m    one forward wide 8x8 board (VL53L7CX-like, 60 deg horizontal) + memory
  s6_m1b    forward 8x8 + one rear single-zone
  s6_m2     forward + rear 8x8 (the ETH Crazyflie layout)
  s6_cam    no new hardware: ground-contact-line ranging from the existing camera, 12 columns over +-60 deg, 15 Hz,
            sigma 0.02 + 0.05 d^2, nothing beyond 1.5 m, 10 % of readings lost, each net wall seen with p 0.5 (per
            episode)                                                         s6_camm + memory
  s6_ld06   one 360-degree 2D lidar (LD06 class, 36 beams, 10 Hz, 2 cm)
batch 2 (after the hardware survey: the Orange Pi Zero 3W has four I2C buses, so up to four sensors need no XSHUT):
  s6_w2     wide 8x8 front + wide 8x8 rear (2 x VL53L7CX, 60 deg each, straight mounts)   s6_w2m  + memory
  s6_w1rbl  wide 8x8 front + right / rear / left single-zone (VL53L7CX + 3 generic VL53L1X, four buses)
  s6_mr4m   s6_mr4 (four single-zone, one per bus) + memory
S7 (2026-10-14): the shield works off a local 3D occupancy map built from multi-zone boards (ecps_map; the brain still
gets the front columns as in S6):
  s7_w2map  front + rear VL53L7CX (60 x 60 deg, 8x8) -> map -> shield
  s7_m2map  front + rear VL53L5CX (45 x 45 deg, 8x8) -> map -> shield
  s7_w1map  front VL53L7CX only -> map -> shield, + the backing corridor (no rear sensor)
python -m rl.shield_screen --seed-from rl/out/struct.jsonl --procs 8 --out rl/out/shield.jsonl
python -m rl.shield_screen --report --out rl/out/shield.jsonl
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from rl import hw_arms
from rl.batch import run_jobs

E = Path(__file__).resolve().parent.parent
V4 = hw_arms.V4
FRONT8 = [-19.69, -14.06, -8.44, -2.81, 2.81, 8.44, 14.06, 19.69]
RING32 = [i * 11.25 for i in range(-15, 17)]


def shield(**kw) -> dict:
    return {"ecps_pilot": {"struct": {"shield": {"enabled": True, **kw}}}}


def tof(**kw) -> dict:
    return {"vision": {"ecps_tof": {"enabled": True, **kw}}}


def speed(c: float) -> dict:
    return {"decoder": {"cruise": c}, "safety": {"max_forward": c}}


CONDS = {
    "sh3": (V4, "intact", shield(beams=[90.0, 180.0, -90.0])),
    "sh4": (V4, "intact", shield()),
    "sh4u": (V4, "intact", shield(unstick_s=1.0)),
    "sh5u": (V4, "intact", shield(beams=[-25.0, 25.0, 90.0, 180.0, -90.0], unstick_s=1.0)),
    "shfu": (V4, "intact", shield(beams=FRONT8 + [90.0, 180.0, -90.0], fov_deg=5.6, rate_hz=15.0, unstick_s=1.0)),
    "shou": (V4, "intact", shield(beams=RING32, fov_deg=0.0, rate_hz=20.0, noise_m=0.0, unstick_s=1.0)),
    "v4_c035": (V4, "intact", speed(0.35)),
    "v4_c065": (V4, "intact", speed(0.65)),
    "shfu_c065": (V4, "intact", {**shield(beams=FRONT8 + [90.0, 180.0, -90.0], fov_deg=5.6, rate_hz=15.0, unstick_s=1.0), **speed(0.65)}),
}
B5 = [-25.0, 25.0, 90.0, 180.0, -90.0]
BF = FRONT8 + [90.0, 180.0, -90.0]
CONDS.update({
    "s5n": (V4, "intact", tof(beams=B5)),
    "s5": (V4, "intact", {**tof(beams=B5), **shield(beams=B5, unstick_s=1.0)}),
    "s5f": (V4, "intact", {**tof(beams=BF, fov_deg=5.6, rate_hz=15.0), **shield(beams=BF, fov_deg=5.6, rate_hz=15.0, unstick_s=1.0)}),
    "s5f_g2": (V4, "intact", {**tof(beams=BF, fov_deg=5.6, rate_hz=15.0, gain=2.0, d0=1.0),
                              **shield(beams=BF, fov_deg=5.6, rate_hz=15.0, unstick_s=1.0)}),
    "s5f_c065": (V4, "intact", {**tof(beams=BF, fov_deg=5.6, rate_hz=15.0), **shield(beams=BF, fov_deg=5.6, rate_hz=15.0, unstick_s=1.0),
                                **speed(0.65)}),
})

F1, F2, F3B, MR4 = [0.0], [-25.0, 25.0], [-25.0, 25.0, 180.0], [0.0, 90.0, 180.0, -90.0]
WIDE8 = [-26.25, -18.75, -11.25, -3.75, 3.75, 11.25, 18.75, 26.25]
REAR8 = [round((180.0 + b + 180.0) % 360.0 - 180.0, 2) for b in FRONT8]
CAM12 = [-55.0 + 10.0 * i for i in range(12)]
RING36 = [i * 10.0 for i in range(-17, 19)]
CAM = dict(noise=0.02, noise_k=0.05, max_m=1.5, p_miss=0.1, p_net=0.5)


def s6(beams, fov, rate=10.0, noise=0.01, mem=False, **model) -> dict:
    sh = dict(beams=beams, fov_deg=fov, rate_hz=rate, noise_m=noise, unstick_s=1.0, **model)
    if mem:
        sh["memory"] = {"enabled": True}
    return {**tof(beams=beams, fov_deg=fov, rate_hz=rate, noise_m=noise, **model), **shield(**sh)}


CONDS.update({
    "s6_f1m": (V4, "intact", s6(F1, 27.0, mem=True)),
    "s6_f2": (V4, "intact", s6(F2, 27.0)),
    "s6_f2m": (V4, "intact", s6(F2, 27.0, mem=True)),
    "s6_f3b": (V4, "intact", s6(F3B, 27.0)),
    "s6_mr4": (V4, "intact", s6(MR4, 27.0)),
    "s6_m1": (V4, "intact", s6(FRONT8, 5.6, rate=15.0)),
    "s6_m1m": (V4, "intact", s6(FRONT8, 5.6, rate=15.0, mem=True)),
    "s6_w1m": (V4, "intact", s6(WIDE8, 7.5, rate=15.0, mem=True)),
    "s6_m1b": (V4, "intact", s6(FRONT8 + [180.0], [5.6] * 8 + [27.0], rate=15.0)),
    "s6_m2": (V4, "intact", s6(FRONT8 + REAR8, 5.6, rate=15.0)),
    "s6_cam": (V4, "intact", s6(CAM12, 10.0, rate=15.0, **CAM)),
    "s6_camm": (V4, "intact", s6(CAM12, 10.0, rate=15.0, mem=True, **CAM)),
    "s6_ld06": (V4, "intact", s6(RING36, 0.0, rate=10.0, noise=0.02)),
})
REARW8 = [round((180.0 + b + 180.0) % 360.0 - 180.0, 2) for b in WIDE8]
CONDS.update({
    "s6_w2": (V4, "intact", s6(WIDE8 + REARW8, 7.5, rate=15.0)),
    "s6_w2m": (V4, "intact", s6(WIDE8 + REARW8, 7.5, rate=15.0, mem=True)),
    "s6_w1rbl": (V4, "intact", s6(WIDE8 + [90.0, 180.0, -90.0], [7.5] * 8 + [27.0] * 3, rate=15.0)),
    "s6_mr4m": (V4, "intact", s6(MR4, 27.0, mem=True)),
})
L7 = dict(hfov=60.0, vfov=60.0, n=8, rate_hz=15.0, noise_m=0.01, max_m=3.5)
L5 = dict(hfov=45.0, vfov=45.0, n=8, rate_hz=15.0, noise_m=0.01, max_m=4.0)


def s7(sensors, brain, brain_fov, mem=None) -> dict:
    sh = dict(beams=[], unstick_s=1.0, map={"enabled": True, "sensors": sensors})
    if mem:
        sh["memory"] = mem
    return {**tof(beams=brain, fov_deg=brain_fov, rate_hz=15.0), **shield(**sh)}


CONDS.update({
    "s7_w2map": (V4, "intact", s7([{**L7, "bearing": 0.0}, {**L7, "bearing": 180.0}], WIDE8, 7.5)),
    "s7_m2map": (V4, "intact", s7([{**L5, "bearing": 0.0}, {**L5, "bearing": 180.0}], FRONT8, 5.6)),
    "s7_w1map": (V4, "intact", s7([{**L7, "bearing": 0.0}], WIDE8, 7.5, mem={"enabled": True, "points": False})),
})


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--conds", nargs="+", default=list(CONDS))
    ap.add_argument("--procs", type=int, default=8)
    ap.add_argument("--out", default=str(E / "rl" / "out" / "shield.jsonl"))
    ap.add_argument("--seed-from", default=None, help="copy v4 (and s3_rng) records from an rl.hw_arms --out file")
    ap.add_argument("--report", action="store_true")
    a = ap.parse_args()
    extra = [c for c in ("s3_rng",)]
    names = ["v4", *extra, *a.conds]
    hw_arms.COND.clear()
    hw_arms.COND.update({c: (V4, "intact") for c in names})
    if a.report:
        R = [json.loads(l) for l in open(a.out)]
        present = {r["tag"] for r in R}
        hw_arms.COND.clear()
        hw_arms.COND.update({c: (V4, "intact") for c in ["v4", *extra, *CONDS] if c in present})
        hw_arms.report(R); return
    out = Path(a.out)
    if a.seed_from and not out.exists():
        with open(out, "w") as f:
            for l in open(a.seed_from):
                r = json.loads(l)
                if r.get("tag") in ("v4", *extra) and "error" not in r:
                    f.write(l if l.endswith("\n") else l + "\n")
    J = []
    for j in hw_arms.jobs():
        c = CONDS.get(j["tag"])
        if c is None:
            continue
        J.append({**j, "variant": c[0], "arm": c[1], "overrides": c[2]})
    if out.exists():
        done = {(r["tag"], r["set"], r["key"]) for r in map(json.loads, open(out)) if "error" not in r}
        J = [j for j in J if (j["tag"], j["set"], j["key"]) not in done]
    print(len(J), "jobs to run", flush=True)
    run_jobs(J, a.procs, str(out))
    main_report = [json.loads(l) for l in open(out)]
    hw_arms.report(main_report)


if __name__ == "__main__":
    main()
