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
