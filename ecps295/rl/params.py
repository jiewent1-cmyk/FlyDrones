"""Parameter vector for the pilot experiment: decoder group A (RL plan P1), started at MiniFly v3_2.

Every entry: (name, cfg path, v3_2 value, low, high, log-scale). CMA-ES works on z in [0, 1]^n (log or linear map
between low and high); `to_overrides` turns a z vector into the nested dict merged over the variant config.
"""

from __future__ import annotations

import math

import numpy as np

SPEC = [
    ("yaw_gain", "decoder.axes.yaw.gain", 0.025, 0.010, 0.060, True),
    ("yaw_dnp03_w", "decoder.axes.yaw.dnp03", 0.8, 0.0, 2.0, False),  # DNp03_L +w, DNp03_R -w
    ("smoothing", "decoder.smoothing", 0.2, 0.05, 0.6, True),
    ("brake_dnp03", "decoder.brake_terms.dnp03", 0.02, 0.005, 0.08, True),
    ("brake_dnp01", "decoder.brake_terms.dnp01", 0.03, 0.005, 0.10, True),
    ("brake_decay", "decoder.ecps.brake_decay", 0.97, 0.85, 0.99, False),
    ("escape_thr", "decoder.escape.threshold_hz", 20.0, 8.0, 40.0, True),
    ("sacc_thr", "decoder.ecps.saccade.threshold_hz", 15.0, 6.0, 30.0, True),
    ("sacc_dur", "decoder.ecps.saccade.duration_s", 1.5, 0.6, 2.5, False),
    ("sacc_backoff", "decoder.ecps.saccade.backoff", 0.3, 0.0, 0.6, False),
    ("sacc_backoff_s", "decoder.ecps.saccade.backoff_s", 0.6, 0.0, 1.5, False),
    ("sacc_on_brake", "decoder.ecps.saccade.on_brake", 0.6, 0.2, 1.2, False),
    ("sacc_refr", "decoder.ecps.saccade.refractory_s", 1.0, 0.0, 3.0, False),
    ("sacc_keep_dir", "decoder.ecps.saccade.keep_direction_s", 5.0, 0.0, 10.0, False),
    ("caut_hold", "decoder.ecps.caution.hold_s", 2.0, 0.0, 4.0, False),
    ("caut_ramp", "decoder.ecps.caution.ramp_s", 2.0, 0.0, 4.0, False),
    ("caut_on_brake", "decoder.ecps.caution.on_brake", 0.6, 0.2, 1.2, False),
]
# es3: bounds widened where es2's best point sat on a bound (smoothing, DNp03/DNp01 brake, escape / saccade
# thresholds, saccade duration, refractory, caution hold / trigger) or close to one (yaw gain)
WIDE = {
    "yaw_gain": (0.004, 0.06),
    "smoothing": (0.05, 0.9),
    "brake_dnp03": (0.001, 0.08),
    "brake_dnp01": (0.005, 0.3),
    "escape_thr": (8.0, 80.0),
    "sacc_thr": (2.0, 30.0),
    "sacc_dur": (0.6, 4.0),
    "sacc_refr": (0.0, 6.0),
    "caut_hold": (0.0, 8.0),
    "caut_on_brake": (0.2, 2.5),
}
SPEC_DEFAULT = list(SPEC)
SPEC_WIDE = [(n, path, v0, *WIDE.get(n, (lo, hi)), lg) for n, path, v0, lo, hi, lg in SPEC]
NAMES = [s[0] for s in SPEC]
N = len(SPEC)


def set_space(name: str) -> None:
    """'default' (es1, es2) or 'wide' (es3). Rebinds SPEC / Z0 for this process and forked workers."""
    global SPEC, Z0
    SPEC = SPEC_WIDE if name == "wide" else SPEC_DEFAULT
    Z0 = to_z(np.array([s[2] for s in SPEC]))


def to_value(z: np.ndarray) -> np.ndarray:
    z = np.clip(np.asarray(z, float), 0.0, 1.0)
    out = []
    for zi, (_, _, _, lo, hi, lg) in zip(z, SPEC):
        out.append(math.exp(math.log(lo) + zi * (math.log(hi) - math.log(lo))) if lg else lo + zi * (hi - lo))
    return np.array(out)


def to_z(values: np.ndarray) -> np.ndarray:
    out = []
    for v, (_, _, _, lo, hi, lg) in zip(values, SPEC):
        out.append((math.log(v) - math.log(lo)) / (math.log(hi) - math.log(lo)) if lg else (v - lo) / (hi - lo))
    return np.clip(np.array(out), 0.0, 1.0)


Z0 = to_z(np.array([s[2] for s in SPEC]))


def to_overrides(z: np.ndarray) -> dict:
    v = {n: float(x) for n, x in zip(NAMES, to_value(z))}
    return {
        "decoder": {
            "smoothing": v["smoothing"],
            "axes": {
                "yaw": {
                    "gain": v["yaw_gain"],
                    "terms": {"DNg02_R": 1.0, "DNg02_L": -1.0, "DNp03_L": v["yaw_dnp03_w"], "DNp03_R": -v["yaw_dnp03_w"]},
                }
            },
            "brake_terms": {"DNp03_L": v["brake_dnp03"], "DNp03_R": v["brake_dnp03"], "DNp01_L": v["brake_dnp01"], "DNp01_R": v["brake_dnp01"]},
            "escape": {"threshold_hz": v["escape_thr"]},
            "ecps": {
                "brake_decay": v["brake_decay"],
                "saccade": {
                    "threshold_hz": v["sacc_thr"],
                    "duration_s": v["sacc_dur"],
                    "backoff": v["sacc_backoff"],
                    "backoff_s": v["sacc_backoff_s"],
                    "on_brake": v["sacc_on_brake"],
                    "refractory_s": v["sacc_refr"],
                    "keep_direction_s": v["sacc_keep_dir"],
                },
                "caution": {"hold_s": v["caut_hold"], "ramp_s": v["caut_ramp"], "on_brake": v["caut_on_brake"]},
            },
        }
    }


def describe(z: np.ndarray) -> dict:
    return {n: round(float(x), 4) for n, x in zip(NAMES, to_value(z))}


# bypass arm (rl.arms): 8 extra gains of the feature -> pseudo-DN map, appended to the decoder vector
BYPASS_SPEC = [
    ("c0", 25.0, 0.0, 60.0, False),
    ("k_flow", 0.3, 0.01, 3.0, True),
    ("k_hal", 0.3, 0.01, 3.0, True),
    ("k_loom", 0.6, 0.01, 5.0, True),
    ("k_blank", 0.3, 0.01, 5.0, True),
    ("k_near", 0.3, 0.01, 5.0, True),
    ("k_gf", 0.6, 0.01, 5.0, True),
    ("th_gf", 30.0, 0.0, 120.0, False),
]


def _map(z, lo, hi, lg):
    z = min(1.0, max(0.0, float(z)))
    return math.exp(math.log(lo) + z * (math.log(hi) - math.log(lo))) if lg else lo + z * (hi - lo)


def _unmap(v, lo, hi, lg):
    return min(1.0, max(0.0, (math.log(v) - math.log(lo)) / (math.log(hi) - math.log(lo)) if lg else (v - lo) / (hi - lo)))


def split(z, arm: str):
    """Decoder part and (for the bypass arm) the extra gains of a CMA vector."""
    z = np.asarray(z, float)
    return (z[:N], z[N:]) if arm == "bypass" else (z, z[:0])


def full_overrides(z, arm: str) -> dict:
    zd, zb = split(z, arm)
    o = to_overrides(zd)
    if arm == "bypass":
        o["bypass"] = {n: _map(x, lo, hi, lg) for x, (n, _, lo, hi, lg) in zip(zb, BYPASS_SPEC)}
    return o


def x0_for(arm: str, zdec):
    if arm != "bypass":
        return np.asarray(zdec, float)
    return np.concatenate([np.asarray(zdec, float), [_unmap(v0, lo, hi, lg) for _, v0, lo, hi, lg in BYPASS_SPEC]])


def describe_full(z, arm: str) -> dict:
    zd, zb = split(z, arm)
    d = describe(zd)
    if arm == "bypass":
        d.update({n: round(_map(x, lo, hi, lg), 4) for x, (n, _, lo, hi, lg) in zip(zb, BYPASS_SPEC)})
    return d
