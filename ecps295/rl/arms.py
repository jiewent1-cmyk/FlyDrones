"""Control arms for "is the behaviour coming from MiniFly?" (RL roadmap A1). Shared by the twin and run_g4.py.

  intact      unchanged
  brainoff    the 6 DN output groups always read 0 Hz (inputs still drive the network; decoder, state machine,
              SafetyGovernor and fence turn unchanged) -> what the scaffolding does on its own
  shuffle<k>  degree-preserving rewiring of MiniFly by Maslov-Sneppen edge switches (20 x edges): every neuron keeps
              exactly its in- and out-degree, each edge keeps its weight and sign (they belong to the pre-synaptic
              neuron); no duplicate edges or self-loops. Stricter than ClutchMedia775's global post permutation, which
              merges duplicate edges. k = rewiring seed
  randread<k> the 6 DN output groups are replaced by random non-input, non-output neurons of the same sizes
  bypass      no LIF: pseudo DN rates are a fixed-sign linear map of the encoder input rates (same anatomy as
              MiniFly: flow + haltere -> DNg02, ipsilateral loom + blank + near -> DNp03, loom - threshold -> DNp01)
              with 8 tunable gains (cfg["bypass"]), so the decoder/state machine can be optimised on raw features
  black       camera frames are all zero            (evaluation only)
  frozen      camera repeats its first frame         (evaluation only)
  noloom      LPLC2 / LC4 / LCb / LCn inputs silenced (evaluation only)
"""

from __future__ import annotations

import numpy as np
from scipy import sparse

OUTS = ["DNg02_L", "DNg02_R", "DNp03_L", "DNp03_R", "DNp01_L", "DNp01_R"]
BYPASS_DEFAULT = {"c0": 25.0, "k_flow": 0.3, "k_hal": 0.3, "k_loom": 0.6, "k_blank": 0.3, "k_near": 0.3, "k_gf": 0.6, "th_gf": 30.0}
_SHUFFLED: dict = {}


def connectome_for(base, arm: str):
    """The connectome an arm runs on (a degree-preserving shuffle for shuffle<k>, else the original)."""
    if not arm.startswith("shuffle"):
        return base
    k = int(arm[7:] or 1)
    key = (id(base), k)
    if key not in _SHUFFLED:
        import copy

        W = base.weights.tocoo()
        pre, post, w = W.col.copy(), W.row.copy(), W.data.copy()
        rng = np.random.default_rng(1000 + k)
        edges = set(zip(post.tolist(), pre.tolist()))
        n_ok, m = 0, len(post)
        for _ in range(20 * m):  # Maslov-Sneppen switches: (a->b, c->d) -> (a->d, c->b), weight stays with its edge
            i, j = rng.integers(m, size=2)
            b, d, a_, c_ = post[i], post[j], pre[i], pre[j]
            if i == j or b == d or a_ == d or c_ == b or (d, a_) in edges or (b, c_) in edges:
                continue
            edges -= {(b, a_), (d, c_)}
            edges |= {(d, a_), (b, c_)}
            post[i], post[j] = d, b
            n_ok += 1
        c = copy.copy(base)
        c.weights = sparse.csc_matrix((w, (post, pre)), shape=W.shape)
        c.meta = {**(getattr(base, "meta", {}) or {}), "shuffle_switches": n_ok}
        c.groups = dict(base.groups)
        _SHUFFLED[key] = c
    return _SHUFFLED[key]


def _mean(rates: dict, name: str) -> float:
    r = rates.get(name)
    return 0.0 if r is None else float(np.mean(r))


def install(arm: str, brain, pilot, drone, cfg: dict, seed: int = 0) -> None:
    """Patch a built brain / pilot / drone in place for the arm (call after the Pilot is constructed)."""
    if arm in ("intact", "") or arm.startswith("shuffle"):
        return
    if arm == "brainoff":
        orig = brain.tick

        def tick(inputs, ms):
            r = orig(inputs, ms)
            return {k: (0.0 if k in OUTS else v) for k, v in r.items()}

        brain.tick = tick
    elif arm.startswith("randread"):
        k = int(arm[8:] or 1)
        c = brain.connectome
        used = np.concatenate([c.group(g) for g in list(brain.input_specs) + list(brain.output_specs)])
        pool = np.setdiff1d(np.arange(c.n), used)
        rng = np.random.default_rng(2000 + k)
        picks = rng.choice(pool, size=sum(c.group(g).size for g in OUTS), replace=False)
        i = 0
        for g in OUTS:
            n = c.group(g).size
            c.groups[g] = np.sort(picks[i : i + n])
            i += n
    elif arm == "bypass":
        p = {**BYPASS_DEFAULT, **(cfg.get("bypass") or {})}
        relu = lambda x: max(0.0, x)  # noqa: E731

        def tick(inputs, ms):
            out = {k: _mean(inputs, k) for k in brain.input_specs}
            for s in "LR":
                flow = 0.5 * (_mean(inputs, f"T4a_{s}") + _mean(inputs, f"T4b_{s}"))
                loom = _mean(inputs, f"LPLC2_{s}") + _mean(inputs, f"LC4_{s}")
                out[f"DNg02_{s}"] = relu(p["c0"] + p["k_flow"] * flow + p["k_hal"] * _mean(inputs, f"HAL_{s}"))
                out[f"DNp03_{s}"] = relu(p["k_loom"] * loom + p["k_blank"] * _mean(inputs, f"LCb_{s}") + p["k_near"] * _mean(inputs, f"LCn_{s}"))
                out[f"DNp01_{s}"] = relu(p["k_gf"] * loom - p["th_gf"])
            brain.last_rates = out
            brain.realtime_factor = float("inf")
            return out

        brain.last_counts = np.zeros(1, np.int64)  # EcpsPilot.tick reads these; the LIF never runs in this arm
        brain.last_raster = []
        brain.tick = tick
    elif arm == "noloom":
        orig = brain.tick
        silence = {f"{g}_{s}" for g in ("LPLC2", "LC4", "LCb", "LCn") for s in "LR"}
        brain.tick = lambda inputs, ms: orig({k: v for k, v in inputs.items() if k not in silence}, ms)
    elif arm in ("black", "frozen"):
        orig = drone.frame
        first: list = []

        def frame():
            f = orig()
            if f is None:
                return f
            if arm == "black":
                return np.zeros_like(f)
            if not first:
                first.append(f.copy())
            return first[0]

        drone.frame = frame
    else:
        raise ValueError(f"unknown arm {arm!r}")


EVAL_ONLY = {"black", "frozen", "noloom"}
