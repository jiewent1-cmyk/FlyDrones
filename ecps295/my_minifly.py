"""Custom MiniFly connectomes for the ECPS295 drone (TechRoute §10 S2).

Every variant starts from the upstream synthetic MiniFly (flydrones.brain.synthetic.build_minifly, same seed) and
only ADDS neurons and synapses, so all upstream pathways are bit-identical and the signs stay biological
(excitatory / inhibitory per cell type).

v2  LCb: "frontal blanking" columnar cells. The G4 suite showed that a plain (textureless) wall never drives the
    flow-based looming detectors: once it fills the view nothing moves and it even hides the textured floor.
    LCb cells receive EcpsRetina's `blank` feature (sudden loss of texture in the central field) and feed the
    looming integrators PVLP (-> DNp03 saccade + brake) with a weak drive to the giant fibre DNp01.

    python my_minifly.py v2 --out minifly/minifly_v2.npz
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from scipy import sparse

from flydrones.brain.connectome import Connectome
from flydrones.brain.synthetic import build_minifly


class Builder:
    """Append populations and synapses to an existing connectome (upstream neurons keep their indices)."""

    def __init__(self, base: Connectome, seed: int = 11):
        self.base = base
        self.rng = np.random.default_rng(seed)
        self.types = list(base.types)
        self.sides = list(base.sides)
        self.sign = np.sign(np.asarray(base.weights.sum(axis=0)).ravel())  # presynaptic sign per upstream neuron
        self.sign[self.sign == 0] = 1.0
        self.new_sign: list[float] = []
        self.rows: list[np.ndarray] = []
        self.cols: list[np.ndarray] = []
        self.vals: list[np.ndarray] = []

    def add(self, name: str, n: int, side: str, sign: float) -> np.ndarray:
        start = len(self.types)
        self.types += [name] * n
        self.sides += [side] * n
        self.new_sign += [sign] * n
        return np.arange(start, start + n)

    def g(self, name: str, side: str) -> np.ndarray:
        t, s = np.asarray(self.types), np.asarray(self.sides)
        return np.flatnonzero((t == name) & (s == side))

    def connect(self, pre: np.ndarray, post: np.ndarray, syn: float, p: float = 1.0, jitter: float = 0.3) -> None:
        P, Q = np.meshgrid(pre, post)
        mask = self.rng.random(P.shape) < p
        w = np.clip(np.round(syn * (1 + jitter * self.rng.standard_normal(P.shape))), 1, None)
        self.rows.append(Q[mask])
        self.cols.append(P[mask])
        self.vals.append(w[mask])

    def build(self, name: str, meta: dict) -> Connectome:
        n0, n = self.base.n, len(self.types)
        sign = np.concatenate([self.sign, np.asarray(self.new_sign, dtype=np.float32)])
        W0 = self.base.weights.tocoo()
        r = np.concatenate([W0.row, *self.rows]) if self.rows else W0.row
        c = np.concatenate([W0.col, *self.cols]) if self.cols else W0.col
        add_v = (np.concatenate(self.vals) * sign[np.concatenate(self.cols)]) if self.vals else np.zeros(0)
        v = np.concatenate([W0.data, add_v]).astype(np.float32)
        W = sparse.csc_matrix((v, (r, c)), shape=(n, n), dtype=np.float32)
        return Connectome(
            name=name,
            weights=W,
            types=np.asarray(self.types),
            sides=np.asarray(self.sides),
            superclass=None,
            body_ids=np.arange(n, dtype=np.int64),
            meta={**self.base.meta, **meta, "base": self.base.name, "added_neurons": n - n0},
        )


def v2(seed: int = 7) -> Connectome:
    b = Builder(build_minifly(seed))
    for s in ("L", "R"):
        b.add("LCb", 24, s, +1.0)  # cholinergic columnar; 24 like LPLC2 (12, like LC4, did not move PVLP at all)
    for s in ("L", "R"):
        o = {"L": "R", "R": "L"}[s]
        lcb, pvlp = b.g("LCb", s), b.g("PVLP", s)
        b.connect(lcb, pvlp, 8, p=0.7)  # a bit above LPLC2 -> PVLP (6): blanking has no LC4 partner
        b.connect(lcb, b.g("DNp01", s), 2, p=1.0)  # weaker than looming (3): blanking alone rarely fires the GF
        b.connect(lcb, b.g("PVLP_inh", o), 2, p=0.5)  # joins the left/right competition for the saccade side
    return b.build("minifly-ecps-v2", {"variant": "v2", "notes": "LCb frontal-blanking input -> PVLP/DNp01"})


VARIANTS = {"v2": v2}

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("variant", choices=list(VARIANTS))
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    con = VARIANTS[a.variant]()
    base = build_minifly(7)
    same = (con.weights[: base.n, : base.n] != base.weights).nnz == 0
    print(con.summary())
    print(
        f"upstream block identical: {same}; added {con.n - base.n} neurons, "
        f"{con.weights.nnz - base.weights.nnz} connections -> {Path(con.save(a.out))}"
    )
