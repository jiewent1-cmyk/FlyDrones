"""Mushroom-body plasticity on top of MiniFly (RL roadmap stage E, template AleksiAero, three changes as planned).

  KC layer    n_kc Kenyon cells, each summing 4-7 random encoder inputs (adaptive z-score of the per-neuron input rates
              the Pilot feeds the brain: flow, loom, blank, near, ventral, haltere), global inhibition as k-winners-take-
              all so a fixed 3-10 % of KCs is active per tick
  MBON        one approach output m = mean g over the active KCs (1 while nothing has been learned); g in [0, 1] only
              ever inhibits (multiplicative depression) and recovers to 1 with tau_rec
  read-out    a scene the KCs tie to punishment (m < 1) slows the forward command (k_fwd) and adds k_sac Hz to both DNp03
              read-outs (saccade drive); m = 1 changes nothing, so g = 1 reproduces the plain variant bit for bit
  learning    eligibility trace tau_e on KC activity; phasic dopamine = teaching signal - slow baseline - threshold;
              PPL1 (punishment: contact 1 s, near miss / governor / fence turn 0.3 s) depresses g of eligible KCs,
              PAM (entering a new coverage cell) lets them recover faster. Applied every tick (50 ms at 20 Hz).
rangers    rng_in > 0 (e5/e6): the S3 rear / left / right rangers (drone.side_ranges()) join the KC input as proximity
              max(0, 1 - d / rng_m), each repeated rng_in times; back = True lets m also slow backing (forward < 0)
No hand-written taxis term. Only the twin provides teaching signals; Gazebo and the real drone load a frozen g.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np

DEFAULT = {"n_kc": 300, "claws": [4, 7], "active_frac": 0.05, "z_tau_s": 10.0, "tau_e_s": 2.0, "tau_rec_s": 900.0,
           "eta_ppl1": 0.7, "eta_pam": 0.5, "teach_ovr": True, "da_base_s": 30.0, "da_thr": 0.05, "k_fwd": 0.6, "k_sac": 0.0, "seed": 0,
           "rng_in": 0, "rng_m": 0.6, "back": False}


class MushroomBody:
    def __init__(self, cfg: dict | None = None, g=None, learn: bool = False):
        self.p = {**DEFAULT, **(cfg or {})}
        self.n = int(self.p["n_kc"])
        self.g = np.ones(self.n) if g is None else np.asarray(g, float).copy()
        assert self.g.shape == (self.n,)
        self.learn = learn
        self.e = np.zeros(self.n)
        self.kc = np.zeros(self.n, bool)
        self.m = 1.0
        self._keys = None
        self._da_base = 0.0
        self.stats = {"ticks": 0, "active": 0, "m_sum": 0.0, "m_min": 1.0, "ppl1_s": 0.0, "pam_s": 0.0}

    def _wire(self, x: np.ndarray) -> None:
        rng = np.random.default_rng(90_000 + int(self.p["seed"]))  # own stream: never touches the brain / drone RNGs
        lo, hi = self.p["claws"]
        self.claws = [rng.choice(x.size, size=int(rng.integers(lo, hi + 1)), replace=False) for _ in range(self.n)]
        self.mu, self.var = x.copy(), np.ones(x.size)
        self.k = max(1, int(round(self.p["active_frac"] * self.n)))

    def _vector(self, inputs: dict, sizes: dict) -> np.ndarray:
        if self._keys is None:
            self._keys = sorted(k for k in inputs if sizes.get(k, 0) > 0)
        return np.concatenate([np.broadcast_to(np.asarray(inputs.get(k, 0.0), float), (sizes[k],)) for k in self._keys])

    def sense(self, inputs: dict, sizes: dict, dt: float) -> float:
        """KC activity and the approach output m for this tick's encoder input rates."""
        x = self._vector(inputs, sizes)
        if not hasattr(self, "claws"):
            self._wire(x)
        a = dt / self.p["z_tau_s"]
        self.mu += a * (x - self.mu)
        self.var += a * ((x - self.mu) ** 2 - self.var)
        z = (x - self.mu) / np.sqrt(self.var + 1e-6)
        drive = np.array([z[c].sum() for c in self.claws])
        kc = np.zeros(self.n, bool)
        top = np.argpartition(-drive, self.k - 1)[: self.k]
        kc[top[drive[top] > 0]] = True
        self.kc = kc
        self.m = float(self.g[kc].mean()) if kc.any() else 1.0
        s = self.stats
        s["ticks"] += 1
        s["active"] += int(kc.sum())
        s["m_sum"] += self.m
        s["m_min"] = min(s["m_min"], self.m)
        return self.m

    def teach(self, ppl1: float, pam: float, dt: float) -> None:
        """Eligibility trace + dopamine update of g (only while learning)."""
        self.e = self.e * math.exp(-dt / self.p["tau_e_s"]) + self.kc
        if not self.learn:
            return
        en = self.e / (self.p["tau_e_s"] / dt)  # ~ share of the recent window the KC was active
        da = max(0.0, ppl1 - self._da_base - self.p["da_thr"])
        self._da_base += dt / self.p["da_base_s"] * (ppl1 - self._da_base)
        self.g -= self.p["eta_ppl1"] * da * en * dt
        self.g += self.p["eta_pam"] * pam * en * (1 - self.g) * dt
        self.g += (1 - self.g) * (1 - math.exp(-dt / self.p["tau_rec_s"]))
        np.clip(self.g, 0.0, 1.0, out=self.g)
        self.stats["ppl1_s"] += dt * (ppl1 > 0)
        self.stats["pam_s"] += dt * (pam > 0)

    def summary(self) -> dict:
        s = self.stats
        t = max(1, s["ticks"])
        return {"mb_active_frac": round(s["active"] / t / self.n, 4), "mb_m_mean": round(s["m_sum"] / t, 4), "mb_m_min": round(s["m_min"], 3),
                "mb_g_mean": round(float(self.g.mean()), 4), "mb_g_low": int((self.g < 0.5).sum()),
                "mb_ppl1_s": round(s["ppl1_s"], 1), "mb_pam_s": round(s["pam_s"], 1)}


def install(mb: MushroomBody, brain, pilot, drone=None) -> None:
    """Wrap brain.tick (KCs see the encoder inputs; m adds saccade drive) and the decoder (m slows forward)."""
    sizes = {k: int(brain.connectome.group(k).size) for k in brain.input_specs}
    orig_tick, orig_update = brain.tick, pilot.decoder.update
    p = mb.p
    rng = int(p["rng_in"]) if drone is not None and hasattr(drone, "side_ranges") else 0
    if rng:
        sizes = {**sizes, **{f"rng_{k}": rng for k in ("rear", "left", "right")}}

    def tick(inputs, ms):
        x = inputs
        if rng:
            r = drone.side_ranges()
            x = {**inputs, **{f"rng_{k}": max(0.0, 1.0 - r[k] / p["rng_m"]) for k in ("rear", "left", "right")}}
        m = mb.sense(x, sizes, ms / 1000.0)
        r = orig_tick(inputs, ms)
        if m < 1.0 and p["k_sac"]:
            r = dict(r)
            for k in ("DNp03_L", "DNp03_R"):
                if k in r:
                    r[k] = r[k] + p["k_sac"] * (1.0 - m)
        return r

    def update(rates, dt):
        cmd = orig_update(rates, dt)
        if mb.m < 1.0 and (cmd.forward > 0 or (p["back"] and cmd.forward < 0)):
            cmd = dataclasses.replace(cmd, forward=cmd.forward * max(0.0, 1.0 - p["k_fwd"] * (1.0 - mb.m)))
        return cmd

    brain.tick = tick
    pilot.decoder.update = update
