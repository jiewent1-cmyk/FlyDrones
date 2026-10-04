"""Retina with an extra `blank` feature for the custom MiniFly v2 (TechRoute §10 S2, LCb cells in my_minifly.py).

Upstream looming is the expansion of optic flow, which needs texture. A plain wall filling the view gives no flow and
even hides the textured floor, so it is never seen (G4 suite T2_plain). `blank` reports a SUDDEN rise in the share of
textureless cells in the central field (camera tilted down: normally floor texture and the far net / scenery), per eye,
relative to a slowly adapting baseline, so a large uniform region that has always been there does not count.
"""

from __future__ import annotations

import numpy as np

from flydrones.senses import Retina
from flydrones.senses.retina import EyeFeatures


class EcpsRetina(Retina):
    blank_cfg: dict = {}

    @classmethod
    def from_config(cls, cfg: dict) -> EcpsRetina:
        r = super().from_config(cfg)
        b = (cfg.get("vision", {}) or {}).get("ecps_blank", {}) or {}
        r.blank_thr = float(b.get("energy_thr", 2e-4))  # mean squared gradient per cell (image in 0..1)
        r.blank_rows = tuple(b.get("rows", (1, 4)))  # grid rows of the central band [start, stop)
        r.blank_gain = float(b.get("gain", 2.5))
        r.blank_min = float(b.get("min_frac", 0.55))  # absolute share needed before it counts
        r.blank_tau = float(b.get("baseline_s", 6.0))
        r.blank_hz = float(b.get("hz", 20.0))
        # v2 saturated both eyes, so the saccade side was a coin flip. The level says "something blank ahead", the
        # side comes from the textureless share across each whole eye (like upstream looming's loom_by_eye).
        r.blank_side_gain = float(b.get("side_gain", 0.0))  # 0 = v2 behaviour
        r._blank_base = {"L": None, "R": None}
        r.last_blank = {"L": 0.0, "R": 0.0, "frac_L": 0.0, "frac_R": 0.0, "side_L": 0.5}
        return r

    def encode(self, frame):
        vf = super().encode(frame)
        if frame is None or self.prev is None:
            for e in "LR":
                vf.eyes[e].grids["blank"] = np.zeros((self.rows, self.cols), np.float32)
            return vf
        img = self.prev  # current blurred, resized gray image from Retina.encode
        iy, ix = np.gradient(img)
        energy = ix * ix + iy * iy
        ncols = self.cols * 2
        rh, cw = self.h // self.rows, ((self.w // 2) * 2) // ncols
        e = energy[: rh * self.rows, : cw * ncols].reshape(self.rows, rh, ncols, cw).mean(axis=(1, 3))
        r0, r1 = self.blank_rows
        half = self.cols // 2
        central = {"L": e[r0:r1, half : self.cols], "R": e[r0:r1, self.cols : self.cols + half]}  # next to the midline
        a = 1.0 / max(1.0, self.blank_tau * self.blank_hz)
        levels = {}
        for eye, cells in central.items():
            frac = float((cells < self.blank_thr).mean())
            base = self._blank_base[eye]
            if base is None:
                base = frac
            level = 0.0
            if frac >= self.blank_min:
                level = float(np.clip((frac - base) / max(1e-3, 1.0 - base) * self.blank_gain, 0.0, 1.0))
            if level < 0.2:  # adapt only while nothing alarming is in front
                base = (1 - a) * base + a * frac
            self._blank_base[eye] = base
            levels[eye] = level
            self.last_blank[f"frac_{eye}"] = frac
        if self.blank_side_gain > 0:
            whole = {"L": e[r0:r1, : self.cols], "R": e[r0:r1, self.cols : ncols]}
            fl, fr = (float((whole[k] < self.blank_thr).mean()) for k in "LR")
            side_l = float(np.clip(0.5 + (fl - fr) * self.blank_side_gain, 0.0, 1.0))
            level = max(levels.values())
            levels = {"L": level * min(1.0, 2 * side_l), "R": level * min(1.0, 2 * (1 - side_l))}
            self.last_blank["side_L"] = side_l
        for eye in "LR":
            self.last_blank[eye] = levels[eye]
            vf.eyes[eye].grids["blank"] = np.full((self.rows, self.cols), levels[eye], np.float32)
        return vf


__all__ = ["EcpsRetina", "EyeFeatures"]
