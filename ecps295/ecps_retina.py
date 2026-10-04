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
        # v3 `near`: centering response. Straight forward flight gives front-to-back flow in BOTH eyes, stronger on
        # the side with nearer surfaces; rotation gives opposite signs, so only count it while both eyes see
        # front-to-back flow and the gyro (yaw_rate_dps, set by EcpsPilot from the previous tick) is quiet.
        n = (cfg.get("vision", {}) or {}).get("ecps_near", {}) or {}
        r.near_on = bool(n.get("enabled", False))
        r.near_rows = tuple(n.get("rows", (2, 6)))  # lower field: floor and obstacle bases
        # lateral field only: frontal flow depends on where the texture is (tape bands, edges), not on distance;
        # centering in bees and flies uses the sides of the eye. outer_cols = columns counted from each eye's edge
        r.near_outer = int(n.get("outer_cols", 0))  # 0 = whole eye (first v3 try)
        r.near_gain = float(n.get("gain", 3.0))
        r.near_min_flow = float(n.get("min_flow", 0.08))  # mean front-to-back level needed in both eyes
        r.near_max_yaw = float(n.get("max_yaw_dps", 12.0))
        r.near_tau = float(n.get("smooth", 0.3))  # EMA weight of the new value per tick
        # a textureless surface gives no flow, so it would look FAR: leave such scenes to `blank`
        r.near_max_blank = float(n.get("max_blank_frac", 0.5))
        r._blank_whole = (0.0, 0.0)
        r.yaw_rate_dps = 0.0
        r._near = {"L": 0.0, "R": 0.0}
        r.last_near = {"L": 0.0, "R": 0.0, "f_L": 0.0, "f_R": 0.0}
        return r

    def _near_feature(self, vf) -> None:
        f = {}
        r0, r1 = self.near_rows
        for e in "LR":
            g = vf.eyes[e].grids
            cols = slice(None)
            if self.near_outer:
                cols = slice(0, self.near_outer) if e == "L" else slice(self.cols - self.near_outer, self.cols)
            f[e] = float((g["ftb"][r0:r1, cols] - g["btf"][r0:r1, cols]).mean()) if "ftb" in g else 0.0
        target = {"L": 0.0, "R": 0.0}
        textured = max(self._blank_whole) <= self.near_max_blank
        if self.near_on and textured and min(f.values()) > self.near_min_flow and abs(self.yaw_rate_dps) < self.near_max_yaw:
            asym = (f["R"] - f["L"]) / (f["R"] + f["L"] + 1e-6)  # > 0: right side nearer
            target = {"R": float(np.clip(asym * self.near_gain, 0, 1)), "L": float(np.clip(-asym * self.near_gain, 0, 1))}
        for e in "LR":
            self._near[e] = (1 - self.near_tau) * self._near[e] + self.near_tau * target[e]
            vf.eyes[e].grids["near"] = np.full((self.rows, self.cols), self._near[e], np.float32)
        self.last_near = {"L": self._near["L"], "R": self._near["R"], "f_L": f["L"], "f_R": f["R"]}

    def encode(self, frame):
        vf = super().encode(frame)
        if frame is None or self.prev is None:
            self._near_feature(vf)
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
        whole = {"L": e[r0:r1, : self.cols], "R": e[r0:r1, self.cols : ncols]}
        fl, fr = (float((whole[k] < self.blank_thr).mean()) for k in "LR")
        self._blank_whole = (fl, fr)
        if self.blank_side_gain > 0:
            side_l = float(np.clip(0.5 + (fl - fr) * self.blank_side_gain, 0.0, 1.0))
            level = max(levels.values())
            levels = {"L": level * min(1.0, 2 * side_l), "R": level * min(1.0, 2 * (1 - side_l))}
            self.last_blank["side_L"] = side_l
        for eye in "LR":
            self.last_blank[eye] = levels[eye]
            vf.eyes[eye].grids["blank"] = np.full((self.rows, self.cols), levels[eye], np.float32)
        self._near_feature(vf)  # after blank: it needs this frame's textureless shares
        return vf


__all__ = ["EcpsRetina", "EyeFeatures"]
