"""Gazebo side of the target-tracking test (2026-10-18, run_g4.py --track <world>_mk.marker.json).

The Gazebo drone camera is a 120 deg equidistant fisheye (model.sdf front_camera); the companion's MarkerDetector
assumes a pinhole camera (hw config marker.hfov_deg), so each colour frame (cam_bridge 640x480 rgb stream) is remapped
to a 640x480 pinhole view with that hfov first. Detection runs synchronously at 10 Hz of flight time inside
run_g4's tick (sim-clock pacing keeps the brain's time right), stamped with time.monotonic() like the companion's
MarkerThread so TargetTrack's pose history (tel.t) matches. Truth (SIM_STATE) scores range / bearing / final pose.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import cv2
import numpy as np

from ecps_target import MarkerDetector
from shm_frames import ShmFrame

FISH_HFOV = 2.09440  # model.sdf front_camera horizontal_fov, equidistant lens, scale_to_hfov


def fisheye_to_pinhole_maps(w: int, h: int, hfov_pin_deg: float, fish_hfov: float = FISH_HFOV):
    fp = (w / 2.0) / math.tan(math.radians(hfov_pin_deg) / 2.0)
    ff = (w / 2.0) / (fish_hfov / 2.0)
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    u, v = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    x, y = (u - cx) / fp, (v - cy) / fp
    rr = np.hypot(x, y)
    th = np.arctan(rr)
    s = np.where(rr > 1e-9, ff * th / np.maximum(rr, 1e-9), ff)
    return (cx + x * s).astype(np.float32), (cy + y * s).astype(np.float32)


class GzMarker:
    def __init__(self, spec_path: str, shm: str, mcfg: dict | None = None, out_dir: str = "."):
        m = mcfg or {}
        self.spec = json.load(open(spec_path))
        self.hfov = float(m.get("hfov_deg", 70.0))
        self.pitch = float(m.get("pitch_deg", 12.0))
        self.det = MarkerDetector(m.get("dictionary", "4x4_50"), self.spec["id"], self.spec["size_m"], self.hfov, self.pitch)
        self.frame = ShmFrame(shm)
        self.maps = None
        self.out = Path(out_dir)
        self.hz = float(m.get("hz", 10.0))
        self._next = 0.0
        self._last = (None, 0.0)
        self.started = False
        self.t_start = None
        self.frames = self.hits = 0
        self.det_ms: list[float] = []
        self.saved = 0
        self.cur: dict = {}

    def latest(self):
        return self._last

    # ------------------------------------------------------------------ per tick
    def _truth(self, drone):
        tru = drone.truth_ned()
        s = drone.m.messages.get("SIM_STATE") if drone.m is not None else None
        if tru is None or s is None:
            return None
        return tru, math.degrees(s.yaw)

    def step(self, t: float, drone) -> None:
        self.cur = {}
        tr = self._truth(drone)
        if tr is not None:
            (n, e, up), yaw = tr
            dn, de = self.spec["n"] - n, self.spec["e"] - e
            self.cur["true_range"] = round(math.hypot(dn, de), 3)
            self.cur["true_bearing"] = round((math.degrees(math.atan2(de, dn)) - yaw + 180) % 360 - 180, 2)
            self.cur["true_yaw"] = round(yaw, 2)
        if t < self._next:
            return
        self._next = t + 1.0 / self.hz
        if not self.frame.read() or self.frame.img is None:
            return
        img = self.frame.img
        h, w = img.shape[:2]
        if self.maps is None:
            self.maps = fisheye_to_pinhole_maps(w, h, self.hfov)
        c0 = time.perf_counter()
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) if img.ndim == 3 else img
        pin = cv2.remap(gray, self.maps[0], self.maps[1], cv2.INTER_LINEAR)
        dets = self.det.detect(pin, time.monotonic())
        self.det_ms.append((time.perf_counter() - c0) * 1000)
        self.frames += 1
        d = dets[0] if dets else None
        self._last = (d, time.monotonic())
        if d is not None:
            self.hits += 1
            self.cur.update(mk_range=round(d.range_m, 3), mk_bearing=round(d.bearing_deg, 2))
        tb = self.cur.get("true_bearing")
        if self.saved < 6 and (d is not None or (tb is not None and abs(tb) < 20 and self.saved < 3)):  # frames for the record
            cv2.imwrite(str(self.out / f"trk_pin_{self.saved}.png"), pin)
            cv2.imwrite(str(self.out / f"trk_fish_{self.saved}.jpg"), img[..., ::-1] if img.ndim == 3 else img)
            self.saved += 1

    def row(self, pilot) -> dict:
        tg = pilot.target
        r = {"trk": tg.state, "mk_range": None, "mk_bearing": None, "true_range": None, "true_bearing": None, "true_yaw": None}
        r.update(self.cur)
        r["est_n"], r["est_e"] = (round(tg.est[0], 3), round(tg.est[1], 3)) if tg.est else (None, None)
        r["goal_n"], r["goal_e"] = (round(tg.goal[0], 3), round(tg.goal[1], 3)) if tg.goal else (None, None)
        return r

    def done(self, pilot, t: float, hold_s: float) -> bool:
        tg = pilot.target
        if tg.state == "ARRIVED" and t - tg.t_state > hold_s:
            return True
        return tg.state == "FAILED" and t - tg.t_state > 3.0

    # ------------------------------------------------------------------ scoring
    def summary(self, pilot, rows: list[dict], mode: str) -> dict:
        tg = pilot.target
        fl = [r for r in rows if r["phase"] == mode]
        log = list(tg.log)
        first = lambda key: next((t for t, s in log if s.startswith(key)), None)  # noqa: E731
        out = {"trk_marker": self.spec, "trk_log": log, "trk_final_state": tg.state, "trk_t_start": self.t_start,
               "trk_t_approach": first("target: approach"), "trk_t_arrived": first("target: arrived"),
               "trk_frames": self.frames, "trk_hits": self.hits, "trk_escapes_ignored": getattr(tg, "escapes_ignored", None),
               "trk_det_ms_p50": round(float(np.median(self.det_ms)), 1) if self.det_ms else None}
        errs = [(r["mk_range"] - r["true_range"], r["mk_bearing"] - r["true_bearing"], r["true_range"]) for r in fl
                if r.get("mk_range") is not None and r.get("true_range") is not None]
        if errs:
            ar = np.abs([x[0] for x in errs])
            rel = np.abs([x[0] / max(x[2], 0.1) for x in errs])
            ab = np.abs([(x[1] + 180) % 360 - 180 for x in errs])
            out.update(trk_range_err_med_m=round(float(np.median(ar)), 3), trk_range_err_rel_med=round(float(np.median(rel)), 3),
                       trk_range_err_p90_m=round(float(np.percentile(ar, 90)), 3), trk_bearing_err_med_deg=round(float(np.median(ab)), 2),
                       trk_det_bias_m=round(float(np.median([x[0] for x in errs])), 3))
        end = [r for r in fl if r.get("true_range") is not None][-10:]
        if end:
            d = float(np.median([r["true_range"] for r in end]))
            fe = float(np.median([abs(r["true_bearing"]) for r in end]))
            out.update(trk_end_dist_m=round(d, 3), trk_end_facing_err_deg=round(fe, 1))
        if tg.est is not None and fl:
            # the estimate lives in the EKF frame; compare after removing the EKF's own position error at the end
            r = fl[-1]
            off_n, off_e = r["ekf_north"] - r["north"], r["ekf_east"] - r["east"]
            out["trk_est_err_m"] = round(math.hypot(tg.est[0] - off_n - self.spec["n"], tg.est[1] - off_e - self.spec["e"]), 3)
            out["trk_est_err_raw_m"] = round(math.hypot(tg.est[0] - self.spec["n"], tg.est[1] - self.spec["e"]), 3)
        arrived = out["trk_t_arrived"] is not None
        out["trk_pass"] = bool(arrived and tg.state == "ARRIVED" and 0.6 <= out.get("trk_end_dist_m", 0) <= 1.5
                               and out.get("trk_end_facing_err_deg", 99) < 20)
        return out
