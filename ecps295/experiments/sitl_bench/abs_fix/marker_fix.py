#!/usr/bin/env python3
"""Companion-side marker position fix -> ArduCopter 4.7 (abs_fix, 2026-11-19).

Portable part: no SITL code in here. abs_sim.py synthesises marker observations from SITL truth and calls these classes
exactly the way the Orange Pi should call them with real ArUco fixes.

Frames and units
  "field frame" = the EKF local frame used by the mission:
     origin = EKF origin (take-off spot, SET_GPS_GLOBAL_ORIGIN there), x = north of the EKF, y = east, metres.
     Without a compass the EKF starts with yaw 0, so EKF north = the vehicle nose direction at power-up.
     => place the vehicle on the take-off spot with its nose along the field x axis, and survey the markers in that frame.
     With a compass, EKF north = magnetic north: rotate the surveyed marker map by the field azimuth once.
  obs = (t_capture_s, x, y) vehicle position in the field frame computed from one marker detection
        (marker map position - R(yaw) * marker-relative vector). All times are companion monotonic seconds.

Two ways to hand the fix to the FC (pick one; see report.md):
  ExtNavSender   ODOMETRY (MAV_FRAME_LOCAL_FRD / MAV_FRAME_BODY_FRD) -> AP_VisualOdom -> EKF3 ExtNav position.
                 FC: VISO_TYPE 1, EK3_SRC1_POSXY 6, EK3_SRC1_VELXY 5 (flow), VISO_QUAL_MIN 1, VISO_DELAY_MS = capture->send latency.
                 Sends quality 100 for real fixes and a quality-0 keep-alive at 5 Hz otherwise (keeps VisOdom 'healthy' for
                 the pre-arm check, but quality < VISO_QUAL_MIN is not passed to the EKF: AP_VisualOdom_MAV.cpp handle_pose_estimate).
                 time_usec = SEND time (not capture time): the FC's JitterCorrection maps the stamp to the arrival time anyway,
                 the real latency must be in VISO_DELAY_MS (max 250).
  ExtPosResetter COMMAND_INT MAV_CMD_EXTERNAL_POSITION_ESTIMATE (43003) -> AP_AHRS::handle_external_position_estimate ->
                 NavEKF3_core::setLatLng (hard NE reset). Needs AP_AHRS_POSITION_RESET_ENABLED in the firmware.
                 One reset per sighting: average >= min_n gated fixes as offsets to the EKF position at capture time.
                 param1 = send time (s), param2 = send - capture (s), param3 = accuracy (m), x/y = lat/lon 1e7, z = NaN.
Both use OutlierGate first.
"""
from __future__ import annotations

import math
from collections import deque

LATLON_TO_M = 0.011131884502145034  # AP Location: metres per 1e-7 degree


def ne_to_latlon(lat0_e7, lon0_e7, n, e):
    """EKF NE offset (m) from the EKF origin -> (lat_e7, lon_e7), same flat-earth maths as AP Location::offset."""
    dlat = n / LATLON_TO_M
    lat_mid = (lat0_e7 + dlat / 2) * 1e-7
    scale = max(0.01, math.cos(math.radians(lat_mid)))
    return int(round(lat0_e7 + dlat)), int(round(lon0_e7 + e / LATLON_TO_M / scale))


class EkfHistory:
    """Recent FC position estimates (LOCAL_POSITION_NED), to pair a fix with the EKF position at capture time."""

    def __init__(self, keep_s=3.0):
        self.buf = deque()
        self.keep = keep_s

    def add(self, t, x, y, vx, vy):
        self.buf.append((t, x, y, vx, vy))
        while self.buf and self.buf[0][0] < t - self.keep:
            self.buf.popleft()

    def at(self, t):
        """EKF (x, y, vx, vy) at time t (nearest sample, extrapolated with its velocity). None if no data."""
        if not self.buf:
            return None
        s = min(self.buf, key=lambda q: abs(q[0] - t))
        dt = t - s[0]
        return s[1] + s[3] * dt, s[2] + s[4] * dt, s[3], s[4]


class OutlierGate:
    """Reject single-frame misdetections before anything goes to the FC.

    - After a gap > reacq_s (or at start) a fix is only accepted once n_confirm consecutive fixes agree within cons_r
      (their spread around the median); the confirmed batch is then released.
    - While tracking, a fix must lie within cons_r of the last accepted fix propagated with the EKF velocity.
      n_confirm consecutive mutually consistent rejections re-acquire (marker map switch / real jump).
    This alone does not catch a *persistent* wrong fix (wrong id in the map, map error, a reflection).
    Optional second stage (ekf_hist given, "gate v2"): a fix is only passed on if its distance to the FC's own position
    estimate at capture time is <= min(innov_max, innov_base + drift_rate * (time since the last fix that passed)).
    The allowance grows with the time the FC has been dead-reckoning on flow (opt_flow2: ~0.8 m / 120 s worst case),
    so a 1 m jump while the marker is tracked continuously is refused, but a real 0.5 m drift after a 30 s gap is not.
    """

    def __init__(self, cons_r=0.3, n_confirm=3, reacq_s=1.0, ekf_hist=None, innov_base=0.3, drift_rate=0.02, innov_max=1.5):
        self.cons_r, self.n, self.reacq = cons_r, n_confirm, reacq_s
        self.last = None  # (t, x, y)
        self.cand = []
        self.n_rej = 0
        self.hist, self.ib, self.dr, self.imax = ekf_hist, innov_base, drift_rate, innov_max
        self.t_ok = None
        self.n_innov_rej = 0

    def _innov_ok(self, fixes):
        if self.hist is None:
            return fixes
        out = []
        for f in fixes:
            e = self.hist.at(f[0])
            if e is None:
                continue
            allowed = self.imax if self.t_ok is None else min(self.imax, self.ib + self.dr * (f[0] - self.t_ok))
            if math.hypot(f[1] - e[0], f[2] - e[1]) <= allowed:
                out.append(f)
                self.t_ok = f[0]
            else:
                self.n_innov_rej += 1
        return out

    def _consistent(self, pts):
        mx = sorted(p[1] for p in pts)[len(pts) // 2]
        my = sorted(p[2] for p in pts)[len(pts) // 2]
        return all(math.hypot(p[1] - mx, p[2] - my) <= self.cons_r for p in pts)

    def push(self, obs, vel=(0.0, 0.0)):
        """obs = (t, x, y). Returns the list of accepted fixes (possibly empty)."""
        return self._innov_ok(self._push(obs, vel))

    def _push(self, obs, vel):
        t, x, y = obs
        if self.last is None or t - self.last[0] > self.reacq:
            self.cand.append(obs)
            self.cand = self.cand[-self.n:]
            if len(self.cand) == self.n and self._consistent(self.cand):
                out, self.cand = self.cand, []
                self.last = out[-1]
                return out
            return []
        dt = t - self.last[0]
        px, py = self.last[1] + vel[0] * dt, self.last[2] + vel[1] * dt
        if math.hypot(x - px, y - py) <= self.cons_r:
            self.last, self.cand, self.n_rej = obs, [], 0
            return [obs]
        self.n_rej += 1
        self.cand.append(obs)
        self.cand = self.cand[-self.n:]
        if len(self.cand) == self.n and self._consistent(self.cand):
            out, self.cand = self.cand, []
            self.last, self.n_rej = out[-1], 0
            return out
        return []


class ExtNavSender:
    """ODOMETRY -> EKF3 ExtNav position (EK3_SRC1_POSXY 6)."""

    def __init__(self, mav, sigma=0.1, keepalive_hz=5.0, yaw_sigma=None, reset_gap_s=None):
        self.mav, self.sigma, self.ka = mav, sigma, 1.0 / keepalive_hz
        self.yaw_sigma = yaw_sigma  # rad, 1-sigma of the yaw in q (EK3_SRC1_YAW 6 fuses it; floored by VISO_YAW_M_NSE and 5 deg)
        self.reset_gap = reset_gap_s  # optional: bump reset_counter on the first fix after a gap -> EKF hard NE reset
        self.reset_counter = 0
        self.t_last = -1e9
        self.t_last_fix = -1e9
        self.last_xyz = (0.0, 0.0, 0.0)
        self.n_sent = 0

    @staticmethod
    def _quat(roll, pitch, yaw):
        cr, sr, cp, sp, cy, sy = (math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2),
                                  math.cos(yaw / 2), math.sin(yaw / 2))
        return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]

    def _send(self, t_send, x, y, z, att, quality):
        cov = [0.0] * 21
        cov[0] = self.sigma ** 2  # EKF uses sqrt(cov[0]+cov[6]+cov[11]) as the per-axis 1-sigma (floored by VISO_POS_M_NSE)
        cov[15] = cov[18] = 0.0
        cov[20] = self.yaw_sigma ** 2 if self.yaw_sigma else 0.3 ** 2
        nan = float("nan")
        self.mav.odometry_send(int(t_send * 1e6), 20, 12, x, y, z, self._quat(*att), nan, nan, nan, nan, nan, nan,
                               cov, [nan] * 21, self.reset_counter, 0, quality)  # 20 = MAV_FRAME_LOCAL_FRD, 12 = MAV_FRAME_BODY_FRD

    def fix(self, t_send, x, y, z, att):
        """att = (roll, pitch, yaw) rad; yaw only matters with EK3_SRC1_YAW 6 (then: marker-derived yaw in the field frame).
        Returns True if this fix carried a new reset_counter (EKF resets NE position to it instead of fusing it)."""
        reset = self.reset_gap is not None and t_send - self.t_last_fix > self.reset_gap
        if reset:
            self.reset_counter = (self.reset_counter + 1) % 256
        self._send(t_send, x, y, z, att, 100)
        self.t_last, self.t_last_fix, self.last_xyz = t_send, t_send, (x, y, z)
        self.n_sent += 1
        return reset

    def tick(self, t_now, att):
        if t_now - self.t_last >= self.ka:
            self._send(t_now, *self.last_xyz, att, 0)  # quality 0 < VISO_QUAL_MIN 1: health only, not fused
            self.t_last = t_now


class ExtPosResetter:
    """MAV_CMD_EXTERNAL_POSITION_ESTIMATE -> EKF3 setLatLng (hard reset of NE position, AID_RELATIVE flow nav stays)."""

    def __init__(self, mav, origin_e7, ekf_hist, min_n=5, window_s=1.5, spread_max=0.25, min_corr=0.05, cooldown_s=3.0,
                 sigma=0.1, target=(1, 1)):
        self.mav, self.origin, self.hist = mav, origin_e7, ekf_hist
        self.min_n, self.win, self.spread, self.min_corr, self.cool = min_n, window_s, spread_max, min_corr, cooldown_s
        self.sigma, self.target = sigma, target
        self.buf = []  # (t_capture, dx, dy) offsets fix - EKF
        self.t_reset = -1e9
        self.events = []

    def push(self, obs, t_now):
        """obs = accepted fix (t_capture, x, y). Returns a dict describing a reset if one was sent."""
        t, x, y = obs
        if t < self.t_reset + 0.5:  # EKF position before/around the last reset: offsets would be stale
            return None
        e = self.hist.at(t)
        if e is None:
            return None
        self.buf.append((t, x - e[0], y - e[1]))
        self.buf = [b for b in self.buf if b[0] >= t - self.win]
        if len(self.buf) < self.min_n or t_now - self.t_reset < self.cool:
            return None
        dx = sorted(b[1] for b in self.buf)[len(self.buf) // 2]
        dy = sorted(b[2] for b in self.buf)[len(self.buf) // 2]
        inl = [b for b in self.buf if math.hypot(b[1] - dx, b[2] - dy) <= self.spread]
        if len(inl) < self.min_n:
            return None
        mx = sum(b[1] for b in inl) / len(inl)
        my = sum(b[2] for b in inl) / len(inl)
        self.buf = []
        if math.hypot(mx, my) < self.min_corr:
            self.t_reset = t_now  # nothing worth a reset; still wait a cooldown
            return {"sent": False, "dx": mx, "dy": my, "n": len(inl)}
        t_ref = inl[-1][0]
        e = self.hist.at(t_ref)
        n, ea = e[0] + mx, e[1] + my
        lat, lon = ne_to_latlon(self.origin[0], self.origin[1], n, ea)
        acc = max(0.05, self.sigma / math.sqrt(len(inl)))
        self.mav.command_int_send(self.target[0], self.target[1], 0, 43003, 0, 0, float(t_now), float(t_now - t_ref), acc,
                                  float("nan"), lat, lon, float("nan"))  # frame 0 = MAV_FRAME_GLOBAL, z must be NaN
        self.t_reset = t_now
        ev = {"sent": True, "dx": mx, "dy": my, "n": len(inl), "t_ref": t_ref, "n_ekf": n, "e_ekf": ea}
        self.events.append(ev)
        return ev
