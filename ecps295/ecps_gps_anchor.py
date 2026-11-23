"""GPS anchor for the flow position hold (2026-11-23): GPS bounds the drift of the optical-flow EKF.

The FC keeps flying on optical flow + compass (EK3_SRC1_POSXY 0, VELXY 5): over seconds flow is far better than GPS.
But the flow position drifts without bound, and the GPS error, however large, does not grow. So the companion compares
the two and moves the hold target slowly:
    d = (EKF - GPS) - (EKF - GPS when the reference was taken)      flow drift plus GPS wander since then
    want = d shortened by the dead band (dead_k x reported accuracy): an apparent drift that the GPS accuracy could
           explain is left alone, only the part beyond it is pulled out
    c -> want with time constant tau, rate limited; c is added to the hold target (the same path as the landmark fix)
The FC's EKF never sees the GPS position: a GPS jump or outage cannot move the estimate or make the EKF switch sources.

SITL study (ArduCopter 4.7.0, 15 min holds, 3 flow noise levels x 3 GPS qualities x 10 repeats, report
项目文档/09_台架测试/gps_hover_20261123/report.md): fusing the GPS position in the FC is the best choice with a good GPS
(0.9 m against 1.8-3.2 m with noisy flow) and by far the worst with a poor one (4 m against 0.7 m) and with a GPS jump
or outage (3.6 m, 6 m). This anchor is the safe middle: 3 m GPS jumps absorbed in 80 / 80 runs, no false jump in 270
fifteen-minute runs, outage = correction held. Its gain is modest and not statistically established (maximum offset
0.6-0.8 x flow alone with a good GPS, about equal otherwise); with good flow and a poor GPS it can be worse (about
1.0 m against 0.7 m). Repeats of the same run differ by 0.3 m (median), so tau 60 against 120 s is not resolved.

Safety behaviour:
  - GPS not good enough (fix type < 3, accuracy > hacc_max_m, satellites < sats_min) or no new sample: c is held.
  - GPS jump (d leaves its own jump_tau low-pass by more than jump_m; flow drift cannot do that in a hover): the jump
    is absorbed into the reference, c is not moved.
  - c is rate limited (rate_max_mps) and limited to max_m.
  - Landmark fixes are more precise: each new landmark fix re-bases the anchor (base = landmark offset, c = 0), so the
    GPS only bounds the drift since the last marker was seen.
  - Not flying: everything is reset; the reference is taken again once the next flight is above min_alt_m.
"""

from __future__ import annotations

import math


class GpsDriftFix:
    """The filter alone (no MAVLink, no config): update() per GPS sample, returns the correction (north, east)."""

    def __init__(self, tau=60.0, ref_s=5.0, gate_m=1.5, hacc_max=2.5, sats_min=7, rate_max=0.10, max_m=3.0, dead_k=1.0, dead_m=0.0,
                 jump_m=1.5, jump_tau=5.0):
        self.tau, self.ref_s, self.gate_m = float(tau), float(ref_s), float(gate_m)
        self.hacc_max, self.sats_min, self.rate_max, self.max_m = float(hacc_max), int(sats_min), float(rate_max), float(max_m)
        self.dead_k, self.dead_m = float(dead_k), float(dead_m)
        self.jump_m, self.jump_tau = float(jump_m), float(jump_tau)
        self.reset()

    def reset(self) -> None:
        self.t0 = self.t = None
        self._sum, self._n = [0.0, 0.0], 0
        self.ref = None  # (EKF - GPS) when the reference was taken, mean over ref_s
        self.c = [0.0, 0.0]  # correction to add to the hold target (north, east), m
        self.d = [0.0, 0.0]
        self.d_lp = None  # low-pass of d for the jump detector
        self.want = [0.0, 0.0]
        self.state = "ref"  # ref | ok | bad_gps | gated
        self.n_gated = self.n_bad = self.n_jump = 0

    def rebase(self) -> None:
        """Start again from the present: d = 0, c = 0 (the caller has a better fix for everything up to now)."""
        if self.ref is not None:
            self.ref = (self.ref[0] + self.d[0], self.ref[1] + self.d[1])
        self.c, self.d, self.want, self.d_lp = [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], None

    def update(self, t, ekf_n, ekf_e, gps_n, gps_e, fix_type=3, hacc=0.0, sats=99):
        dt = 0.0 if self.t is None else max(0.0, t - self.t)
        self.t = t
        if fix_type < 3 or hacc > self.hacc_max or sats < self.sats_min:
            self.state, self.n_bad = "bad_gps", self.n_bad + 1
            return tuple(self.c)
        if self.ref is None:
            if self.t0 is None:
                self.t0 = t
            self._sum[0] += ekf_n - gps_n
            self._sum[1] += ekf_e - gps_e
            self._n += 1
            if t - self.t0 >= self.ref_s:
                self.ref = (self._sum[0] / self._n, self._sum[1] / self._n)
            self.state = "ref"
            return tuple(self.c)
        self.d = [ekf_n - gps_n - self.ref[0], ekf_e - gps_e - self.ref[1]]
        if self.jump_m > 0:
            if self.d_lp is None:
                self.d_lp = list(self.d)
            jn, je = self.d[0] - self.d_lp[0], self.d[1] - self.d_lp[1]
            if math.hypot(jn, je) > self.jump_m:
                self.ref = (self.ref[0] + jn, self.ref[1] + je)
                self.d = list(self.d_lp)
                self.n_jump += 1
            else:
                b = 1.0 - math.exp(-dt / self.jump_tau)
                self.d_lp = [self.d_lp[0] + b * jn, self.d_lp[1] + b * je]
        dead, n = self.dead_m + self.dead_k * hacc, math.hypot(*self.d)
        self.want = [0.0, 0.0] if n <= dead else [self.d[0] * (1 - dead / n), self.d[1] * (1 - dead / n)]
        if math.hypot(self.want[0] - self.c[0], self.want[1] - self.c[1]) > self.gate_m:  # too far to be drift: hold
            self.state, self.n_gated = "gated", self.n_gated + 1
            return tuple(self.c)
        a = 1.0 - math.exp(-dt / self.tau)
        step = [a * (self.want[0] - self.c[0]), a * (self.want[1] - self.c[1])]
        lim, n = self.rate_max * dt, math.hypot(*step)
        if n > lim > 0:
            step = [step[0] * lim / n, step[1] * lim / n]
        self.c = [self.c[0] + step[0], self.c[1] + step[1]]
        n = math.hypot(*self.c)
        if n > self.max_m:
            self.c = [self.c[0] * self.max_m / n, self.c[1] * self.max_m / n]
        self.state = "ok"
        return tuple(self.c)


class GpsAnchor:
    """Config + plumbing around GpsDriftFix. EcpsPilot calls step() every tick; `sample` is drone.gps_sample():
    None or {"stamp": any value that changes with each new GPS message, "lat": deg, "lon": deg, "fix": fix type,
    "hacc": m, "sats": n}. offset = base + c is EKF minus world (north, east), as LandmarkFix.offset."""

    R_EARTH = 6378137.0

    def __init__(self, cfg: dict | None = None):
        c = cfg or {}
        self.enabled = bool(c.get("enabled", False))
        self.stale_s = float(c.get("stale_s", 2.0))  # no new GPS sample for this long: state "no_gps", c held
        self.min_alt = float(c.get("min_alt_m", 0.5))  # the reference is not taken below this height (take-off)
        self.hacc_floor = float(c.get("hacc_floor_m", 0.5))  # a receiver that reports better than this is not believed
        self.fix = GpsDriftFix(tau=c.get("tau_s", 60.0), ref_s=c.get("ref_s", 5.0), gate_m=c.get("gate_m", 1.5),
                               hacc_max=c.get("hacc_max_m", 2.5), sats_min=c.get("sats_min", 7),
                               rate_max=c.get("rate_max_mps", 0.10), max_m=c.get("max_m", 3.0), dead_k=c.get("dead_k", 1.0),
                               dead_m=c.get("dead_m", 0.0), jump_m=c.get("jump_m", 1.5), jump_tau=c.get("jump_tau_s", 5.0))
        self.base = (0.0, 0.0)  # landmark offset at the last landmark fix
        self.offset = (0.0, 0.0)
        self.state = "off" if not self.enabled else "idle"
        self.samples = 0
        self._stamp = None
        self._t_sample: float | None = None
        self._ll0: tuple[float, float] | None = None
        self._lm_fixes = 0
        self.log: list[tuple[float, str]] = []

    def reset(self) -> None:
        self.fix.reset()
        self.base, self.offset = (0.0, 0.0), (0.0, 0.0)
        self._ll0, self._lm_fixes = None, 0
        self.state = "idle"

    def _note(self, t: float, what: str, on_event=None) -> None:
        self.log.append((round(t, 2), what))
        if on_event is not None:
            on_event(t, what)

    def step(self, now: float, ekf_n: float, ekf_e: float, sample: dict | None, flying: bool, t: float = 0.0,
             lm_offset: tuple[float, float] | None = None, lm_fixes: int = 0, on_event=None,
             alt_m: float | None = None) -> tuple[float, float]:
        """now: monotonic seconds; ekf_n / ekf_e: the FC's raw position; lm_offset / lm_fixes: LandmarkFix.offset and
        .fixes when the landmark fix is on and has fixes. Returns the offset to use (EKF minus world)."""
        if not self.enabled:
            return self.offset
        if not flying:
            if self.state != "idle":
                self.reset()
            return self.offset
        if self.fix.ref is None and self.fix.t0 is None and alt_m is not None and alt_m < self.min_alt:
            return self.offset  # still taking off: no reference yet
        if lm_offset is not None and lm_fixes != self._lm_fixes:  # a new landmark fix: it knows better
            self._lm_fixes = lm_fixes
            self.base = (float(lm_offset[0]), float(lm_offset[1]))
            self.fix.rebase()
        new = sample is not None and sample.get("stamp") != self._stamp
        if new:
            self._stamp, self._t_sample = sample["stamp"], now
            lat, lon = float(sample["lat"]), float(sample["lon"])
            if self._ll0 is None:
                self._ll0 = (lat, lon)
            gn = math.radians(lat - self._ll0[0]) * self.R_EARTH
            ge = math.radians(lon - self._ll0[1]) * self.R_EARTH * math.cos(math.radians(self._ll0[0]))
            jumps, before = self.fix.n_jump, self.fix.state
            self.fix.update(now, ekf_n, ekf_e, gn, ge, int(sample.get("fix", 0)),
                            max(self.hacc_floor, float(sample.get("hacc", 99.0))), int(sample.get("sats", 0)))
            self.samples += 1
            self.state = self.fix.state
            if self.fix.n_jump != jumps:
                self._note(t, f"gps anchor: GPS jump absorbed ({self.fix.n_jump})", on_event)
            if before == "ref" and self.fix.state != "ref":
                self._note(t, "gps anchor: reference taken", on_event)
        elif self._t_sample is None or now - self._t_sample > self.stale_s:
            self.state = "no_gps"
        self.offset = (self.base[0] + self.fix.c[0], self.base[1] + self.fix.c[1])
        return self.offset

    def summary(self) -> dict:
        return {"enabled": self.enabled, "state": self.state, "samples": self.samples,
                "offset": [round(self.offset[0], 3), round(self.offset[1], 3)],
                "c": [round(self.fix.c[0], 3), round(self.fix.c[1], 3)], "jumps": self.fix.n_jump, "gated": self.fix.n_gated,
                "bad_gps": self.fix.n_bad, "log": self.log[-20:]}
