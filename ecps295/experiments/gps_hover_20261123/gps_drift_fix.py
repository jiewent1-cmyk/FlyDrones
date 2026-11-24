"""Slow GPS anchor for a flow-based position hold (2026-11-23). No MAVLink in here: usable in SITL and on the companion.

The FC holds its EKF position (optical flow, drifts without bound). GPS position is noisy over seconds but its error
does not grow. d = (EKF - GPS) - (EKF - GPS at the start of the hold) is the flow drift plus the GPS wander. A low-pass
of d (time constant tau) is added to the hold target, so the drift is pulled out slowly and GPS noise is not followed.
"""
import math


class GpsDriftFix:
    def __init__(self, tau=60.0, ref_s=5.0, gate_m=1.5, hacc_max=2.5, sats_min=7, rate_max=0.10, max_m=3.0, dead_k=0.0, dead_m=0.0,
                 jump_m=0.0, jump_tau=5.0):
        self.tau, self.ref_s, self.gate_m = float(tau), float(ref_s), float(gate_m)
        self.hacc_max, self.sats_min, self.rate_max, self.max_m = float(hacc_max), int(sats_min), float(rate_max), float(max_m)
        # dead band = dead_m + dead_k * reported GPS accuracy: an apparent drift inside it is left alone (it may be GPS wander)
        self.dead_k, self.dead_m = float(dead_k), float(dead_m)
        # jump detector (0 = off): d moves by more than jump_m against its own jump_tau low-pass -> the GPS position
        # jumped (flow drift cannot do that in a hover); the jump is absorbed into the reference, the correction stays
        self.jump_m, self.jump_tau = float(jump_m), float(jump_tau)
        self.reset()

    def reset(self):
        self.t0 = self.t = None
        self._sum, self._n = [0.0, 0.0], 0
        self.ref = None                 # (EKF - GPS) at the start of the hold, mean over ref_s
        self.c = [0.0, 0.0]             # correction to add to the hold target (north, east), m
        self.d = [0.0, 0.0]
        self.want = [0.0, 0.0]
        self.state = "ref"              # ref | ok | bad_gps | gated
        self.n_gated = self.n_bad = self.n_jump = 0
        self.d_lp = None                # low-pass of d for the jump detector

    def update(self, t, ekf_n, ekf_e, gps_n, gps_e, fix_type=3, hacc=0.0, sats=99):
        """Call at each new GPS sample. Returns the correction (north, east)."""
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
        if math.hypot(self.want[0] - self.c[0], self.want[1] - self.c[1]) > self.gate_m:   # GPS jump: hold the correction
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
