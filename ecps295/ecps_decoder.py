"""ECPS295 read-out for the custom MiniFly (TechRoute §10 S2).

Same transparent linear read-out as flydrones.motor.MotorDecoder, plus behaviour the G4 flight tests showed was
missing (all switchable from YAML under decoder.ecps, so every change can be A/B tested):

  saccade escape   DNp03 (evasive saccades) or the giant fibre DNp01 trigger a turn AWAY from the threatened side,
                   with a short back-off, instead of a climb (the 1.0 m ceiling rules climbing out)
  caution          after an escape (or a brake >= on_brake) the cruise stays off for hold_s and then ramps back over
                   ramp_s, so a drone that stopped in front of a wall does not creep into it once looming goes quiet
  brake memory     slower decay of the DNp03/DNp01 brake (upstream x0.93 per tick ~ 0.5 s half-life)
  efference copy   throttle += k * forward command: cancels the "rising" flow the floor makes while flying forward
  yaw decoupling   throttle -= k * |DNg02_R - DNg02_L|: turning drives one DNg02 up and should not read as climb
  retreat (v4)     MDN ("moonwalker" descending neurons, backward walking in Drosophila) above threshold: back up
                   along the way the drone came, no turn, until MDN has been quiet for hold_s (min min_s), then one
                   saccade so it does not fly straight back. Driven by LCv (obstacle under the drone): turning in
                   place, which the saccade does, kept the drone over the box while the flow EKF height drifted

Nothing here changes the connectome; MiniFly variants live in my_minifly.py.
"""

from __future__ import annotations

import random

from flydrones.motor import FlightCommand, MotorDecoder
from flydrones.motor.command import AXES


class EcpsDecoder(MotorDecoder):
    def __init__(self, cfg: dict):
        super().__init__(cfg)
        e = (cfg.get("decoder", {}) or {}).get("ecps", {}) or {}
        self.brake_decay = float(e.get("brake_decay", 0.93))
        s = e.get("saccade", {}) or {}
        self.sacc_on = bool(s.get("enabled", False))
        self.sacc_terms = list(s.get("terms", ["DNp03_L", "DNp03_R"]))
        self.sacc_threshold = float(s.get("threshold_hz", 20.0))
        self.sacc_duration = float(s.get("duration_s", 1.2))
        self.sacc_yaw = float(s.get("yaw", 1.0))
        self.sacc_backoff = float(s.get("backoff", 0.3))
        self.sacc_backoff_s = float(s.get("backoff_s", 0.6))
        self.sacc_on_brake = float(s.get("on_brake", 0.0))  # a scare that stops the drone also turns it away
        self.sacc_refractory = float(s.get("refractory_s", 0.0))  # no brake-triggered saccade right after one
        self.sacc_keep_dir_s = float(s.get("keep_direction_s", 0.0))  # no side evidence: keep turning the same way
        c = e.get("caution", {}) or {}
        self.caution_hold = float(c.get("hold_s", 0.0))
        self.caution_ramp = float(c.get("ramp_s", 0.0))
        self.caution_on_brake = float(c.get("on_brake", 0.0))  # a brake this strong also starts the caution period
        r = e.get("retreat", {}) or {}
        self.ret_on = bool(r.get("enabled", False))
        self.ret_terms = list(r.get("terms", ["MDN_L", "MDN_R"]))
        self.ret_threshold = float(r.get("threshold_hz", 20.0))
        self.ret_speed = float(r.get("speed", 0.5))
        self.ret_min_s = float(r.get("min_s", 1.0))
        self.ret_hold_s = float(r.get("hold_s", 0.5))
        # blind: nothing looks backwards (S7b D1: a 4 s retreat backed into a box). After a capped retreat, MDN has to
        # go quiet before the next one, so the drone turns away instead of backing on in steps.
        self.ret_max_s = float(r.get("max_s", 1e9))
        self._ret_wait_quiet = False
        self.ret_then_saccade = bool(r.get("then_saccade", True))
        self._mdn = 0.0
        self._ret_start = -1.0
        self._ret_last = -1e9  # last tick MDN was above threshold
        self._ret_active = False
        self.k_fwd = float(e.get("throttle_per_forward", 0.0))
        self.k_yaw = float(e.get("throttle_yaw_decouple", 0.0))
        self._d03 = 0.0
        self._sacc_until = -1.0
        self._sacc_start = -1.0
        self._sacc_dir = 0.0
        self._side = 0.0  # smoothed right-minus-left threat (DNp03, DNp01), ~1 s memory
        self._caution_from = -1e9
        self._rng = random.Random(int(e.get("seed", 0)))
        self.escapes: list[tuple[float, str, float]] = []  # (time, trigger, direction)

    def _caution_factor(self) -> float:
        if self.caution_hold <= 0 and self.caution_ramp <= 0:
            return 1.0
        dt = self._elapsed - self._caution_from
        if dt < self.caution_hold:
            return 0.0
        if self.caution_ramp > 0 and dt < self.caution_hold + self.caution_ramp:
            return (dt - self.caution_hold) / self.caution_ramp
        return 1.0

    def update(self, rates: dict[str, float], dt: float) -> FlightCommand:
        self._elapsed += dt
        if not self.settled:
            for k, v in rates.items():
                self._settle_sum[k] = self._settle_sum.get(k, 0.0) + v
            self._settle_n += 1
            return FlightCommand.hover("settling: measuring resting rates")
        if self._settle_n and not self.baseline:
            self.baseline = {k: v / self._settle_n for k, v in self._settle_sum.items()}

        def d(g: str) -> float:
            return rates.get(g, 0.0) - self.baseline.get(g, 0.0)

        raw = {}
        for axis, spec in self.axes.items():
            raw[axis] = float(spec.get("gain", 0.0)) * sum(w * d(g) for g, w in spec.get("terms", {}).items())
        if self.k_yaw:
            raw["throttle"] -= self.k_yaw * abs(d("DNg02_R") - d("DNg02_L"))
        brake = sum(w * max(0.0, d(g)) for g, w in self.brake_terms.items())
        side_now = (d("DNp03_R") - d("DNp03_L")) + 0.5 * (d("DNp01_R") - d("DNp01_L"))
        self._side = 0.95 * self._side + 0.05 * side_now
        self._brake = max(brake, self._brake * self.brake_decay)
        if self.caution_on_brake and self._brake >= self.caution_on_brake:
            self._caution_from = max(self._caution_from, self._elapsed)
        raw["forward"] += self.cruise * max(0.0, 1.0 - self._brake) * self._caution_factor()

        a = self.smoothing
        for axis in AXES:
            v = max(-1.0, min(1.0, raw[axis]))
            self._state[axis] = (1 - a) * self._state[axis] + a * v
        out = {k: (0.0 if abs(v) < self.deadzone else v) for k, v in self._state.items()}
        if self.k_fwd:  # efference copy, after smoothing so it tracks the forward command
            out["throttle"] = max(-1.0, min(1.0, out["throttle"] + self.k_fwd * self._state["forward"]))
        cmd = FlightCommand(**out)

        # escape triggers: giant fibre (upstream) and, optionally, sustained DNp03
        trigger = None
        if self.escape_terms:
            self._gf = 0.6 * self._gf + 0.4 * max(rates.get(g, 0.0) for g in self.escape_terms)
            if self._gf >= self.escape_threshold:
                trigger = "DNp01"
        if self.sacc_on:
            self._d03 = 0.6 * self._d03 + 0.4 * max(rates.get(g, 0.0) for g in self.sacc_terms)
            if trigger is None and self._d03 >= self.sacc_threshold:
                trigger = "DNp03"
            refractory = self._elapsed - self._sacc_until < self.sacc_refractory
            if trigger is None and self.sacc_on_brake and brake >= self.sacc_on_brake and not refractory:
                trigger = "brake"  # this tick's brake, not the decaying memory (v1.2 re-triggered on it)
        # retreat (v4): MDN -> back up; ends with a saccade (handled as an ordinary trigger below)
        if self.ret_on:
            self._mdn = 0.6 * self._mdn + 0.4 * max(rates.get(g, 0.0) for g in self.ret_terms)
            capped = self._ret_active and self._elapsed - self._ret_start >= self.ret_max_s
            if self._mdn < self.ret_threshold:
                self._ret_wait_quiet = False
            if self._mdn >= self.ret_threshold and not capped and not self._ret_wait_quiet:
                if not self._ret_active:
                    self._ret_active, self._ret_start = True, self._elapsed
                    self._sacc_until = min(self._sacc_until, self._elapsed)  # retreat overrides a saccade in progress
                    self.escapes.append((round(self._elapsed, 2), "MDN", 0.0))
                self._ret_last = self._elapsed
            elif self._ret_active and (
                capped or (self._elapsed - self._ret_last > self.ret_hold_s and self._elapsed - self._ret_start >= self.ret_min_s)
            ):
                self._ret_active = False
                self._ret_wait_quiet = capped
                self._caution_from = self._elapsed
                if self.ret_then_saccade and self.sacc_on:
                    trigger = "retreat"
        if self._ret_active:
            cmd.escape = True
            cmd.forward = -self.ret_speed
            cmd.yaw = 0.0
            cmd.throttle = 0.0
            cmd.note = "retreat (MDN)"
            return cmd

        busy = self._elapsed <= max(self._escape_until, self._sacc_until)
        if trigger and not busy:
            if self.sacc_on:
                # turn away from the threatened side: right-side threat (DNp03_R / DNp01_R) -> yaw left (negative)
                side = side_now + self._side
                recent = self._sacc_dir != 0.0 and self._elapsed - self._sacc_until < self.sacc_keep_dir_s
                if abs(side) > 2:
                    self._sacc_dir = -1.0 if side > 0 else 1.0
                elif not recent:
                    self._sacc_dir = self._rng.choice((-1.0, 1.0))
                self._sacc_start, self._sacc_until = self._elapsed, self._elapsed + self.sacc_duration
                self._caution_from = self._sacc_until
                self._brake = max(self._brake, 1.0)
            else:
                self._escape_until = self._elapsed + self.escape_duration
            self.last_escape_t = self._elapsed
            self.escapes.append((round(self._elapsed, 2), trigger, self._sacc_dir if self.sacc_on else 0.0))

        if self._elapsed <= self._sacc_until:
            cmd.escape = True
            cmd.yaw = self._sacc_dir * self.sacc_yaw
            cmd.forward = -self.sacc_backoff if self._elapsed - self._sacc_start < self.sacc_backoff_s else 0.0
            cmd.throttle = 0.0
            cmd.note = f"saccade {'right' if self._sacc_dir > 0 else 'left'}"
        elif self._elapsed <= self._escape_until:
            cmd.escape = True
            cmd.forward = 0.0
            if self.escape_mode == "climb":
                cmd.throttle = self.escape_strength
            elif self.escape_mode == "drop":
                cmd.throttle = -self.escape_strength
            cmd.note = f"giant fiber escape ({self.escape_mode})"
        return cmd
