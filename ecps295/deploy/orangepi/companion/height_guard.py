"""Height guard (2026-11-21): the barometer vouches for the downward rangefinder.

Why: with the rangefinder as the EKF height source (EK3_SRC1_POSZ 2) a reading that is valid but wrong is believed.
A sensor that under-reads from take-off (the VL53L0X in daylight, 2026-11-19: 0.07-0.24 m while held at 1 m and more)
makes the FC fly at 4 m while it reports 2 m, and neither the EKF gates nor the fence notice (SITL val4 / val6).

What: baro_range_monitor.BaroRangeMonitor compares the two (SITL val7, 576 runs). On a mismatch this guard
  1. tells the pilot (STATUSTEXT, console event),
  2. switches the EKF to the source set that takes its height from the barometer (MAV_CMD_SET_EKF_SOURCE_SET),
  3. half a second later asks the FC to LAND,
and latches until the FC is disarmed. In SITL the true height then stayed at 1.3-1.8 m instead of 4.0 m and the
landing was at the normal ~0.5 m/s.

It acts on its own path, like the console's emergency stop: whether or not the companion is in control, because the
fault is in the FC's own height and an AUTO mission flown by the FC is exactly the case to protect. It writes nothing
while the drone is muted (bench mode).

Self-check: a guard that has silently stopped is worse than none (SITL val7 step 5: monitor stopped or pressure stream
lost = no action, 4.0 m again). While the FC is armed, missing RANGEFINDER / SCALED_PRESSURE / ATTITUDE for longer
than blind_s is reported ("height guard blind") on STATUSTEXT and in status()["blind"].

Defaults = level criterion only (rate criterion off, R = 99): 0 false alarms in 220 SITL flights that included baro
drift of 1.2 m/min, 2.5x baro noise and +-0.5 m baro steps; the rate criterion alarmed on the steps. The SITL
barometer has no prop wash and no ground effect: check e (status()["e"]) on real flight logs before relying on E.

Additions of 2026-11-22 (safety ablation, 9796 SITL flights; report 项目文档/09_台架测试/ECPS295_安全性消融_补充实验_20261121.md):
  oor_s         the range is invalid (out of limits) for this long while clearly airborne: trip. Before, an out-of-range
                reading was "not judged" and a rangefinder that went out of range for good was never caught (10/10
                unsafe). 0/200 false trips.
  skip_invalid  invalid readings stay out of the monitor's low-pass. Without it a 0.2 s out-of-range blip tripped the
                guard (10/10).
  t_s 0.2       hold time, was 0.5: a short collapse is caught 8/8 instead of 9/20, 0/48 false trips. Not yet confirmed
                on the bench.
  b_rate_m      the bias moves more than this within b_window_s: trip ("bias drift"). The bias tracker absorbs a slow
                rangefinder drift as if it were baro drift (0.05 m/s: never caught, 3.8 m). With 0.3 m per 10 s it is
                caught after 23 s (2.8 m), 0/56 false trips with 1.2 m/min baro drift. Margin 1.5x only: check the
                real barometer's drift after power-up on a bench log before flying with it. An absolute cap on the
                bias was tested and rejected (false trips grow with the flight time).
  synth         synthetic rangefinder, see _synth(). After a trip the median sideways run during the landing fell from
                0.59 m to 0.18 m (worst 7.9 m -> 2.4 m). Needs a second rangefinder on the FC (RNGFND2_TYPE 10,
                RNGFND2_ORIENT 25, RNGFND2_MIN / MAX / GNDCLR as sensor 1) and is therefore off by default.
  supervision   tick() never raises. An exception inside the guard is reported ("HEIGHT GUARD FAILED", status()["fault"])
                and stalled() tells the heartbeat thread that tick() is no longer called. In the ablation an exception
                in the guard was silent and those flights were as unsafe as flights without a guard.
Still not detected: a rangefinder drift as slow as plausible baro drift (0.02 m/s = 1.2 m/min); the two sensors alone
cannot tell which one is drifting.
"""

from __future__ import annotations

import math
import time
from collections import deque

from baro_range_monitor import BaroRangeMonitor

DEFAULTS = {
    "enabled": False,       # off until the thresholds have been checked on this airframe's own logs
    "fc_hz": 0.5,           # low-pass of both heights
    "tau_s": 10.0,          # bias (baro drift) tracker
    "e_m": 0.7,             # level criterion: |baro - range - bias| above this ...
    "t_s": 0.2,             # ... for this long
    "rate_m": 99.0,         # rate criterion over 2 s; 99 = off
    "source_set": 3,        # EKF source set with the barometer as height (EK3_SRC3_POSZ 1, EK3_SRC3_VELXY 5)
    "land_delay_s": 0.5,    # LAND this long after the source-set switch
    "blind_s": 1.0,         # armed and an input older than this: report "blind"
    "rng_min_m": 0.02,
    "rng_max_m": 7.9,
    "oor_s": 0.3,           # range invalid this long above oor_h_m (baro): trip; 0 = off
    "oor_h_m": 0.5,
    "skip_invalid": True,   # invalid readings stay out of the monitor's low-pass
    "b_rate_m": 0.3,        # bias moves more than this within b_window_s: trip; 0 = off
    "b_window_s": 10.0,
    "synth": False,         # synthetic rangefinder 2 from the barometer; needs RNGFND2_TYPE 10 on the FC
    "rng1_min": None,       # RNGFND1_MIN to restore after a trip if the FC's own value could not be read
    "stall_s": 1.0,         # armed and no tick() for this long: stalled()
}
MAV_CMD_SET_EKF_SOURCE_SET = 42007
RESEND_S = 0.3              # repeat the switch / LAND until the FC shows LAND
RNG1_MIN_TRIP = 50.0        # RNGFND1_MIN after a trip with synth: every reading of the faulty sensor is below it
PARAM_REAL32 = 9
SYNTH_BIAS_AGE_S = 5.0      # the synthetic range uses the bias from this long before the trip
SYNTH_MAX_M = 7.5


class _Rng:
    """The downward sensor itself (DISTANCE_SENSOR id 0) in the shape of a RANGEFINDER message."""

    def __init__(self, distance: float, stamp: float):
        self.distance, self._timestamp = distance, stamp


def _param_name(msg) -> str:
    n = msg.param_id
    return (n.decode(errors="ignore") if isinstance(n, bytes) else str(n)).rstrip("\0")


class HeightGuard:
    def __init__(self, cfg: dict | None = None):
        self.cfg = {**DEFAULTS, **(cfg or {})}
        self.enabled = bool(self.cfg["enabled"])
        self.mon = self._new_monitor()
        self.tripped_t: float | None = None   # monotonic time of the trip; None = not tripped
        self.reason = ""
        self.blind = ""                       # "" or what is missing
        self._rng_stamp = self._press_stamp = None
        self._sent_t = 0.0
        self._was_armed = False
        self.e = 0.0
        self.active = False                   # the monitor is judging (armed and off the ground)
        self.fault = ""                       # "" or the exception that stopped a tick (kept until disarm)
        self.faults = 0
        self._tick_t: float | None = None     # monotonic time of the last tick() call
        self._oor_t: float | None = None      # since when the range has been invalid in the air
        # synthetic rangefinder (_synth)
        self.synth = bool(self.cfg["synth"])
        self._sp_hist: deque = deque()        # (t, pressure) while disarmed: reference pressure at arming
        self._b_hist: deque = deque()         # (t, monitor bias) of the last SYNTH_BIAS_AGE_S
        self._sp_stamp = self._sp0 = self._sh = None
        self._s_armed = self._synth_on = False
        # RNGFND1_MIN is raised at a trip (synth) and put back after the disarm (_rng1_min)
        self._p_min: float | None = None      # the FC's own RNGFND1_MIN, read while not tripped
        self._p_req_t = self._p_set_t = 0.0
        self._restore = False

    def _new_monitor(self) -> BaroRangeMonitor:
        c = self.cfg
        return BaroRangeMonitor(fc=c["fc_hz"], tau=c["tau_s"], E=c["e_m"], T=c["t_s"], R=c["rate_m"],
                                rng_min=c["rng_min_m"], rng_max=c["rng_max_m"], skip_invalid=c["skip_invalid"],
                                b_rate=c["b_rate_m"], b_window=c["b_window_s"])

    def status(self) -> dict:
        return {"enabled": self.enabled, "active": self.active, "tripped": self.tripped_t is not None,
                "reason": self.reason, "blind": self.blind, "e": round(self.e, 3), "fault": self.fault}

    def note(self) -> str:
        if not self.enabled:
            return ""
        if self.tripped_t is not None:
            return f"height guard: TRIPPED ({self.reason})"
        if self.fault:
            return f"height guard: FAILED ({self.fault})"
        return f"height guard: blind ({self.blind})" if self.blind else ""

    def stalled(self, armed: bool) -> bool:
        """For another thread (the FC heartbeat): enabled and armed, but tick() has not been called for stall_s."""
        t = self._tick_t
        return bool(self.enabled and armed and t is not None and time.monotonic() - t > float(self.cfg["stall_s"]))

    def _valid(self, distance: float) -> bool:
        return float(self.cfg["rng_min_m"]) <= distance <= float(self.cfg["rng_max_m"])

    # ------------------------------------------------------------------ once per companion tick
    def tick(self, drone) -> None:
        """drone: RealDrone (m.messages, armed(), fc_mode(), mute, _text(), _gevent()). Never raises."""
        if not self.enabled or drone.m is None:
            return
        self._tick_t = time.monotonic()
        try:
            self._tick(drone)
        except Exception as e:  # noqa: BLE001 - a guard that died silently is worse than none: say so, keep trying
            self.faults += 1
            self.fault = f"{type(e).__name__}: {e}"
            try:
                drone._gevent(f"HEIGHT GUARD FAILED: {self.fault}")
                drone._text("ECPS: HEIGHT GUARD FAILED", every_s=2.0)
            except Exception:  # noqa: BLE001
                pass

    def _tick(self, drone) -> None:
        now = time.monotonic()
        armed = drone.armed()
        if self._was_armed and not armed:     # disarmed: release the latch, start again for the next flight
            if self.synth and self.tripped_t is not None:
                self._restore = True
            self.mon = self._new_monitor()
            self.tripped_t, self.reason, self.blind, self.active = None, "", "", False
            self.fault, self._oor_t = "", None
        self._was_armed = armed
        msgs = drone.m.messages
        rng, att, prs = msgs.get("RANGEFINDER"), msgs.get("ATTITUDE"), msgs.get("SCALED_PRESSURE")
        src = "RANGEFINDER"
        if self.synth:
            # RANGEFINDER is the FC's pick of its rangefinders and would show the synthetic one: watch sensor 0 itself
            src, d0 = "DISTANCE_SENSOR", msgs.get("DISTANCE_SENSOR[0]")
            rng = None if d0 is None else _Rng(d0.current_distance * 0.01, d0._timestamp)
            self._rng1_min(drone, armed, now)
            self._synth(drone, rng, att, prs, armed, now)
        wall = time.time()
        age = {n: (wall - m._timestamp if m is not None else float("inf"))
               for n, m in ((src, rng), ("ATTITUDE", att), ("SCALED_PRESSURE", prs))}
        missing = [n for n, a in age.items() if a > float(self.cfg["blind_s"])]
        self.blind = ",".join(missing) if armed and missing else ""
        if self.blind and self.tripped_t is None:
            drone._gevent(f"height guard blind: {self.blind}")
            drone._text("ECPS: height guard blind")
        if self.tripped_t is not None:
            self._act(drone, now)
            return
        if rng is None or att is None or rng._timestamp == self._rng_stamp:
            return                            # the monitor is clocked by new RANGEFINDER messages
        self._rng_stamp = rng._timestamp
        press = None
        if prs is not None and prs._timestamp != self._press_stamp:
            self._press_stamp, press = prs._timestamp, prs.press_abs
        s = self.mon.step(rng._timestamp, rng.distance, att.roll, att.pitch, press, armed, drone.fc_mode())
        self.e, self.active = float(s["e"] or 0.0), bool(s["enabled"])
        reason = "+".join({"A": "level", "B": "rate", "C": "bias drift"}[k] for k in (s["reason"] or "")) if s["trigger"] else ""
        # the range is invalid while clearly airborne: the monitor does not judge it, and the FC only stops using it
        oor = float(self.cfg["oor_s"])
        if oor > 0 and armed and self.active and (self.mon.h_b or 0.0) > float(self.cfg["oor_h_m"]) \
                and not self._valid(rng.distance):
            self._oor_t = self._oor_t or now
            if now - self._oor_t >= oor and not reason:
                reason = "range lost"
        else:
            self._oor_t = None
        if reason and armed:
            self.tripped_t = now
            self.reason = reason
            what = f"range invalid for {now - self._oor_t:.1f} s" if reason == "range lost" else \
                f"rangefinder and barometer disagree by {self.e:+.2f} m"
            drone._gevent(f"HEIGHT GUARD: {what} ({self.reason}) -> baro height, LAND")
            self._act(drone, now)

    def _rng1_min(self, drone, armed: bool, now: float) -> None:
        """synth only. A trip sets RNGFND1_MIN to RNG1_MIN_TRIP, and the FC stores that: it must be put back once the
        drone is disarmed, or the next flight starts without the downward rangefinder. The FC's own value is read while
        the guard is not tripped; cfg rng1_min is used if that never worked. A value of RNG1_MIN_TRIP found on a
        disarmed FC is a left-over of an earlier trip (companion restarted before the restore) and is put back too."""
        m = drone.m
        pv = m.messages.get("PARAM_VALUE")
        cur = float(pv.param_value) if pv is not None and _param_name(pv) == "RNGFND1_MIN" else None
        left_over = cur is not None and math.isclose(cur, RNG1_MIN_TRIP, abs_tol=1e-3)
        if cur is not None and not left_over:
            self._p_min = cur
        if self.tripped_t is not None or drone.mute:
            return
        if left_over and not armed and not self._restore:
            self._restore = True
        if self._restore and not armed:
            target = self._p_min if self._p_min is not None else self.cfg["rng1_min"]
            if target is None:
                drone._gevent(f"height guard: RNGFND1_MIN is {RNG1_MIN_TRIP:g} after a trip and the old value is unknown:"
                              f" set it by hand")
                drone._text("ECPS: set RNGFND1_MIN by hand")
                self._restore = False
            elif cur is not None and math.isclose(cur, float(target), abs_tol=1e-3):
                drone._gevent(f"height guard: RNGFND1_MIN restored to {float(target):g}")
                self._restore = False
            elif now - self._p_set_t >= 1.0:
                self._p_set_t = now
                m.mav.param_set_send(m.target_system, m.target_component, b"RNGFND1_MIN", float(target), PARAM_REAL32)
            return
        if self._p_min is None and now - self._p_req_t >= 5.0:
            self._p_req_t = now
            m.mav.param_request_read_send(m.target_system, m.target_component, b"RNGFND1_MIN", -1)

    def _synth(self, drone, rng, att, prs, armed: bool, now: float) -> None:
        """Second downward rangefinder (RNGFND2_TYPE 10, MAVLink) made from the barometer.

        Why: after a trip the EKF still scales the optical flow with the height above ground it gets from the faulty
        rangefinder (HAGL 0.05 m at a true 2 m: velocity estimate ~0, the position loop has no damping), and with the
        range out of limits the FC reports "no relative position" and LAND levels the attitude: the wind then carries
        the drone (up to 7.9 m in SITL). Both need a believable range. Sent on the ground (the arming check wants data
        from every rangefinder), silent in normal flight (the EKF would mix it with the real one), sent again once
        the guard has tripped or the real range is invalid. Height = baro height minus the monitor's bias from
        SYNTH_BIAS_AGE_S ago (the bias of the last seconds before a trip already follows the fault)."""
        if prs is not None and prs._timestamp != self._sp_stamp:
            self._sp_stamp = prs._timestamp
            if not armed:
                self._sp_hist.append((now, prs.press_abs))
                while now - self._sp_hist[0][0] > 3.0:
                    self._sp_hist.popleft()
            elif self._sp0:
                hb = 44330.0 * (1.0 - (prs.press_abs / self._sp0) ** 0.1903)
                self._sh = hb if self._sh is None else self._sh + 0.25 * (hb - self._sh)
        if armed and not self._s_armed:
            p = [v for t, v in self._sp_hist if now - t <= 2.0]
            self._sp0, self._sh = (sum(p) / len(p) if p else None), None
        if not armed:
            self._sp0, self._synth_on = None, False
        self._s_armed = armed
        if armed and rng is not None and self.active and (self.mon.h_b or 0.0) > float(self.cfg["oor_h_m"]) \
                and not self._valid(rng.distance):
            self._synth_on = True             # the real range is invalid in the air
        if self.tripped_t is not None:
            self._synth_on = True
        if armed and not self._synth_on:
            if self.mon.b_set:
                self._b_hist.append((now, self.mon.b))
                while now - self._b_hist[0][0] > SYNTH_BIAS_AGE_S:
                    self._b_hist.popleft()
            return
        if not armed:
            self._b_hist.clear()
        if drone.mute:
            return
        h = 0.05
        if armed and self._sh is not None:
            h = max(0.03, self._sh - (self._b_hist[0][1] if self._b_hist else self.mon.b))
        c = math.cos(att.roll) * math.cos(att.pitch) if att is not None else 1.0
        d = min(SYNTH_MAX_M, h / max(0.5, c))
        # min 1 cm, max 800 cm, type 0 (laser), id 2, orientation 25 (down), covariance unknown
        drone.m.mav.distance_sensor_send(int(now * 1000) & 0xFFFFFFFF, 1, 800, int(round(d * 100)), 0, 2, 25, 255)

    def _act(self, drone, now: float) -> None:
        if drone.mute or not drone.armed():
            return
        if drone.fc_mode() == "LAND" and now - self.tripped_t > float(self.cfg["land_delay_s"]):
            return                            # the FC is landing on the baro height: nothing more to send
        if now - self._sent_t < RESEND_S:
            return
        self._sent_t = now
        m = drone.m
        m.mav.command_long_send(m.target_system, m.target_component, MAV_CMD_SET_EKF_SOURCE_SET, 0,
                                float(self.cfg["source_set"]), 0, 0, 0, 0, 0, 0)
        if self.synth:                        # the FC drops the faulty rangefinder and uses the synthetic one
            m.mav.param_set_send(m.target_system, m.target_component, b"RNGFND1_MIN", RNG1_MIN_TRIP, PARAM_REAL32)
        drone._text("ECPS: RANGEFINDER BAD - baro height, LAND", every_s=1.0)
        if now - self.tripped_t >= float(self.cfg["land_delay_s"]):
            m.set_mode("LAND")
