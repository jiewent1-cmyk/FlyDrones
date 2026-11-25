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
Not detected: a rangefinder error that grows slower than the bias tracker follows (0.1 m/s in SITL).
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
    "t_s": 0.5,             # ... for this long
    "rate_m": 99.0,         # rate criterion over 2 s; 99 = off
    "source_set": 3,        # EKF source set with the barometer as height (EK3_SRC3_POSZ 1, EK3_SRC3_VELXY 5)
    "land_delay_s": 0.5,    # LAND this long after the source-set switch
    "blind_s": 1.0,         # armed and an input older than this: report "blind"
    "rng_min_m": 0.02,
    "rng_max_m": 7.9,
}
MAV_CMD_SET_EKF_SOURCE_SET = 42007
RESEND_S = 0.3              # repeat the switch / LAND until the FC shows LAND


class _Rng:
    def __init__(self, distance, stamp):
        self.distance, self._timestamp = distance, stamp


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
        # EXPERIMENT (round 4): synthetic rangefinder from the barometer, see _synth()
        self._sp_hist = deque()
        self._b_hist = deque()
        self._sp_stamp = self._sp0 = self._sh = None
        self._s_armed = self._synth_on = False

    def _new_monitor(self) -> BaroRangeMonitor:
        c = self.cfg
        m = BaroRangeMonitor(fc=c["fc_hz"], tau=c["tau_s"], E=c["e_m"], T=c["t_s"], R=c["rate_m"],
                             rng_min=c["rng_min_m"], rng_max=c["rng_max_m"])
        m.skip_invalid = bool(c.get("skip_inv"))      # EXPERIMENT round 4b
        return m

    def status(self) -> dict:
        return {"enabled": self.enabled, "active": self.active, "tripped": self.tripped_t is not None,
                "reason": self.reason, "blind": self.blind, "e": round(self.e, 3)}

    def note(self) -> str:
        if not self.enabled:
            return ""
        if self.tripped_t is not None:
            return f"height guard: TRIPPED ({self.reason})"
        return f"height guard: blind ({self.blind})" if self.blind else ""

    # ------------------------------------------------------------------ once per companion tick
    def tick(self, drone) -> None:
        """drone: RealDrone (m.messages, armed(), fc_mode(), mute, _text(), _gevent())."""
        if not self.enabled or drone.m is None:
            return
        now = time.monotonic()
        armed = drone.armed()
        if self._was_armed and not armed:     # disarmed: release the latch, start again for the next flight
            self.mon = self._new_monitor()
            self.tripped_t, self.reason, self.blind, self.active = None, "", "", False
        self._was_armed = armed
        msgs = drone.m.messages
        rng, att, prs = msgs.get("RANGEFINDER"), msgs.get("ATTITUDE"), msgs.get("SCALED_PRESSURE")
        if self.cfg.get("synth"):
            # RANGEFINDER is the FC's pick of the two sensors, so it would show the synthetic one: watch sensor 0 itself
            d0 = msgs.get("DISTANCE_SENSOR[0]")
            if d0 is not None:
                rng = _Rng(d0.current_distance * 0.01, d0._timestamp)
            self._synth(drone, rng, att, prs, armed, now)
        wall = time.time()
        age = {n: (wall - m._timestamp if m is not None else float("inf"))
               for n, m in (("RANGEFINDER", rng), ("ATTITUDE", att), ("SCALED_PRESSURE", prs))}
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
        # EXPERIMENT (ablation round 2, not in b3a6691): trip when the range is invalid for oor_s while clearly airborne
        oor = float(self.cfg.get("oor_s", 0.0))
        if oor > 0 and armed and self.active and (self.mon.h_b or 0.0) > 0.5 \
                and not (float(self.cfg["rng_min_m"]) <= rng.distance <= float(self.cfg["rng_max_m"])):
            self._oor_t = getattr(self, "_oor_t", None) or now
            if now - self._oor_t >= oor:
                s = {**s, "trigger": True, "reason": "range lost"}
        else:
            self._oor_t = None
        if s["trigger"] and armed:
            self.tripped_t = now
            self.reason = {"A": "level", "B": "rate", "AB": "level+rate"}.get(s["reason"], str(s["reason"]))
            drone._gevent(f"HEIGHT GUARD: rangefinder and barometer disagree by {self.e:+.2f} m ({self.reason})"
                          f" -> baro height, LAND")
            self._act(drone, now)

    def _synth(self, drone, rng, att, prs, armed, now) -> None:
        """EXPERIMENT (round 4). Second downward rangefinder (RNGFND2_TYPE 10, MAVLink) made from the barometer.

        Why: after a trip the EKF still scales the optical flow with the height above ground it gets from the faulty
        rangefinder (HAGL 0.05 m at a true 2 m: velocity estimate ~0, the position loop has no damping), and with the
        range out of limits the FC reports "no relative position" and LAND levels the attitude. Both need a believable
        range. Sent on the ground (the arming check wants data from every rangefinder), silent in normal flight (the EKF
        would mix it with the real one), sent again once the guard has tripped or the real range is invalid."""
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
        if armed and rng is not None and self.active and (self.mon.h_b or 0.0) > 0.5 \
                and not (float(self.cfg["rng_min_m"]) <= rng.distance <= float(self.cfg["rng_max_m"])):
            self._synth_on = True             # the real range is invalid in the air
        if self.tripped_t is not None:
            self._synth_on = True
        if armed and not self._synth_on:
            if self.mon.b_set:                # bias history: the last seconds before a trip already follow the fault
                self._b_hist.append((now, self.mon.b))
                while now - self._b_hist[0][0] > 5.0:
                    self._b_hist.popleft()
            return
        if not armed:
            self._b_hist.clear()
        h = 0.05
        if armed and self._sh is not None:
            h = max(0.03, self._sh - (self._b_hist[0][1] if self._b_hist else self.mon.b))
        c = math.cos(att.roll) * math.cos(att.pitch) if att is not None else 1.0
        d = min(7.5, h / max(0.5, c))
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
        if self.cfg.get("synth"):             # the FC drops the faulty rangefinder: every reading is below its minimum
            m.mav.param_set_send(m.target_system, m.target_component, b"RNGFND1_MIN", 50.0, 9)
        m.mav.command_long_send(m.target_system, m.target_component, MAV_CMD_SET_EKF_SOURCE_SET, 0,
                                float(self.cfg["source_set"]), 0, 0, 0, 0, 0, 0)
        drone._text("ECPS: RANGEFINDER BAD - baro height, LAND", every_s=1.0)
        if now - self.tripped_t >= float(self.cfg["land_delay_s"]):
            m.set_mode("LAND")
