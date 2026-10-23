"""Pilot with the watchdogs the G4 suite showed were missing (TechRoute §10 S6).

F2      Pilot.tick never passed brain_age_s to SafetyGovernor.filter, so the "brain stalled -> hover" watchdog could
        not fire. Here the age is the wall time since the previous tick finished.
fence   (optional, fence_turn=True) upstream SafetyGovernor only zeroes the forward command beyond
        geofence_radius_m, so a cruising drone parks on the boundary. Near the fence and heading outward, this
        safety layer yaws the drone back toward the take-off point (not brain behaviour; logged as such). While the
        brain is in a saccade it only blocks forward motion and leaves the yaw to the saccade.
camera  a frozen camera (USB hang, driver stuck) went unnoticed and the brain flew blind into the wall
        (G4 T4_freeze). If the drone reports no new frame for stale_s, command hover; after land_after_s, land.
height source  (route B) the companion's height module feeds the FC; if it has not sent for stale_s, hover; after
        land_after_s, land (S7b D2: it stopped, the FC fell back to baro and a box threw the drone to 2.2 m)
height  (optional, height_guard) without GPS a box under the drone pulls the flow EKF height down and the EKF then
        rejects the true range as an outlier: after backing off the box it still read 0.75 m at a true 1.23 m, and
        ArduPilot's own altitude hold carried the drone through the 1.3 m hard fence (v4 smoke test). The ventral
        cue's height above the floor (baro minus the floor baseline, frozen over obstacles) does not depend on the
        EKF; above ceiling_m this layer commands a descent, below floor_m a climb. Logged as a watchdog event.
waypoints  (optional, 2026-10-17) goal bias for waypoint flight: points are local NED (m) from the take-off point, the
        frame of the EKF position. Outside escapes the layer adds a yaw toward the current point to the brain's yaw
        and scales the brain's forward command down with the heading error and near the point; the brain's brakes,
        saccades and retreats are kept, and the fence turn and the range shield still act after it. After a saccade
        the bias stays off for post_escape_s so the drone first leaves the obstacle. A point is reached within
        radius_m (EKF), skipped when the distance has not shrunk by stuck_progress_m for stuck_s (e.g. a stack on
        it). When all points are done the cruise stops (hover; the pilot or the test harness takes over).
target  (2026-10-18) ArUco target tracking (ecps_target.TargetTrack): off until track_start() (ground console TRACK);
        it searches on the spot, sets the waypoint goal standoff_m short of the marker and keeps moving it with the
        marker, then hovers facing it. It runs before the waypoint layer, so everything after it still applies.
rtl     (2026-10-22, optional `rtl:`) return to the take-off point and land: rtl_start() (ground console RTL, console
        link lost, or the battery below rtl.battery_pct) flies home through the waypoint layer (brain escapes, fence
        turn and range shield still apply), hovers settle_s on the point, then requests LAND from the FC (the FC lands
        and disarms). Home not reached (the waypoint layer skipped it as stuck, or timeout_s) and farther than
        land_far_m from it: lands where it is. The governor's own min_battery_pct still lands at once.
landmark (2026-10-22, optional `landmark:`, ecps_landmark.LandmarkFix) fixed ArUco markers measure the flow EKF's drift;
        every layer here sees the corrected pose (EKF - offset) and the drone gets the offset (pos_offset) for its
        hover-hold target. Off unless enabled and a marker detector is attached (landmark.source).

Same sequence as flydrones.runtime.Pilot.tick otherwise (upstream left untouched).
"""

from __future__ import annotations

import copy
import math
import time

from ecps_struct import StructGuard

from flydrones.motor import FlightCommand
from flydrones.runtime import Pilot, TickInfo


class EcpsPilot(Pilot):
    def __init__(
        self,
        *args,
        stale_s: float = 0.3,
        land_after_s: float = 3.0,
        fence_turn: bool = False,
        fence_margin_m: float = 0.35,
        height_guard: dict | None = None,
        height_source: dict | None = None,
        struct: dict | None = None,
        waypoints: dict | None = None,
        target: dict | None = None,
        rtl: dict | None = None,
        landmark: dict | None = None,
        **kw,
    ):
        super().__init__(*args, **kw)
        wp = waypoints or {}
        self.wp_on = bool(wp.get("enabled", False))
        self.wp_points = [(float(p[0]), float(p[1])) for p in wp.get("points", [])]
        self.wp_radius = float(wp.get("radius_m", 0.35))
        self.wp_gain = float(wp.get("gain", 1.0))  # yaw command per 60 deg of heading error
        self.wp_brain_w = float(wp.get("brain_yaw_w", 1.0))  # weight of the brain's own yaw
        self.wp_slow_r = float(wp.get("slow_r_m", 0.8))
        self.wp_min_fwd = float(wp.get("min_forward_frac", 0.3))
        self.wp_post_escape = float(wp.get("post_escape_s", 2.0))
        self.wp_stuck_s = float(wp.get("stuck_s", 20.0))
        self.wp_stuck_dm = float(wp.get("stuck_progress_m", 0.25))
        self.wp_idx = 0
        self.wp_active = True  # the companion clears it while the pilot flies: the mission pauses, then resumes
        self.wp_log: list[tuple[float, int, str, float]] = []  # (time, index, reached/skipped, EKF distance)
        self.wp_done_t: float | None = None
        self.wp_ticks = 0
        self._wp_best = float("inf")
        self._wp_best_t = 0.0
        self._wp_escape_t = -1e9
        hg = height_guard or {}
        self.hg_on = bool(hg.get("enabled", False))
        self.hg_ceiling = float(hg.get("ceiling_m", 1.0))
        self.hg_floor = float(hg.get("floor_m", 0.35))
        self.hg_gain = float(hg.get("gain", 1.5))
        self.hg_ticks = 0
        self.last_height_est: float | None = None
        # companion height watchdog (S7b D2): the FC height comes from our own height module (route B); if it stops,
        # the FC falls back to baro, and a box under the drone then threw it to 2.2 m. Do not keep patrolling.
        hs = height_source or {}
        self.ext_wd_on = bool(hs.get("enabled", True))
        self.ext_stale_s = float(hs.get("stale_s", 0.5))
        self.ext_land_after_s = float(hs.get("land_after_s", 3.0))
        self.fence_turn = fence_turn
        self.fence_margin = fence_margin_m
        self._turning_back = False
        self.fence_turn_ticks = 0
        self.stale_s = stale_s
        self.land_after_s = land_after_s
        self._last_tick_end: float | None = None
        self._t = 0.0  # time of the last tick (track_start / track_stop called between ticks)
        self._t_prev = 0.0
        self._last_yaw_dps = 0.0
        self._last_fwd = 0.0
        self.watchdog_events: list[tuple[float, str]] = []
        from ecps_target import TargetTrack

        self.target = TargetTrack(target)  # OFF until track_start(); the companion attaches its MarkerThread as source
        self.target.on_event = self._event
        rc = rtl or {}
        self.rtl_on = bool(rc.get("enabled", True))
        self.rtl_home = tuple(float(v) for v in rc.get("home", (0.0, 0.0)))
        self.rtl_radius = float(rc.get("radius_m", 0.25))
        self.rtl_settle_s = float(rc.get("settle_s", 2.0))
        self.rtl_timeout_s = float(rc.get("timeout_s", 90.0))
        self.rtl_land_far = float(rc.get("land_far_m", 0.6))
        self.rtl_battery = float(rc.get("battery_pct", 0.0))  # 0 = no battery trigger
        self.rtl_state = "OFF"  # OFF / RETURN / SETTLE / LAND
        self.rtl_why = ""
        self._rtl_t = 0.0
        self._rtl_radius0: float | None = None
        self._rtl_batt_done = False
        from ecps_landmark import LandmarkFix

        self.landmark = LandmarkFix(landmark)  # off unless landmark.enabled; the companion attaches the MarkerThread
        sg = StructGuard(struct)  # 2026-10-13 blind-side fixes (ecps_struct.py), off unless configured
        self.struct = sg if sg.enabled else None

    def _event(self, t: float, what: str) -> None:
        if not self.watchdog_events or self.watchdog_events[-1][1] != what:
            self.watchdog_events.append((round(t, 2), what))

    def _fence_turn(self, cmd: FlightCommand, tel, t: float) -> FlightCommand:
        r = math.hypot(tel.x_m, tel.y_m)
        out_bearing = math.atan2(tel.y_m, tel.x_m)  # NED: x north, y east; yaw 0 = north, clockwise
        heading = math.radians(tel.yaw_deg)
        outward = math.cos(heading - out_bearing)
        if not self._turning_back and r > self.safety.fence - self.fence_margin and outward > 0.0:
            self._turning_back = True
            self._event(t, "fence: turning back")
        if self._turning_back:
            if outward < -0.5 or r < self.safety.fence - 2 * self.fence_margin:
                self._turning_back = False
            elif cmd.escape or cmd.forward <= 0.0:
                # the brain is in a saccade: do not fight its yaw (patrol v3/v3.1: both yawing next to the box stack
                # that sits at the fence edge ended in contact); only keep it from moving outward. Same when nothing
                # moves the drone forward (console HOLD, waypoints done, brakes): no pointless spin on the spot
                cmd = FlightCommand(throttle=cmd.throttle, yaw=cmd.yaw, forward=min(cmd.forward, 0.0), escape=cmd.escape, note=cmd.note)
            else:
                err = math.atan2(math.sin(out_bearing + math.pi - heading), math.cos(out_bearing + math.pi - heading))
                cmd = FlightCommand(
                    throttle=cmd.throttle,
                    yaw=max(-1.0, min(1.0, err / math.radians(60))),
                    forward=min(cmd.forward, 0.0),
                    escape=cmd.escape,
                    note="fence turn-back",
                )
                self.fence_turn_ticks += 1
        return cmd

    def set_waypoints(self, points) -> None:
        """Replace the mission (e.g. a go-to point from the ground console) and start it from its first point."""
        self.wp_points = [(float(p[0]), float(p[1])) for p in points]
        self.wp_on = True
        self.wp_idx, self.wp_done_t = 0, None
        self._wp_best, self._wp_escape_t = float("inf"), -1e9

    def track_start(self, t: float | None = None) -> None:
        """Start target tracking (search -> approach -> hover facing the marker); replaces any waypoint mission."""
        self.wp_on = False
        self.target.start(self._t if t is None else t)

    def track_stop(self, t: float | None = None) -> None:
        self.target.stop(self._t if t is None else t, self)
        self.wp_on = False

    def rtl_start(self, why: str = "", t: float | None = None) -> bool:
        """Return home and land (replaces tracking and any waypoint mission). False when the layer is off."""
        if not self.rtl_on:
            return False
        t = self._t if t is None else t
        if self.target.active:
            self.target.stop(t, self)
        if self._rtl_radius0 is None:
            self._rtl_radius0 = self.wp_radius
        self.wp_radius = self.rtl_radius
        self.set_waypoints([self.rtl_home])
        self.rtl_state, self.rtl_why, self._rtl_t = "RETURN", why, t
        self._event(t, "rtl: returning home" + (f" ({why})" if why else ""))
        return True

    def rtl_stop(self, t: float | None = None) -> None:
        """Cancel the return (the pilot or the console takes over); a LAND already requested stays requested."""
        if self.rtl_state == "OFF":
            return
        if self.rtl_state in ("RETURN", "SETTLE"):
            self.wp_on = False
        if self._rtl_radius0 is not None:
            self.wp_radius, self._rtl_radius0 = self._rtl_radius0, None
        self.rtl_state = "OFF"
        self._event(self._t if t is None else t, "rtl: cancelled")

    def _rtl_land(self, t: float, what: str) -> None:
        self.rtl_state, self._rtl_t = "LAND", t
        self.wp_on = False
        self.safety.land_requested = True
        self._event(t, what)

    def _rtl(self, cmd: FlightCommand, tel, t: float) -> FlightCommand:
        if not self.wp_active:  # the pilot flies (or the console holds): the timers wait
            self._rtl_t = t if self.rtl_state == "SETTLE" else self._rtl_t + (t - self._t_prev)
            return cmd
        d = math.hypot(self.rtl_home[0] - tel.x_m, self.rtl_home[1] - tel.y_m)
        if self.rtl_state == "RETURN":
            if self.wp_done_t is not None:
                if d <= self.rtl_land_far:
                    self.rtl_state, self._rtl_t = "SETTLE", t
                    self.wp_on = False
                    self._event(t, f"rtl: home ({d:.2f} m), settling")
                else:
                    self._rtl_land(t, f"rtl: home not reached ({d:.2f} m) -> landing here")
            elif t - self._rtl_t > self.rtl_timeout_s:
                self._rtl_land(t, f"rtl: timeout {self.rtl_timeout_s:.0f} s ({d:.2f} m from home) -> landing here")
        elif self.rtl_state == "SETTLE" and t - self._rtl_t > self.rtl_settle_s:
            self._rtl_land(t, f"rtl: landing ({d:.2f} m from home)")
        if self.rtl_state in ("SETTLE", "LAND") and not cmd.escape:  # hover on the point (position hold in the FC link)
            return FlightCommand(throttle=0.0, yaw=0.0, forward=0.0, note=f"rtl: {self.rtl_state.lower()}")
        return cmd

    def _rtl_watch(self, tel, t: float) -> None:
        """Per tick: a landed / disarmed drone resets the layer; the battery below rtl.battery_pct starts the return."""
        if getattr(tel, "flying", None) is False and self.rtl_state in ("OFF", "LAND"):  # landed / disarmed: the next flight starts clean
            if self.rtl_state == "LAND":
                self.rtl_state = "OFF"
                if self._rtl_radius0 is not None:
                    self.wp_radius, self._rtl_radius0 = self._rtl_radius0, None
            self._rtl_batt_done = False
        if (self.rtl_on and self.rtl_battery > 0 and not self._rtl_batt_done and getattr(tel, "battery_pct", None) is not None
                and tel.battery_pct < self.rtl_battery and getattr(tel, "flying", True) and self.rtl_state == "OFF"):
            self._rtl_batt_done = True  # once per flight: the operator / pilot may cancel it
            self.rtl_start(f"battery {tel.battery_pct:.0f} %", t)

    def _waypoint(self, cmd: FlightCommand, tel, t: float) -> FlightCommand:
        if not self.wp_active:
            self._wp_best = float("inf")  # restart the stuck timer on resume
            return cmd
        if self.wp_done_t is None and self.wp_idx >= len(self.wp_points):
            self.wp_done_t = t
            self._event(t, "waypoints: done -> hover")
        if self.wp_done_t is not None:
            return FlightCommand(throttle=cmd.throttle, yaw=cmd.yaw, forward=min(cmd.forward, 0.0), escape=cmd.escape, note="waypoints done")
        n, e = self.wp_points[self.wp_idx]
        d = math.hypot(n - tel.x_m, e - tel.y_m)
        if d < self._wp_best - self.wp_stuck_dm:
            self._wp_best, self._wp_best_t = d, t
        if d < self.wp_radius or t - self._wp_best_t > self.wp_stuck_s:
            what = "reached" if d < self.wp_radius else "skipped"
            self.wp_log.append((round(t, 2), self.wp_idx, what, round(d, 3)))
            self._event(t, f"waypoint {self.wp_idx} {what}")
            self.wp_idx += 1
            self._wp_best, self._wp_best_t = float("inf"), t
            return self._waypoint(cmd, tel, t)
        if cmd.escape:
            self._wp_escape_t = t
            return cmd
        if t - self._wp_escape_t < self.wp_post_escape:  # let the drone leave the obstacle before steering back
            return cmd
        heading = math.radians(tel.yaw_deg)
        b = math.atan2(e - tel.y_m, n - tel.x_m)
        err = math.atan2(math.sin(b - heading), math.cos(b - heading))
        yaw = max(-1.0, min(1.0, self.wp_brain_w * cmd.yaw + self.wp_gain * err / math.radians(60)))
        fwd = cmd.forward
        if fwd > 0:  # keep brakes and retreats; slow down to turn and to stop on the point
            fwd *= max(0.0, math.cos(err)) * max(self.wp_min_fwd, min(1.0, d / self.wp_slow_r))
        self.wp_ticks += 1
        return FlightCommand(throttle=cmd.throttle, yaw=yaw, forward=fwd, escape=cmd.escape, note=f"waypoint {self.wp_idx}")

    def tick(self, t: float, dt: float) -> TickInfo:
        now = time.monotonic()
        brain_age = 0.0 if self._last_tick_end is None else now - self._last_tick_end  # F2
        frame = self.drone.frame() if self.drone.has_camera else None
        cam = self.webcam.read() if self.webcam is not None else None
        if hasattr(self.retina, "yaw_rate_dps"):  # EcpsRetina centering needs the gyro (previous tick)
            self.retina.yaw_rate_dps = self._last_yaw_dps
        if hasattr(self.retina, "fwd_cmd"):  # C1 efference copy: forward command of the previous tick
            self.retina.fwd_cmd = self._last_fwd
        if hasattr(self.retina, "range_m"):  # v4 ventral cue: downward ToF + baro height (latest telemetry)
            self.retina.range_m = getattr(self.drone, "rangefinder_m", lambda: None)()
            self.retina.baro_m = getattr(self.drone, "baro_m", lambda: None)()
        if getattr(self.retina, "tof_on", False) and hasattr(self.drone, "ranges"):  # S5: horizontal ToF -> near grids
            rt = self.retina
            rt.tof = self.drone.ranges(rt.tof_beams, rt.tof_fov, rt.tof_rate, rt.tof_noise, **getattr(rt, "tof_kw", {}))
        vision = self.retina.encode(frame)
        g = None
        if self.gestures is not None:
            g = self.gestures.read(t, cam)
            vision = self.illusion.apply(vision, g, t)
        tel = self.drone.telemetry()
        if self.landmark.enabled and tel.x_m is not None and tel.y_m is not None:
            now_m = tel.t if getattr(tel, "t", None) is not None else time.monotonic()
            self.landmark.update(now_m, tel.x_m, tel.y_m, tel.yaw_deg, t, self._event)
            if self.landmark.fixes:
                tel = copy.copy(tel)  # the drone keeps its raw EKF telemetry
                tel.x_m, tel.y_m = self.landmark.correct(tel.x_m, tel.y_m)
            self.drone.pos_offset = self.landmark.offset
        self._last_yaw_dps = tel.yaw_rate_dps or 0.0
        inputs = self.encoder.encode(vision, tel.yaw_rate_dps)
        rates = self.brain.tick(inputs, ms=dt * 1000.0)
        raw = self.decoder.update(rates, dt)
        cmd = raw
        stop_reason = ""
        invalid_raw = not all(math.isfinite(getattr(raw, a)) for a in ("throttle", "yaw", "forward", "lateral"))
        if brain_age > self.safety.brain_timeout:
            self._event(t, f"brain stalled {brain_age:.2f} s -> hover")
        stale = getattr(self.drone, "frame_age_s", lambda: 0.0)() if self.drone.has_camera else 0.0
        if stale > self.stale_s:
            stop_reason = f"camera stale {stale:.1f} s"
            self._event(t, "camera stale -> hover")
            if stale > self.land_after_s:
                self.safety.land_requested = True
                self._event(t, "camera stale -> land")
        ext_age = getattr(self.drone, "ext_age_s", lambda: 0.0)()
        if self.ext_wd_on and ext_age > self.ext_stale_s and getattr(self.drone, "ext_height", False):
            stop_reason = f"companion height stale {min(ext_age, 99):.1f} s"
            self._event(t, "height source stale -> hover")
            if ext_age > self.ext_land_after_s:
                self.safety.land_requested = True
                self._event(t, "height source stale -> land")
        if self.hg_on and hasattr(self.retina, "ventral"):
            h = self.retina.ventral.height_above_floor(self.retina.baro_m)
            self.last_height_est = h
            if h is not None and h > self.hg_ceiling:
                thr = max(-1.0, min(cmd.throttle, -0.2 - self.hg_gain * (h - self.hg_ceiling)))
                cmd = FlightCommand(throttle=thr, yaw=cmd.yaw, forward=cmd.forward, escape=cmd.escape, note="height guard: high")
                self.hg_ticks += 1
                self._event(t, "height guard: too high")
            elif h is not None and h < self.hg_floor:
                thr = min(1.0, max(cmd.throttle, 0.2 + self.hg_gain * (self.hg_floor - h)))
                cmd = FlightCommand(throttle=thr, yaw=cmd.yaw, forward=cmd.forward, escape=cmd.escape, note="height guard: low")
                self.hg_ticks += 1
                self._event(t, "height guard: too low")
        self._rtl_watch(tel, t)
        self._t_prev, self._t = self._t, t
        if self.rtl_state != "OFF" and tel.x_m is not None and tel.y_m is not None:
            cmd = self._rtl(cmd, tel, t)
        if self.target.active and tel.x_m is not None and tel.y_m is not None and tel.yaw_deg is not None:
            cmd = self.target.update(self, cmd, tel, t)
        if self.wp_on and tel.x_m is not None and tel.y_m is not None and tel.yaw_deg is not None:
            cmd = self._waypoint(cmd, tel, t)
        if self.fence_turn and tel.x_m is not None and tel.y_m is not None and tel.yaw_deg is not None:
            cmd = self._fence_turn(cmd, tel, t)
        if self.struct is not None:
            cmd = self.struct.filter(cmd, tel, self.decoder, self.drone, t)
        # No steering layer may overwrite the governor or a watchdog stop.
        # Preserve invalid raw input even if an intermediate clamp hid its NaN.
        cmd = self.safety.filter(raw if invalid_raw else cmd, tel, dt, brain_age_s=brain_age)
        if stop_reason:
            cmd = self.safety.hover(stop_reason)
        if self.safety.land_requested:
            self.drone.land()
        else:
            self.drone.send(cmd)
        self._last_fwd = cmd.forward
        self.history.append(
            {
                "t": t,
                "alt": tel.alt_m,
                "x": tel.x_m,
                "y": tel.y_m,
                "yaw": tel.yaw_deg,
                **{f"cmd_{k}": getattr(cmd, k) for k in ("throttle", "yaw", "forward")},
                "escape": cmd.escape,
                **{f"hz_{k}": v for k, v in rates.items() if k.startswith("DN")},
            }
        )
        self._last_tick_end = time.monotonic()
        del self.history[:-1200]  # one minute at 20 Hz; full telemetry is already streamed to ticks.csv
        return TickInfo(
            t,
            cam if cam is not None else frame,
            rates,
            raw,
            cmd,
            tel,
            g,
            self.illusion.mode if g is not None else "camera",
            self.brain.last_raster,
            self.brain.realtime_factor,
            int(self.brain.last_counts.sum()),
            self.brain.net.t_ms,
        )
