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

Same sequence as flydrones.runtime.Pilot.tick otherwise (upstream left untouched).
"""

from __future__ import annotations

import math
import time

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
        **kw,
    ):
        super().__init__(*args, **kw)
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
        self._last_yaw_dps = 0.0
        self._last_fwd = 0.0
        self.watchdog_events: list[tuple[float, str]] = []

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
            elif cmd.escape:
                # the brain is in a saccade: do not fight its yaw (patrol v3/v3.1: both yawing next to the box stack
                # that sits at the fence edge ended in contact); only keep it from moving outward
                cmd = FlightCommand(throttle=cmd.throttle, yaw=cmd.yaw, forward=min(cmd.forward, 0.0), escape=True, note=cmd.note)
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
        vision = self.retina.encode(frame)
        g = None
        if self.gestures is not None:
            g = self.gestures.read(t, cam)
            vision = self.illusion.apply(vision, g, t)
        tel = self.drone.telemetry()
        self._last_yaw_dps = tel.yaw_rate_dps or 0.0
        inputs = self.encoder.encode(vision, tel.yaw_rate_dps)
        rates = self.brain.tick(inputs, ms=dt * 1000.0)
        raw = self.decoder.update(rates, dt)
        cmd = self.safety.filter(raw, tel, dt, brain_age_s=brain_age)
        if brain_age > self.safety.brain_timeout:
            self._event(t, f"brain stalled {brain_age:.2f} s -> hover")
        stale = getattr(self.drone, "frame_age_s", lambda: 0.0)() if self.drone.has_camera else 0.0
        if stale > self.stale_s:
            cmd = FlightCommand.hover(f"camera stale {stale:.1f} s")
            self._event(t, "camera stale -> hover")
            if stale > self.land_after_s:
                self.safety.land_requested = True
                self._event(t, "camera stale -> land")
        ext_age = getattr(self.drone, "ext_age_s", lambda: 0.0)()
        if self.ext_wd_on and ext_age > self.ext_stale_s and getattr(self.drone, "ext_height", False):
            cmd = FlightCommand.hover(f"companion height stale {min(ext_age, 99):.1f} s")
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
        if self.fence_turn and tel.x_m is not None and tel.y_m is not None and tel.yaw_deg is not None:
            cmd = self._fence_turn(cmd, tel, t)
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
