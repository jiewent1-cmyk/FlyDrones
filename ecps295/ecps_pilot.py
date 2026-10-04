"""Pilot with the watchdogs the G4 suite showed were missing (TechRoute §10 S6).

F2      Pilot.tick never passed brain_age_s to SafetyGovernor.filter, so the "brain stalled -> hover" watchdog could
        not fire. Here the age is the wall time since the previous tick finished.
fence   (optional, fence_turn=True) upstream SafetyGovernor only zeroes the forward command beyond
        geofence_radius_m, so a cruising drone parks on the boundary. Near the fence and heading outward, this
        safety layer yaws the drone back toward the take-off point (not brain behaviour; logged as such).
camera  a frozen camera (USB hang, driver stuck) went unnoticed and the brain flew blind into the wall
        (G4 T4_freeze). If the drone reports no new frame for stale_s, command hover; after land_after_s, land.

Same sequence as flydrones.runtime.Pilot.tick otherwise (upstream left untouched).
"""

from __future__ import annotations

import math
import time

from flydrones.motor import FlightCommand
from flydrones.runtime import Pilot, TickInfo


class EcpsPilot(Pilot):
    def __init__(
        self, *args, stale_s: float = 0.3, land_after_s: float = 3.0, fence_turn: bool = False, fence_margin_m: float = 0.35, **kw
    ):
        super().__init__(*args, **kw)
        self.fence_turn = fence_turn
        self.fence_margin = fence_margin_m
        self._turning_back = False
        self.fence_turn_ticks = 0
        self.stale_s = stale_s
        self.land_after_s = land_after_s
        self._last_tick_end: float | None = None
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
        vision = self.retina.encode(frame)
        g = None
        if self.gestures is not None:
            g = self.gestures.read(t, cam)
            vision = self.illusion.apply(vision, g, t)
        tel = self.drone.telemetry()
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
        if self.fence_turn and tel.x_m is not None and tel.y_m is not None and tel.yaw_deg is not None:
            cmd = self._fence_turn(cmd, tel, t)
        if self.safety.land_requested:
            self.drone.land()
        else:
            self.drone.send(cmd)
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
