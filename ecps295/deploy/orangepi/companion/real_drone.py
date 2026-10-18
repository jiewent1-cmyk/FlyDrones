"""Drone backends for the ECPS295 companion on the Orange Pi (the real-hardware counterpart of gz_camera_drone.py).

RealDrone   MAVLink to the Matek F405-WMO (through mavlink-router), USB camera, front + rear VL53L7CX. Same interface
            as GazeboCameraMavlinkDrone, so EcpsPilot / EcpsRetina / RangeShield + LocalMap run unchanged:
              frame(), frame_age_s()          latest 192x144 gray frame from the camera thread
              zones(bearing, ...)             latest 8x8 frame of the sensor facing `bearing` (0 front, 180 rear); a new
                                              array object per frame, as the map update expects
              ranges(beams, fov, ...)         S5 retina beams = columns of the FRONT 8x8 frame (nearest of the column,
                                              minus 0.06 m, clipped to 0..2 m, 2.0 = no echo), the beam convention of
                                              mavlink_twin._zones3
              telemetry(), rangefinder_m(), baro_m(), send_ext_height()   inherited (route-B height at 20 Hz)
            Pilot-in-command gate: velocity setpoints and LAND are sent ONLY when the companion runs with
            --mode guided AND the flight controller reports GUIDED AND armed. The companion never arms, never takes
            off, never changes mode except LAND requested by a watchdog while it is in control. The pilot takes off in
            LOITER / ALT_HOLD and hands over with the RC mode switch (GUIDED); switching away takes control back at once.
            Heartbeat (sysid 254) and the route-B height are sent in every mode except bench.
BenchDrone  no flight controller: fixed hover telemetry, commands recorded only (bench tests and profiling).
"""

from __future__ import annotations

import math
import time

import numpy as np

from flydrones.motor.command import FlightCommand
from flydrones.safety import Telemetry
from mavlink_twin import Ecps295MavlinkDrone


class _Sensors:
    """Camera + ToF access shared by the real and the bench backend."""

    has_camera = True

    def _init_sensors(self, cam, tofs: dict, hfov: float = 60.0, n: int = 8):
        self.cam = cam
        self.tofs = tofs  # {bearing_deg (int): sensor}
        self.frames_new = 0
        self.frames_repeated = 0
        self._last_stamp = 0.0
        self._rng_cache = (None, None, None)  # (key, frame object, result)
        self.tof_hfov, self.tof_n = hfov, n

    def frame(self):
        """Latest frame, never blocking (2026-10-16): the old 5 s wait for a first frame ran inside the control loop and
        the brain warm-up, so without a camera the watchdogs stalled and SIGTERM took minutes. Before the first frame
        this returns None (the retina encodes a blank view) and frame_age_s() is inf: the camera watchdog hovers and
        the hand-over gate refuses control."""
        if self.cam is None:
            return None
        img, st = self.cam.latest()
        if img is None:
            return None
        if st != self._last_stamp:
            self.frames_new += 1
            self._last_stamp = st
        else:
            self.frames_repeated += 1
        return img

    def frame_age_s(self) -> float:
        if self.cam is None or not self.cam.stamp:
            return float("inf")
        return time.monotonic() - self.cam.stamp

    def zones(self, bearing: float, hfov: float = 60.0, vfov: float = 60.0, n: int = 8, rate_hz: float = 15.0,
              noise_m: float = 0.01, max_m: float = 3.5):
        s = self.tofs.get(int(round(bearing)) % 360)
        return None if s is None else s.frame

    def ranges(self, beams, fov_deg=0.0, rate_hz: float = 10.0, noise_m: float = 0.01, **model) -> list[float]:
        s = self.tofs.get(0)
        f = None if s is None else s.frame
        key = tuple(beams)
        if f is None:
            return [2.0] * len(beams)
        if self._rng_cache[0] == key and self._rng_cache[1] is f:
            return self._rng_cache[2]
        g = f.reshape(self.tof_n, self.tof_n)
        pitch = self.tof_hfov / self.tof_n
        out = []
        for b in beams:
            c = int(round((b + self.tof_hfov / 2) / pitch - 0.5))
            col = g[:, min(self.tof_n - 1, max(0, c))]
            d = np.nanmin(col) if np.isfinite(col).any() else np.inf
            out.append(2.0 if not np.isfinite(d) else max(0.0, min(2.0, float(d) - 0.06)))
        self._rng_cache = (key, f, out)
        return out

    def truth_ned(self):
        return None

    def side_ranges(self) -> dict:
        return {"rear": 2.0, "left": 2.0, "right": 2.0}


class RealDrone(_Sensors, Ecps295MavlinkDrone):
    GUIDED = "GUIDED"

    def __init__(self, url: str, cam, tofs: dict, *, allow_cmd: bool = False, ext_height: bool = True,
                 handover_alt_m: float = 0.3, rangefinder_hz: float = 20.0, guard: dict | None = None,
                 max_flight_s: float = 120.0, stop_check=lambda: False,
                 ekf_origin: tuple[float, float, float] = (33.6430, -117.8420, 20.0), **kw):
        self.rangefinder_hz = rangefinder_hz  # set before the base __init__, which connects and requests the streams
        Ecps295MavlinkDrone.__init__(self, url, nav="flow", ext_height=ext_height, origin=tuple(ekf_origin), **kw)
        self._origin_t = 0.0
        self.origin_sent = 0
        self._init_sensors(cam, tofs)
        self.allow_cmd = allow_cmd  # --mode guided
        self.handover_alt_m = handover_alt_m
        self._latched = False
        self.passive = False
        self.frozen = False
        self.cmd_sent = 0
        self.cmd_blocked = 0
        self.land_sent = 0
        self.link_msgs = 0
        g = guard or {}
        self.tof_stale_s = float(g.get("tof_stale_s", 0.3))  # 15 Hz sensors: ~4 missed frames
        self.tof_land_s = float(g.get("tof_land_s", 2.0))
        self.cam_ready_s = float(g.get("camera_ready_s", 0.3))
        self.max_flight_s = float(max_flight_s)
        self.stop_check = stop_check
        self._bad_since: float | None = None
        self._latch_t: float | None = None
        self._hold = False
        self._last_text = 0.0
        self.guard_note = ""
        self.guard_events: list[tuple[float, str]] = []
        self.guard_ticks = {"tof_hover": 0, "tof_land": 0, "time_hold": 0, "handover_refused": 0}

    def _request_streams(self) -> None:
        """As the twin, but RANGEFINDER at rangefinder_hz (2026-10-16): the route-B height is the FC's last RANGEFINDER,
        measured in the Pi SITL from RANGEFINDER arrival to the VISION_POSITION_ESTIMATE sent: mean 47 ms at the twin's
        10 Hz, 24 ms at 20 Hz (+ ~6 ms UART, + the sensor's own delay) -> VISO_DELAY_MS 40. mavlink_twin.py keeps 10 Hz."""
        Ecps295MavlinkDrone._request_streams(self)
        mav = self.mavutil.mavlink
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                     mav.MAVLINK_MSG_ID_RANGEFINDER, 1e6 / self.rangefinder_hz, 0, 0, 0, 0, 0)

    # ---------------------------------------------------------------- freshness (2026-10-16, Gazebo PIL)
    # The route-B height must never come from a stale reading. In the first PIL run the companion's stream requests
    # went to a SITL that was shutting down; the new FC never streamed RANGEFINDER, the cached 0.005 m ground reading
    # was sent as the height for the whole flight, the EKF believed the drone was on the floor and it climbed to 2.9 m.
    # The same happens on the real drone after an FC reboot. Readings older than STALE_S count as missing (no height
    # is sent: the FC's ExtNav times out instead of trusting a frozen value), and the stream requests are repeated
    # every REQ_EVERY_S so a rebooted FC streams again.
    STALE_S = 0.5
    REQ_EVERY_S = 5.0

    def _fresh(self, name: str):
        m = None if self.m is None else self.m.messages.get(name)
        return m if m is not None and time.time() - m._timestamp <= self.STALE_S else None

    def rangefinder_m(self) -> float | None:
        r = self._fresh("RANGEFINDER")
        return None if r is None else r.distance

    def baro_m(self) -> float | None:
        return Ecps295MavlinkDrone.baro_m(self) if self._fresh("SCALED_PRESSURE") is not None else None

    def connect(self) -> None:
        """As the twin's connect, but waits for the FC heartbeat indefinitely (the companion may boot before the FC is
        powered) and gives up only when the process is asked to stop (SIGTERM / SIGINT)."""
        mu = self.mavutil
        if "," in self.conn_str:
            dev, baud = self.conn_str.split(",", 1)
            self.m = mu.mavlink_connection(dev, baud=int(baud), source_system=self.source_system)
        else:
            self.m = mu.mavlink_connection(self.conn_str, source_system=self.source_system)
        t0 = time.time()
        while True:
            if self.stop_check():
                print("stopped while waiting for the flight controller", flush=True)
                raise SystemExit(0)  # a requested stop, not a failure (systemd Restart=on-failure)
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if hb and hb.get_srcSystem() == self.fc_sysid and hb.autopilot != mu.mavlink.MAV_AUTOPILOT_INVALID:
                self.m.target_system, self.m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
                break
            if time.time() - t0 > 10:
                print(f"waiting for the autopilot heartbeat (sysid {self.fc_sysid}) on {self.conn_str} ...", flush=True)
                t0 = time.time()
        print(f"MAVLink heartbeat from system {self.m.target_system}", flush=True)
        self._request_streams()

    # ---------------------------------------------------------------- sensor guard + time hold (2026-10-16)
    # EcpsPilot watches the camera and the route-B height, but nothing watched the two VL53L7CX: a sensor that never
    # initialised gave ranges() = 2.0 m (no echo = clear) and no map input, and a hung I2C bus kept the last frame
    # forever, so the brain and RangeShield flew on a frozen view. Now (companion only, the simulation code unchanged):
    #   hand-over  control is only taken while the camera and every configured ToF delivered within the last
    #              camera_ready_s / tof_stale_s
    #   in flight  a ToF older than tof_stale_s -> hover; still bad after tof_land_s -> LAND
    #   time limit after max_flight_s under our control: hover in place and tell the pilot (STATUSTEXT) instead of the
    #              governor's descend-in-place LAND (PIL: once landed on a box edge); the pilot takes over with the
    #              mode switch. Battery-low and the other watchdog LANDs are unchanged.
    def tof_stale(self) -> str:
        for s in self.tofs.values():
            if s.age_s() > self.tof_stale_s:
                return f"ToF {s.name} stale ({s.status})"
        return ""

    def sensors_ready(self) -> str:
        """'' when the camera and all ToF sensors are fresh and the FC streams its EKF position, otherwise the reason."""
        if self.cam is not None and self.frame_age_s() > self.cam_ready_s:
            return f"camera stale ({getattr(self.cam, 'status', '?')})"
        if self._fresh("LOCAL_POSITION_NED") is None:  # map, fence turn and waypoints all work in the EKF frame
            return "no EKF position"
        return self.tof_stale()

    # ---------------------------------------------------------------- EKF origin (2026-10-17)
    # Without GPS the FC streams no LOCAL_POSITION_NED until an EKF origin is set (mavlink_twin._takeoff_flow does it in
    # the simulation; the companion never took off, so on the real drone nobody did). Without it there is no height for
    # the hand-over gate, no pose for the 3D map, the fence turn or the waypoints. Set it while disarmed, every 2 s until
    # the position stream starts; the origin is fixed, the waypoints are metres from the point where the EKF starts.
    def _keep_origin(self) -> None:
        now = time.monotonic()
        if (self.m is None or self.mute or self.armed() or self._fresh("LOCAL_POSITION_NED") is not None
                or now - self._origin_t < 2.0):
            return
        self._origin_t = now
        lat, lon, alt = self.origin
        self.m.mav.set_gps_global_origin_send(self.m.target_system, int(lat * 1e7), int(lon * 1e7), int(alt * 1000))
        self.origin_sent += 1

    def _gevent(self, what: str) -> None:
        if not self.guard_events or self.guard_events[-1][1] != what:
            self.guard_events.append((round(time.monotonic(), 2), what))
            print(f"[guard] {what}", flush=True)

    def _text(self, msg: str, every_s: float = 5.0) -> None:
        now = time.monotonic()
        if self.m is not None and not self.mute and now - self._last_text >= every_s:
            self._last_text = now
            self.m.mav.statustext_send(4, msg[:50].encode())  # MAV_SEVERITY_WARNING, shown in Mission Planner / OSD

    def _guard(self, cmd):
        """Applied to every command while in control; returns the command to send, or None for LAND."""
        now = time.monotonic()
        bad = self.tof_stale()
        if bad:
            if self._bad_since is None:
                self._bad_since = now
            if now - self._bad_since > self.tof_land_s:
                self.guard_ticks["tof_land"] += 1
                self._gevent(f"{bad} -> land")
                self._text(f"ECPS: {bad[:30]} LAND", 1.0)
                return None
            self.guard_ticks["tof_hover"] += 1
            self._gevent(f"{bad} -> hover")
            self.guard_note = f"guard: {bad}"
            return FlightCommand.hover(self.guard_note)
        self._bad_since = None
        if self._latch_t is not None and now - self._latch_t > self.max_flight_s:
            if not self._hold:
                self._hold = True
                self._gevent(f"flight time {self.max_flight_s:.0f} s -> hover, pilot takes over")
            self.guard_ticks["time_hold"] += 1
            self._text("ECPS: time limit, hovering - take over")
            self.guard_note = "guard: time limit hold"
            return FlightCommand.hover(self.guard_note)
        self.guard_note = ""
        return cmd

    # ---------------------------------------------------------------- gate
    def _fc_state(self):
        """pymavlink's per-sysid state of the flight controller only (never another node's heartbeat)."""
        return None if self.m is None else self.m.sysid_state.get(self.fc_sysid)

    def fc_hb_age_s(self) -> float:
        st = self._fc_state()
        hb = None if st is None else st.messages.get("HEARTBEAT")
        return time.time() - hb._timestamp if hb is not None else float("inf")

    def fc_mode(self) -> str:
        st = self._fc_state()
        return getattr(st, "flightmode", "?") if st is not None else "?"

    def armed(self) -> bool:
        st = self._fc_state()
        return bool(getattr(st, "armed", False)) if st is not None else False

    def in_control(self) -> bool:
        """GUIDED + armed per the FC's own heartbeat, heartbeat younger than 2 s, and launched with --mode guided.
        Hand-over latch (SITL test 2026-10-15): control is only TAKEN while the FC reports >= handover_alt_m (the pilot
        has taken off); once taken it is kept while GUIDED + armed. Arming in GUIDED on the ground therefore never hands
        the motors to the brain (in SITL the companion's setpoints had overridden the GCS take-off)."""
        ok = (self.allow_cmd and not self.mute and self.fc_hb_age_s() < 2.0 and self.fc_mode() == self.GUIDED
              and self.armed())
        if not ok:
            if self._latched:
                self._latch_t, self._hold, self._bad_since = None, False, None
            self._latched = False
        elif not self._latched and (self._tel.alt_m or 0.0) >= self.handover_alt_m:
            why = self.sensors_ready()
            if why:  # never hand the motors to a brain that cannot see (guard, 2026-10-16)
                self.guard_ticks["handover_refused"] += 1
                self._gevent(f"hand-over refused: {why}")
                self._text(f"ECPS: not taking control: {why[:22]}")
            else:
                self._latched = True
                self._latch_t = time.monotonic()
        return ok and self._latched

    def send(self, cmd) -> None:
        if self.in_control():
            cmd = self._guard(cmd)
            if cmd is None:
                return self.land()
            self.cmd_sent += 1
            return Ecps295MavlinkDrone.send(self, cmd)
        self.guard_note = ""
        self.cmd_blocked += 1
        if not self.mute:
            self._heartbeat()
            self.send_ext_height()

    def land(self) -> None:
        if self.in_control():
            self.land_sent += 1
            self.m.set_mode("LAND")

    def takeoff(self) -> None:
        raise RuntimeError("the companion never takes off: the pilot takes off and switches to GUIDED")

    def telemetry(self) -> Telemetry:
        """Called once per tick by EcpsPilot whatever the command path: heartbeat and route-B height go out here, so
        they never depend on send() (a watchdog that requests LAND while we are not in control calls land(), not
        send(); in SITL that starved the heartbeat and the height, and the FC refused to arm: GCS failsafe, VisOdom)."""
        t = Ecps295MavlinkDrone.telemetry(self)
        now = time.monotonic()
        if self.m is not None and now - getattr(self, "_req_t", 0.0) >= self.REQ_EVERY_S:
            self._req_t = now
            self._request_streams()
        if not self.mute:
            self._heartbeat()
            self.send_ext_height()
        self._keep_origin()
        t.flying = self.armed()
        return t


class BenchDrone(_Sensors):
    """No flight controller: hover at 0.8 m at the origin, heading north; commands are only recorded."""

    ext_height = False

    def __init__(self, cam, tofs: dict, v_max: float = 0.5, vz_max: float = 0.3, yaw_rate_max_dps: float = 60.0, **kw):
        self._init_sensors(cam, tofs)
        self.v_max, self.vz_max, self.yr_max = v_max, vz_max, math.radians(yaw_rate_max_dps)
        self._tel = Telemetry(alt_m=0.8, vz_mps=0.0, yaw_deg=0.0, yaw_rate_dps=0.0, x_m=0.0, y_m=0.0, battery_pct=100.0,
                              flying=True)
        self.last_cmd = None
        self.cmd_sent = self.cmd_blocked = self.land_sent = 0
        self.mute = False
        self.m = None

    def connect(self) -> None:
        pass

    def telemetry(self) -> Telemetry:
        self._tel.t = time.monotonic()
        return self._tel

    def rangefinder_m(self):
        return 0.8

    def baro_m(self):
        return 0.8

    def ext_age_s(self) -> float:
        return 0.0

    def in_control(self) -> bool:
        return False

    def fc_mode(self) -> str:
        return "BENCH"

    def armed(self) -> bool:
        return False

    def send(self, cmd) -> None:
        self.last_cmd = cmd
        self.cmd_blocked += 1

    def land(self) -> None:
        self.land_sent += 1
