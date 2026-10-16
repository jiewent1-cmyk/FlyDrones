"""MavlinkDrone with the feasibility-study fixes, kept out of upstream code.

F1  request LOCAL_POSITION_NED / ATTITUDE / SYS_STATUS with SET_MESSAGE_INTERVAL; refuse takeoff without telemetry
F3  v_max / vz_max / yaw rate / takeoff_alt come from the caller (YAML), not hard-coded defaults
F4  takeoff waits for EKF, arm (with STATUSTEXT reasons), takeoff COMMAND_ACK and reaching altitude; aborts on timeout
F7  only accept the autopilot's heartbeat (target_system = sysid of a non-GCS heartbeat)
F9  send our own HEARTBEAT at 1 Hz from a dedicated sysid (default 254)
nav="flow"  no GPS (Matek 3901-L0X flow + ToF, sitl/fhb_delta.parm): set the EKF origin, take off in ALT_HOLD with an RC
    throttle override, switch to GUIDED once flow gives a position (same sequence as s7_flow.py)
truth  SIM_STATE (simulator ground truth, SITL only) is streamed so runs score clearance on where the drone really is,
    not on the EKF estimate, which drifts under optical flow
ext_height  (S7e, TechRoute route B) the companion computer supplies the flight controller's height. A box under the
    drone pulled the flow EKF height down 0.4-1 m and the EKF then rejected the true range as an outlier; ArduPilot's
    altitude hold climbed through the 1.3 m fence. The companion knows better: height above the FLOOR from the ToF
    while nothing is under the drone, and baro minus the floor baseline while something is (ecps_ventral.VentralCue,
    the same cue as the brain's LCv input, own instance). Sent as VISION_POSITION_ESTIMATE z at 20 Hz from connect on;
    the FC uses it with EK3_SRC1_POSZ 6 (ExternalNav, sitl/variants/G_extnav_height.parm) and falls back to baro by
    itself if the messages stop for 0.5 s. x/y echo the EKF's own estimate and are not fused (EK3_SRC1_POSXY 0).
"""

from __future__ import annotations

import math
import time

from flydrones.drones.mavlink import MavlinkDrone


class TakeoffError(RuntimeError):
    pass


class Ecps295MavlinkDrone(MavlinkDrone):
    def __init__(
        self,
        connection: str = "tcp:127.0.0.1:5760",
        *,
        source_system: int = 254,
        fc_sysid: int = 1,
        stream_hz: float = 20.0,
        takeoff_timeout_s: float = 90.0,
        nav: str = "gps",
        origin: tuple[float, float, float] = (33.6430, -117.8420, 20.0),
        ext_height: bool = False,
        **kw,
    ):
        super().__init__(connection, **kw)
        self.source_system = source_system
        self.fc_sysid = fc_sysid
        self.stream_hz = stream_hz
        self.takeoff_timeout_s = takeoff_timeout_s
        self._last_hb = 0.0
        self.nav = nav
        self.origin = origin
        self.rng_m: float | None = None
        self._p0: float | None = None  # first SCALED_PRESSURE (hPa): baro_m() is relative to it
        self.ext_height = ext_height
        self._surface = None
        if ext_height:
            from ecps_ventral import VentralCue

            self._surface = VentralCue()
        self._ext_last = 0.0
        self.ext_h_m: float | None = None  # last height sent to the FC
        self.ext_src = ""  # "tof" / "baro" / "ground"
        # fault injection (S7b): ext_paused stops only the height messages, mute stops everything the companion sends
        # (height, velocity setpoints, heartbeat) as if the Orange Pi had hung; telemetry is still read for scoring
        self.ext_paused = False
        self._ext_sent = 0.0  # monotonic time of the last height actually sent (EcpsPilot height-source watchdog)
        self.mute = False
        # N4 stress (2026-10-15): ECPS_DRIFT_MPS > 0 adds a steady world-frame velocity (direction drift_dir, rad from north,
        # set by run_g4 from the brain seed) to every setpoint sent through send(): an uncompensated drift the companion did
        # not command, so the range shield never sees it (rl.twin ood="wind" without the gusts)
        import os

        self.drift_mps = float(os.environ.get("ECPS_DRIFT_MPS", "0") or 0.0)
        self.drift_dir = 0.0

    # ------------------------------------------------------------------ link
    def connect(self) -> None:
        mu = self.mavutil
        if "," in self.conn_str:
            dev, baud = self.conn_str.split(",", 1)
            self.m = mu.mavlink_connection(dev, baud=int(baud), source_system=self.source_system)
        else:
            self.m = mu.mavlink_connection(self.conn_str, source_system=self.source_system)
        # F7: wait for the flight controller, not any GCS heartbeat
        t0 = time.time()
        while time.time() - t0 < 30:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb and hb.get_srcSystem() == self.fc_sysid and hb.autopilot != mu.mavlink.MAV_AUTOPILOT_INVALID:
                self.m.target_system, self.m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
                break
        else:
            raise SystemExit(f"no autopilot heartbeat from sysid {self.fc_sysid}")
        print(f"MAVLink heartbeat from system {self.m.target_system}")
        self._request_streams()

    def _request_streams(self) -> None:
        # F1
        mav = self.mavutil.mavlink
        for mid, hz in (
            (mav.MAVLINK_MSG_ID_LOCAL_POSITION_NED, self.stream_hz),
            (mav.MAVLINK_MSG_ID_ATTITUDE, self.stream_hz),
            (mav.MAVLINK_MSG_ID_SYS_STATUS, 2.0),
            (mav.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2.0),
            (mav.MAVLINK_MSG_ID_SIM_STATE, self.stream_hz),
            (mav.MAVLINK_MSG_ID_RANGEFINDER, 10.0),
            (mav.MAVLINK_MSG_ID_SCALED_PRESSURE, self.stream_hz),
            (mav.MAVLINK_MSG_ID_RC_CHANNELS, 2.0),
        ):
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, 1e6 / hz, 0, 0, 0, 0, 0
            )

    def _heartbeat(self) -> None:
        # F9
        if self.mute:
            return
        now = time.monotonic()
        if now - self._last_hb >= 1.0:
            mav = self.mavutil.mavlink
            self.m.mav.heartbeat_send(mav.MAV_TYPE_ONBOARD_CONTROLLER, mav.MAV_AUTOPILOT_INVALID, 0, 0, mav.MAV_STATE_ACTIVE)
            self._last_hb = now

    def _pump(self, timeout: float = 0.1):
        self._heartbeat()
        self.send_ext_height()
        msg = self.m.recv_match(blocking=True, timeout=timeout)
        if msg is not None:
            k = msg.get_type()
            if k == "STATUSTEXT":
                print("  [FC]", msg.text)
            elif k == "LOCAL_POSITION_NED":
                self._tel.x_m, self._tel.y_m, self._tel.alt_m, self._tel.vz_mps = msg.x, msg.y, -msg.z, -msg.vz
            elif k == "RANGEFINDER":
                self.rng_m = msg.distance
        return msg

    def _rc_throttle(self, pwm: int | None) -> None:
        """RC override: roll/pitch/yaw centred, throttle pwm; None releases all channels."""
        ch = (0, 0, 0, 0) if pwm is None else (1500, 1500, pwm, 1500)
        # overrides are only accepted from MAV_GCS_SYSID (255): the take-off stands in for the pilot / GCS, so send
        # these as the GCS and keep our own sysid for everything else
        own, self.m.mav.srcSystem = self.m.mav.srcSystem, 255
        try:
            self.m.mav.rc_channels_override_send(self.m.target_system, self.m.target_component, *ch, 0, 0, 0, 0)
        finally:
            self.m.mav.srcSystem = own

    def _ekf_flags(self) -> int:
        e = self.m.messages.get("EKF_STATUS_REPORT")
        return e.flags if e is not None else 0

    def _takeoff_flow(self) -> None:
        m = self.m
        deadline = time.time() + self.takeoff_timeout_s
        while time.time() < deadline and not self._ekf_flags() & 0x01:
            self._pump()
        if not self._ekf_flags() & 0x01:
            raise TakeoffError("EKF attitude not ready")
        lat, lon, alt = self.origin
        last_origin = 0.0
        while time.time() < deadline and self._tel.alt_m is None:  # LOCAL_POSITION_NED starts once the origin is set
            if time.time() - last_origin > 2:
                m.mav.set_gps_global_origin_send(m.target_system, int(lat * 1e7), int(lon * 1e7), int(alt * 1000))
                last_origin = time.time()
            self._pump()
        if self._tel.alt_m is None:
            raise TakeoffError("no LOCAL_POSITION_NED after setting the EKF origin")
        print(f"flow take-off: EKF origin set, flags 0x{self._ekf_flags():04x}, rangefinder {self.rng_m}", flush=True)
        # GUIDED take-off as with GPS once the EKF expects a relative position (0x08 now, or 0x100 "predicted" after
        # lift-off): ALT_HOLD has no horizontal hold and a 2.5 deg pitch bias drifted the drone into a box at 0.9 m/s
        t1 = time.time()
        while time.time() < deadline and time.time() - t1 < 10 and not self._ekf_flags() & 0x108:
            self._pump()
        print(f"flow take-off: EKF flags 0x{self._ekf_flags():04x} after {time.time() - t1:.1f} s", flush=True)
        if self._ekf_flags() & 0x108:
            return self._takeoff_guided(deadline, ekf_need=0x01 | 0x02 | 0x04)
        m.set_mode("ALT_HOLD")
        while time.time() < deadline and not m.motors_armed():
            self._rc_throttle(1000)
            m.arducopter_arm()
            t1 = time.time()
            while time.time() - t1 < 1.5 and not m.motors_armed():
                self._rc_throttle(1000)
                self._pump()
        if not m.motors_armed():
            raise TakeoffError("arming failed (see [FC] messages)")
        print("flow take-off: armed in ALT_HOLD, climbing", flush=True)
        # ALT_HOLD has no horizontal hold, so leave it as soon as GUIDED is accepted (flow is fused once airborne)
        guided, last_try = 4, 0.0  # ArduCopter custom_mode
        while time.time() < deadline:
            self._rc_throttle(1700 if (self._tel.alt_m or 0.0) < 0.95 * self.takeoff_alt else 1500)
            msg = self._pump()
            if (self._tel.alt_m or 0.0) > 0.10 and time.time() - last_try > 0.5:
                m.set_mode("GUIDED")
                last_try = time.time()
            if msg is not None and msg.get_type() == "HEARTBEAT" and msg.get_srcSystem() == m.target_system:
                if msg.custom_mode == guided:
                    break
        else:
            raise TakeoffError("could not enter GUIDED on optical flow")
        print(f"flow take-off: GUIDED at {self._tel.alt_m:.2f} m (rangefinder {self.rng_m})", flush=True)
        self._rc_throttle(None)

        def low() -> bool:  # stop on either: the EKF height lagged the ToF by 0.8 m in a climb and hit the 1.3 m fence
            rng = self.rangefinder_m()
            return (self._tel.alt_m or 0.0) < 0.95 * self.takeoff_alt and (rng is None or rng < 0.95 * self.takeoff_alt)

        while time.time() < deadline and low():
            self._send_velocity(0.0, 0.0, -0.3, 0.0)
            self._pump()
        t1 = time.time()
        while time.time() - t1 < 1.0:
            self._send_velocity(0.0, 0.0, 0.0, 0.0)
            self._pump()
        self.flying = True

    def rangefinder_m(self) -> float | None:
        r = self.m.messages.get("RANGEFINDER")
        return r.distance if r is not None else None

    def baro_m(self) -> float | None:
        """Barometric height (m) relative to the first pressure seen; independent of the ToF and the EKF (v4 LCv)."""
        p = self.m.messages.get("SCALED_PRESSURE")
        if p is None or p.press_abs <= 0:
            return None
        if self._p0 is None:
            self._p0 = p.press_abs
        return 44330.0 * (1.0 - (p.press_abs / self._p0) ** (1 / 5.255))

    def truth_ned(self) -> tuple[float, float, float] | None:
        """Simulator ground truth (north, east, height above the origin) from SIM_STATE, or None."""
        s = self.m.messages.get("SIM_STATE")
        if s is None:
            return None
        lat0, lon0, alt0 = self.origin
        k = 111319.49 * 1e-7  # m per 1e-7 deg of latitude
        n = (s.lat_int - lat0 * 1e7) * k
        e = (s.lon_int - lon0 * 1e7) * k * math.cos(math.radians(lat0))
        return n, e, s.alt - alt0

    # ------------------------------------------------------------------ S3 / S4 rangers (route review 2026-10-13)
    def _ray(self, n: float, e: float, a: float, ang: float, skip: frozenset = frozenset()) -> float:
        """Horizontal distance from (n, e) at height a to the first layout box (nets included) along bearing ang, max
        2 m; the same slab test as rl.twin.TwinDrone._ray. `skip`: box names this ray does not see (S6 camera model)."""
        best = 2.0
        dn, de = math.cos(ang), math.sin(ang)
        for o in getattr(getattr(self, "room", None), "boxes", None) or []:
            if not o.lo[2] <= a <= o.hi[2] or (skip and _box_name(o) in skip):
                continue
            t0, t1 = 0.0, best
            for p, d, lo, hi in ((n, dn, o.lo[0], o.hi[0]), (e, de, o.lo[1], o.hi[1])):
                if abs(d) < 1e-9:
                    if not lo <= p <= hi:
                        t0 = math.inf
                        break
                else:
                    u, v = (lo - p) / d, (hi - p) / d
                    t0, t1 = max(t0, min(u, v)), min(t1, max(u, v))
            if t0 <= t1:
                best = min(best, t0)
        return best

    def _ray3(self, p: tuple, d: tuple) -> float:
        """3D slab test from p = (north, east, up) along the unit vector d (north, east, up) against the layout boxes
        and, in flight, the floor; max 2 m."""
        best = min(2.0, p[2] / -d[2]) if d[2] < -1e-9 and p[2] > 0.3 else 2.0
        for o in getattr(getattr(self, "room", None), "boxes", None) or []:
            t0, t1 = 0.0, best
            for i in range(3):
                if abs(d[i]) < 1e-9:
                    if not o.lo[i] <= p[i] <= o.hi[i]:
                        t0 = math.inf
                        break
                else:
                    u, v = (o.lo[i] - p[i]) / d[i], (o.hi[i] - p[i]) / d[i]
                    t0, t1 = max(t0, min(u, v)), min(t1, max(u, v))
            if t0 <= t1:
                best = min(best, t0)
        return best

    def _cone3(self, n: float, e: float, a: float, s, b: float, h: float) -> float:
        """Shortest slant range over the whole ToF cone (axis + 8 rim rays at half-angle h) for body bearing b (rad),
        rotated by the SIM_STATE attitude. Unlike the planar fan it also sees box tops just below the sensor."""
        cr, sr, cp, sp = math.cos(s.roll), math.sin(s.roll), math.cos(s.pitch), math.sin(s.pitch)
        cy, sy = math.cos(s.yaw), math.sin(s.yaw)
        best = 2.0
        for k in range(9):
            r, phi = (0.0, 0.0) if k == 0 else (h, (k - 1) * math.pi / 4)
            az, el = b + r * math.cos(phi), r * math.sin(phi)
            x, y, z = math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), -math.sin(el)  # body FRD
            dn = cp * cy * x + (sr * sp * cy - cr * sy) * y + (cr * sp * cy + sr * sy) * z
            de = cp * sy * x + (sr * sp * sy + cr * cy) * y + (cr * sp * sy - sr * cy) * z
            dd = -sp * x + sr * cp * y + cr * cp * z
            best = min(best, self._ray3((n, e, a), (dn, de, -dd)))
        return best

    def ranges(self, beams, fov_deg=0.0, rate_hz: float = 10.0, noise_m: float = 0.01, **model) -> list[float]:
        """Horizontal ToF beams (body bearings in deg, + = right) emulated from SIM_STATE truth and the world layout
        (run_g4 --layout sets self.room): same geometry, cone, noise and sample-and-hold as rl.twin.TwinDrone.ranges().
        SITL only; reads 2.0 (no echo) until truth arrives.
        S6 (2026-10-14): fov_deg may be a per-beam list and `model` adds the camera-style range model, as in the twin;
        see _ranges_model() for the geometry per sensor kind. Without either the old path runs unchanged."""
        import os
        import random

        if not hasattr(self, "_beams"):
            self._beams = {}
        fk = tuple(float(f) for f in fov_deg) if isinstance(fov_deg, (list, tuple)) else fov_deg
        key = (tuple(beams), fk, rate_hz, noise_m, tuple(sorted(model.items())))
        st = self._beams.get(key)
        if st is None:
            st = self._beams[key] = [-1.0, [2.0] * len(beams), random.Random(86_000 + len(self._beams))]
        now = time.monotonic()
        if now - st[0] >= 1.0 / rate_hz - 1e-6:
            tru = self.truth_ned()
            s = self.m.messages.get("SIM_STATE") if self.m is not None else None
            if tru is not None and s is not None:
                st[0] = now
                n, e, a = tru
                if model or isinstance(fk, tuple):
                    st[1] = self._ranges_model(n, e, a, s, beams, fk, noise_m, st, **model)
                else:
                    h = math.radians(fov_deg) / 2
                    if h > 0 and os.environ.get("ECPS_TOF_CONE3D") == "1":  # N1b: full cone + attitude (box tops, floor);
                        # S7: a multi-zone column (fov < 10 deg = zone pitch) is its 8 zone rays, as in _ranges_model
                        raw = [self._zones3(n, e, a, s, math.radians(b), math.radians(fov_deg)) if fov_deg < 10.0
                               else self._cone3(n, e, a, s, math.radians(b), h) for b in beams]
                    else:
                        offs = (-h, 0.0, h) if h > 0 else (0.0,)
                        raw = [min(self._ray(n, e, a, s.yaw + math.radians(b) + o) for o in offs) for b in beams]
                    st[1] = [max(0.0, min(2.0, r - 0.06 + (st[2].gauss(0.0, noise_m) if noise_m > 0 else 0.0))) for r in raw]
        return st[1]

    def _ranges_model(self, n, e, a, s, beams, fov, noise_m, st, noise_k=0.0, max_m=2.0, p_miss=0.0, p_net=1.0):
        """S6 per-beam geometry (with ECPS_TOF_CONE3D=1; planar otherwise): single-zone ToF (fov >= 10 deg) = the full
        cone (_cone3); multi-zone column (0 < fov < 10 deg, zone pitch = fov) = the 8 zone rays of that 8x8 column
        (_zones3), nearest of the column; 2D lidar (fov 0) and the camera model (noise_k > 0, ground-contact-line
        ranging at flight height) stay planar. Camera model as in the twin: sigma = noise_m + noise_k d^2, nothing
        beyond max_m, p_miss lost readings, each net wall seen with p_net for the whole run."""
        import os

        rng = st[2]
        if len(st) < 4:
            boxes = getattr(getattr(self, "room", None), "boxes", None) or []
            st.append(frozenset(_box_name(o) for o in boxes if _box_name(o).startswith("net_") and rng.random() >= p_net))
        skip = st[3]
        cone = os.environ.get("ECPS_TOF_CONE3D") == "1"
        fovs = fov if isinstance(fov, tuple) else (float(fov),) * len(beams)
        out = []
        for b, fv in zip(beams, fovs):
            br, h = math.radians(b), math.radians(fv) / 2
            if cone and noise_k == 0 and fv >= 10.0:
                d = self._cone3(n, e, a, s, br, h)
            elif cone and noise_k == 0 and 0 < fv < 10.0:
                d = self._zones3(n, e, a, s, br, math.radians(fv))
            else:
                offs = (-h, 0.0, h) if h > 0 else (0.0,)
                d = min(self._ray(n, e, a, s.yaw + br + o, skip) for o in offs)
            d -= 0.06
            sd = noise_m + noise_k * d * d
            if sd > 0:
                d += rng.gauss(0.0, sd)
            if (p_miss > 0 and rng.random() < p_miss) or d > max_m:
                d = 2.0
            out.append(max(0.0, min(2.0, d)))
        return out

    def _zones3(self, n: float, e: float, a: float, s, b: float, pitch: float) -> float:
        """S6 multi-zone ToF column (VL53L5CX / L7CX 8x8): the 8 zone-centre rays of the column at body bearing b,
        elevations (k - 3.5) * pitch, rotated by the SIM_STATE attitude like _cone3; nearest of the column (box tops
        and, low enough, the floor included)."""
        cr, sr, cp, sp = math.cos(s.roll), math.sin(s.roll), math.cos(s.pitch), math.sin(s.pitch)
        cy, sy = math.cos(s.yaw), math.sin(s.yaw)
        best = 2.0
        for k in range(8):
            el = (k - 3.5) * pitch
            x, y, z = math.cos(el) * math.cos(b), math.cos(el) * math.sin(b), -math.sin(el)  # body FRD
            dn = cp * cy * x + (sr * sp * cy - cr * sy) * y + (cr * sp * cy + sr * sy) * z
            de = cp * sy * x + (sr * sp * sy + cr * cy) * y + (cr * sp * sy - sr * cy) * z
            dd = -sp * x + sr * cp * y + cr * cp * z
            best = min(best, self._ray3((n, e, a), (dn, de, -dd)))
        return best

    def zones(self, bearing: float, hfov: float, vfov: float, n: int = 8, rate_hz: float = 15.0, noise_m: float = 0.01,
              max_m: float = 3.5):
        """S7 (2026-10-14): one multi-zone ToF (n x n zones, level mount) facing body bearing `bearing` (deg), emulated
        from SIM_STATE truth (position + attitude) and the layout boxes + floor, as rl.twin.TwinDrone.zones(): slant range
        per zone-centre ray (row 0 = top), nan beyond max_m; None until truth arrives."""
        import numpy as np

        from ecps_map import zone_dirs

        if not hasattr(self, "_zones"):
            self._zones = {}
        key = (bearing, hfov, vfov, n, rate_hz, noise_m, max_m)
        st = self._zones.get(key)
        if st is None:
            st = self._zones[key] = [-1.0, None, np.random.default_rng(87_000 + len(self._zones)), zone_dirs(bearing, hfov, vfov, n)]
        now = time.monotonic()
        if now - st[0] >= 1.0 / rate_hz - 1e-6:
            tru = self.truth_ned()
            s = self.m.messages.get("SIM_STATE") if self.m is not None else None
            if tru is not None and s is not None:
                st[0] = now
                tn, te, ta = tru
                cr, sr, cp, sp = math.cos(s.roll), math.sin(s.roll), math.cos(s.pitch), math.sin(s.pitch)
                cy, sy = math.cos(s.yaw), math.sin(s.yaw)
                B = st[3]
                x, y, z = B[:, 0], B[:, 1], -B[:, 2]  # body FRD
                dn = cp * cy * x + (sr * sp * cy - cr * sy) * y + (cr * sp * cy + sr * sy) * z
                de = cp * sy * x + (sr * sp * sy + cr * cy) * y + (cr * sp * sy - sr * cy) * z
                dd = -sp * x + sr * cp * y + cr * cp * z
                D = np.stack([dn, de, -dd], axis=1)
                b = s.yaw + math.radians(bearing)
                o = np.array([tn + 0.06 * math.cos(b), te + 0.06 * math.sin(b), ta])
                r = self._rays3np(o, D)
                if noise_m > 0:
                    r = r + st[2].normal(0.0, noise_m, len(r))
                st[1] = np.where(r > max_m, np.nan, np.maximum(r, 0.0))
        return st[1]

    def _rays3np(self, o, D):
        """Vectorised _ray3: distance along each unit ray D (N, 3) (north, east, up) from o to the first layout box or,
        in flight, the floor; inf if none."""
        import numpy as np

        if not hasattr(self, "_box_lo"):
            boxes = getattr(getattr(self, "room", None), "boxes", None) or []
            self._box_lo = np.array([b.lo for b in boxes], float).reshape(-1, 3)
            self._box_hi = np.array([b.hi for b in boxes], float).reshape(-1, 3)
        t = np.full(len(D), np.inf)
        if len(self._box_lo):
            Dz = np.where(np.abs(D) < 1e-12, 1e-12, D)
            t1 = (self._box_lo[None, :, :] - o[None, None, :]) / Dz[:, None, :]
            t2 = (self._box_hi[None, :, :] - o[None, None, :]) / Dz[:, None, :]
            tmin = np.maximum(np.minimum(t1, t2).max(axis=2), 0.0)
            tmax = np.maximum(t1, t2).min(axis=2)
            t = np.where(tmax >= tmin, tmin, np.inf).min(axis=1)
        if o[2] > 0.3:
            t = np.minimum(t, np.where(D[:, 2] < -1e-9, o[2] / np.maximum(-D[:, 2], 1e-9), np.inf))
        return t

    def side_ranges(self) -> dict:
        """S3 rear / left / right rangers (rl.twin.TwinDrone.side_ranges: single rays, 10 Hz, 1 cm noise)."""
        r = self.ranges((180.0, -90.0, 90.0), 0.0, 10.0, 0.01)
        return {"rear": r[0], "left": r[1], "right": r[2]}

    # ------------------------------------------------------------------ takeoff (F4)
    def takeoff(self) -> None:
        if self.autopilot != "ardupilot":
            return super().takeoff()
        if self.nav == "flow":
            return self._takeoff_flow()
        return self._takeoff_guided(time.time() + self.takeoff_timeout_s)

    def _takeoff_guided(self, deadline: float, ekf_need: int = 0x01 | 0x02 | 0x04 | 0x08) -> None:
        mav = self.mavutil.mavlink
        m = self.m
        ekf_ok = False
        while time.time() < deadline and not ekf_ok:
            msg = self._pump()
            ekf_ok = bool(
                msg is not None
                and msg.get_type() == "EKF_STATUS_REPORT"
                and (msg.flags & ekf_need) == ekf_need
                and not msg.flags & (1 << 7)
            )
        if not ekf_ok:
            raise TakeoffError("EKF not ready")
        if self._tel.alt_m is None:
            raise TakeoffError("no LOCAL_POSITION_NED telemetry (F1): refusing to fly without altitude")
        m.set_mode("GUIDED")
        while time.time() < deadline and not m.motors_armed():
            m.arducopter_arm()
            t1 = time.time()
            while time.time() - t1 < 1.5 and not m.motors_armed():
                self._pump()
        if not m.motors_armed():
            raise TakeoffError("arming failed (see [FC] messages)")
        m.mav.command_long_send(
            m.target_system, m.target_component, mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, self.takeoff_alt
        )
        acked = False
        while time.time() < deadline:
            msg = self._pump()
            if msg is None:
                continue
            k = msg.get_type()
            if k == "COMMAND_ACK" and msg.command == mav.MAV_CMD_NAV_TAKEOFF:
                if msg.result != mav.MAV_RESULT_ACCEPTED:
                    raise TakeoffError(f"takeoff rejected, result={msg.result}")
                acked = True
            elif k == "LOCAL_POSITION_NED":
                if acked and -msg.z > 0.95 * self.takeoff_alt and abs(msg.vz) < 0.1:
                    self.flying = True
                    return
        raise TakeoffError("did not reach takeoff altitude")

    def send(self, cmd) -> None:
        if self.mute:
            return
        self._heartbeat()
        self.send_ext_height()
        if self.drift_mps > 0 and self.m is not None:
            att = self.m.messages.get("ATTITUDE")
            yw = att.yaw if att is not None else 0.0
            dn, de = self.drift_mps * math.cos(self.drift_dir), self.drift_mps * math.sin(self.drift_dir)
            self._send_velocity(cmd.forward * self.v_max + math.cos(yw) * dn + math.sin(yw) * de,
                                cmd.lateral * self.v_max - math.sin(yw) * dn + math.cos(yw) * de,
                                -cmd.throttle * self.vz_max, cmd.yaw * self.yr_max)
            return
        super().send(cmd)

    def floor_height(self) -> tuple[float | None, str]:
        """Height of the ToF above the floor: the tilt-corrected range while nothing is under the drone, otherwise
        (obstacle under it, or out of range) baro minus the floor baseline."""
        rng, baro = self.rangefinder_m(), self.baro_m()
        att = self.m.messages.get("ATTITUDE")
        tilt = math.cos(att.roll) * math.cos(att.pitch) if att is not None else 1.0
        now = time.monotonic()
        dt = min(0.2, now - self._ext_last) if self._ext_last else 0.05
        level = self._surface.update(now, rng, baro, dt)
        if rng is not None and rng < self._surface.range_valid[0]:
            return max(0.0, rng * tilt), "ground"  # on the ground (below the cue's valid range)
        if level == 0.0 and rng is not None and rng <= self._surface.range_valid[1]:
            return rng * tilt, "tof"
        h = self._surface.height_above_floor(baro)
        return (None, "") if h is None else (h, "baro")

    def send_ext_height(self) -> None:
        if not self.ext_height or self.m is None or self.mute:
            return
        if self.ext_paused:
            self.ext_src = "paused"
            return
        now = time.monotonic()
        if now - self._ext_last < 0.05:  # 20 Hz
            return
        h, src = self.floor_height()
        self._ext_last = now
        if h is None:
            return
        self.ext_h_m, self.ext_src = h, src
        lp = self.m.messages.get("LOCAL_POSITION_NED")
        att = self.m.messages.get("ATTITUDE")
        x, y = (lp.x, lp.y) if lp is not None else (0.0, 0.0)
        r, p, yw = (att.roll, att.pitch, att.yaw) if att is not None else (0.0, 0.0, 0.0)
        self.m.mav.vision_position_estimate_send(int(now * 1e6), x, y, -h, r, p, yw)
        self._ext_sent = now

    def ext_age_s(self) -> float:
        """Seconds since the companion last sent the FC its height (inf before the first, 0 when not in use)."""
        if not self.ext_height:
            return 0.0
        return time.monotonic() - self._ext_sent if self._ext_sent else float("inf")


def _box_name(o) -> str:
    """Layout box name (world_layout.room_from_layout passes the layout name to flydrones Box), '' if none."""
    for k in ("name", "label", "id"):
        v = getattr(o, k, None)
        if isinstance(v, str):
            return v
    return ""
