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
            (mav.MAVLINK_MSG_ID_RC_CHANNELS, 2.0),
        ):
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, 1e6 / hz, 0, 0, 0, 0, 0
            )

    def _heartbeat(self) -> None:
        # F9
        now = time.monotonic()
        if now - self._last_hb >= 1.0:
            mav = self.mavutil.mavlink
            self.m.mav.heartbeat_send(mav.MAV_TYPE_ONBOARD_CONTROLLER, mav.MAV_AUTOPILOT_INVALID, 0, 0, mav.MAV_STATE_ACTIVE)
            self._last_hb = now

    def _pump(self, timeout: float = 0.1):
        self._heartbeat()
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
        self._heartbeat()
        super().send(cmd)
