"""MavlinkDrone with the feasibility-study fixes, kept out of upstream code.

F1  request LOCAL_POSITION_NED / ATTITUDE / SYS_STATUS with SET_MESSAGE_INTERVAL; refuse takeoff without telemetry
F3  v_max / vz_max / yaw rate / takeoff_alt come from the caller (YAML), not hard-coded defaults
F4  takeoff waits for EKF, arm (with STATUSTEXT reasons), takeoff COMMAND_ACK and reaching altitude; aborts on timeout
F7  only accept the autopilot's heartbeat (target_system = sysid of a non-GCS heartbeat)
F9  send our own HEARTBEAT at 1 Hz from a dedicated sysid (default 254)
"""

from __future__ import annotations

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
        **kw,
    ):
        super().__init__(connection, **kw)
        self.source_system = source_system
        self.fc_sysid = fc_sysid
        self.stream_hz = stream_hz
        self.takeoff_timeout_s = takeoff_timeout_s
        self._last_hb = 0.0

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
        return msg

    # ------------------------------------------------------------------ takeoff (F4)
    def takeoff(self) -> None:
        if self.autopilot != "ardupilot":
            return super().takeoff()
        mav = self.mavutil.mavlink
        m = self.m
        deadline = time.time() + self.takeoff_timeout_s
        ekf_need = 0x01 | 0x02 | 0x04 | 0x08
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
