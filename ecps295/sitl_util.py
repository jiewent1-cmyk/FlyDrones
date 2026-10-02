"""Shared pymavlink helpers for ECPS295 SITL experiments."""

from __future__ import annotations

import time

from pymavlink import mavutil

MAV = mavutil.mavlink
# body-frame velocity + yaw rate, same as flydrones.drones.mavlink
TYPE_MASK_VEL_YAWRATE = 0b0000_0101_1100_0111


def connect(url: str = "tcp:127.0.0.1:5760", rates: dict | None = None):
    m = mavutil.mavlink_connection(url, source_system=255)
    m.wait_heartbeat(timeout=60)
    rates = rates or {"LOCAL_POSITION_NED": 50, "ATTITUDE": 50, "SYS_STATUS": 2, "EKF_STATUS_REPORT": 2}
    for name, hz in rates.items():
        set_rate(m, name, hz)
    return m


def set_rate(m, name: str, hz: float) -> None:
    mid = getattr(MAV, f"MAVLINK_MSG_ID_{name}")
    m.mav.command_long_send(
        m.target_system, m.target_component, MAV.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, 1e6 / hz, 0, 0, 0, 0, 0
    )


def drain_text(m) -> None:
    while s := m.recv_match(type="STATUSTEXT", blocking=False):
        print("  [FC]", s.text, flush=True)


def get_param(m, name: str):
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < 5:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg and msg.param_id == name:
            return msg.param_value
    return None


def set_param(m, name: str, value: float) -> None:
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(), value, MAV.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.2)


def wait_ekf(m, timeout: float = 120, need_abs: bool = True) -> float:
    """Wait until EKF reports attitude + velocity + relative horizontal position (and absolute if need_abs)."""
    need = 0x01 | 0x02 | 0x04 | 0x08 | (0x10 if need_abs else 0)
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = m.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=2)
        drain_text(m)
        if r and (r.flags & need) == need and not (r.flags & (1 << 7)):
            return time.time() - t0
    raise SystemExit(f"EKF not ready after {timeout}s")


def arm_takeoff(m, alt: float, settle: float = 12) -> None:
    m.set_mode("GUIDED")
    for _ in range(30):
        m.arducopter_arm()
        m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        drain_text(m)
        if m.motors_armed():
            break
        time.sleep(1)
    if not m.motors_armed():
        raise SystemExit("ARM FAILED")
    m.mav.command_long_send(m.target_system, m.target_component, MAV.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    t0 = time.time()
    while time.time() - t0 < settle:
        m.recv_match(blocking=True, timeout=0.5)
        drain_text(m)


def wait_steady(m, alt: float, tol_v: float = 0.15, tol_alt: float = 0.05, hold: float = 3.0, timeout: float = 40) -> float:
    """Stream zero-velocity setpoints until |v| < tol_v and |alt - target| < tol_alt for `hold` seconds."""
    t0 = time.time()
    ok_since = None
    nxt = 0.0
    while time.time() - t0 < timeout:
        if time.time() >= nxt:
            send_vel(m, 0, 0, 0, 0)
            nxt = time.time() + 0.05
        p = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.1)
        drain_text(m)
        if p is None:
            continue
        steady = (p.vx**2 + p.vy**2 + p.vz**2) ** 0.5 < tol_v and abs(-p.z - alt) < tol_alt
        ok_since = (ok_since or time.time()) if steady else None
        if ok_since and time.time() - ok_since > hold:
            return time.time() - t0
    raise SystemExit(f"not steady after {timeout}s")


def send_vel(m, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
    m.mav.set_position_target_local_ned_send(
        0,
        m.target_system,
        m.target_component,
        MAV.MAV_FRAME_BODY_OFFSET_NED,
        TYPE_MASK_VEL_YAWRATE,
        0,
        0,
        0,
        vx,
        vy,
        vz,
        0,
        0,
        0,
        0,
        yaw_rate,
    )


def land(m, timeout: float = 60) -> bool:
    m.set_mode("LAND")
    t0 = time.time()
    while time.time() - t0 < timeout and m.motors_armed():
        m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        drain_text(m)
    return not m.motors_armed()
