"""Hover-precision test for run_g4.py --mode hovertest (2026-10-18).

The brain still ticks (passive, logged) but the setpoints come from this script, so the test measures the flight
controller + flow EKF, not the decoder. ECPS_HOLD picks how a hover is commanded:

  vel  (default, what the companion does today) body-frame velocity 0 + yaw rate, SET_POSITION_TARGET_LOCAL_NED
  pos  (step 1) a LOCAL_NED position target at the anchor (the EKF position when the hover block started) + yaw rate,
       so the FC's position loop pulls the drone back to where the EKF believes the anchor is

Plan (simulated seconds): block A = H1 hover 40, Y1 yaw +90 deg (30 deg/s, 3 s), H2 hover 20; then M forward 0.2 m/s
for 2 s; block B = H3 hover 20 (stop and hold), Y2 yaw -90 deg, H4 hover 15. The anchor of a block is taken at its
first segment and kept through the yaw, so 'pos' holds one point per block; metrics are relative to the truth at the
block start (hover_stats.py).
"""

from __future__ import annotations

import math
import os

YAW_DPS = float(os.environ.get("ECPS_YAW_DPS", "30"))  # 2026-10-19 sweep: the 90 deg turns at this rate
PLAN = [
    ("H1", 40.0, "hold", 0.0),
    ("Y1", 90.0 / YAW_DPS, "yaw", YAW_DPS),
    ("H2", 20.0, "hold", 0.0),
    ("M", 2.0, "move", 0.2),
    ("H3", 20.0, "hold", 0.0),
    ("Y2", 90.0 / YAW_DPS, "yaw", -YAW_DPS),
    ("H4", 15.0, "hold", 0.0),
]
BLOCK_START = {"H1", "H3"}

# SET_POSITION_TARGET_LOCAL_NED type_mask bits: 0-2 pos, 3-5 vel, 6-8 acc, 10 yaw, 11 yaw rate (1 = ignore)
MASK_VEL_YAWRATE = 0b0000_0101_1100_0111  # as sitl_util.TYPE_MASK_VEL_YAWRATE
MASK_POS_YAWRATE = 0b0000_0101_1111_1000


class HoverTest:
    def __init__(self, drone, hold: str = "vel"):
        assert hold in ("vel", "pos"), hold
        self.drone, self.hold = drone, hold
        self.kind, self.arg = "hold", 0.0
        self.anchor: tuple[float, float, float] | None = None
        self.sent = 0

    def begin(self, name: str, kind: str, arg: float) -> None:
        self.kind, self.arg = kind, arg
        if kind == "move":
            self.anchor = None
        elif name in BLOCK_START or self.anchor is None:
            lp = self.drone.m.messages.get("LOCAL_POSITION_NED")
            self.anchor = (lp.x, lp.y, lp.z) if lp is not None else None

    def step(self) -> None:
        """Called once per tick: heartbeat + route-B height (passive send() skips them) + the setpoint."""
        d = self.drone
        d._heartbeat()
        d.send_ext_height()
        yaw_rate = math.radians(self.arg) if self.kind == "yaw" else 0.0
        if self.kind == "move":
            self._vel(self.arg, yaw_rate)
        elif self.hold == "pos" and self.anchor is not None:
            n, e, z = self.anchor
            d.m.mav.set_position_target_local_ned_send(
                0, d.m.target_system, d.m.target_component, d.mavutil.mavlink.MAV_FRAME_LOCAL_NED, MASK_POS_YAWRATE,
                n, e, z, 0, 0, 0, 0, 0, 0, 0, yaw_rate)
        else:
            self._vel(0.0, yaw_rate)
        self.sent += 1

    def _vel(self, vx: float, yaw_rate: float) -> None:
        d = self.drone
        d.m.mav.set_position_target_local_ned_send(
            0, d.m.target_system, d.m.target_component, d.mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, MASK_VEL_YAWRATE,
            0, 0, 0, vx, 0, 0, 0, 0, 0, 0, yaw_rate)
