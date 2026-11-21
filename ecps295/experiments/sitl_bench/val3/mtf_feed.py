#!/usr/bin/env python3
"""MTF-01 MAVLink feeder for SITL (task V3).

Sends OPTICAL_FLOW (MAVLink id 100) and DISTANCE_SENSOR (id 132) to the ArduPilot
SITL instance, like the real MTF-01 module does over its own MAVLink link.

Two modes:
  fixed : constant distance / constant flow values (part 1, driver acceptance tests)
  truth : distance/flow computed from the SITL truth (SIM_STATE), part 2 closed loop

Usage:
  mtf_feed.py --url tcp:127.0.0.1:6160 [--mode fixed|truth] ...
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

from pymavlink import mavutil

# ---------------------------------------------------------------------------
# port of libraries/AP_OpticalFlow/AP_OpticalFlow_SITL.cpp (ArduPilot 4.7.0)
# lines 30-105: same rotation / velocity / range / flow maths.
#   rotmat from euler (roll,pitch,yaw)        -> SITL.cpp:53-56
#   range = (height_agl - relPosSensorEF.z)/rotmat.c.z   -> SITL.cpp:63-70
#   relVelSensor = rotmat^T * velocity_NED                -> SITL.cpp:78
#   relVelSensor += gyro % posRelSensorBF                 -> SITL.cpp:81
#   flowRate.x = (-relVelSensor.y/range + gyro.x) * fsx  -> SITL.cpp:96
#   flowRate.y = ( relVelSensor.x/range + gyro.y) * fsy  -> SITL.cpp:97
# ---------------------------------------------------------------------------


def euler_to_dcm(roll, pitch, yaw):
    """Body->earth rotation matrix, exactly Matrix3f::from_euler().

    Source: libraries/AP_Math/matrix3.cpp:26-44 (a, b, c are the columns of the matrix):
        a = (cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy)
        b = (cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy)
        c = (-sp,   sr*cp,             cr*cp)
    so that rotmat.c.z (AP_OpticalFlow_SITL.cpp:67) = cr*cp, and the rows are:
        row0 = (a.x, b.x, c.x), row1 = (a.y, b.y, c.y), row2 = (a.z, b.z, c.z)
    """
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    r00 = cp * cy
    r01 = cp * sy
    r02 = -sp
    r10 = sr * sp * cy - cr * sy
    r11 = sr * sp * sy + cr * cy
    r12 = sr * cp
    r20 = cr * sp * cy + sr * sy
    r21 = cr * sp * sy - sr * cy
    r22 = cr * cp
    return (r00, r01, r02), (r10, r11, r12), (r20, r21, r22)


def dcm_mul_t(dcm, v):
    """rotmat.transpose() * v  (relVelSensor = rotmat.mul_transpose(velocity))"""
    (a, b, c), (d, e, f), (g, h, i) = dcm
    return (a * v[0] + b * v[1] + c * v[2],
            d * v[0] + e * v[1] + f * v[2],
            g * v[0] + h * v[1] + i * v[2])


def dcm_mul(dcm, v):
    (a, b, c), (d, e, f), (g, h, i) = dcm
    return (a * v[0] + b * v[1] + c * v[2],
            d * v[0] + e * v[1] + f * v[2],
            g * v[0] + h * v[1] + i * v[2])


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def sitl_flow(rate_ned, roll, pitch, yaw, gyro_rad, height_agl, pos_offset_bf=(0.0, 0.0, 0.0),
              flow_scaler=(0.0, 0.0)):
    """Same maths as AP_OpticalFlow_SITL::update().  Returns (flow_x, flow_y) rad/s
    (already multiplied by the flow scaler, exactly like the SITL backend does)."""
    dcm = euler_to_dcm(roll, pitch, yaw)
    pos_rel_ef = dcm_mul(dcm, pos_offset_bf)
    if dcm[2][2] > 0.05 and height_agl > 0:
        rng = (height_agl - pos_rel_ef[2]) / dcm[2][2]
    else:
        rng = 1e38
    rel_vel = dcm_mul_t(dcm, rate_ned)
    rel_vel = tuple(rel_vel[k] + cross(gyro_rad, pos_offset_bf)[k] for k in range(3))
    fsx = 1.0 + 0.001 * flow_scaler[0]
    fsy = 1.0 + 0.001 * flow_scaler[1]
    fx = (-rel_vel[1] / rng + gyro_rad[0]) * fsx
    fy = (rel_vel[0] / rng + gyro_rad[1]) * fsy
    return fx, fy


def sched_lookup(sched, el):
    """Value tuple of the schedule entry active at time el."""
    t = 0.0
    for item in sched:
        dur = item[-1]
        if el < t + dur:
            return item[:-1]
        t += dur
    return sched[-1][:-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", required=True)
    ap.add_argument("--sysid", type=int, default=200)
    ap.add_argument("--compid", type=int, default=191)
    ap.add_argument("--mode", choices=["fixed", "truth"], default="fixed")
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--dist", type=float, default=-1.0, help="m; <0 = do not send DISTANCE_SENSOR")
    ap.add_argument("--dist-max", type=float, default=8.0, help="m; max_distance field of the message")
    ap.add_argument("--dist-min", type=float, default=0.01, help="m; min_distance field of the message")
    ap.add_argument("--orient", type=int, default=25, help="MAV_SENSOR_ROTATION of the message")
    ap.add_argument("--flow", action="store_true", help="send OPTICAL_FLOW")
    ap.add_argument("--flow-x", type=float, default=0.0, help="rad/s")
    ap.add_argument("--flow-y", type=float, default=0.0, help="rad/s")
    ap.add_argument("--flow-legacy", action="store_true",
                    help="use the int16 flow_x/flow_y fields instead of flow_rate_x/flow_rate_y")
    ap.add_argument("--quality", type=int, default=100)
    ap.add_argument("--sensor-id", type=int, default=1)
    ap.add_argument("--stop-dist-at", type=float, default=-1.0, help="s; stop DISTANCE_SENSOR after t")
    ap.add_argument("--stop-flow-at", type=float, default=-1.0, help="s; stop OPTICAL_FLOW after t")
    ap.add_argument("--home-alt", type=float, default=20.0, help="m; ground altitude for truth mode")
    ap.add_argument("--rng-noise", type=float, default=0.0, help="m; uniform +- noise on the distance")
    ap.add_argument("--flow-noise", type=float, default=0.0, help="rad/s; uniform +- noise on the flow")
    ap.add_argument("--out", default="feed.json")
    ap.add_argument("--arm-ground", action="store_true",
                    help="arm the FC on the ground (STABILIZE, zero throttle) so that the dataflash log starts")
    ap.add_argument("--dist-schedule", default="", help="val:dur,val:dur  (m)")
    ap.add_argument("--flow-schedule", default="", help="x:y:dur,x:y:dur  (rad/s)")
    ap.add_argument("--quality-schedule", default="", help="q:dur,q:dur")
    ap.add_argument("--hz-schedule", default="", help="hz:dur,hz:dur")
    args = ap.parse_args()

    def parse_sched(s):
        out = []
        for part in s.split(","):
            if not part:
                continue
            f = [float(v) for v in part.split(":")]
            out.append(tuple(f))
        return out

    args.dist_schedule = parse_sched(args.dist_schedule)
    args.flow_schedule = parse_sched(args.flow_schedule)
    args.quality_schedule = parse_sched(args.quality_schedule)
    args.hz_schedule = parse_sched(args.hz_schedule)

    m = mavutil.mavlink_connection(args.url, source_system=args.sysid, source_component=args.compid)
    if m.wait_heartbeat(timeout=30) is None:
        sys.exit("no heartbeat on " + args.url)
    print("connected, sysid=%d compid=%d" % (args.sysid, args.compid), flush=True)
    # ask the FC to stream the messages we want to observe
    for mid, hz in ((132, 5), (100, 5), (153, 20), (1, 2), (193, 2), (30, 10)):
        m.mav.command_long_send(1, 1, 511, 0, mid, int(1e6 / hz), 0, 0, 0, 0, 0)
        time.sleep(0.02)
    for name in ("FLOW_TYPE", "RNGFND1_TYPE", "RNGFND1_MAX", "RNGFND1_MIN", "RNGFND1_ORIENT"):
        m.mav.param_request_read_send(1, 1, name.encode(), -1)

    rec = {"args": vars(args), "events": [], "fc_distance_sensor": [], "fc_optical_flow": [],
           "fc_rangefinder": [], "fc_sys_status": [], "statustext": [], "sent": {"dist": 0, "flow": 0}, "sent_ts": {"dist": [], "flow": []}}

    t0 = time.time()
    period = 1.0 / args.hz
    next_t = t0
    qual = args.quality
    hz = args.hz
    last_hb = 0.0
    last_rc = 0.0
    last_arm = 0.0
    truth = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "vn": 0.0, "ve": 0.0, "vd": 0.0,
             "gr": 0.0, "gp": 0.0, "gy": 0.0, "alt": args.home_alt, "have": False}
    last_flow = None

    def note(kind, **kw):
        rec["events"].append([round(time.time() - t0, 3), kind] + [kw])

    while True:
        now = time.time()
        el = now - t0
        if el > args.duration:
            break
        if args.hz_schedule:
            hz_new = sched_lookup(args.hz_schedule, el)[0]
            if hz_new != hz:
                hz = hz_new
                period = 1.0 / hz
                next_t = now + period
        if now < next_t:
            m.recv_match(blocking=True, timeout=0.002)
            time.sleep(min(0.001, max(0.0, next_t - now)))
            continue
        next_t += period
        if next_t < now:
            next_t = now + period
        if now - last_hb > 1.0:
            m.mav.heartbeat_send(18, 8, 0, 0, 0)
            last_hb = now
        if args.arm_ground and el > 1.0:
            if el < 2.0:
                m.mav.command_long_send(1, 1, 176, 0, 1, 0, 0, 0, 0, 0, 0)  # STABILIZE
            if now - last_rc > 0.2:
                m.mav.rc_channels_override_send(1, 1, 1500, 1500, 1000, 1500, 0, 0, 0, 0)
                last_rc = now
            if el < 20.0 and now - last_arm > 3.0:
                m.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
                last_arm = now

        # ---- receive whatever the FC sends -------------------------------
        while True:
            x = m.recv_match(blocking=True, timeout=0.002)
            if x is None:
                break
            k = x.get_type()
            t = round(time.time() - t0, 3)
            rec.setdefault("rx_counts", {})[k] = rec.get("rx_counts", {}).get(k, 0) + 1
            if k == "DISTANCE_SENSOR":
                rec["fc_distance_sensor"].append(
                    [t, x.current_distance, x.min_distance, x.max_distance, x.orientation,
                     getattr(x, "signal_quality", -1), x.get_srcSystem()])
            elif k == "OPTICAL_FLOW":
                rec["fc_optical_flow"].append(
                    [t, x.flow_x, x.flow_y, getattr(x, "flow_rate_x", None),
                     getattr(x, "flow_rate_y", None), x.quality, x.ground_distance,
                     x.get_srcSystem()])
            elif k == "RANGEFINDER":
                rec["fc_rangefinder"].append([t, x.distance, x.get_srcSystem()])
            elif k == "SYS_STATUS":
                rec["fc_sys_status"].append(
                    [t, x.onboard_control_sensors_present, x.onboard_control_sensors_enabled,
                     x.onboard_control_sensors_health])
            elif k == "STATUSTEXT":
                rec["statustext"].append([t, x.text, x.get_srcSystem()])
            elif k == "PARAM_VALUE":
                rec.setdefault("params", {})[x.param_id] = x.param_value
            elif k == "COMMAND_ACK":
                rec.setdefault("acks", []).append([t, x.command, x.result])
            elif k == "SIM_STATE":
                truth.update(roll=x.roll, pitch=x.pitch, yaw=x.yaw,
                             vn=x.vx, ve=x.vy, vd=x.vz,
                             gr=x.xacc, gp=x.yacc, gy=x.zacc, alt=x.z,
                             have=True)

        # ---- truth based values -----------------------------------------
        if args.mode == "truth":
            height_agl = truth["alt"] - args.home_alt
            dcm = euler_to_dcm(truth["roll"], truth["pitch"], truth["yaw"])
            # range = height / cos(tilt) : sensor looks straight down, tilted by pitch/roll
            tilt_cos = max(0.05, dcm[2][2])
            dist_m = height_agl / tilt_cos
            if height_agl <= 0.05:
                dist_m = 0.02
            if args.rng_noise > 0:
                import random as _r
                dist_m += args.rng_noise * _r.uniform(-1, 1)
            fx, fy = sitl_flow((truth["vn"], truth["ve"], truth["vd"]),
                               truth["roll"], truth["pitch"], truth["yaw"],
                               (truth["gr"], truth["gp"], truth["gy"]),
                               height_agl)
            if args.flow_noise > 0:
                import random as _r
                fx += args.flow_noise * _r.uniform(-1, 1)
                fy += args.flow_noise * _r.uniform(-1, 1)
        else:
            dist_m = args.dist
            fx, fy = args.flow_x, args.flow_y
        if args.dist_schedule:
            dist_m = sched_lookup(args.dist_schedule, el)[0]
        if args.flow_schedule:
            fx, fy = sched_lookup(args.flow_schedule, el)
        if args.quality_schedule:
            qual = int(sched_lookup(args.quality_schedule, el)[0])

        # ---- send -------------------------------------------------------
        send_dist = args.mode == "truth" or args.dist >= 0 or bool(args.dist_schedule)
        if send_dist and (args.stop_dist_at < 0 or el < args.stop_dist_at):
            cm = int(round(dist_m * 100))
            m.mav.distance_sensor_send(
                int((time.time() - t0) * 1000) & 0xFFFFFFFF,
                int(round(args.dist_min * 100)), int(round(args.dist_max * 100)), cm,
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                args.sensor_id, args.orient, 0, 0.0, 0.0, [1.0, 0.0, 0.0, 0.0], 100)
            rec["sent"]["dist"] += 1
            rec["sent_ts"]["dist"].append(round(el, 4))
            last_flow = None
        if args.flow and (args.stop_flow_at < 0 or el < args.stop_flow_at):
            if args.flow_legacy:
                m.mav.optical_flow_send(
                    int((time.time() - t0) * 1000) & 0xFFFFFFFF, args.sensor_id,
                    int(round(fx)), int(round(fy)), 0.0, 0.0, qual, -1.0, 0.0, 0.0)
            else:
                m.mav.optical_flow_send(
                    int((time.time() - t0) * 1000) & 0xFFFFFFFF, args.sensor_id,
                    int(round(fx)), int(round(fy)), 0.0, 0.0, qual, -1.0,
                    float(fx), float(fy))
            rec["sent"]["flow"] += 1
            rec["sent_ts"]["flow"].append(round(el, 4))

    with open(args.out, "w") as f:
        json.dump(rec, f, indent=1)
    print("done: sent dist=%d flow=%d, fc DISTANCE_SENSOR=%d OPTICAL_FLOW=%d RANGEFINDER=%d" % (
        rec["sent"]["dist"], rec["sent"]["flow"], len(rec["fc_distance_sensor"]),
        len(rec["fc_optical_flow"]), len(rec["fc_rangefinder"])), flush=True)


if __name__ == "__main__":
    main()
