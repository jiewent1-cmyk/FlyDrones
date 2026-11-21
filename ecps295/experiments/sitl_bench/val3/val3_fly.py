#!/usr/bin/env python3
"""Task V3 part 2: closed-loop flight with MAVLink-fed optical flow + rangefinder.

One MAVLink TCP connection to the SITL instance carries both the GCS/mission traffic
and the simulated MTF-01 sensor traffic (source_system = --sysid, default 200).

Truth source: SIM_STATE (MAVLink id 108) that SITL serves on request
(libraries/GCS_MAVLink/GCS_Common.cpp:6711 -> SITL::sim_state_send, libraries/SITL/SITL.cpp:1606).

Optical-flow math: Python port of libraries/AP_OpticalFlow/AP_OpticalFlow_SITL.cpp
(see mtf_feed.py for the line-by-line mapping).
Range: height_agl / rotmat.c.z  (same as AP_OpticalFlow_SITL.cpp:66-72).

Mission harness (upload / AUTO / fly) follows /home/jacky/sim/opt_flow2/opt2_sim.py
(read-only reference) so that both sensor paths fly the same mission.
"""
import argparse
import json
import math
import os
import sys
import time

from pymavlink import mavutil

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mtf_feed import euler_to_dcm, sitl_flow  # noqa: E402

FIELD_PARAMS = {"RTL_ALT_M": 0.0, "WP_SPD": 0.4, "WP_SPD_UP": 0.5, "WP_SPD_DN": 0.4,
                "WP_RADIUS_M": 0.2, "WP_ACC": 1.0, "LOIT_SPEED_MS": 1.0, "FENCE_MARGIN": 0.3}

ap = argparse.ArgumentParser()
ap.add_argument("mission", nargs="?", default="HOV", choices=["HOV", "SQ"])
ap.add_argument("--url", required=True)
ap.add_argument("--alt", type=float, default=2.0)
ap.add_argument("--side", type=float, default=2.0)
ap.add_argument("--laps", type=int, default=1)
ap.add_argument("--hold", type=float, default=60.0)
ap.add_argument("--seconds", type=float, default=180.0)
ap.add_argument("--sysid", type=int, default=200)
ap.add_argument("--compid", type=int, default=191)
ap.add_argument("--send", type=int, default=1, help="1 = feed OPTICAL_FLOW/DISTANCE_SENSOR, 0 = native SITL drivers only")
ap.add_argument("--hz", type=float, default=50.0)
ap.add_argument("--quality", type=int, default=100)
ap.add_argument("--home-alt", type=float, default=-1.0, help="ground altitude in SIM_STATE; <0 = auto-calibrate on the ground")
ap.add_argument("--calib", type=float, default=6.0, help="s of ground calibration")
ap.add_argument("--settle", type=float, default=25.0)
ap.add_argument("--out", default="run.json")
ap.add_argument("--truth-log", default="truth.csv")
args = ap.parse_args()

T0 = time.time()
clock = lambda: time.time() - T0  # noqa: E731


def say(msg):
    print("[%6.1f] %s" % (clock(), msg), flush=True)


m = mavutil.mavlink_connection(args.url, source_system=args.sysid, source_component=args.compid)
st = {"mode": None, "armed": False, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "lat": 0, "lon": 0,
      "rng": float("nan"), "seq": -1, "ekf": 0, "origin": False, "texts": [], "reached": [], "tboot": 0}
truth = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "vn": 0.0, "ve": 0.0, "vd": 0.0,
         "gx": 0.0, "gy": 0.0, "gz": 0.0, "alt": 0.0, "have": False}
ground_alt = args.home_alt if args.home_alt >= 0 else None
alt_samples = []
rec = {"fc_optical_flow": [], "fc_distance_sensor": [], "sent": {"dist": 0, "flow": 0}, "rx_counts": {}}
tf = open(args.truth_log, "w")
tf.write("t,alt,height_agl,roll,pitch,yaw,vn,ve,vd,gx,gy,gz,dist_m,flow_x,flow_y,fc_of_x,fc_of_y\n")
tf.flush()
truth_rows = []
rc = {"thr": None}
t_hb = t_rc = t_org = 0.0
next_send = 0.0
last_fc_of = (None, None)


def pump(sec=0.05):
    global t_hb, t_rc, t_org, next_send, ground_alt, last_fc_of
    t_end = clock() + sec
    while True:
        now_t = time.time()
        if now_t >= next_send:
            next_send = now_t + 1.0 / args.hz
            send_sensors(now_t)
        if now_t - t_hb >= 1.0:
            m.mav.heartbeat_send(18, 8, 0, 0, 0)
            t_hb = now_t
        if not st["origin"] and now_t - t_org >= 2.0:
            m.mav.set_gps_global_origin_send(1, int(33.6430e7), int(-117.8420e7), 20000)
            t_org = now_t
        if rc["thr"] is not None and now_t - t_rc >= 0.1:
            m.mav.rc_channels_override_send(1, 1, 1500, 1500, rc["thr"], 1500, 0, 0, 0, 0)
            t_rc = now_t
        x = m.recv_match(blocking=True, timeout=0.01)
        if x is not None:
            k = x.get_type()
            rec["rx_counts"][k] = rec.get("rx_counts", {}).get(k, 0) + 1
            if k == "SIM_STATE":
                truth.update(roll=x.roll, pitch=x.pitch, yaw=x.yaw, vn=x.vn, ve=x.ve, vd=x.vd,
                             gx=x.xgyro, gy=x.ygyro, gz=x.zgyro, alt=x.alt, have=True)
                if ground_alt is None and clock() < args.calib:
                    alt_samples.append(x.alt)
                elif ground_alt is None:
                    ground_alt = sorted(alt_samples)[len(alt_samples) // 2] if alt_samples else x.alt
                    say("ground altitude (SIM_STATE) = %.3f m (%d samples)" % (ground_alt, len(alt_samples)))
            elif k == "HEARTBEAT" and x.get_srcSystem() == 1:
                mode = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
                        6: "RTL", 9: "LAND", 21: "SMART_RTL"}.get(x.custom_mode, str(x.custom_mode))
                armed = bool(x.base_mode & 128)
                if (mode, armed) != (st["mode"], st["armed"]):
                    st["mode"], st["armed"] = mode, armed
                    say("FC mode %s %s (alt %.2f m)" % (mode, "ARMED" if armed else "disarmed", -st["z"]))
            elif k == "STATUSTEXT":
                st["texts"].append(x.text)
            elif k == "LOCAL_POSITION_NED":
                st["x"], st["y"], st["z"] = x.x, x.y, x.z
            elif k == "ATTITUDE":
                st["yaw"], st["tboot"] = x.yaw, x.time_boot_ms
            elif k == "GLOBAL_POSITION_INT":
                st["lat"], st["lon"] = x.lat, x.lon
            elif k == "RANGEFINDER":
                st["rng"] = x.distance
            elif k == "EKF_STATUS_REPORT":
                st["ekf"] = x.flags
            elif k == "GPS_GLOBAL_ORIGIN":
                st["origin"] = True
            elif k == "MISSION_CURRENT":
                st["seq"] = x.seq
            elif k == "MISSION_ITEM_REACHED":
                if x.seq not in st["reached"]:
                    st["reached"].append(x.seq)
                    say("mission item %d REACHED (x %.2f y %.2f alt %.2f)" % (x.seq, st["x"], st["y"], -st["z"]))
            elif k == "OPTICAL_FLOW":
                rec["fc_optical_flow"].append([round(clock(), 3), x.flow_rate_x, x.flow_rate_y, x.quality])
                last_fc_of = (x.flow_rate_x, x.flow_rate_y)
            elif k == "DISTANCE_SENSOR":
                rec["fc_distance_sensor"].append([round(clock(), 3), x.current_distance, x.orientation])
        if clock() >= t_end:
            return


def send_sensors(now_t):
    """Compute the truth based sensor values, log them, and (MAVLink path) send them."""
    if not truth["have"]:
        return
    ga = ground_alt if ground_alt is not None else truth["alt"]
    height_agl = truth["alt"] - ga
    dcm = euler_to_dcm(truth["roll"], truth["pitch"], truth["yaw"])
    tilt_cos = dcm[2][2]
    if height_agl > 0.05 and tilt_cos > 0.05:
        dist_m = height_agl / tilt_cos
    else:
        dist_m = 0.02
    fx, fy = sitl_flow((truth["vn"], truth["ve"], truth["vd"]), truth["roll"], truth["pitch"], truth["yaw"],
                       (truth["gx"], truth["gy"], truth["gz"]), height_agl)
    cm = int(round(dist_m * 100))
    if cm > 65535 or cm < 0:
        rec["clamped"] = rec.get("clamped", 0) + 1
        say("DISTANCE_SENSOR clamped: %.1f m -> %d cm" % (dist_m, max(0, min(65535, cm))))
        cm = max(0, min(65535, cm))
    t_ms = int((now_t - T0) * 1000) & 0xFFFFFFFF
    if args.send:
        m.mav.optical_flow_send(t_ms, 1, int(round(fx)), int(round(fy)), 0.0, 0.0, args.quality, -1.0,
                                float(fx), float(fy))
        m.mav.distance_sensor_send(t_ms, 1, 800, cm,
                                    mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, 1, 25, 0, 0.0, 0.0,
                                    [1.0, 0.0, 0.0, 0.0], 100)
        rec["sent"]["flow"] += 1
        rec["sent"]["dist"] += 1
    truth_rows.append([round(clock(), 3), round(truth["alt"], 4), round(height_agl, 4),
                       round(truth["roll"], 5), round(truth["pitch"], 5), round(truth["yaw"], 5),
                       round(truth["vn"], 4), round(truth["ve"], 4), round(truth["vd"], 4),
                       round(truth["gx"], 5), round(truth["gy"], 5), round(truth["gz"], 5),
                       round(dist_m, 4), round(fx, 6), round(fy, 6),
                       last_fc_of[0], last_fc_of[1]])
    tf.write(",".join(str(v) for v in truth_rows[-1]) + "\n")
    tf.flush()


def streams():
    for mid, hz in ((108, 20), (132, 5), (100, 5), (30, 10), (32, 10), (33, 5), (42, 2), (74, 5), (193, 2), (0, 2)):
        m.mav.command_long_send(1, 1, 511, 0, mid, int(1e6 / hz), 0, 0, 0, 0, 0)


def pget(name):
    for _ in range(5):
        m.mav.param_request_read_send(1, 1, name.encode(), -1)
        t = time.time()
        while time.time() - t < 1.0:
            x = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
            if x and x.param_id == name:
                return x.param_value, x.param_type
    return None, None


def pset(name, value):
    old, typ = pget(name)
    if old is None:
        return None, None
    for _ in range(5):
        m.mav.param_set_send(1, 1, name.encode(), float(value), typ)
        got, _ = pget(name)
        if got is not None and abs(got - value) < 1e-4:
            return old, got
    return old, pget(name)[0]


def set_mode(num):
    m.mav.command_long_send(1, 1, 176, 0, 1, num, 0, 0, 0, 0, 0)


def mission_items(name):
    s, h, al = args.side, args.hold, args.alt
    if name == "HOV":
        return [(16, 0, 0, 0, 0), (22, 0, 0, al, 0), (19, 0, 0, al, h), (21, 0, 0, 0, 0)]
    pts = [(s, 0, h), (s, s, h), (0, s, h), (0, 0, h)] * args.laps
    items = [(16, 0, 0, 0, 0), (22, 0, 0, al, 0)]
    items += [(16, x, y, al, hold) for x, y, hold in pts]
    items.append((21, 0, 0, 0, 0))
    return items


def upload(name):
    for _ in range(60):
        pump(0.1)
        if st["lat"] and st["origin"]:
            break
    lat0, lon0, yaw = st["lat"] / 1e7, st["lon"] / 1e7, st["yaw"]
    items = mission_items(name)
    wire = []
    for seq, (cmd, x, y, alt, hold) in enumerate(items):
        north, east = x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw)
        lat = lat0 + north / 111319.5
        lon = lon0 + east / (111319.5 * math.cos(math.radians(lat0)))
        if cmd == 21:
            lat = lon = 0.0
        wire.append((seq, cmd, hold if cmd in (16, 19) else 0.0, int(round(lat * 1e7)), int(round(lon * 1e7)), alt))
    say("uploading %s: alt %.2f side %.2f yaw %.1f deg" % (name, args.alt, args.side, math.degrees(yaw)))
    m.mav.mission_clear_all_send(1, 1, 0)
    pump(0.5)
    m.mav.mission_count_send(1, 1, len(wire), 0)
    t = time.time()
    while time.time() - t < 15:
        x = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True, timeout=1)
        if x is None:
            continue
        if x.get_type() == "MISSION_ACK":
            say("mission ACK type %d" % x.type)
            return x.type == 0
        seq, cmd, p1, lat, lon, alt = wire[x.seq]
        m.mav.mission_item_int_send(1, 1, seq, 6, cmd, 0, 1, p1, 0, 0, 0, lat, lon, alt, 0)
    return False


def fly_report(seconds):
    track = []
    tb0 = st["tboot"]
    t_wall = clock()
    while (st["tboot"] - tb0) / 1000.0 < seconds:
        pump(0.1)
        if st["armed"]:
            track.append((round((st["tboot"] - tb0) / 1000.0, 2), round(-st["z"], 3), st["mode"]))
        elif track:
            break
        elif clock() - t_wall > 25.0:
            say("not armed after %.0f s, giving up" % (clock() - t_wall))
            break
    if not track:
        return {"flew": False}
    return {"flew": True, "seconds": round(track[-1][0] - track[0][0], 1),
            "est_alt_max_m": round(max(p[1] for p in track), 2),
            "items_reached": list(st["reached"]), "modes": sorted({p[2] for p in track}),
            "disarmed": not st["armed"], "alt_track": track[::10]}


if m.wait_heartbeat(timeout=60) is None:
    sys.exit("no heartbeat on " + args.url)
say("connected, sysid=%d send=%d" % (args.sysid, args.send))
streams()
pump(1.0)
say("waiting for the EKF and the origin")
t = clock()
while clock() - t < 60 and not (st["origin"] and st["lat"] and st["ekf"] & 0x08):
    pump(0.5)
say("origin %s, EKF flags 0x%03x, rangefinder %.2f m" % (st["origin"], st["ekf"], st["rng"]))
for k, v in FIELD_PARAMS.items():
    say("param %-16s %s -> %s" % (k, *pset(k, v)))
pump(args.settle)
if ground_alt is None:
    ground_alt = sorted(alt_samples)[len(alt_samples) // 2] if alt_samples else 0.0
    say("ground altitude (SIM_STATE) = %.3f m" % ground_alt)

out = {}
say("=== AUTO mission %s" % args.mission)
st["reached"].clear()
if not upload(args.mission):
    print("RESULT_JSON " + json.dumps({"error": "upload failed"}))
    sys.exit(1)
set_mode(5)
rc["thr"] = 1000
pump(2.0)
for _ in range(15):
    m.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
    pump(2.0)
    if st["armed"]:
        break
say("armed %s in %s; switching to AUTO" % (st["armed"], st["mode"]))
out["auto_armed"] = st["armed"]
set_mode(3)
pump(1.0)
rc["thr"] = 1550
out["auto"] = fly_report(args.seconds)
out["ground_alt"] = ground_alt
out["sent"] = rec["sent"]
out["rx_counts"] = rec["rx_counts"]
out["texts"] = [x for x in st["texts"] if not x.startswith(("ArduCopter", "ChibiOS", "Frame", "RCOut", "IMU"))][-40:]
with open(args.out, "w") as f:
    json.dump({"result": out, "fc_optical_flow": rec["fc_optical_flow"],
               "fc_distance_sensor": rec["fc_distance_sensor"]}, f, indent=1)
with open(args.truth_log, "a") as f:
    pass
tf.close()
print("RESULT_JSON " + json.dumps(out))
