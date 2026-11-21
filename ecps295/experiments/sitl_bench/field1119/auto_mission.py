#!/usr/bin/env python3
"""ECPS295 field tool (2026-11-19): no-GPS AUTO waypoint mission for the course's 20' cage flight.

The FC flies the mission itself (ArduCopter AUTO) on optical flow + the companion's route-B height; MiniFly is not in
control. Missions are metres from the point where the script is run, x = ahead of the nose, y = to the right, converted
to latitude / longitude with the FC's own GLOBAL_POSITION_INT and yaw (there is no GPS and no compass: the EKF origin is
the fixed one the companion sets, and "north" is wherever the EKF's yaw says).

Real drone, on the Pi, companion running in shadow (it sends the heartbeat, the height and the origin):
    auto_mission.py params --apply            # field parameters (backup of the old values is printed and saved)
    auto_mission.py params --restore FILE
    auto_mission.py upload D1|D2|D3 [--alt 0.8] [--side 1.0]
    auto_mission.py auto                      # pilot has armed in LOITER: switch the FC to AUTO and report the flight
    auto_mission.py watch                     # only report

SITL rehearsal (the script also plays the companion and the pilot):
    auto_mission.py rehearse D3 --url tcp:127.0.0.1:5780
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

from pymavlink import mavutil

FIELD_PARAMS = {  # see the field plan, section 3
    "RTL_ALT_M": 0.0, "WP_SPD": 0.4, "WP_SPD_UP": 0.5, "WP_SPD_DN": 0.4, "WP_RADIUS_M": 0.2, "WP_ACC": 1.0,
    "LOIT_SPEED_MS": 1.0,
    # FENCE_MARGIN was 2 m with FENCE_ALT_MAX 1.3 m: in LOITER the fence avoidance then never lets the drone climb and
    # backs it down at AVOID_BACKZ_SPD (SITL 2026-11-18: no climb at full stick, "Hit ground at 0.77 m/s"). 0.3 m is
    # the value all the Gazebo / SITL work used (sitl/fhb_delta.parm).
    "FENCE_MARGIN": 0.3,
}
MODES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND"}
MAV_CMD = {16: "WAYPOINT", 21: "LAND", 22: "TAKEOFF"}

ap = argparse.ArgumentParser()
ap.add_argument("action", choices=["params", "upload", "auto", "watch", "rehearse"])
ap.add_argument("mission", nargs="?", default="D3", choices=["D1", "D2", "D3"])
ap.add_argument("--url", default="udpin:127.0.0.1:14570", help="Pi: the mavlink-router diagnostics tap")
ap.add_argument("--alt", type=float, default=0.8)
ap.add_argument("--side", type=float, default=1.0)
ap.add_argument("--hold", type=float, default=2.0, help="seconds at each waypoint")
ap.add_argument("--apply", action="store_true")
ap.add_argument("--restore", default="")
ap.add_argument("--seconds", type=float, default=150.0)
ap.add_argument("--no-loiter-test", action="store_true", help="rehearse: skip the manual LOITER take-off")
ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help="extra / overriding field parameter (params --apply, rehearse)")
a = ap.parse_args()

for kv in a.set:
    FIELD_PARAMS[kv.split("=")[0]] = float(kv.split("=")[1])
SIM = a.action == "rehearse"
m = mavutil.mavlink_connection(a.url, source_system=254 if SIM else 250, source_component=191)
st = {"mode": None, "armed": False, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "lat": 0, "lon": 0, "rng": float("nan"),
      "thr": 0, "seq": -1, "volt": 0.0, "ekf": 0, "origin": False, "texts": [], "reached": [], "t_rng": 0.0}
rc = {"thr": None}
t_hb = t_rc = t_org = 0.0
T0 = time.time()


def say(msg):
    print("[%6.1f] %s" % (time.time() - T0, msg), flush=True)


def pump(sec=0.05):
    """Read messages for `sec`; in SITL also act as the companion (heartbeat, height, origin) and the pilot's sticks."""
    global t_hb, t_rc, t_org
    t_end = time.time() + sec
    while True:
        now = time.time()
        if SIM:
            if now - t_hb >= 1.0:
                m.mav.heartbeat_send(18, 8, 0, 0, 0)  # MAV_TYPE_ONBOARD_CONTROLLER
                t_hb = now
            if not st["origin"] and now - t_org >= 2.0:
                m.mav.set_gps_global_origin_send(1, int(33.6430e7), int(-117.8420e7), 20000)
                m.mav.command_long_send(1, 1, 512, 0, 49, 0, 0, 0, 0, 0, 0)
                t_org = now
            if rc["thr"] is not None and now - t_rc >= 0.1:
                m.mav.rc_channels_override_send(1, 1, 1500, 1500, rc["thr"], 1500, 0, 0, 0, 0)
                t_rc = now
        x = m.recv_match(blocking=True, timeout=0.02)
        if x is not None:
            k = x.get_type()
            if k == "HEARTBEAT" and x.get_srcSystem() == 1 and x.get_srcComponent() == 1:
                mode, armed = MODES.get(x.custom_mode, str(x.custom_mode)), bool(x.base_mode & 128)
                if (mode, armed) != (st["mode"], st["armed"]):
                    st["mode"], st["armed"] = mode, armed
                    say("FC mode %s %s  (alt %.2f m, x %.2f y %.2f)" % (mode, "ARMED" if armed else "disarmed", -st["z"], st["x"], st["y"]))
            elif k == "STATUSTEXT":
                st["texts"].append(x.text)
                say("FC text: " + x.text)
            elif k == "LOCAL_POSITION_NED":
                st["x"], st["y"], st["z"] = x.x, x.y, x.z
            elif k == "ATTITUDE":
                st["yaw"] = x.yaw
            elif k == "GLOBAL_POSITION_INT":
                st["lat"], st["lon"] = x.lat, x.lon
            elif k == "RANGEFINDER":
                st["rng"], st["t_rng"] = x.distance, now
                if SIM:  # route-B height, as mavlink_twin.send_ext_height
                    m.mav.vision_position_estimate_send(int(now * 1e6), st["x"], st["y"], -x.distance, 0.0, 0.0, st["yaw"])
            elif k == "VFR_HUD":
                st["thr"] = x.throttle
            elif k == "SYS_STATUS":
                st["volt"] = x.voltage_battery / 1000
            elif k == "EKF_STATUS_REPORT":
                st["ekf"] = x.flags
            elif k == "GPS_GLOBAL_ORIGIN":
                st["origin"] = True
            elif k == "MISSION_CURRENT":
                if x.seq != st["seq"]:
                    st["seq"] = x.seq
                    say("mission item %d active" % x.seq)
            elif k == "MISSION_ITEM_REACHED":
                st["reached"].append(x.seq)
                say("mission item %d REACHED  (x %.2f y %.2f alt %.2f)" % (x.seq, st["x"], st["y"], -st["z"]))
        if time.time() >= t_end:
            return


def streams():
    for mid, hz in ((0, 2), (1, 2), (30, 10), (32, 10), (33, 5), (42, 2), (74, 5), (173, 20), (193, 2)):
        m.mav.command_long_send(1, 1, 511, 0, mid, 1e6 / hz, 0, 0, 0, 0, 0)


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
    """[(command, x_ahead_m, y_right_m, alt_m, hold_s)]; item 0 is the home slot ArduPilot keeps for itself."""
    s, h, al = a.side, a.hold, a.alt
    pts = {"D1": [(0, 0, 5.0)], "D2": [(s, 0, h), (0, 0, h)], "D3": [(s, 0, h), (s, s, h), (0, s, h), (0, 0, h)]}[name]
    items = [(16, 0, 0, 0, 0), (22, 0, 0, al, 0)]
    items += [(16, x, y, al, hold) for x, y, hold in pts]
    items.append((21, 0, 0, 0, 0))
    return items


def upload(name):
    for _ in range(60):  # the reference point: where the drone stands now
        pump(0.1)
        if st["lat"] and st["origin"] or (st["lat"] and not SIM):
            break
    if not st["lat"]:
        say("no GLOBAL_POSITION_INT from the FC (EKF origin not set?) - cannot place the mission")
        return False
    lat0, lon0, yaw = st["lat"] / 1e7, st["lon"] / 1e7, st["yaw"]
    items = mission_items(name)
    wire = []
    for seq, (cmd, x, y, alt, hold) in enumerate(items):
        north, east = x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw)
        lat = lat0 + north / 111319.5
        lon = lon0 + east / (111319.5 * math.cos(math.radians(lat0)))
        if cmd == 21:
            lat = lon = 0.0  # LAND where it is
        wire.append((seq, cmd, hold if cmd == 16 else 0.0, int(round(lat * 1e7)), int(round(lon * 1e7)), alt))
    say("uploading %s: %d items, alt %.2f m, side %.2f m, nose yaw %.0f deg, reference %.7f %.7f" % (name, len(wire), a.alt, a.side, math.degrees(yaw), lat0, lon0))
    m.mav.mission_clear_all_send(1, 1, 0)
    pump(0.5)
    m.mav.mission_count_send(1, 1, len(wire), 0)
    t = time.time()
    done = False
    while time.time() - t < 15 and not done:
        x = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True, timeout=1)
        if x is None:
            continue
        if x.get_type() == "MISSION_ACK":
            done = x.type == 0
            if not done:
                say("mission upload refused: MISSION_ACK type %d" % x.type)
                return False
        else:
            seq, cmd, p1, lat, lon, alt = wire[x.seq]
            m.mav.mission_item_int_send(1, 1, seq, 6, cmd, 0, 1, p1, 0, 0, 0, lat, lon, alt, 0)  # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    if not done:
        say("mission upload timed out")
        return False
    m.mav.mission_request_list_send(1, 1, 0)  # read back
    cnt = m.recv_match(type="MISSION_COUNT", blocking=True, timeout=5)
    ok = cnt is not None and cnt.count == len(wire)
    for seq in range(len(wire) if ok else 0):
        m.mav.mission_request_int_send(1, 1, seq, 0)
        it = m.recv_match(type="MISSION_ITEM_INT", blocking=True, timeout=3)
        want = wire[seq]
        if it is None or it.command != want[1] or (seq > 0 and (abs(it.z - want[5]) > 0.01 or abs(it.x - want[3]) > 2 or abs(it.y - want[4]) > 2)):
            ok = False
            say("read-back mismatch at item %d: %s" % (seq, it))
        elif seq > 0:
            say("  item %d %-8s x %+.2f y %+.2f alt %.2f hold %.0f s" % (seq, MAV_CMD.get(it.command, it.command), items[seq][1], items[seq][2], it.z, it.param1))
    m.mav.mission_ack_send(1, 1, 0, 0)
    say("mission %s read back %s" % (name, "OK" if ok else "FAILED"))
    return ok


def fly_report(seconds, need_arm=True):
    """Follow one flight until the FC disarms; returns a summary."""
    track, t_start, was_armed = [], time.time(), False
    x0, y0 = st["x"], st["y"]
    while time.time() - t_start < seconds:
        pump(0.1)
        if st["armed"]:
            was_armed = True
            track.append((time.time() - t_start, st["x"] - x0, st["y"] - y0, -st["z"], st["mode"], st["thr"]))
        elif was_armed:
            break
    if not track:
        return {"flew": False}
    alt = [p[3] for p in track]
    r = [math.hypot(p[1], p[2]) for p in track]
    return {"flew": True, "seconds": round(track[-1][0] - track[0][0], 1), "alt_max_m": round(max(alt), 2),
            "radius_max_m": round(max(r), 2), "landing_offset_m": round(r[-1], 2), "items_reached": list(st["reached"]),
            "modes": sorted({p[4] for p in track}), "disarmed": not st["armed"], "batt_v": round(st["volt"], 2)}


# ---------------------------------------------------------------------------------------------------------------
if m.wait_heartbeat(timeout=30) is None:
    sys.exit("no heartbeat on " + a.url)
streams()
pump(1.0)

if a.action == "params":
    if a.restore:
        old = json.load(open(a.restore))
        for k, v in old.items():
            print("restore %-14s -> %s" % (k, pset(k, v)[1]))
    else:
        cur = {k: pget(k)[0] for k in FIELD_PARAMS}
        for k, v in FIELD_PARAMS.items():
            print("%-14s now %-8s field value %s" % (k, cur[k], v))
        if a.apply:
            fn = "field_params_before_%s.json" % time.strftime("%Y%m%d_%H%M%S")
            json.dump(cur, open(fn, "w"), indent=1)
            for k, v in FIELD_PARAMS.items():
                print("set %-14s %s -> %s" % (k, *pset(k, v)))
            print("old values saved to", fn)
elif a.action == "upload":
    sys.exit(0 if upload(a.mission) else 1)
elif a.action == "watch":
    print(json.dumps(fly_report(a.seconds)))
elif a.action == "auto":
    pump(2.0)
    if not st["armed"] or st["mode"] != "LOITER":
        sys.exit("refusing: the pilot must have armed in LOITER first (now %s, %s)" % (st["mode"], "armed" if st["armed"] else "disarmed"))
    say("switching the FC to AUTO")
    set_mode(3)
    print(json.dumps(fly_report(a.seconds)))
else:  # rehearse (SITL)
    say("waiting for the EKF and the origin")
    t = time.time()
    while time.time() - t < 60 and not (st["origin"] and st["lat"] and st["ekf"] & 0x08):
        pump(0.5)
    say("origin %s, EKF flags 0x%03x, rangefinder %.2f m" % (st["origin"], st["ekf"], st["rng"]))
    for k, v in FIELD_PARAMS.items():
        say("param %-14s %s -> %s" % (k, *pset(k, v)))
    pump(25.0)  # let the simulated IMUs settle ("Arm: Accels inconsistent" right after boot)
    out = {}
    if not a.no_loiter_test:  # plan step B2: manual LOITER take-off, hover, land
        say("=== LOITER: arm, throttle up, hover, land")
        set_mode(5)
        pump(1.0)
        rc["thr"] = 1000
        pump(1.0)
        m.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
        pump(2.0)
        rc["thr"] = 1750
        t = time.time()
        while time.time() - t < 8 and -st["z"] < 0.7:
            pump(0.1)
        say("LOITER climb: alt %.2f m after %.1f s" % (-st["z"], time.time() - t))
        rc["thr"] = 1500
        alt_pk, t = -st["z"], time.time()
        while time.time() - t < 4:
            pump(0.1)
            alt_pk = max(alt_pk, -st["z"])
        say("LOITER stick centred: peak alt %.2f m" % alt_pk)
        rc["thr"] = 1500
        x0, y0, drift = st["x"], st["y"], 0.0
        t = time.time()
        while time.time() - t < 15:
            pump(0.2)
            drift = max(drift, math.hypot(st["x"] - x0, st["y"] - y0))
        out["loiter"] = {"armed": st["armed"], "alt_m": round(-st["z"], 2), "hover_drift_m": round(drift, 2)}
        say("LOITER hover: %s" % out["loiter"])
        rc["thr"] = 1250
        t = time.time()
        while time.time() - t < 25 and st["armed"]:
            pump(0.2)
        rc["thr"] = 1000
        pump(3.0)
        if st["armed"]:
            m.mav.command_long_send(1, 1, 400, 0, 0, 21196, 0, 0, 0, 0, 0)
            pump(2.0)
    say("=== AUTO mission %s" % a.mission)
    st["reached"].clear()
    if not upload(a.mission):
        sys.exit(1)
    set_mode(5)
    rc["thr"] = 1000
    pump(2.0)
    m.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
    pump(2.0)
    say("armed %s in %s; switching to AUTO" % (st["armed"], st["mode"]))
    set_mode(3)
    pump(1.0)
    rc["thr"] = 1550  # the manual's "advance the throttle a tiny bit"
    out["auto"] = fly_report(a.seconds)
    out["texts"] = [t for t in st["texts"] if not t.startswith(("ArduCopter", "ChibiOS", "Frame", "RCOut", "IMU"))][-25:]
    print(json.dumps(out, indent=1))
