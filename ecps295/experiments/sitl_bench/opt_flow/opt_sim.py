#!/usr/bin/env python3
"""opt_sim.py - harsh-scenario driver for MTF-01 flow + rangefinder EKF tuning (2026-11-19).

Derived from /home/jacky/sim/mtf01/mtf_sim.py (same companion / pilot logic).

Differences vs mtf_sim.py:
  * NO VISION_POSITION_ESTIMATE is sent.  After the MTF-01 change the height source is the
    rangefinder itself (EK3_SRC1_POSZ 2), the Orange Pi is out of the height path.
    The companion only sets the EKF origin and keeps a heartbeat.
  * scenario "hover"  : LOITER position hold at --alt for --hold sim-seconds (default 120),
                        then descend and disarm.
  * scenario "mission": AUTO square (--side) flown --laps times at --alt, then land.
  * fault injection --fault T:NAME=VAL[:DUR]  (T = sim seconds after the scenario start
    marker, DUR>0 restores the original value).
Prints RESULT_JSON; the truth-based metrics come from analyze_opt.py (dataflash).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

from pymavlink import mavutil

FIELD_PARAMS = {"RTL_ALT_M": 0.0, "WP_SPD": 0.4, "WP_SPD_UP": 0.5, "WP_SPD_DN": 0.4, "WP_RADIUS_M": 0.2,
                "WP_ACC": 1.0, "LOIT_SPEED_MS": 1.0, "FENCE_MARGIN": 0.3}
MODES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND", 21: "SMART_RTL"}

ap = argparse.ArgumentParser()
ap.add_argument("--url", required=True)
ap.add_argument("--scenario", default="hover", choices=["hover", "mission"])
ap.add_argument("--alt", type=float, default=2.0)
ap.add_argument("--side", type=float, default=2.0)
ap.add_argument("--laps", type=int, default=1)
ap.add_argument("--hold", type=float, default=120.0)
ap.add_argument("--seconds", type=float, default=400.0)
ap.add_argument("--speed", type=float, default=1.0)
ap.add_argument("--fault", action="append", default=[])
ap.add_argument("--dump", action="store_true")
a = ap.parse_args()

S = a.speed
m = mavutil.mavlink_connection(a.url, source_system=254, source_component=191)
st = {"mode": None, "armed": False, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "lat": 0, "lon": 0, "rng": float("nan"),
      "seq": -1, "ekf": 0, "origin": False, "texts": [], "reached": [], "tboot": 0.0}
rc = {"thr": None}
t_hb = t_rc = t_org = 0.0
T0 = time.time()
out = {}


def clock():
    return (time.time() - T0) * S


def say(msg):
    print("[%6.1f] %s" % (clock(), msg), flush=True)


def pump(sec=0.05):
    global t_hb, t_rc, t_org
    t_end = clock() + sec
    while True:
        now = clock()
        if now - t_hb >= 1.0:
            m.mav.heartbeat_send(18, 8, 0, 0, 0)
            t_hb = now
        if not st["origin"] and now - t_org >= 2.0:
            m.mav.set_gps_global_origin_send(1, int(33.6430e7), int(-117.8420e7), 20000)
            m.mav.command_long_send(1, 1, 512, 0, 49, 0, 0, 0, 0, 0, 0)
            t_org = now
        if rc["thr"] is not None and now - t_rc >= 0.1:
            m.mav.rc_channels_override_send(1, 1, 1500, 1500, rc["thr"], 1500, 0, 0, 0, 0)
            t_rc = now
        x = m.recv_match(blocking=True, timeout=0.01)
        if x is not None:
            k = x.get_type()
            if k == "HEARTBEAT" and x.get_srcSystem() == 1 and x.get_srcComponent() == 1:
                mode, armed = MODES.get(x.custom_mode, str(x.custom_mode)), bool(x.base_mode & 128)
                if (mode, armed) != (st["mode"], st["armed"]):
                    st["mode"], st["armed"] = mode, armed
                    say("FC mode %s %s  (alt %.2f m, x %.2f y %.2f)" % (mode, "ARMED" if armed else "disarmed",
                                                                      -st["z"], st["x"], st["y"]))
            elif k == "STATUSTEXT":
                st["texts"].append(x.text)
                say("FC text: " + x.text)
            elif k == "LOCAL_POSITION_NED":
                st["x"], st["y"], st["z"] = x.x, x.y, x.z
            elif k == "ATTITUDE":
                st["yaw"], st["tboot"] = x.yaw, x.time_boot_ms
            elif k == "GLOBAL_POSITION_INT":
                st["lat"], st["lon"] = x.lat, x.lon
            elif k == "RANGEFINDER":
                st["rng"] = x.distance
                # the Orange Pi is still connected and still sends the vision stream: the FC needs a
                # healthy vision source for the arming check (VISO_TYPE 1).  The EKF does NOT use it
                # for height (EK3_SRC1_POSZ 2 = rangefinder) nor for position (EK3_SRC1_POSXY 0).
                m.mav.vision_position_estimate_send(int(st["tboot"] * 1000), st["x"], st["y"], -x.distance, 0.0, 0.0, st["yaw"])
            elif k == "EKF_STATUS_REPORT":
                st["ekf"] = x.flags
            elif k == "GPS_GLOBAL_ORIGIN":
                st["origin"] = True
            elif k == "MISSION_CURRENT":
                st["seq"] = x.seq
            elif k == "MISSION_ITEM_REACHED":
                if x.seq not in st["reached"]:
                    st["reached"].append(x.seq)
                    say("mission item %d REACHED  (x %.2f y %.2f alt %.2f)" % (x.seq, st["x"], st["y"], -st["z"]))
        if clock() >= t_end:
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


def pset_fast(name, value):
    m.mav.param_set_send(1, 1, name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


def set_mode(num):
    m.mav.command_long_send(1, 1, 176, 0, 1, num, 0, 0, 0, 0, 0)


def mission_items():
    s, al = a.side, a.alt
    items = [(16, 0, 0, 0, 0), (22, 0, 0, al, 0)]
    for _ in range(a.laps):
        for (x, y) in ((s, 0), (s, s), (0, s), (0, 0)):
            items.append((16, x, y, al, 2.0))
    items.append((21, 0, 0, 0, 0))
    return items


def upload():
    for _ in range(60):
        pump(0.1)
        if st["lat"] and st["origin"]:
            break
    lat0, lon0, yaw = st["lat"] / 1e7, st["lon"] / 1e7, st["yaw"]
    items = mission_items()
    wire = []
    for seq, (cmd, x, y, alt, hold) in enumerate(items):
        north, east = x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw)
        lat = lat0 + north / 111319.5
        lon = lon0 + east / (111319.5 * math.cos(math.radians(lat0)))
        if cmd == 21:
            lat = lon = 0.0
        wire.append((seq, cmd, hold if cmd == 16 else 0.0, int(round(lat * 1e7)), int(round(lon * 1e7)), alt))
    say("uploading mission: alt %.2f side %.2f laps %d yaw %.1f deg" % (a.alt, a.side, a.laps, math.degrees(yaw)))
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


# ---------------------------------------------------------------- faults
FAULTS = []
for f in a.fault:
    parts = f.split(":")
    t = float(parts[0])
    kv = parts[1]
    name, val = kv.split("=")
    dur = float(parts[2]) if len(parts) > 2 else 0.0
    FAULTS.append({"t": t, "name": name, "val": float(val), "dur": dur, "done": False, "restored": False, "orig": None})


def setup_faults():
    for f in FAULTS:
        v, _ = pget(f["name"])
        f["orig"] = v
        say("FAULT orig %s = %s" % (f["name"], v))


def run_faults(dt):
    for f in FAULTS:
        if not f["done"] and dt >= f["t"]:
            pset_fast(f["name"], f["val"])
            f["done"] = True
            say("FAULT: %s = %s" % (f["name"], f["val"]))
        if f["done"] and not f["restored"] and f["dur"] > 0 and dt >= f["t"] + f["dur"]:
            pset_fast(f["name"], f["orig"])
            f["restored"] = True
            say("FAULT restored: %s = %s" % (f["name"], f["orig"]))


# ---------------------------------------------------------------- scenarios
def do_hover():
    set_mode(5)
    pump(1.0)
    rc["thr"] = 1000
    pump(1.0)
    m.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
    pump(2.0)
    out["armed"] = st["armed"]
    # climb with a P controller on the estimated altitude (the throttle is a climb-rate command
    # in LOITER); a constant throttle overshoots by ~0.6 m and can trip the alt fence
    t = clock()
    while clock() - t < 45:
        pump(0.1)
        err = a.alt - (-st["z"])
        rc["thr"] = 1500 + int(max(-100, min(100, 150 * err)))
        if abs(err) < 0.03 and clock() - t > 10:
            break
    say("climb: est alt %.2f m after %.1f s" % (-st["z"], clock() - t))
    pump(4.0)  # settle
    say("HOVER_START alt %.2f" % a.alt)
    t0 = clock()
    while clock() - t0 < a.hold:
        pump(0.2)
        err = a.alt - (-st["z"])
        rc["thr"] = 1500 + int(max(-100, min(100, 150 * err)))
        run_faults(clock() - t0)
    say("HOVER_END")
    rc["thr"] = 1250
    t = clock()
    while clock() - t < 40 and st["armed"]:
        pump(0.2)
    rc["thr"] = 1000
    pump(3.0)
    if st["armed"]:
        m.mav.command_long_send(1, 1, 400, 0, 0, 21196, 0, 0, 0, 0, 0)
        pump(2.0)
    out["disarmed"] = not st["armed"]
    out["est_alt_max_m"] = round(-st["z"], 2)


def do_mission():
    if not upload():
        say("upload failed")
        return
    set_mode(5)
    rc["thr"] = 1000
    pump(2.0)
    m.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
    pump(2.0)
    say("armed %s in %s; switching to AUTO" % (st["armed"], st["mode"]))
    out["armed"] = st["armed"]
    set_mode(3)
    pump(1.0)
    rc["thr"] = 1500   # neutral: in AUTO the RC throttle is an altitude override, 1550 added ~0.8 m
                       # to the takeoff and tripped the alt fence
    t0 = clock()
    track = []
    while clock() - t0 < a.seconds:
        pump(0.1)
        run_faults(clock() - t0)
        if st["armed"]:
            track.append((clock() - t0, -st["z"], st["mode"]))
        else:
            break
    if track:
        out["seconds"] = round(track[-1][0] - track[0][0], 1)
        out["est_alt_max_m"] = round(max(p[1] for p in track), 2)
        out["items_reached"] = list(st["reached"])
        out["modes"] = sorted({p[2] for p in track})
    out["disarmed"] = not st["armed"]


if m.wait_heartbeat(timeout=60) is None:
    sys.exit("no heartbeat on " + a.url)
streams()
pump(1.0)
say("waiting for the EKF and the origin")
t = clock()
while clock() - t < 60 and not (st["origin"] and st["lat"] and st["ekf"] & 0x08):
    pump(0.5)
say("origin %s, EKF flags 0x%03x, rangefinder %.2f m" % (st["origin"], st["ekf"], st["rng"]))
for k, v in FIELD_PARAMS.items():
    say("param %-16s %s -> %s" % (k, *pset(k, v)))
if a.dump:
    for k in ("EK3_SRC1_POSZ", "EK3_SRC2_POSZ", "EK3_RNG_M_NSE", "EK3_RNG_I_GATE", "EK3_FLOW_M_NSE",
              "EK3_FLOW_I_GATE", "EK3_FLOW_DELAY", "EK3_HGT_I_GATE", "EK3_HGT_DELAY", "EK3_ALT_M_NSE",
              "EK3_TERR_GRAD", "EK3_RNG_USE_SPD", "RNGFND1_POS_Z", "RNGFND1_GNDCLR", "RNGFND1_MAX",
              "FLOW_POS_X", "FLOW_POS_Y", "FLOW_POS_Z", "FLOW_FXSCALER", "FLOW_FYSCALER", "FLOW_ORIENT_YAW",
              "SIM_SONAR_POS_Z", "SIM_SONAR_POS_X", "SIM_FLOW_POS_X", "SIM_FLOW_POS_Z", "SIM_FLOW_RND",
              "SIM_SONAR_RND", "SIM_WIND_SPD", "SIM_WIND_DIR", "FENCE_RADIUS", "FENCE_ALT_MAX",
              "FS_EKF_ACTION", "FS_EKF_THRESH", "FS_OPTIONS"):
        v, _ = pget(k)
        say("PARAMDUMP %s = %s" % (k, v))
setup_faults()
pump(25.0)   # EKF tilt alignment / accel consistency check needs time
if a.scenario == "hover":
    do_hover()
else:
    do_mission()
out["texts"] = [t for t in st["texts"] if not t.startswith(("ArduPilot Ready", "ChibiOS", "Frame", "RCOut", "IMU"))][-40:]
print("RESULT_JSON " + json.dumps(out))
