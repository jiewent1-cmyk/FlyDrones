#!/usr/bin/env python3
"""GPS+flow SITL rehearsal (2026-11-19), copied from ~/sim/mtf01/mtf_sim.py. Additions (gps_flow):
  --ch8 PWM / --switch T:PWM (T after AUTO) / --hover-switch T:PWM (T after hover start): RC8 = EK3 source set switch (RC8_OPTION 90)
  --gps-walk SIGMA[:TAU]  Gauss-Markov horizontal GPS error via SIM_GPS1_GLTCH_X/Y (deg), 2 Hz, seeded by --seed
  --gps-step T:DN,DE[:DUR] extra GPS offset (m) from T after AUTO (DUR<=0: permanent)
  --no-origin / --origin-ofs M / --no-vis  companion behaviour
  --settle S  idle time after param set (was 25)
Original header follows.
MTF-01 SITL rehearsal (2026-11-19). Derived from field1119/auto_mission.py `rehearse` (same companion/pilot logic).

Changes vs auto_mission.py:
  --speed S       SITL --speedup; all script waits are in *sim* seconds (wall = sim / S), companion heartbeat 1 Hz sim.
  VISION_POSITION_ESTIMATE time_usec = FC time_boot (sim clock) instead of wall clock.
  --hover-alt H   LOITER take-off climbs to H (was fixed 0.7 m) and hovers 15 s.
  --vis-cut T     companion stops sending the route-B height T sim-s after the switch to AUTO (heartbeat continues).
  --hb-cut        with --vis-cut: the companion heartbeat stops too (whole companion dead).
  --fault T:NAME=VAL   set a parameter T sim-s after AUTO (repeatable; used for SIM_SONAR_POS_Z range faults).
  --no-auto       only the LOITER part.
Prints a JSON summary; the metrics against truth come from the dataflash log (analyze.py).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
import random

from pymavlink import mavutil

FIELD_PARAMS = {"RTL_ALT_M": 0.0, "WP_SPD": 0.4, "WP_SPD_UP": 0.5, "WP_SPD_DN": 0.4, "WP_RADIUS_M": 0.2, "WP_ACC": 1.0,
                "LOIT_SPEED_MS": 1.0, "FENCE_MARGIN": 0.3}
MODES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND", 21: "SMART_RTL"}

ap = argparse.ArgumentParser()
ap.add_argument("mission", nargs="?", default="D3", choices=["D1", "D2", "D3"])
ap.add_argument("--url", required=True)
ap.add_argument("--alt", type=float, default=0.8)
ap.add_argument("--side", type=float, default=1.0)
ap.add_argument("--hold", type=float, default=2.0)
ap.add_argument("--seconds", type=float, default=150.0)
ap.add_argument("--speed", type=float, default=1.0)
ap.add_argument("--hover-alt", type=float, default=0.7)
ap.add_argument("--no-loiter-test", action="store_true")
ap.add_argument("--no-auto", action="store_true")
ap.add_argument("--vis-cut", type=float, default=-1)
ap.add_argument("--hb-cut", action="store_true")
ap.add_argument("--fault", action="append", default=[])
ap.add_argument("--set", action="append", default=[])
ap.add_argument("--ch8", type=int, default=1000)
ap.add_argument("--switch", action="append", default=[])
ap.add_argument("--hover-switch", action="append", default=[])
ap.add_argument("--gps-walk", default="")
ap.add_argument("--gps-step", action="append", default=[])
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--no-origin", action="store_true")
ap.add_argument("--origin-ofs", type=float, default=0.0)
ap.add_argument("--no-vis", action="store_true")
ap.add_argument("--settle", type=float, default=25.0)
ap.add_argument("--hover-time", type=float, default=15.0)
a = ap.parse_args()
for kv in a.set:
    FIELD_PARAMS[kv.split("=")[0]] = float(kv.split("=")[1])
S = a.speed
m = mavutil.mavlink_connection(a.url, source_system=254, source_component=191)
st = {"mode": None, "armed": False, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "lat": 0, "lon": 0, "rng": float("nan"),
      "seq": -1, "ekf": 0, "origin": False, "texts": [], "reached": [], "tboot": 0}
rc = {"thr": None, "ch8": a.ch8}
rng = random.Random(a.seed)
gw = {"sig": 0.0, "tau": 20.0, "n": 0.0, "e": 0.0, "dn": 0.0, "de": 0.0, "t": -1.0}
if a.gps_walk:
    _g = [float(v) for v in a.gps_walk.split(":")]
    gw["sig"] = _g[0]
    gw["tau"] = _g[1] if len(_g) > 1 else 20.0
    gw["n"], gw["e"] = rng.gauss(0, gw["sig"]), rng.gauss(0, gw["sig"])
comp = {"vis": True, "hb": True}
t_hb = t_rc = t_org = 0.0
T0 = time.time()


def clock():  # sim seconds (approx.)
    return (time.time() - T0) * S


def say(msg):
    print("[%6.1f] %s" % (clock(), msg), flush=True)


def pump(sec=0.05):
    global t_hb, t_rc, t_org
    t_end = clock() + sec
    while True:
        now = clock()
        if comp["hb"] and now - t_hb >= 1.0:
            m.mav.heartbeat_send(18, 8, 0, 0, 0)
            t_hb = now
        if not st["origin"] and now - t_org >= 2.0:
            if not a.no_origin:
                m.mav.set_gps_global_origin_send(1, int((33.6430 + a.origin_ofs / 111319.5) * 1e7), int(-117.8420e7), 20000)
            m.mav.command_long_send(1, 1, 512, 0, 49, 0, 0, 0, 0, 0, 0)
            t_org = now
        if (gw["sig"] > 0 or gw["dn"] or gw["de"]) and now - gw["t"] >= 0.5:
            if gw["sig"] > 0 and gw["t"] >= 0:
                al = math.exp(-(now - gw["t"]) / gw["tau"])
                q = gw["sig"] * math.sqrt(1 - al * al)
                gw["n"], gw["e"] = al * gw["n"] + q * rng.gauss(0, 1), al * gw["e"] + q * rng.gauss(0, 1)
            gw["t"] = now
            pset_fast("SIM_GPS1_GLTCH_X", (gw["n"] + gw["dn"]) / 111319.5)
            pset_fast("SIM_GPS1_GLTCH_Y", (gw["e"] + gw["de"]) / (111319.5 * math.cos(math.radians(33.643))))
        if rc["thr"] is not None and now - t_rc >= 0.1:
            m.mav.rc_channels_override_send(1, 1, 1500, 1500, rc["thr"], 1500, 0, 0, 0, rc["ch8"])
            t_rc = now
        x = m.recv_match(blocking=True, timeout=0.01)
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
                st["yaw"], st["tboot"] = x.yaw, x.time_boot_ms
            elif k == "GLOBAL_POSITION_INT":
                st["lat"], st["lon"] = x.lat, x.lon
            elif k == "RANGEFINDER":
                st["rng"] = x.distance
                if comp["vis"] and not a.no_vis:  # route-B height, as mavlink_twin.send_ext_height
                    m.mav.vision_position_estimate_send(int(st["tboot"] * 1000), st["x"], st["y"], -x.distance, 0.0, 0.0, st["yaw"])
            elif k == "EKF_STATUS_REPORT":
                st["ekf"] = x.flags
            elif k == "GPS_GLOBAL_ORIGIN":
                st["origin"] = True
            elif k == "MISSION_CURRENT":
                if x.seq != st["seq"]:
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


def pset_fast(name, value):  # in-flight: no read-back loop (keeps the companion pump running)
    m.mav.param_set_send(1, 1, name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


def set_mode(num):
    m.mav.command_long_send(1, 1, 176, 0, 1, num, 0, 0, 0, 0, 0)


def mission_items(name):
    s, h, al = a.side, a.hold, a.alt
    pts = {"D1": [(0, 0, 5.0)], "D2": [(s, 0, h), (0, 0, h)], "D3": [(s, 0, h), (s, s, h), (0, s, h), (0, 0, h)]}[name]
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
        wire.append((seq, cmd, hold if cmd == 16 else 0.0, int(round(lat * 1e7)), int(round(lon * 1e7)), alt))
    say("uploading %s: alt %.2f side %.2f yaw %.1f deg" % (name, a.alt, a.side, math.degrees(yaw)))
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
    track, t_start, was_armed = [], clock(), False
    faults = []
    for f in a.fault:
        t, kv = f.split(":")
        faults.append([float(t), kv.split("=")[0], float(kv.split("=")[1]), False])
    sws = [[float(f.split(":")[0]), int(f.split(":")[1]), False] for f in a.switch]
    steps = []
    for f in a.gps_step:
        p = f.split(":")
        dn, de = [float(v) for v in p[1].split(",")]
        steps.append([float(p[0]), dn, de, False, float(p[2]) if len(p) > 2 else 0.0, False])
    while clock() - t_start < seconds:
        pump(0.1)
        dt = clock() - t_start
        if a.vis_cut >= 0 and dt >= a.vis_cut and comp["vis"]:
            comp["vis"] = False
            comp["hb"] = not a.hb_cut
            say("COMPANION: route-B height stopped%s" % (" + heartbeat stopped" if a.hb_cut else ""))
        for f in sws:
            if not f[2] and dt >= f[0]:
                rc["ch8"] = f[1]; f[2] = True
                say("SWITCH ch8 -> %d (auto)  est x %.2f y %.2f" % (f[1], st["x"], st["y"]))
        for f in steps:
            if not f[3] and dt >= f[0]:
                gw["dn"], gw["de"] = f[1], f[2]; f[3] = True
                say("GPS STEP %.1f %.1f m" % (f[1], f[2]))
            if f[3] and not f[5] and f[4] > 0 and dt >= f[0] + f[4]:
                gw["dn"] = gw["de"] = 0.0; f[5] = True
                say("GPS STEP end")
        for f in faults:
            if not f[3] and dt >= f[0]:
                pset_fast(f[1], f[2])
                f[3] = True
                say("FAULT: %s = %s" % (f[1], f[2]))
        if st["armed"]:
            was_armed = True
            track.append((dt, -st["z"], st["mode"]))
        elif was_armed:
            break
    if not track:
        return {"flew": False}
    return {"flew": True, "seconds": round(track[-1][0] - track[0][0], 1), "est_alt_max_m": round(max(p[1] for p in track), 2),
            "items_reached": list(st["reached"]), "modes": sorted({p[2] for p in track}), "disarmed": not st["armed"]}


if m.wait_heartbeat(timeout=60) is None:
    sys.exit("no heartbeat on " + a.url)
streams()
rc["thr"] = 1000
pump(1.0)
say("waiting for the EKF and the origin")
t = clock()
while clock() - t < 60 and not (st["origin"] and st["lat"] and st["ekf"] & 0x08):
    pump(0.5)
say("origin %s, EKF flags 0x%03x, rangefinder %.2f m" % (st["origin"], st["ekf"], st["rng"]))
for k, v in FIELD_PARAMS.items():
    say("param %-16s %s -> %s" % (k, *pset(k, v)))
pump(a.settle)
say("EKF flags 0x%03x before arming" % st["ekf"])
out = {}
if not a.no_loiter_test:
    say("=== LOITER: arm, throttle up to %.2f m, hover 15 s, land" % a.hover_alt)
    set_mode(5)
    pump(1.0)
    rc["thr"] = 1000
    pump(1.0)
    m.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
    pump(2.0)
    out["loiter_armed"] = st["armed"]
    rc["thr"] = 1750
    t = clock()
    while clock() - t < 12 and -st["z"] < a.hover_alt - 0.1:
        pump(0.1)
    say("LOITER climb: est alt %.2f m after %.1f s" % (-st["z"], clock() - t))
    rc["thr"] = 1500
    pump(4.0)
    say("LOITER_HOVER_START")
    x0, y0, drift = st["x"], st["y"], 0.0
    t = clock()
    hsw = [[float(f.split(":")[0]), int(f.split(":")[1]), False] for f in a.hover_switch]
    while clock() - t < a.hover_time:
        pump(0.2)
        drift = max(drift, math.hypot(st["x"] - x0, st["y"] - y0))
        for f in hsw:
            if not f[2] and clock() - t >= f[0]:
                rc["ch8"] = f[1]; f[2] = True
                say("SWITCH ch8 -> %d (hover)  est x %.2f y %.2f" % (f[1], st["x"], st["y"]))
    say("LOITER_HOVER_END")
    out["loiter"] = {"armed": st["armed"], "est_alt_m": round(-st["z"], 2), "est_hover_drift_m": round(drift, 2)}
    say("LOITER hover: %s" % out["loiter"])
    rc["thr"] = 1250
    t = clock()
    while clock() - t < 30 and st["armed"]:
        pump(0.2)
    rc["thr"] = 1000
    pump(3.0)
    if st["armed"]:
        m.mav.command_long_send(1, 1, 400, 0, 0, 21196, 0, 0, 0, 0, 0)
        pump(2.0)
if not a.no_auto:
    say("=== AUTO mission %s" % a.mission)
    st["reached"].clear()
    if not upload(a.mission):
        print(json.dumps({"error": "upload failed"}))
        sys.exit(1)
    set_mode(5)
    rc["thr"] = 1000
    pump(2.0)
    m.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
    pump(2.0)
    say("armed %s in %s; switching to AUTO" % (st["armed"], st["mode"]))
    out["auto_armed"] = st["armed"]
    set_mode(3)
    pump(1.0)
    rc["thr"] = 1550
    out["auto"] = fly_report(a.seconds)
out["texts"] = [t for t in st["texts"] if not t.startswith(("ArduCopter", "ChibiOS", "Frame", "RCOut", "IMU"))][-40:]
print("RESULT_JSON " + json.dumps(out))
