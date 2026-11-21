#!/usr/bin/env python3
"""abs_fix SITL driver (2026-11-19): AUTO mission + synthetic ArUco marker fixes from SITL truth -> FC.

Derived from /home/jacky/sim/opt_flow2/opt2_sim.py (same origin / arming / mission upload / RC3 1500 logic; the FC holds
altitude itself). Added:
  truth          SIM_STATE (SITL only) at 25 Hz, paired with ATTITUDE.time_boot_ms. All times here are FC time_boot (sim s).
  field frame    origin = truth at the first SIM_STATE (vehicle on the take-off spot), x axis = true yaw at that moment
                 + --field-rot deg. The mission is uploaded in the EKF frame (EKF yaw 0 at boot = nose), i.e. in the field
                 frame when --field-rot 0.
  markers        --markers hov|sq1|sq4 (positions below). Camera: forward, pitched 12 deg down, HFOV 70 / VFOV 55.4 deg;
                 detection gates as in wt/rtl ecps_landmark.py: horizontal range 0.4-3.0 m, |bearing| <= 30 deg,
                 marker facing angle <= 60 deg; plus the vertical FOV with the true vehicle pitch.
  --vis          all | first10 | p30_5 : extra time window counted from the moment the hover item becomes current
                 (first10: [0,10) s, p30_5: (t mod 30) < 5 s). Geometry still applies.
  observation    every 1/--rate s while visible: field position = truth + N(0, --sigma) per axis; with probability
                 --outlier-p a 1.0 m error in a random direction (misdetection); --badfix t0:dur:dx:dy adds a persistent
                 error to all fixes captured in [t0, t0+dur) s after AUTO. --yaw-mode fc: the fix is computed with the
                 FC's yaw (as the existing companion code does) instead of the marker's own orientation.
                 Released to the companion logic --lat s after capture (FC clock).
  --method       none | extnav | extpos   (marker_fix.ExtNavSender / marker_fix.ExtPosResetter after marker_fix.OutlierGate)
  --comp-stop T  companion stops sending anything T s after AUTO.
Writes comp.csv (every observation / gate / send) in the cwd. Prints RESULT_JSON.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
import time

from pymavlink import mavutil

sys.path.insert(0, "/home/jacky/sim/abs_fix")
from marker_fix import EkfHistory, ExtNavSender, ExtPosResetter, OutlierGate, LATLON_TO_M  # noqa: E402

FIELD_PARAMS = {"RTL_ALT_M": 0.0, "WP_SPD": 0.4, "WP_SPD_UP": 0.5, "WP_SPD_DN": 0.4, "WP_RADIUS_M": 0.2, "WP_ACC": 1.0,
                "LOIT_SPEED_MS": 1.0, "FENCE_MARGIN": 0.3}
MODES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND", 21: "SMART_RTL"}
ORIGIN = (int(33.6430e7), int(-117.8420e7))
# marker map, field frame: (x, y, centre height m, facing direction deg = direction of the marker normal)
MARKERS = {
    "hov": [(2.0, 0.0, 1.0, 180.0)],                       # 2 m in front of the take-off spot, facing it
    "sq1": [(0.0, -1.0, 1.0, 90.0)],                       # 1 m beyond the take-off corner on the last leg's line
    "sq4": [(3.0, 0.0, 1.0, 180.0), (2.0, 3.0, 1.0, 270.0), (-1.0, 2.0, 1.0, 0.0), (0.0, -1.0, 1.0, 90.0)],  # 1 m beyond each corner
    "none": [],
}
CAM_PITCH, HFOV_HALF, VFOV_HALF = -12.0, 35.0, 27.7
R_MIN, R_MAX, BEAR_MAX, FACE_MAX = 0.4, 3.0, 30.0, 60.0

ap = argparse.ArgumentParser()
ap.add_argument("mission", choices=["HOV", "HOVW", "SQ"])
ap.add_argument("--url", required=True)
ap.add_argument("--alt", type=float, default=2.0)
ap.add_argument("--side", type=float, default=2.0)
ap.add_argument("--hold", type=float, default=120.0)
ap.add_argument("--laps", type=int, default=3)
ap.add_argument("--seconds", type=float, default=300.0)
ap.add_argument("--speed", type=float, default=1.0)
ap.add_argument("--settle", type=float, default=20.0)
ap.add_argument("--method", default="none", choices=["none", "extnav", "extpos"])
ap.add_argument("--markers", default="none", choices=list(MARKERS))
ap.add_argument("--vis", default="all", choices=["all", "first10", "p30_5"])
ap.add_argument("--sigma", type=float, default=0.05)
ap.add_argument("--claim", type=float, default=-1, help="accuracy reported to the FC (default = --sigma)")
ap.add_argument("--lat", type=float, default=0.1)
ap.add_argument("--rate", type=float, default=10.0)
ap.add_argument("--outlier-p", type=float, default=0.03)
ap.add_argument("--badfix", default="")
ap.add_argument("--yaw-mode", default="marker", choices=["marker", "fc"])
ap.add_argument("--yaw-sigma", type=float, default=3.0, help="deg, noise of the marker-derived yaw")
ap.add_argument("--yaw-out", default="marker", choices=["marker", "echo"],
                help="extnav: yaw in ODOMETRY.q = marker-derived yaw (field frame) or the FC's own yaw echoed back")
ap.add_argument("--field-rot", type=float, default=0.0)
ap.add_argument("--comp-stop", type=float, default=-1)
ap.add_argument("--no-gate", action="store_true", help="skip OutlierGate (EKF innovation gate only)")
ap.add_argument("--gate2", action="store_true", help="OutlierGate + time-growing innovation check against the FC estimate")
ap.add_argument("--reset-gap", type=float, default=-1, help="extnav: bump ODOMETRY.reset_counter on the first fix after a gap > s")
ap.add_argument("--ekf-frame", default="field", choices=["field", "ned"],
                help="ned: EKF frame is NED (compass); the companion rotates field fixes by the surveyed field azimuth")
ap.add_argument("--azi-err", type=float, default=0.0, help="deg, error of the surveyed field azimuth (--ekf-frame ned)")
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--set", action="append", default=[])
ap.add_argument("--fault", action="append", default=[])  # T:NAME=VAL parameter change T s after AUTO
a = ap.parse_args()
for kv in a.set:
    FIELD_PARAMS[kv.split("=")[0]] = float(kv.split("=")[1])
claim = a.sigma if a.claim < 0 else a.claim
rng = random.Random(a.seed)
S = a.speed
m = mavutil.mavlink_connection(a.url, source_system=254, source_component=191)
st = {"mode": None, "armed": False, "x": 0.0, "y": 0.0, "vx": 0.0, "vy": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
      "lat": 0, "lon": 0, "rng": float("nan"), "seq": -1, "ekf": 0, "origin": False, "texts": [], "reached": [], "tboot": 0,
      "truth": None, "t_seq": {}}
rc = {"thr": None}
comp = {"on": True}
t_hb = t_rc = t_org = 0.0
T0 = time.time()
field = {"o": None}  # (n0, e0, psi0) truth origin and field x-axis direction
hist = EkfHistory()
gate = OutlierGate(ekf_hist=hist, innov_base=max(0.3, 3 * claim)) if a.gate2 else OutlierGate()
sender = None
resetter = None
pending = []  # (t_release, t_capture, x, y, truth_x, truth_y, kind)
t_next_obs = 0.0
t_auto = None
diag = {"t": 0.0}
badfix = [float(v) for v in a.badfix.split(":")] if a.badfix else None
cfh = open("comp.csv", "w", newline="")
cw = csv.writer(cfh)
cw.writerow(["t", "ev", "tx", "ty", "mx", "my", "ex", "ey", "info"])


def tnow():
    return st["tboot"] / 1000.0


def say(msg):
    print("[%7.2f] %s" % (tnow(), msg), flush=True)


def truth_field(tr):
    n0, e0, psi0 = field["o"]
    n = (tr["lat"] - ORIGIN[0]) * LATLON_TO_M - n0
    e = (tr["lon"] - ORIGIN[1]) * LATLON_TO_M * math.cos(math.radians(ORIGIN[0] * 1e-7)) - e0
    c, s = math.cos(psi0), math.sin(psi0)
    return c * n + s * e, -s * n + c * e, tr["yaw"] - psi0


def visible(px, py, alt, yawf, pitch):
    for mx, my, mh, mf in MARKERS[a.markers]:
        dx, dy = mx - px, my - py
        r = math.hypot(dx, dy)
        if not (R_MIN <= r <= R_MAX):
            continue
        bear = math.degrees(math.atan2(dy, dx) - yawf)
        bear = (bear + 180) % 360 - 180
        if abs(bear) > min(BEAR_MAX, HFOV_HALF):
            continue
        face = math.degrees(math.atan2(-dy, -dx)) - mf
        face = (face + 180) % 360 - 180
        if abs(face) > FACE_MAX:
            continue
        elev = math.degrees(math.atan2(mh - alt, r))
        if abs(elev - (math.degrees(pitch) + CAM_PITCH)) > VFOV_HALF:
            continue
        return (mx, my, r)
    return None


def window_open(t):
    if a.vis == "all":
        return True
    t2 = st["t_seq"].get(2)
    if t2 is None:
        return False
    h = t - t2
    if a.vis == "first10":
        return 0 <= h < 10
    return h >= 0 and (h % 30) < 5


def companion_step():
    """Synthesise observations, release them after the latency, run gate + method."""
    global t_next_obs
    tr = st["truth"]
    t = tnow()
    if tr is None or field["o"] is None:
        return
    if not comp["on"]:
        return
    att_fc = (st["roll"], st["pitch"], st["yaw"])
    if t >= t_next_obs:
        t_next_obs = t + 1.0 / a.rate
        px, py, yawf = truth_field(tr)
        alt = tr["alt"]
        vis = visible(px, py, alt, yawf, tr["pitch"]) if window_open(t) else None
        if vis:
            mx, my, r = vis
            azi = (field["o"][2] + math.radians(a.azi_err)) if a.ekf_frame == "ned" else 0.0  # field -> EKF frame rotation
            if a.yaw_mode == "fc":  # companion rotates the marker-relative vector with the FC (EKF) yaw
                dyaw = (st["yaw"] - azi) - yawf
                rx, ry = px - mx, py - my
                ox = mx + math.cos(dyaw) * rx - math.sin(dyaw) * ry
                oy = my + math.sin(dyaw) * rx + math.cos(dyaw) * ry
            else:
                ox, oy = px, py
            ox += rng.gauss(0, a.sigma)
            oy += rng.gauss(0, a.sigma)
            kind = "obs"
            if rng.random() < a.outlier_p:
                th = rng.uniform(0, 2 * math.pi)
                ox, oy, kind = ox + math.cos(th), oy + math.sin(th), "obs_outlier"
            if badfix and t_auto is not None and badfix[0] <= t - t_auto < badfix[0] + badfix[1]:
                ox, oy, kind = ox + badfix[2], oy + badfix[3], "obs_bad"
            yaw_meas = yawf + math.radians(rng.gauss(0, a.yaw_sigma)) + azi
            if azi:
                ox, oy = math.cos(azi) * ox - math.sin(azi) * oy, math.sin(azi) * ox + math.cos(azi) * oy
            pending.append((t + a.lat, t, ox, oy, px, py, kind, yaw_meas))
    while pending and pending[0][0] <= t:
        _, tc, ox, oy, px, py, kind, yaw_meas = pending.pop(0)
        cw.writerow(["%.3f" % t, kind, "%.3f" % px, "%.3f" % py, "%.3f" % ox, "%.3f" % oy, "%.3f" % st["x"], "%.3f" % st["y"], "%.3f" % tc])
        if a.method == "none":
            continue
        acc = [(tc, ox, oy)] if a.no_gate else gate.push((tc, ox, oy), (st["vx"], st["vy"]))
        for f in acc:
            if a.method == "extnav":
                if f[0] == tc:  # only the newest of a confirmed batch is sent (older ones are stale by then)
                    yaw_out = yaw_meas if a.yaw_out == "marker" else st["yaw"]
                    fx, fy = f[1], f[2]
                    if a.reset_gap > 0 and len(acc) > 1 and t - sender.t_last_fix > a.reset_gap:  # reset with the batch mean
                        fx, fy = sum(q[1] for q in acc) / len(acc), sum(q[2] for q in acc) / len(acc)
                    rs = sender.fix(t, fx, fy, -(st["rng"] if st["rng"] == st["rng"] else 0.0), (st["roll"], st["pitch"], yaw_out))
                    cw.writerow(["%.3f" % t, "sent", "", "", "%.3f" % fx, "%.3f" % fy, "", "", "reset" if rs else ""])
            else:
                ev = resetter.push(f, t)
                if ev:
                    cw.writerow(["%.3f" % t, "reset" if ev["sent"] else "reset_skip", "", "", "%.3f" % ev["dx"], "%.3f" % ev["dy"], "", "", ev["n"]])
                    if ev["sent"]:
                        say("EXT POS reset: offset %.2f %.2f (n %d)" % (ev["dx"], ev["dy"], ev["n"]))
        if not acc:
            cw.writerow(["%.3f" % t, "gated", "", "", "%.3f" % ox, "%.3f" % oy, "", "", ""])
    if a.method == "extnav":
        sender.tick(t, att_fc)
    if t - diag["t"] >= 5.0:  # link backlog check: FC clock vs wall clock * speedup (should stay constant)
        diag["t"] = t
        cw.writerow(["%.3f" % t, "clock", "", "", "", "", "", "", "%.3f" % ((time.time() - T0) * S - t)])


def pump(sec=0.05):
    global t_hb, t_rc, t_org
    t_end = time.time() + sec / S
    while True:
        now = time.time()
        if comp["on"] and now - t_hb >= 1.0 / S:
            m.mav.heartbeat_send(18, 8, 0, 0, 0)
            t_hb = now
        if not st["origin"] and now - t_org >= 2.0 / S:
            m.mav.set_gps_global_origin_send(1, ORIGIN[0], ORIGIN[1], 20000)
            m.mav.command_long_send(1, 1, 512, 0, 49, 0, 0, 0, 0, 0, 0)
            t_org = now
        if rc["thr"] is not None and now - t_rc >= 0.1 / S:
            m.mav.rc_channels_override_send(1, 1, 1500, 1500, rc["thr"], 1500, 0, 0, 0, 0)
            t_rc = now
        x = m.recv_match(blocking=True, timeout=0.002)
        nmsg = 0
        while x is not None:
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
                st["x"], st["y"], st["z"], st["vx"], st["vy"] = x.x, x.y, x.z, x.vx, x.vy
                hist.add(x.time_boot_ms / 1000.0, x.x, x.y, x.vx, x.vy)
            elif k == "ATTITUDE":
                st["roll"], st["pitch"], st["yaw"], st["tboot"] = x.roll, x.pitch, x.yaw, x.time_boot_ms
            elif k == "SIM_STATE":  # SITL truth: lat_int/lon_int 1e7 deg, alt m AMSL, yaw/pitch rad
                if field["o"] is None and st["tboot"] > 0:
                    n0 = (x.lat_int - ORIGIN[0]) * LATLON_TO_M
                    e0 = (x.lon_int - ORIGIN[1]) * LATLON_TO_M * math.cos(math.radians(ORIGIN[0] * 1e-7))
                    field["o"] = (n0, e0, x.yaw + math.radians(a.field_rot))
                    field["alt0"] = x.alt
                    say("field origin from truth: n0 %.3f e0 %.3f, x axis %.1f deg (true yaw %.1f + rot %.1f)" % (
                        n0, e0, math.degrees(field["o"][2]), math.degrees(x.yaw), a.field_rot))
                if field["o"] is not None:
                    st["truth"] = {"lat": x.lat_int, "lon": x.lon_int, "yaw": x.yaw, "pitch": x.pitch, "alt": x.alt - field["alt0"]}
            elif k == "GLOBAL_POSITION_INT":
                st["lat"], st["lon"] = x.lat, x.lon
            elif k == "RANGEFINDER":
                st["rng"] = x.distance
            elif k == "EKF_STATUS_REPORT":
                st["ekf"] = x.flags
            elif k == "GPS_GLOBAL_ORIGIN":
                st["origin"] = True
            elif k == "MISSION_CURRENT":
                if x.seq != st["seq"]:
                    st["seq"] = x.seq
                    st["t_seq"].setdefault(x.seq, tnow())
            elif k == "MISSION_ITEM_REACHED":
                if x.seq not in st["reached"]:
                    st["reached"].append(x.seq)
                    say("mission item %d REACHED  (x %.2f y %.2f alt %.2f)" % (x.seq, st["x"], st["y"], -st["z"]))
            nmsg += 1
            x = m.recv_match(blocking=False) if nmsg < 20 else None  # bounded: the companion step must run often
        companion_step()
        if time.time() >= t_end:
            return


def streams():
    # 0 HB, 1 SYS_STATUS, 30 ATTITUDE, 32 LOCAL_POSITION_NED, 33 GLOBAL_POSITION_INT, 42 MISSION_CURRENT, 74 VFR, 173 RANGEFINDER,
    # 193 EKF_STATUS, 108 SIM_STATE (SITL truth)
    for mid, hz in ((0, 2), (1, 2), (30, 25), (32, 25), (33, 5), (42, 5), (74, 2), (173, 10), (193, 2), (108, 25)):
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
    s, h, al = a.side, a.hold, a.alt
    if name == "HOV":
        return [(16, 0, 0, 0, 0), (22, 0, 0, al, 0), (19, 0, 0, al, h), (21, 0, 0, 0, 0)]
    if name == "HOVW":  # hover as a NAV_WAYPOINT with hold time: AUTO WP submode -> EKF resets move the vehicle (MoveVehicle)
        return [(16, 0, 0, 0, 0), (22, 0, 0, al, 0), (16, 0, 0, al, h), (21, 0, 0, 0, 0)]
    pts = [(s, 0, 0), (s, s, 0), (0, s, 0), (0, 0, 0)] * a.laps
    return [(16, 0, 0, 0, 0), (22, 0, 0, al, 0)] + [(16, x, y, al, hold) for x, y, hold in pts] + [(21, 0, 0, 0, 0)]


def upload(name):
    for _ in range(60):
        pump(0.1)
        if st["lat"] and st["origin"]:
            break
    yaw = st["yaw"]  # EKF yaw (0 at boot without compass): mission in the EKF frame = field frame
    wire = []
    for seq, (cmd, x, y, alt, hold) in enumerate(items := mission_items(name)):
        north, east = x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw)
        lat = ORIGIN[0] + north / LATLON_TO_M
        lon = ORIGIN[1] + east / (LATLON_TO_M * math.cos(math.radians(ORIGIN[0] * 1e-7)))
        if cmd == 21:
            lat = lon = 0.0
        wire.append((seq, cmd, hold if cmd in (16, 19) else 0.0, int(round(lat)), int(round(lon)), alt))
    say("uploading %s: alt %.2f side %.2f EKF yaw %.1f deg, %d items" % (name, a.alt, a.side, math.degrees(yaw), len(items)))
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


def fly(seconds):
    global t_auto
    t_auto = tnow()
    faults = [[float(f.split(":")[0]), f.split(":")[1].split("=")[0], float(f.split("=")[1]), False] for f in a.fault]
    was_armed, n = False, 0
    while tnow() - t_auto < seconds:
        pump(0.1)
        dt = tnow() - t_auto
        if a.comp_stop >= 0 and dt >= a.comp_stop and comp["on"]:
            comp["on"] = False
            say("COMPANION STOPPED (no heartbeat, no fixes)")
        for f in faults:
            if not f[3] and dt >= f[0]:
                m.mav.param_set_send(1, 1, f[1].encode(), f[2], mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                f[3] = True
                say("FAULT: %s = %s" % (f[1], f[2]))
        if st["armed"]:
            was_armed = True
            n += 1
        elif was_armed:
            break
    return {"flew": was_armed, "seconds": round(tnow() - t_auto, 1), "items_reached": list(st["reached"]), "disarmed": not st["armed"]}


if m.wait_heartbeat(timeout=60) is None:
    sys.exit("no heartbeat on " + a.url)
streams()
pump(1.0)
if a.method == "extnav":
    sender = ExtNavSender(m.mav, sigma=claim, yaw_sigma=math.radians(a.yaw_sigma) if a.yaw_out == "marker" else 0.3,
                          reset_gap_s=a.reset_gap if a.reset_gap > 0 else None)
elif a.method == "extpos":
    resetter = ExtPosResetter(m.mav, ORIGIN, hist, sigma=claim)
say("waiting for the EKF and the origin")
t = time.time()
while time.time() - t < 60 / S + 30 and not (st["origin"] and st["lat"] and st["ekf"] & 0x08):
    pump(0.5)
say("origin %s, EKF flags 0x%03x, rangefinder %.2f m" % (st["origin"], st["ekf"], st["rng"]))
for k, v in FIELD_PARAMS.items():
    say("param %-16s %s -> %s" % (k, *pset(k, v)))
pump(a.settle)
out = {}
say("=== AUTO mission %s method %s markers %s vis %s sigma %.2f lat %.2f rate %.0f" % (a.mission, a.method, a.markers, a.vis, a.sigma, a.lat, a.rate))
st["reached"].clear()
if not upload(a.mission):
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
rc["thr"] = 1500
out["auto"] = fly(a.seconds)
out["gate_innov_rej"] = gate.n_innov_rej
if resetter:
    out["resets"] = len(resetter.events)
if sender:
    out["fixes_sent"] = sender.n_sent
out["texts"] = [t for t in st["texts"] if not t.startswith(("ArduCopter", "ChibiOS", "Frame", "RCOut", "IMU"))][-40:]
cfh.close()
print("RESULT_JSON " + json.dumps(out))
