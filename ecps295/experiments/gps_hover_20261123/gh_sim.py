#!/usr/bin/env python3
"""GPS hover study (2026-11-23): GUIDED take-off, position hold for --hover s on the EKF position, LAND.
Time base = FC time_boot (ATTITUDE), so the speed-up does not matter.
  --gps-walk S:TAU[:S2:TAU2]  Gauss-Markov horizontal GPS position error (m, s), set through SIM_GPS1_GLTCH_X/Y
  --gps-step T:DN,DE[:DUR]    extra GPS offset T s after the start of the hold (DUR <= 0: for good)
  --fault T:NAME=VAL          parameter change T s after the start of the hold
  --fix TAU                   companion drift fix (gps_drift_fix.GpsDriftFix), 0 = off
"""
import argparse, json, math, os, random, sys, time
from pymavlink import mavutil
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gps_drift_fix import GpsDriftFix

ap = argparse.ArgumentParser()
ap.add_argument("--url", required=True)
ap.add_argument("--speed", type=float, default=1.0)
ap.add_argument("--alt", type=float, default=2.0)
ap.add_argument("--hover", type=float, default=300.0)
ap.add_argument("--settle", type=float, default=25.0)
ap.add_argument("--gps-walk", default="")
ap.add_argument("--gps-step", action="append", default=[])
ap.add_argument("--fault", action="append", default=[])
ap.add_argument("--fix", type=float, default=0.0)
ap.add_argument("--fix-gate", type=float, default=1.5)
ap.add_argument("--fix-dead-k", type=float, default=0.0)
ap.add_argument("--fix-jump", type=float, default=0.0)
ap.add_argument("--seed", type=int, default=1)
a = ap.parse_args()
LAT0, LON0 = 33.6430, -117.8420
MODES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND"}
m = mavutil.mavlink_connection(a.url, source_system=254, source_component=191)
st = {"mode": None, "armed": False, "x": 0.0, "y": 0.0, "z": 0.0, "tb": 0.0, "ekf": 0, "origin": False, "texts": [],
      "gps": None, "gps_new": False, "thr": 1000}
rng = random.Random(a.seed)
g = [float(v) for v in a.gps_walk.split(":")] if a.gps_walk else []
walk = [{"s": g[i], "tau": g[i + 1], "n": rng.gauss(0, g[i]), "e": rng.gauss(0, g[i])} for i in range(0, len(g), 2)]
gw = {"t": -1.0, "dn": 0.0, "de": 0.0}
t_hb = t_org = t_rc = 0.0
W0 = time.time()


def now():
    return st["tb"]


def say(s):
    print("[%7.1f] %s" % (now(), s), flush=True)


def pset_fast(name, value):
    m.mav.param_set_send(1, 1, name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


def pump(sec):
    """Run the link for sec seconds of FC time (wall time before the first ATTITUDE)."""
    global t_hb, t_org, t_rc
    t_end, w_end = now() + sec, time.time() + sec
    while True:
        w = time.time()
        if w - t_hb >= 1.0 / a.speed:
            m.mav.heartbeat_send(18, 8, 0, 0, 0)
            t_hb = w
        if w - t_rc >= 0.1:
            m.mav.rc_channels_override_send(1, 1, 1500, 1500, st["thr"], 1500, 0, 0, 0, 0)
            t_rc = w
        if not st["origin"] and w - t_org >= 1.0:
            m.mav.set_gps_global_origin_send(1, int(LAT0 * 1e7), int(LON0 * 1e7), 20000)
            m.mav.command_long_send(1, 1, 512, 0, 49, 0, 0, 0, 0, 0, 0)
            t_org = w
        t = now()
        if (walk or gw["dn"] or gw["de"]) and t - gw["t"] >= 0.2:
            dt = t - gw["t"] if gw["t"] >= 0 else 0.0
            for c in walk:
                al = math.exp(-dt / c["tau"])
                q = c["s"] * math.sqrt(1 - al * al)
                c["n"], c["e"] = al * c["n"] + q * rng.gauss(0, 1), al * c["e"] + q * rng.gauss(0, 1)
            gw["t"] = t
            pset_fast("SIM_GPS1_GLTCH_X", (sum(c["n"] for c in walk) + gw["dn"]) / 111319.5)
            pset_fast("SIM_GPS1_GLTCH_Y", (sum(c["e"] for c in walk) + gw["de"]) / (111319.5 * math.cos(math.radians(LAT0))))
        x = m.recv_match(blocking=True, timeout=0.01)
        if x is not None:
            k = x.get_type()
            if k == "HEARTBEAT" and x.get_srcSystem() == 1 and x.get_srcComponent() == 1:
                mode, armed = MODES.get(x.custom_mode, str(x.custom_mode)), bool(x.base_mode & 128)
                if (mode, armed) != (st["mode"], st["armed"]):
                    st["mode"], st["armed"] = mode, armed
                    say("FC mode %s %s (alt %.2f)" % (mode, "ARMED" if armed else "disarmed", -st["z"]))
            elif k == "STATUSTEXT":
                st["texts"].append(x.text)
                say("FC text: " + x.text)
            elif k == "LOCAL_POSITION_NED":
                st["x"], st["y"], st["z"] = x.x, x.y, x.z
            elif k == "ATTITUDE":
                st["tb"] = x.time_boot_ms / 1000.0
            elif k == "EKF_STATUS_REPORT":
                st["ekf"] = x.flags
            elif k == "GPS_GLOBAL_ORIGIN":
                st["origin"] = True
            elif k == "GPS_RAW_INT":
                st["gps"] = ((x.lat / 1e7 - LAT0) * 111319.5, (x.lon / 1e7 - LON0) * 111319.5 * math.cos(math.radians(LAT0)),
                             x.fix_type, x.h_acc / 1000.0 if x.h_acc else 0.0, x.satellites_visible)
                st["gps_new"] = True
        if (st["tb"] > 0 and now() >= t_end) or (st["tb"] == 0 and time.time() >= w_end):
            return
        if time.time() - W0 > 1500:
            sys.exit("wall-clock limit")


if m.wait_heartbeat(timeout=60) is None:
    sys.exit("no heartbeat")
for mid, hz in ((0, 2), (1, 2), (24, 5), (30, 20), (32, 20), (193, 2)):
    m.mav.command_long_send(1, 1, 511, 0, mid, 1e6 / hz, 0, 0, 0, 0, 0)
pump(2.0)
t = now()
while now() - t < 60 and not (st["origin"] and st["ekf"] & 0x08):
    pump(0.5)
say("origin %s, EKF flags 0x%03x" % (st["origin"], st["ekf"]))
pump(a.settle)
say("EKF flags 0x%03x before arming, gps %s" % (st["ekf"], st["gps"]))
out = {"armed": False}
m.mav.command_long_send(1, 1, 176, 0, 1, 4, 0, 0, 0, 0, 0)
pump(1.0)
for _ in range(5):
    m.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
    pump(2.0)
    if st["armed"]:
        break
out["armed"] = st["armed"]
if st["armed"]:
    st["thr"] = 1500
    m.mav.command_long_send(1, 1, 22, 0, 0, 0, 0, 0, 0, 0, a.alt)
    t = now()
    while now() - t < 20 and -st["z"] < a.alt - 0.15:
        pump(0.1)
    say("climb: est alt %.2f after %.1f s" % (-st["z"], now() - t))
    pump(5.0)
    ax, ay, az = st["x"], st["y"], st["z"]
    fix = GpsDriftFix(tau=a.fix, gate_m=a.fix_gate, dead_k=a.fix_dead_k, jump_m=a.fix_jump) if a.fix > 0 else None
    cn = ce = 0.0
    steps = []
    for f in a.gps_step:
        p = f.split(":")
        dn, de = [float(v) for v in p[1].split(",")]
        steps.append([float(p[0]), dn, de, float(p[2]) if len(p) > 2 else 0.0, 0])
    faults = [[float(f.split(":")[0]), f.split(":")[1].split("=")[0], float(f.split("=")[1]), False] for f in a.fault]
    csv = open("fix.csv", "w")
    csv.write("t,ekf_n,ekf_e,gps_n,gps_e,fix,hacc,sats,d_n,d_e,c_n,c_e,state\n")
    t0 = now()
    say("HOVER_START tboot=%.3f anchor %.2f %.2f %.2f" % (t0, ax, ay, az))
    while now() - t0 < a.hover and st["armed"] and st["mode"] == "GUIDED":
        pump(0.2)
        dt = now() - t0
        for s in steps:
            if s[4] == 0 and dt >= s[0]:
                gw["dn"], gw["de"], s[4] = s[1], s[2], 1
                say("GPS STEP %.1f %.1f" % (s[1], s[2]))
            if s[4] == 1 and s[3] > 0 and dt >= s[0] + s[3]:
                gw["dn"] = gw["de"] = 0.0
                s[4] = 2
                say("GPS STEP end")
        for f in faults:
            if not f[3] and dt >= f[0]:
                pset_fast(f[1], f[2])
                f[3] = True
                say("FAULT %s = %s" % (f[1], f[2]))
        if st["gps_new"] and st["gps"]:
            st["gps_new"] = False
            gn, ge, ft, hacc, sats = st["gps"]
            if fix:
                cn, ce = fix.update(now(), st["x"], st["y"], gn, ge, ft, hacc, sats)
            csv.write("%.2f,%.3f,%.3f,%.3f,%.3f,%d,%.2f,%d,%.3f,%.3f,%.3f,%.3f,%s\n" % (
                dt, st["x"] - ax, st["y"] - ay, gn, ge, ft, hacc, sats, fix.d[0] if fix else 0, fix.d[1] if fix else 0, cn, ce,
                fix.state if fix else "off"))
        m.mav.set_position_target_local_ned_send(int(now() * 1000), 1, 1, 1, 0x0DF8, ax + cn, ay + ce, az, 0, 0, 0, 0, 0, 0, 0, 0)
    t1 = now()
    say("HOVER_END tboot=%.3f mode %s armed %s" % (t1, st["mode"], st["armed"]))
    out.update(hover_s=round(t1 - t0, 1), end_mode=st["mode"], t0=t0, t1=t1, fix_c=[round(cn, 3), round(ce, 3)],
               fix_gated=fix.n_gated if fix else 0, fix_bad=fix.n_bad if fix else 0, fix_jump=fix.n_jump if fix else 0)
    csv.close()
    m.mav.command_long_send(1, 1, 176, 0, 1, 9, 0, 0, 0, 0, 0)
    t = now()
    while now() - t < 40 and st["armed"]:
        pump(0.2)
    out["disarmed"] = not st["armed"]
out["texts"] = [t for t in st["texts"] if not t.startswith(("ArduCopter", "ChibiOS", "Frame", "RCOut", "IMU"))][-40:]
out["wall_s"] = round(time.time() - W0, 1)
out["sim_s"] = round(now(), 1)
print("RESULT_JSON " + json.dumps(out))
