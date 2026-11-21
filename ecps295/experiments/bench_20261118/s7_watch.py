#!/usr/bin/env python3
"""ECPS295 bench S7: wait until the FC is armed in a given mode above a height, inject one fault, time what the FC does.

Runs on the Orange Pi against the mavlink-router diagnostics tap (udpin:127.0.0.1:14570); sends nothing but stream
requests. Props off.

    s7_watch.py --name 7.1 --fault 'pkill -9 -f "^/opt/ecps295/venv/bin/python -u /opt/ecps295/companion/ecps_companion"'
    s7_watch.py --name 7.8 --fault ''          # no injected fault: the operator switches the transmitter off

It prints one line per change of FC mode / armed state and every STATUSTEXT, with the time since the fault.
"""
import argparse
import subprocess
import sys
import time

from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--name", default="S7")
ap.add_argument("--fault", default="", help="shell command run once the start condition has held; empty = operator fault")
ap.add_argument("--mode", default="GUIDED", help="FC mode required before the fault")
ap.add_argument("--alt", type=float, default=0.25, help="rangefinder height required before the fault [m]")
ap.add_argument("--hold", type=float, default=4.0, help="the start condition must hold this long [s]")
ap.add_argument("--wait", type=float, default=300.0, help="give up waiting for the start condition after this [s]")
ap.add_argument("--after", type=float, default=25.0, help="record this long after the fault [s]")
ap.add_argument("--takeoff", type=float, default=0.0, metavar="ALT_M",
                help="bench only, props off: once the start condition holds, send a GUIDED takeoff to this height so the "
                     "FC spools up and counts as flying (a hand-held drone otherwise stays 'landed' and a failsafe just "
                     "disarms); the fault follows when the throttle output has been above 15 %% for 2 s")
ap.add_argument("--end", default="", help="shell command run at the very end (e.g. undo the fault)")
ap.add_argument("--set-param", default="", metavar="NAME=VALUE",
                help="fault = set this FC parameter (e.g. BATT_LOW_VOLT=9.5); the old value is written back at the end "
                     "and read back, also when the script is interrupted")
a = ap.parse_args()

m = mavutil.mavlink_connection("udpin:127.0.0.1:14570", source_system=250, source_component=191)
m.wait_heartbeat(timeout=10)
for mid, us in ((0, 100000), (1, 500000), (65, 50000), (173, 100000), (74, 200000)):  # HEARTBEAT 10 Hz, SYS_STATUS, RC_CHANNELS 20 Hz, RANGEFINDER, VFR_HUD
    m.mav.command_long_send(1, 1, 511, 0, mid, us, 0, 0, 0, 0, 0)

st = {"mode": None, "armed": None, "rng": float("nan"), "thr": 0, "rc": 0, "sb": 0, "comp_hb": 0.0, "volt": 0.0, "alt": float("nan"), "band": None}
t_fault = None
events = []


def log(msg):
    dt = "      " if t_fault is None else "%+6.2f" % (time.time() - t_fault)
    line = f"[{a.name}] {dt} s  {msg}"
    events.append(line)
    print(line, flush=True)


def pump(timeout=0.2):
    x = m.recv_match(blocking=True, timeout=timeout)
    if x is None:
        return
    k = x.get_type()
    if k == "HEARTBEAT":
        if x.get_srcSystem() == 1 and x.get_srcComponent() == 1 and x.autopilot == 3:
            mode, armed = mavutil.mode_string_v10(x), bool(x.base_mode & 128)
            if mode != st["mode"] or armed != st["armed"]:
                st["mode"], st["armed"] = mode, armed
                log(f"FC mode {mode}  {'ARMED' if armed else 'disarmed'}  (height {st['rng']:.2f} m, throttle out {st['thr']} %)")
        elif x.get_srcSystem() == 254:
            st["comp_hb"] = time.time()
    elif k == "STATUSTEXT":
        log(f"FC text: {x.text}")
    elif k == "RANGEFINDER":
        st["rng"] = x.distance
    elif k == "VFR_HUD":
        st["thr"], st["alt"] = x.throttle, x.alt
    elif k == "RC_CHANNELS":
        st["rc"], st["sb"] = x.rssi, x.chan6_raw
        band = "low" if x.chan6_raw < 1300 else "high" if x.chan6_raw > 1700 else "mid"
        if x.rssi and band != st["band"]:
            if st["band"] is not None and t_fault is not None:
                log(f"pilot mode switch (ch 6) {st['band']} -> {band}")
            st["band"] = band
    elif k == "SYS_STATUS":
        st["volt"] = x.voltage_battery / 1000


def param_get(name):
    for _ in range(5):
        m.mav.param_request_read_send(1, 1, name.encode(), -1)
        t = time.time()
        while time.time() - t < 1.0:
            x = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
            if x and x.param_id == name:
                return x.param_value, x.param_type
    return None, None


def param_set(name, value, ptype):
    for _ in range(5):
        m.mav.param_set_send(1, 1, name.encode(), float(value), ptype)
        got, _ = param_get(name)
        if got is not None and abs(got - value) < 1e-4:
            return got
    return param_get(name)[0]


p_name = p_old = p_type = None
if a.set_param:
    p_name, p_new = a.set_param.split("=")
    p_new = float(p_new)
    p_old, p_type = param_get(p_name)
    if p_old is None:
        print(f"[{a.name}] cannot read {p_name} - stopping", flush=True)
        sys.exit(4)
    print(f"[{a.name}] {p_name} is {p_old:g}; the fault will set it to {p_new:g} and the end will restore {p_old:g}", flush=True)

print(f"[{a.name}] waiting for: ARMED + {a.mode} + height >= {a.alt} m for {a.hold} s (give up after {a.wait:.0f} s)", flush=True)
t0, ok_since, last_note = time.time(), None, 0.0
while True:
    pump()
    now = time.time()
    ok = st["armed"] and st["mode"] == a.mode and st["rng"] >= a.alt
    ok_since = (ok_since or now) if ok else None
    if now - last_note > 10:
        last_note = now
        print(f"[{a.name}]   ... mode {st['mode']} armed {st['armed']} height {st['rng']:.2f} m throttle {st['thr']} % "
              f"rc {st['rc']} batt {st['volt']:.2f} V", flush=True)
    if ok_since and now - ok_since >= a.hold:
        break
    if now - t0 > a.wait:
        print(f"[{a.name}] start condition not reached in {a.wait:.0f} s - nothing injected", flush=True)
        sys.exit(2)

if a.takeoff > 0:
    m.mav.command_long_send(1, 1, 22, 0, 0, 0, 0, 0, 0, 0, a.takeoff)  # MAV_CMD_NAV_TAKEOFF
    print(f"[{a.name}] GUIDED takeoff to {a.takeoff} m sent; waiting for throttle output > 15 % for 2 s", flush=True)
    t1, up_since = time.time(), None
    while True:
        pump()
        now = time.time()
        up_since = (up_since or now) if st["thr"] > 15 else None
        if up_since and now - up_since >= 2.0:
            print(f"[{a.name}] flying state: throttle out {st['thr']} %, height {st['rng']:.2f} m", flush=True)
            break
        if not st["armed"] or now - t1 > 12:
            print(f"[{a.name}] takeoff did not spool up (armed {st['armed']}, mode {st['mode']}, throttle {st['thr']} %) "
                  f"- nothing injected", flush=True)
            sys.exit(3)

comp_alive = time.time() - st["comp_hb"] < 2.5
t_fault = time.time()
if p_name:
    try:
        got = param_set(p_name, p_new, p_type)
        t_fault = time.time()
        log(f"FAULT injected: {p_name} {p_old:g} -> {got:g}  (battery {st['volt']:.2f} V)")
    except BaseException:
        param_set(p_name, p_old, p_type)
        raise
elif a.fault:
    r = subprocess.run(a.fault, shell=True, capture_output=True, text=True)
    log(f"FAULT injected: {a.fault}  (exit {r.returncode}; companion heartbeat was {'present' if comp_alive else 'ABSENT'})")
else:
    log("start condition met - operator: make the fault NOW")
t_end = time.time() + a.after
last_hb = st["comp_hb"]
t_thr = time.time()
while time.time() < t_end:
    try:
        pump()
    except BaseException:
        if p_name:
            param_set(p_name, p_old, p_type)
        raise
    if st["armed"] and time.time() - t_thr >= 2.0:
        t_thr = time.time()
        log(f"   throttle out {st['thr']} %, rangefinder {st['rng']:.2f} m, EKF alt {st['alt']:.2f} m, battery {st['volt']:.2f} V")
    if last_hb and st["comp_hb"] == last_hb and time.time() - last_hb > 2.5:
        log("companion heartbeat (sysid 254) gone")
        last_hb = 0.0
    elif not last_hb and time.time() - st["comp_hb"] < 1.0:
        log("companion heartbeat (sysid 254) back")
        last_hb = st["comp_hb"]
    else:
        last_hb = st["comp_hb"] if last_hb else last_hb
if p_name:
    got = param_set(p_name, p_old, p_type)
    log(f"restored {p_name} = {got:g}" + ("" if got is not None and abs(got - p_old) < 1e-4 else f"  *** NOT RESTORED, expected {p_old:g} ***"))
if a.end:
    r = subprocess.run(a.end, shell=True, capture_output=True, text=True)
    log(f"end command: {a.end}  (exit {r.returncode})")
print(f"[{a.name}] end: mode {st['mode']} armed {st['armed']} height {st['rng']:.2f} m rc {st['rc']} batt {st['volt']:.2f} V", flush=True)
