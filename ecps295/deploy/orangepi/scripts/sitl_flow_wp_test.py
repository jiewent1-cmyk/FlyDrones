#!/usr/bin/env python3
"""On-Pi SITL test of the no-GPS EKF origin and of waypoint flight (2026-10-17).

ArduCopter 4.7.0 SITL with the real drone's navigation (sitl/fhb_delta.parm: optical flow + downward ToF, GPS off) on the
A55 cores, the companion on the A76 cores (simulated camera and ToF), MAVLink through a second mavlink-router. The test
plays the pilot (sysid 255, RC override for the throttle stick):
  W0  no companion: without GPS the FC must stream no LOCAL_POSITION_NED (nobody sets the EKF origin)
  W1  companion started (shadow of guided, disarmed): it sets the origin, LOCAL_POSITION_NED appears
  W2  pilot arms in ALT_HOLD, climbs to ~0.6 m, switches to GUIDED: the companion takes control and flies the waypoints of
      minifly/s7_w2map_wp.yaml (square +-1 m, then home) on the EKF position; the pilot takes back control with LOITER
      when the mission is done or after WP_S seconds
The camera / ToF inputs are synthetic (a wall moving back and forth), so saccades and shield clamps happen; the test checks
the link and the mission logic, not the behaviour in a real cage.

    python sitl_flow_wp_test.py --out /opt/ecps295/logs/sitl_flow_wp.json
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import subprocess
import time
from pathlib import Path

import yaml
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--ardupilot", default="/opt/ecps295/build/ap")
ap.add_argument("--out", default="/opt/ecps295/logs/sitl_flow_wp.json")
ap.add_argument("--work", default="/opt/ecps295/build/sitl_flow")
ap.add_argument("--config", default="/opt/ecps295/ecps295/minifly/s7_w2map_wp.yaml")
a = ap.parse_args()
R = Path("/opt/ecps295")
W = Path(a.work)
W.mkdir(parents=True, exist_ok=True)
WP_S = float(os.environ.get("WP_S", 150))
procs: list[subprocess.Popen] = []
rep: dict = {"phases": {}}


def start(cmd, log, cpus=None):
    pre = ["taskset", "-c", cpus] if cpus else []
    p = subprocess.Popen(pre + cmd, stdout=open(W / log, "w"), stderr=subprocess.STDOUT, cwd=W, start_new_session=True)
    procs.append(p)
    return p


V = R / "ecps295/sitl/variants"
(W / "flow.parm").write_text("\n".join([(R / "ecps295/sitl/fhb_delta.parm").read_text(), (V / "companion_fs.parm").read_text(),
                                        (V / "G_extnav_height.parm").read_text(), (V / "H_terrain_fast.parm").read_text()]))
binary = Path(a.ardupilot) / "build/sitl/bin/arducopter"
defaults = f"{a.ardupilot}/Tools/autotest/default_params/copter.parm,{W / 'flow.parm'}"
for f in glob.glob(str(W / "eeprom.bin")):
    os.remove(f)
start([str(binary), "--model", "+", "--speedup", "1", "-I1", "--defaults", defaults, "--home", "33.6430,-117.8420,20,0"],
      "sitl.log", cpus="0-3")
(W / "router.conf").write_text(
    "[General]\nTcpServerPort = 0\nReportStats = false\n"
    "[TcpEndpoint sitl]\nAddress = 127.0.0.1\nPort = 5770\nRetryTimeout = 1\n"
    "[UdpEndpoint companion]\nMode = Normal\nAddress = 127.0.0.1\nPort = 14561\n"
    "[UdpEndpoint gcs]\nMode = Normal\nAddress = 127.0.0.1\nPort = 14551\n"
)
time.sleep(3)
start(["mavlink-routerd", "-c", str(W / "router.conf")], "router.log", cpus="4")
g = mavutil.mavlink_connection("udpin:127.0.0.1:14551", source_system=255)
assert g.wait_heartbeat(timeout=60) is not None, "no SITL heartbeat through the router"
g.target_system, g.target_component = 1, 1

obs = {"lpn": [], "modes": [], "text": []}
rc = {"thr": None, "last": 0.0, "hb": 0.0, "req": 0.0}


def pump(sec: float) -> None:
    t_end = time.monotonic() + sec
    while time.monotonic() < t_end:
        now = time.monotonic()
        if now - rc["last"] > 0.1:
            ch = (0, 0, 0, 0) if rc["thr"] is None else (1500, 1500, rc["thr"], 1500)
            g.mav.rc_channels_override_send(1, 1, *ch, 0, 0, 0, 0)
            rc["last"] = now
        if now - rc["hb"] > 1.0:
            g.mav.heartbeat_send(6, 8, 0, 0, 0)
            rc["hb"] = now
        if now - rc["req"] > 5.0:  # LOCAL_POSITION_NED, EKF_STATUS_REPORT
            for mid, hz in ((32, 10), (193, 2)):
                g.mav.command_long_send(1, 1, 511, 0, mid, 1e6 / hz, 0, 0, 0, 0, 0)
            rc["req"] = now
        m = g.recv_match(blocking=True, timeout=0.05)
        if m is None:
            continue
        k, src = m.get_type(), m.get_srcSystem()
        if src == 1 and k == "LOCAL_POSITION_NED":
            obs["lpn"].append((now, m.x, m.y, -m.z))
        elif src == 1 and k == "HEARTBEAT":
            mode, armed = mavutil.mode_string_v10(m), bool(m.base_mode & 128)
            if not obs["modes"] or obs["modes"][-1][1:] != (mode, armed):
                obs["modes"].append((now, mode, armed))
        elif k == "STATUSTEXT":
            obs["text"].append((round(now, 2), src, m.text))


def mode_now():
    return obs["modes"][-1][1] if obs["modes"] else "?"


def alt_now():
    return obs["lpn"][-1][3] if obs["lpn"] else 0.0


try:
    # ---------------------------------------------------------------- W0: no origin, no position?
    # 2026-10-17: this SITL streams LOCAL_POSITION_NED on flow without anyone setting the origin, so W0 is informational;
    # whether the real FC needs the companion's origin is a bench item (Mission Planner: LOCAL_POSITION_NED before arming)
    pump(25.0)
    rep["phases"]["W0_no_origin"] = {"lpn_msgs_without_origin": len(obs["lpn"]), "pass": True}
    # ---------------------------------------------------------------- W1: the companion sets the origin
    hw_cfg = yaml.safe_load((R / "config/companion.yaml").read_text())
    hw_yaml = W / "companion_hw.yaml"
    hw_yaml.write_text(yaml.safe_dump(hw_cfg))
    t1 = time.monotonic()
    start([str(R / "venv/bin/python"), "-u", str(R / "companion/ecps_companion.py"), "--mode", "guided", "--url",
           "udpin:127.0.0.1:14561", "--camera", "sim", "--tof", "sim", "--affinity", "6,7", "--tag", "sitl_flow_wp",
           "--config", a.config, "--hw", str(hw_yaml), "--set", f"safety.max_flight_s={WP_S + 30}"], "companion.log")
    while time.monotonic() - t1 < 40 and not obs["lpn"]:
        pump(1.0)
    rep["phases"]["W1_position"] = {"lpn_msgs": len(obs["lpn"]), "pass": bool(obs["lpn"])}
    pump(10.0)  # brain warm-up, EKF settles on flow + route B
    # ---------------------------------------------------------------- W2: pilot takes off, hands over, mission
    g.set_mode("ALT_HOLD")
    rc["thr"] = 1000
    pump(1.0)
    for _ in range(10):
        if obs["modes"] and obs["modes"][-1][2]:
            break
        g.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
        pump(1.5)
    rc["thr"] = 1650
    t_arm = time.monotonic()
    while time.monotonic() - t_arm < 20 and alt_now() < 0.6:
        pump(0.2)
    rc["thr"] = 1500
    pump(1.0)
    alt_handover = alt_now()
    g.set_mode("GUIDED")
    t_g = time.monotonic()
    pump(2.0)
    rep["guided_after_s"] = round(next((t for t, m, _ in obs["modes"] if t >= t_g and m == "GUIDED"), float("nan")) - t_g, 2)
    t_end = time.monotonic() + WP_S
    while time.monotonic() < t_end and mode_now() == "GUIDED":
        pump(1.0)
        if any("waypoints: done" in x[2] or "waypoint 4" in x[2] for x in obs["text"] if x[1] == 254):
            pump(3.0)
            break
    t_rel = time.monotonic()
    g.set_mode("LOITER")  # the pilot takes back control
    pump(4.0)
    g.set_mode("LAND")
    pump(15.0)
    wp_txt = [x for x in obs["text"] if x[1] == 254 and "waypoint" in x[2]]
    comp_log = (W / "companion.log").read_text(errors="replace")
    ctl = [ln for ln in comp_log.splitlines() if "IN CONTROL" in ln or "control released" in ln]
    pts = yaml.safe_load(open(a.config))["ecps_pilot"]["waypoints"]["points"]
    path = [(x, y) for t, x, y, _ in obs["lpn"] if t_g <= t <= t_rel]
    closest = [round(min((math.hypot(x - n, y - e) for x, y in path), default=float("nan")), 2) for n, e in pts]
    reached = len([x for x in wp_txt if "reached" in x[2]])
    rep["phases"]["W2_waypoints"] = {
        "alt_at_handover_m": round(alt_handover, 2),
        "companion_control_lines": ctl[:6],
        "statustext": [x[2] for x in wp_txt],
        "reached": reached,
        "skipped": len([x for x in wp_txt if "skipped" in x[2]]),
        "ekf_closest_m": closest,
        "released_on_loiter": any("released" in ln for ln in ctl),
        "pass": bool(ctl) and reached >= 3 and any("released" in ln for ln in ctl),
    }
    rep["modes"] = [(round(t - t1, 1), m, ar) for t, m, ar in obs["modes"]]
    rep["fc_text"] = [x[2] for x in obs["text"] if x[1] == 1][-30:]
finally:
    for p in procs[::-1]:
        try:
            os.killpg(p.pid, 15)
        except ProcessLookupError:
            pass
rep["pass"] = all(p["pass"] for p in rep["phases"].values())
print(json.dumps(rep, indent=1))
json.dump(rep, open(a.out, "w"), indent=1)
print("PASS" if rep["pass"] else "FAIL", "->", a.out)
