#!/usr/bin/env python3
"""On-Pi SITL test of ArUco target tracking (2026-10-18), closed loop: rendered camera -> detector -> TargetTrack ->
waypoint layer -> ArduPilot (flow + downward ToF, no GPS), in the cage20 room (ToF from SIM_STATE + layout). Runs from
a test copy of the companion tree (--root, ECPS_ROOT) on its own SITL instance and ports (-I2, 5780 / 14562 / 14552).

Derived from sitl_flow_wp_test.py (2026-10-17, no-GPS EKF origin and waypoint flight); its plan:

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
ap.add_argument("--root", default=os.environ.get("ECPS_ROOT", "/opt/ecps295"), help="companion tree (test copy)")
ap.add_argument("--out", default="")
ap.add_argument("--work", default="")
ap.add_argument("--marker", default="-1.6,0.8,0.7,-27,7", help="north,east,up,face yaw,id (behind the take-off heading)")
ap.add_argument("--layout", default="/opt/ecps295/twin/ecps295/gazebo/worlds/ecps295_cage20.layout.json")
a = ap.parse_args()
R = Path(a.root)
a.out = a.out or str(R / "logs/sitl_track.json")
a.work = a.work or str(R / "build/sitl_track")
a.config = str(R / "ecps295/minifly/s7_w2map.yaml")
W = Path(a.work)
W.mkdir(parents=True, exist_ok=True)
TRK_S = float(os.environ.get("TRK_S", 120))
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
MK = [float(v) for v in a.marker.split(",")]
binary = Path(a.ardupilot) / "build/sitl/bin/arducopter"
defaults = f"{a.ardupilot}/Tools/autotest/default_params/copter.parm,{W / 'flow.parm'}"
for f in glob.glob(str(W / "eeprom.bin")):
    os.remove(f)
start([str(binary), "--model", "+", "--speedup", "1", "-I2", "--defaults", defaults, "--home", "33.6430,-117.8420,20,0"],
      "sitl.log", cpus="0-3")
(W / "router.conf").write_text(
    "[General]\nTcpServerPort = 0\nReportStats = false\n"
    "[TcpEndpoint sitl]\nAddress = 127.0.0.1\nPort = 5780\nRetryTimeout = 1\n"
    "[UdpEndpoint companion]\nMode = Normal\nAddress = 127.0.0.1\nPort = 14562\n"
    "[UdpEndpoint gcs]\nMode = Normal\nAddress = 127.0.0.1\nPort = 14552\n"
)
time.sleep(3)
start(["mavlink-routerd", "-c", str(W / "router.conf")], "router.log", cpus="4")
g = mavutil.mavlink_connection("udpin:127.0.0.1:14552", source_system=255)
assert g.wait_heartbeat(timeout=60) is not None, "no SITL heartbeat through the router"
g.target_system, g.target_component = 1, 1

obs = {"lpn": [], "modes": [], "text": [], "truth": []}
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
            for mid, hz in ((32, 10), (193, 2), (108, 10)):  # + SIM_STATE (truth)
                g.mav.command_long_send(1, 1, 511, 0, mid, 1e6 / hz, 0, 0, 0, 0, 0)
            rc["req"] = now
        m = g.recv_match(blocking=True, timeout=0.05)
        if m is None:
            continue
        k, src = m.get_type(), m.get_srcSystem()
        if src == 1 and k == "SIM_STATE":
            obs["truth"].append((now, m.lat_int, m.lon_int, m.yaw))
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
    # ---------------------------------------------------------------- T0: companion up, EKF position
    hw_cfg = yaml.safe_load((R / "config/companion.yaml").read_text())
    hw_cfg["marker"] = {**(hw_cfg.get("marker") or {}), "enabled": True, "marker_id": int(MK[4]), "cpus": "5"}
    hw_yaml = W / "companion_hw.yaml"
    hw_yaml.write_text(yaml.safe_dump(hw_cfg))
    pump(5.0)
    t1 = time.monotonic()
    procs.append(subprocess.Popen(
        [str(R / "venv/bin/python"), "-u", str(R / "companion/ecps_companion.py"), "--mode", "guided", "--url",
         "udpin:127.0.0.1:14562", "--camera", "sitl_marker:" + ",".join(str(v) for v in MK), "--tof", "sitl:" + a.layout,
         "--affinity", "6,7", "--tag", "sitl_track", "--config", a.config, "--hw", str(hw_yaml), "--console", "0",
         "--track", "--log-dir", str(R / "logs"), "--set", f"safety.max_flight_s={TRK_S + 60}"],
        stdout=open(W / "companion.log", "w"), stderr=subprocess.STDOUT, cwd=W, start_new_session=True,
        env=dict(os.environ, ECPS_ROOT=str(R))))
    while time.monotonic() - t1 < 40 and not obs["lpn"]:
        pump(1.0)
    rep["phases"]["T0_position"] = {"lpn_msgs": len(obs["lpn"]), "pass": bool(obs["lpn"])}
    pump(10.0)  # brain warm-up, EKF settles on flow + route B
    # ---------------------------------------------------------------- T1: pilot takes off, hands over, tracking
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
    while time.monotonic() - t_arm < 20 and alt_now() < 0.7:
        pump(0.2)
    rc["thr"] = 1500
    pump(1.0)
    alt_handover = alt_now()
    g.set_mode("GUIDED")
    t_g = time.monotonic()
    pump(2.0)  # the FC reports the new mode in its next heartbeat
    rep["guided_after_s"] = round(next((t for t, m, _ in obs["modes"] if t >= t_g and m == "GUIDED"), float("nan")) - t_g, 2)
    t_end = t_g + TRK_S
    t_arr = None
    while time.monotonic() < t_end and mode_now() == "GUIDED":
        pump(1.0)
        if t_arr is None and any("target: arrived" in x[2] for x in obs["text"] if x[1] == 254):
            t_arr = time.monotonic()
        if t_arr is not None and time.monotonic() - t_arr > 10.0:  # hover 10 s in front of it
            break
    t_rel = time.monotonic()
    tr_end = obs["truth"][-1] if obs["truth"] else None
    g.set_mode("LOITER")  # the pilot takes back control
    pump(4.0)
    g.set_mode("LAND")
    pump(15.0)
    lat0, lon0 = 33.6430, -117.8420
    k = 111319.49 * 1e-7

    def ned(lat, lng):
        return (lat - lat0 * 1e7) * k, (lng - lon0 * 1e7) * k * math.cos(math.radians(lat0))

    trk_txt = [x[2] for x in obs["text"] if x[1] == 254 and ("target" in x[2] or "waypoint" in x[2])]
    comp_log = (W / "companion.log").read_text(errors="replace")
    ctl = [ln for ln in comp_log.splitlines() if "IN CONTROL" in ln or "control released" in ln]
    res = {"alt_at_handover_m": round(alt_handover, 2), "companion_control_lines": ctl[:6], "statustext": trk_txt,
           "arrived_after_s": round(t_arr - t_g, 1) if t_arr else None}
    if tr_end is not None:
        n, e = ned(tr_end[1], tr_end[2])
        d = math.hypot(MK[0] - n, MK[1] - e)
        b = math.degrees(math.atan2(MK[1] - e, MK[0] - n))
        face = abs((b - math.degrees(tr_end[3]) + 180) % 360 - 180)
        path = [ned(la, lo) for t, la, lo, _ in obs["truth"] if t_g <= t <= t_rel]
        rmax = max((math.hypot(*p) for p in path), default=0.0)
        res.update({"truth_end_ne": [round(n, 2), round(e, 2)], "truth_dist_to_marker_m": round(d, 2),
                    "truth_facing_err_deg": round(face, 1), "truth_r_max_m": round(rmax, 2)})
        res["pass"] = bool(t_arr) and 0.75 <= d <= 1.35 and face < 15 and rmax < 2.0
    else:
        res["pass"] = False
    rep["phases"]["T1_track"] = res
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
