#!/usr/bin/env python3
"""On-Pi SITL test of return-home and land (2026-10-22, EcpsPilot rtl layer), derived from sitl_flow_wp_test.py.

ArduCopter 4.7.0 SITL with the real drone's navigation (flow + downward ToF, GPS off) on the A55 cores, the companion on
the A76 cores (simulated camera and ToF), MAVLink through a second mavlink-router. The test plays the pilot:
  R0  companion started: it sets the EKF origin, LOCAL_POSITION_NED appears
  R1  pilot arms in ALT_HOLD, climbs to ~0.6 m, switches to GUIDED: the companion flies the waypoints of
      minifly/s7_w2map_wp.yaml (square +-1 m) and, rtl_after seconds after the hand-over (companion --rtl-after), returns
      home by itself, hovers, requests LAND; the FC lands and disarms. Nobody touches the mode switch.
Pass: the return started away from home (> 0.5 m), "rtl: landing" (not "landing here"), FC LAND within the rtl timeout,
disarmed within 40 s of LAND, landed within 0.4 m of home (EKF) and 0.5 m (SITL truth, SIM_STATE).
--scenario console_lost: no timer; a console (ecl_client.py, a fresh test key) connects before the hand-over, sends GOTO
(1.0, 0.6) after it and then goes silent: the companion must HOLD (link lost) and start RTL lost_rtl_s (5 s) later.
The environment passes through to the companion (ECPS_HOLD=pos ... : the hover hold while it settles and in LAND).

    python sitl_rtl_test.py --root /root/ecps_rtl_test --out /root/ecps_rtl_test/logs/sitl_rtl.json
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import subprocess
import time
from pathlib import Path

import yaml
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--root", default=os.environ.get("ECPS_ROOT", "/opt/ecps295"), help="companion tree (test copy)")
ap.add_argument("--ardupilot", default="/opt/ecps295/build/ap")
ap.add_argument("--out", default="")
ap.add_argument("--work", default="")
ap.add_argument("--config", default="", help="default: <root>/ecps295/minifly/s7_w2map_wp.yaml")
ap.add_argument("--rtl-after", type=float, default=25.0)
ap.add_argument("--scenario", choices=["timer", "console_lost"], default="timer")
ap.add_argument("--ecl-client", default="", help="console_lost: path of deploy/console/tools/ecl_client.py")
ap.add_argument("--silent-after", type=float, default=12.0, help="console_lost: the console goes silent this long after GOTO")
a = ap.parse_args()
R = Path(a.root)
a.out = a.out or str(R / "logs/sitl_rtl.json")
a.config = a.config or str(R / "ecps295/minifly/s7_w2map_wp.yaml")
W = Path(a.work or R / "build/sitl_rtl")
W.mkdir(parents=True, exist_ok=True)
Path(a.out).parent.mkdir(parents=True, exist_ok=True)
procs: list[subprocess.Popen] = []
rep: dict = {"phases": {}, "env": {k: v for k, v in os.environ.items() if k.startswith("ECPS_")}}


def start(cmd, log, cpus=None, env=None):
    pre = ["taskset", "-c", cpus] if cpus else []
    p = subprocess.Popen(pre + cmd, stdout=open(W / log, "w"), stderr=subprocess.STDOUT, cwd=W, start_new_session=True,
                         env=env)
    procs.append(p)
    return p


V = R / "ecps295/sitl/variants"
(W / "flow.parm").write_text("\n".join([(R / "ecps295/sitl/fhb_delta.parm").read_text(), (V / "companion_fs.parm").read_text(),
                                        (V / "G_extnav_height.parm").read_text(), (V / "H_terrain_fast.parm").read_text()]))
binary = Path(a.ardupilot) / "build/sitl/bin/arducopter"
defaults = f"{a.ardupilot}/Tools/autotest/default_params/copter.parm,{W / 'flow.parm'}"
for f in glob.glob(str(W / "eeprom.bin")):
    os.remove(f)
# own instance and ports (-I3: 5790), next to sitl_flow_wp (-I1) and sitl_track (-I2)
start([str(binary), "--model", "+", "--speedup", "1", "-I3", "--defaults", defaults, "--home", "33.6430,-117.8420,20,0"],
      "sitl.log", cpus="0-3")
(W / "router.conf").write_text(
    "[General]\nTcpServerPort = 0\nReportStats = false\n"
    "[TcpEndpoint sitl]\nAddress = 127.0.0.1\nPort = 5790\nRetryTimeout = 1\n"
    "[UdpEndpoint companion]\nMode = Normal\nAddress = 127.0.0.1\nPort = 14564\n"
    "[UdpEndpoint gcs]\nMode = Normal\nAddress = 127.0.0.1\nPort = 14554\n"
)
time.sleep(3)
start(["mavlink-routerd", "-c", str(W / "router.conf")], "router.log", cpus="4")
g = mavutil.mavlink_connection("udpin:127.0.0.1:14554", source_system=255)
assert g.wait_heartbeat(timeout=60) is not None, "no SITL heartbeat through the router"
g.target_system, g.target_component = 1, 1

obs = {"lpn": [], "modes": [], "text": [], "truth": []}
ecl_state = {"client": None, "live": False, "hello": 0.0}
rc = {"thr": None, "last": 0.0, "hb": 0.0, "req": 0.0}
LAT0, LON0 = 33.6430, -117.8420
K = 111319.49 * 1e-7


def ned(lat, lng):
    return (lat - LAT0 * 1e7) * K, (lng - LON0 * 1e7) * K * math.cos(math.radians(LAT0))


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
        if now - rc["req"] > 5.0:  # LOCAL_POSITION_NED, EKF_STATUS_REPORT, SIM_STATE (truth)
            for mid, hz in ((32, 10), (193, 2), (108, 10)):
                g.mav.command_long_send(1, 1, 511, 0, mid, 1e6 / hz, 0, 0, 0, 0, 0)
            rc["req"] = now
        if ecl_state["client"] is not None and ecl_state["live"] and now - ecl_state["hello"] > 0.2:
            ecl_state["client"].hello_tick()
            ecl_state["hello"] = now
            try:
                while ecl_state["client"].recv() is not None:
                    pass
            except (BlockingIOError, OSError):
                pass
        m = g.recv_match(blocking=True, timeout=0.05)
        if m is None:
            continue
        k, src = m.get_type(), m.get_srcSystem()
        if src == 1 and k == "LOCAL_POSITION_NED":
            obs["lpn"].append((now, m.x, m.y, -m.z))
        elif src == 1 and k == "SIM_STATE":
            obs["truth"].append((now, *ned(m.lat_int, m.lon_int)))
        elif src == 1 and k == "HEARTBEAT":
            mode, armed = mavutil.mode_string_v10(m), bool(m.base_mode & 128)
            if not obs["modes"] or obs["modes"][-1][1:] != (mode, armed):
                obs["modes"].append((now, mode, armed))
        elif k == "STATUSTEXT":
            obs["text"].append((round(now, 2), src, m.text))


def mode_now():
    return obs["modes"][-1][1] if obs["modes"] else "?"


def armed_now():
    return obs["modes"][-1][2] if obs["modes"] else False


def alt_now():
    return obs["lpn"][-1][3] if obs["lpn"] else 0.0


def ecps(sub: str):
    return [(t, x) for t, s, x in obs["text"] if s == 254 and sub in x]


try:
    # ---------------------------------------------------------------- R0: companion, EKF origin
    pump(15.0)
    hw_cfg = yaml.safe_load((R / "config/companion.yaml").read_text())
    ecl = None
    if a.scenario == "console_lost":
        import importlib.util
        import secrets

        key = W / "console.key"
        key.write_bytes(secrets.token_bytes(32))
        key.chmod(0o600)
        hw_cfg.setdefault("console", {}).update(port=14705, key_file=str(key), handover="hold", lost_rtl_s=5.0)
        spec = importlib.util.spec_from_file_location("ecl_client", a.ecl_client)
        ecl_mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ecl_mod)
    else:
        hw_cfg.setdefault("console", {})["port"] = 0  # no console: the companion returns by itself (--rtl-after)
    hw_yaml = W / "companion_hw.yaml"
    hw_yaml.write_text(yaml.safe_dump(hw_cfg))
    t1 = time.monotonic()
    start([str(R / "venv/bin/python"), "-u", str(R / "companion/ecps_companion.py"), "--mode", "guided", "--url",
           "udpin:127.0.0.1:14564", "--camera", "sim", "--tof", "sim", "--affinity", "6,7", "--tag", "sitl_rtl",
           "--config", a.config, "--hw", str(hw_yaml), "--sitl", "--set", "safety.max_flight_s=240",
           *(["--rtl-after", str(a.rtl_after)] if a.scenario == "timer" else [])],
          "companion.log", env=dict(os.environ, ECPS_ROOT=str(R)))
    while time.monotonic() - t1 < 40 and not obs["lpn"]:
        pump(1.0)
    rep["phases"]["R0_position"] = {"lpn_msgs": len(obs["lpn"]), "pass": bool(obs["lpn"])}
    if a.scenario == "console_lost":
        c = ecl_mod.Link("127.0.0.1", 14705, name="sitl_rtl_test", cam="off", key=(W / "console.key").read_bytes())
        c.sock.setblocking(False)
        ecl_state.update(client=c, live=True)
    pump(10.0)
    # ---------------------------------------------------------------- R1: take-off, hand-over, mission, return, land
    g.set_mode("ALT_HOLD")
    rc["thr"] = 1000
    pump(1.0)
    for _ in range(10):
        if armed_now():
            break
        g.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
        pump(1.5)
    rc["thr"] = 1650
    t_arm = time.monotonic()
    while time.monotonic() - t_arm < 20 and alt_now() < 0.6:
        pump(0.2)
    rc["thr"] = 1500  # stick centred: the FC's LAND does the descent, the pilot does not touch it
    pump(1.0)
    alt_handover = alt_now()
    g.set_mode("GUIDED")
    t_g = time.monotonic()
    t_land = t_disarm = None
    t_silent = None
    if a.scenario == "console_lost":
        pump(3.0)
        ecl_state["client"].cmd("goto", x=1.0, y=0.6)
        pump(a.silent_after)
        ecl_state["live"] = False  # the console goes silent (no HELLO): link lost after link_timeout_s
        t_silent = time.monotonic()
    while time.monotonic() - t_g < a.rtl_after + 120:
        pump(0.5)
        if t_land is None and mode_now() == "LAND":
            t_land = time.monotonic()
        if t_land is not None and not armed_now():
            t_disarm = time.monotonic()
            break
    pump(3.0)
    rtl_txt = [(round(t - t_g, 1), x) for t, x in ecps("rtl")]
    if t_silent is not None:
        t_r = next((t for t, x in ecps("rtl: returning home")), None)
        rep["console_silent_to_rtl_s"] = round(t_r - t_silent, 1) if t_r else None
        rep["console_lost_reason_ok"] = any("console lost" in x for _, x in rtl_txt)
    t_rtl = next((t for t, x in ecps("rtl: returning home")), None)
    comp_log = (W / "companion.log").read_text(errors="replace")
    ctl = [ln for ln in comp_log.splitlines() if "IN CONTROL" in ln or "control released" in ln]

    def at(series, t):
        return min(series, key=lambda r: abs(r[0] - t)) if series and t is not None else None

    p_rtl = at(obs["lpn"], t_rtl)
    end_ekf = obs["lpn"][-1] if obs["lpn"] else None
    end_tru = obs["truth"][-1] if obs["truth"] else None
    tru_rtl = at(obs["truth"], t_rtl)
    res = {
        "alt_at_handover_m": round(alt_handover, 2),
        "companion_control_lines": ctl[:6],
        "statustext": rtl_txt,
        "waypoints": [x for _, x in ecps("waypoint")][:8],
        "rtl_after_handover_s": round(t_rtl - t_g, 1) if t_rtl else None,
        "dist_at_rtl_ekf_m": round(math.hypot(p_rtl[1], p_rtl[2]), 2) if p_rtl else None,
        "dist_at_rtl_truth_m": round(math.hypot(tru_rtl[1], tru_rtl[2]), 2) if tru_rtl else None,
        "land_after_rtl_s": round(t_land - t_rtl, 1) if t_land and t_rtl else None,
        "disarm_after_land_s": round(t_disarm - t_land, 1) if t_disarm and t_land else None,
        "end_ekf_m": round(math.hypot(end_ekf[1], end_ekf[2]), 2) if end_ekf else None,
        "end_truth_m": round(math.hypot(end_tru[1], end_tru[2]), 2) if end_tru else None,
    }
    landed_home = any("rtl: landing" in x for _, x in rtl_txt) and not any("landing here" in x for _, x in rtl_txt)
    res["pass"] = bool(
        t_rtl and (res["dist_at_rtl_ekf_m"] or 0) > 0.5 and landed_home and t_land and t_disarm
        and (res["disarm_after_land_s"] or 99) < 40 and (res["end_ekf_m"] if res["end_ekf_m"] is not None else 9) < 0.4
        and (res["end_truth_m"] if res["end_truth_m"] is not None else 9) < 0.5
    )
    rep["phases"]["R1_return_land"] = res
    rep["modes"] = [(round(t - t1, 1), m, ar) for t, m, ar in obs["modes"]]
    rep["fc_text"] = [x[2] for x in obs["text"] if x[1] == 1][-30:]
finally:
    for p in procs[::-1]:
        try:
            os.killpg(p.pid, 15)
        except ProcessLookupError:
            pass
rep["pass"] = all(p["pass"] for p in rep["phases"].values())
if a.scenario == "console_lost":  # link_timeout 1.5 s + lost_rtl_s 5 s
    rep["pass"] = rep["pass"] and bool(rep.get("console_lost_reason_ok")) and 5.0 <= (rep.get("console_silent_to_rtl_s") or 0) <= 9.0
print(json.dumps(rep, indent=1))
json.dump(rep, open(a.out, "w"), indent=1)
print("PASS" if rep["pass"] else "FAIL", "->", a.out)
