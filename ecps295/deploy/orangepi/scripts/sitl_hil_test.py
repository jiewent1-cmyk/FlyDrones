#!/usr/bin/env python3
"""On-Pi SITL link test (ArduCopter 4.7.0 SITL on the A55 cores, companion on the A76 cores, MAVLink through a second
mavlink-router instance, i.e. the same software path as on the drone except the UART).

The test plays the pilot / GCS (sysid 255, router UDP endpoint 14551). Phases:
  T1  disarmed: companion must send heartbeat (1 Hz) + route-B height (VISION_POSITION_ESTIMATE), never take control
  T2  GCS arms in GUIDED ON THE GROUND: the companion must NOT take control (hand-over latch, >= 0.3 m)
  T3  GCS take-off to 0.8 m: the companion takes control once airborne; setpoints at 20 Hz (companion tick log) and
      the FC's own POSITION_TARGET_LOCAL_NED echo
  T4  GCS switches to LOITER: control released (latency from the FC heartbeat to the companion's first released tick)
  T5  back to GUIDED (airborne): control resumes
  T6  companion killed (SIGKILL, a hung Orange Pi): FC GCS failsafe must LAND (FS_GCS_ENABLE 5, FS_GCS_TIMEOUT 2 s)
Setpoint timing comes from the companion's ticks.csv (column mono = CLOCK_MONOTONIC, shared with this process): the
router forwards targeted messages only toward the FC, so the GCS endpoint cannot see them.

    python sitl_hil_test.py --ardupilot /opt/ecps295/build/ap --out /opt/ecps295/logs/sitl_hil.json
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import yaml
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--ardupilot", default="/opt/ecps295/build/ap")
ap.add_argument("--out", default="/opt/ecps295/logs/sitl_hil.json")
ap.add_argument("--work", default="/opt/ecps295/build/sitl_run")
a = ap.parse_args()
R = Path("/opt/ecps295")
W = Path(a.work)
W.mkdir(parents=True, exist_ok=True)
procs: list[subprocess.Popen] = []
rep: dict = {"phases": {}}


def start(cmd, log, cpus=None):
    pre = ["taskset", "-c", cpus] if cpus else []
    p = subprocess.Popen(pre + cmd, stdout=open(W / log, "w"), stderr=subprocess.STDOUT, cwd=W, start_new_session=True)
    procs.append(p)
    return p


# ------------------------------------------------------------------ SITL + router
(W / "hil.parm").write_text(
    "\n".join(
        [
            (R / "ecps295/sitl/variants/companion_fs.parm").read_text(),
            (R / "ecps295/sitl/variants/G_extnav_height.parm").read_text(),
            "RNGFND1_TYPE 100",  # SITL downward rangefinder: the route-B input
            "RNGFND1_MAX 3",
            "RNGFND1_MIN 0.05",
            "RNGFND1_ORIENT 25",
            "GUID_TIMEOUT 1",
        ]
    )
)
binary = Path(a.ardupilot) / "build/sitl/bin/arducopter"
defaults = f"{a.ardupilot}/Tools/autotest/default_params/copter.parm,{W / 'hil.parm'}"
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
for mid, hz in ((32, 10), (30, 10), (193, 2), (85, 10)):  # LOCAL_POSITION_NED, ATTITUDE, EKF_STATUS, POSITION_TARGET_LOCAL_NED
    g.mav.command_long_send(1, 1, 511, 0, mid, 1e6 / hz, 0, 0, 0, 0, 0)

obs = {"viso": [], "hb254": [], "modes": [], "ptgt": [], "alt": [], "rng": [], "viso_age": []}
rc = {"thr": None, "last": 0.0, "hb": 0.0, "silent": False}  # pilot's throttle stick (RC override from the GCS, sysid 255)


def pump(sec: float) -> None:
    t_end = time.monotonic() + sec
    while time.monotonic() < t_end:
        if not rc["silent"] and time.monotonic() - rc["last"] > 0.1:
            ch = (0, 0, 0, 0) if rc["thr"] is None else (1500, 1500, rc["thr"], 1500)
            g.mav.rc_channels_override_send(1, 1, *ch, 0, 0, 0, 0)
            rc["last"] = time.monotonic()
        if not rc["silent"] and time.monotonic() - rc["hb"] > 1.0:  # like Mission Planner: 1 Hz GCS heartbeat
            g.mav.heartbeat_send(6, 8, 0, 0, 0)  # MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID
            rc["hb"] = time.monotonic()
        m = g.recv_match(blocking=True, timeout=0.05)
        if m is None:
            continue
        now = time.monotonic()
        k, src = m.get_type(), m.get_srcSystem()
        if src == 254:
            if k == "VISION_POSITION_ESTIMATE":
                obs["viso"].append((now, -m.z))
                if obs["rng"]:  # age of the FC rangefinder sample it was built from (both seen through the same router)
                    obs["viso_age"].append(now - obs["rng"][-1])
            elif k == "HEARTBEAT":
                obs["hb254"].append(now)
        elif src == 1:
            if k == "HEARTBEAT":
                mode = mavutil.mode_string_v10(m)
                armed = bool(m.base_mode & 128)
                if not obs["modes"] or obs["modes"][-1][1:] != (mode, armed):
                    obs["modes"].append((now, mode, armed))
            elif k == "POSITION_TARGET_LOCAL_NED":
                obs["ptgt"].append((now, m.vx, m.vy, m.vz, m.type_mask))
            elif k == "RANGEFINDER":
                obs["rng"].append(now)
            elif k == "LOCAL_POSITION_NED":
                obs["alt"].append((now, -m.z))
            elif k == "STATUSTEXT":
                rep.setdefault("fc_text", []).append(m.text)


def mode_now():
    return obs["modes"][-1][1] if obs["modes"] else "?"


def alt_now():
    return obs["alt"][-1][1] if obs["alt"] else None


def n_in(lst, t0, t1):
    return sum(1 for x in lst if t0 <= (x[0] if isinstance(x, tuple) else x) < t1)


t0 = time.monotonic()
while time.monotonic() - t0 < 120:  # EKF ready on GPS + baro
    pump(1.0)
    e = g.messages.get("EKF_STATUS_REPORT")
    if e is not None and (e.flags & 0x1F) == 0x1F and time.monotonic() - t0 > 20:
        break
rep["ekf_ready_s"] = round(time.monotonic() - t0, 1)
# RNG_HZ=10 reproduces the twin's RANGEFINDER rate (baseline for the route-B age); default = companion.yaml
hw_cfg = yaml.safe_load((R / "config/companion.yaml").read_text())
if os.environ.get("RNG_HZ"):
    hw_cfg["mavlink"]["rangefinder_hz"] = float(os.environ["RNG_HZ"])
hw_yaml = W / "companion_hw.yaml"
hw_yaml.write_text(yaml.safe_dump(hw_cfg))
rep["rangefinder_hz_requested"] = hw_cfg["mavlink"].get("rangefinder_hz")
comp = start([str(R / "venv/bin/python"), "-u", str(R / "companion/ecps_companion.py"), "--mode", "guided", "--url",
              "udpin:127.0.0.1:14561", "--camera", "sim", "--tof", "sim", "--affinity", "6,7", "--tag", "sitl_hil",
              "--no-fence-turn", "--hw", str(hw_yaml)], "companion.log")
pump(15.0)  # connect + brain warm-up
mark = {}

mark["T1"] = time.monotonic()
pump(5.0)
mark["T2"] = time.monotonic()
g.set_mode("GUIDED")
pump(1.0)
g.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)  # arm on the ground in GUIDED
pump(3.0)
mark["T3"] = time.monotonic()
g.mav.command_long_send(1, 1, 22, 0, 0, 0, 0, 0, 0, 0, 0.8)  # take off (the pilot's part)
pump(15.0)
mark["T4"] = time.monotonic()
rc["thr"] = 1500  # the pilot holds the stick at mid before taking control back
pump(0.5)
g.set_mode("LOITER")
pump(5.0)
mark["T5"] = time.monotonic()
g.set_mode("GUIDED")
pump(6.0)
rc["thr"] = None
# T6: the GCS falls silent (as with no ground station in the cage); in the first SITL run it kept sending RC-override
# packets as sysid 255, which is inside MAV_GCS_SYSID..MAV_GCS_SYSID_HI (254-255): the FC then never declared the
# GCS failsafe after the companion died (a connected Mission Planner masks the companion-loss failsafe)
# FLIGHT_GCS=1: the flight setting (config/fc_flight.parm, MAV_GCS_SYSID_HI 254) with Mission Planner still connected;
# the HI change is made only here because T4/T5 need the sysid-255 RC override (SITL has no RC transmitter)
flight_gcs = os.environ.get("FLIGHT_GCS") == "1"
if flight_gcs:
    g.mav.param_set_send(1, 1, b"MAV_GCS_SYSID_HI", 254, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    pump(1.0)
    pv = g.messages.get("PARAM_VALUE")
    rep["flight_gcs_param"] = {"id": pv.param_id, "value": pv.param_value} if pv else None
rc["silent"] = os.environ.get("GCS_ALIVE") != "1" and not flight_gcs
mark["T6"] = time.monotonic()
os.killpg(comp.pid, signal.SIGKILL)
pump(10.0)
mark["end"] = time.monotonic()
for p in procs:
    try:
        os.killpg(p.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass

# ------------------------------------------------------------------ companion tick log
d = sorted(glob.glob(str(R / "logs/*_guided_sitl_hil")))[-1]
T = [r for r in csv.DictReader(open(Path(d) / "ticks.csv"))]
tick = [(float(r["mono"]), int(r["in_control"]), r["alt"], r["ext_h"]) for r in T]


def ctl_ticks(t0, t1):
    return [t for t, c, *_ in tick if t0 <= t < t1 and c]


def mode_time(after, mode):
    return next((t for t, m, _ in obs["modes"] if t >= after and m == mode), None)


rep["companion_log"] = d
rep["phases"]["T1_disarmed"] = {
    "control_ticks": len(ctl_ticks(mark["T1"], mark["T2"])),
    "hb254_hz": round(n_in(obs["hb254"], mark["T1"], mark["T2"]) / 5, 2),
    "viso_hz": round(n_in(obs["viso"], mark["T1"], mark["T2"]) / 5, 1),
}
rep["phases"]["T1_disarmed"]["pass"] = (rep["phases"]["T1_disarmed"]["control_ticks"] == 0
                                        and rep["phases"]["T1_disarmed"]["hb254_hz"] >= 0.8
                                        and rep["phases"]["T1_disarmed"]["viso_hz"] >= 10)
armed_t = next((t for t, m, ar in obs["modes"] if t >= mark["T2"] and ar), None)
rep["phases"]["T2_armed_on_ground"] = {"armed": armed_t is not None, "control_ticks": len(ctl_ticks(mark["T2"], mark["T3"])),
                                       "pass": armed_t is not None and len(ctl_ticks(mark["T2"], mark["T3"])) == 0}
c3 = ctl_ticks(mark["T3"], mark["T4"])
alt_at = None
if c3:
    alt_at = min(((abs(t - c3[0]), h) for t, h in obs["alt"]), default=(0, None))[1]
gaps = [b - a_ for a_, b in zip(c3, c3[1:])]
pt = [x for x in obs["ptgt"] if c3 and c3[0] + 0.5 <= x[0] < mark["T4"]]
rep["phases"]["T3_takeoff_handover"] = {
    "takeover_after_takeoff_s": round(c3[0] - mark["T3"], 2) if c3 else None,
    "alt_at_takeover_m": None if alt_at is None else round(alt_at, 2),
    "setpoint_hz": round(len(c3) / (mark["T4"] - c3[0]), 1) if c3 else 0.0,
    "setpoint_gap_max_ms": round(1000 * max(gaps), 1) if gaps else None,
    "fc_target_echo_hz": round(len(pt) / max(1e-3, mark["T4"] - c3[0] - 0.5), 1) if c3 else 0.0,
    "fc_target_v_nonzero": sum(1 for x in pt if abs(x[1]) + abs(x[2]) + abs(x[3]) > 1e-3),
    "alt_end_m": alt_now(),
}
rep["phases"]["T3_takeoff_handover"]["pass"] = bool(c3) and (alt_at or 0) >= 0.3 and rep["phases"]["T3_takeoff_handover"]["setpoint_hz"] > 18
t_l = mode_time(mark["T4"], "LOITER")
after = [t for t in ctl_ticks(mark["T4"], mark["T5"]) if t_l and t > t_l]
rel = next((t for t, c, *_ in tick if t_l and t > t_l and not c), None)
rep["phases"]["T4_loiter_release"] = {"fc_loiter_seen": t_l is not None,
                                      "release_latency_ms": round(1000 * (rel - t_l), 1) if rel and t_l else None,
                                      "control_ticks_after_switch": len(after),
                                      "pass": t_l is not None and rel is not None and rel - t_l < 0.2}
t_g = mode_time(mark["T5"], "GUIDED")
c5 = ctl_ticks(mark["T5"], mark["T6"])
rep["phases"]["T5_guided_resume"] = {"resume_latency_ms": round(1000 * (c5[0] - t_g), 1) if c5 and t_g else None,
                                     "setpoint_hz": round(len(c5) / (mark["T6"] - c5[0]), 1) if c5 else 0.0,
                                     "pass": bool(c5) and t_g is not None and c5[0] - t_g < 0.3}
land = mode_time(mark["T6"], "LAND")
last_hb = max([t for t in obs["hb254"] if t < mark["T6"] + 0.5], default=None)
rep["phases"]["T6_companion_killed"] = {"modes_after": [m for t, m, _ in obs["modes"] if t >= mark["T6"]],
                                        "land_after_kill_s": round(land - mark["T6"], 2) if land else None,
                                        "land_after_last_heartbeat_s": round(land - last_hb, 2) if land and last_hb else None,
                                        "pass": land is not None and 1.5 < land - mark["T6"] < 4.0}
vh = [h for _, h in obs["viso"]]
rep["route_b_height_m"] = {"n": len(vh), "min": round(min(vh), 3) if vh else None, "max": round(max(vh), 3) if vh else None}
rep["fc_text"] = rep.get("fc_text", [])[-40:]
rep["modes"] = [(round(t - t0, 2), m, ar) for t, m, ar in obs["modes"]]
rep["gcs_alive_during_T6"] = not rc["silent"]
rep["flight_gcs_sysid_hi_254"] = flight_gcs
rep["pass"] = all(p["pass"] for p in rep["phases"].values())
print(json.dumps(rep["phases"], indent=1))
print("route-B height:", rep["route_b_height_m"])
ra = sorted(obs["viso_age"])
if ra:
    t_rng = [t for t in obs["rng"] if mark["T1"] <= t < mark["T6"]]
    rep["route_b_age"] = {"rangefinder_hz": round((len(t_rng) - 1) / (t_rng[-1] - t_rng[0]), 1) if len(t_rng) > 1 else None,
                          "age_since_rangefinder_ms": {"mean": round(1e3 * sum(ra) / len(ra), 1),
                                                       "p50": round(1e3 * ra[len(ra) // 2], 1),
                                                       "p95": round(1e3 * ra[int(0.95 * (len(ra) - 1))], 1),
                                                       "max": round(1e3 * ra[-1], 1)}}
    print("route-B age:", rep["route_b_age"])
json.dump(rep, open(a.out, "w"), indent=1)
print("PASS" if rep["pass"] else "FAIL", "->", a.out)
