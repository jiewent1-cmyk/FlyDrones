#!/usr/bin/env python3
"""SITL rig for the ground console on the Orange Pi (2026-10-17).

ArduCopter 4.7.0 SITL with the real drone's navigation (optical flow + downward rangefinder + route-B ExternalNav height,
no GPS: sitl/fhb_delta.parm + companion_fs + G_extnav_height + H_terrain_fast), a second mavlink-router and the companion
in guided mode with the simulated camera, the two ToF sensors emulated from SIM_STATE + the cage layout
(companion/sitl_world.py) and the console link on. /opt/ecps295 is only read (venv, ecps295 tree, SITL build); the
companion code, config, logs and the SITL working files live in --work. Runs until SIGTERM / SIGINT or --seconds.

    python3 sitl_rig.py --work /root/ecps_console_test --seconds 1800

Ports: SITL -I3 (TCP 5790); router: companion UDP 14563, console MAVLink UDP 14650 (server), test pilot UDP 14651
(server); console link UDP 14701. The pilot (take-off, mode switch) is played by tools/console_e2e.py from the Mac.

--deployed tests the installed runtime instead (2026-10-17, after the deployment): the companion and its config from
--opt, started as ecps-companion.service starts it (nice -10 on CPUs 6-7, A76 governor performance) but in guided mode,
and the production ports: router companion UDP 14560, ground station UDP 14550 (server), console link from the config
(14700), so the Mac app connects with its default settings. mavlink-router.service holds 14550, so the rig stops it for
the run and starts it again at the end (and puts the governor back).
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
import traceback
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--work", default="/root/ecps_console_test")
ap.add_argument("--opt", default="/opt/ecps295")
ap.add_argument("--seconds", type=float, default=1800)
ap.add_argument("--layout", default="ecps295_cage20.layout.json")
ap.add_argument("--console-port", type=int, default=14701)
ap.add_argument("--deployed", action="store_true", help="the installed companion + config and the production ports (see above)")
a = ap.parse_args()
W, R = Path(a.work), Path(a.opt)
(W / "sitl").mkdir(parents=True, exist_ok=True)
(W / "logs").mkdir(parents=True, exist_ok=True)
procs: list[subprocess.Popen] = []
restore: list = []  # undone at the end, last first: the production router, the CPU governor
GOV = Path("/sys/devices/system/cpu/cpufreq/policy6/scaling_governor")


def start(cmd, log, cpus=None, cwd=None, env=None):
    pre = ["taskset", "-c", cpus] if cpus else []
    p = subprocess.Popen(pre + cmd, stdout=open(W / "logs" / log, "w"), stderr=subprocess.STDOUT, cwd=cwd or W,
                         start_new_session=True, env=env)
    procs.append(p)
    print(f"started {log.split('.')[0]} pid {p.pid}", flush=True)
    return p


def stop_all(*_):
    signal.signal(signal.SIGTERM, signal.SIG_IGN)  # once
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    for p in procs[::-1]:
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    t0 = time.time()
    while time.time() - t0 < 10 and any(p.poll() is None for p in procs):
        time.sleep(0.2)
    for p in procs:
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGKILL)
    for what, undo in restore[::-1]:
        try:
            undo()
            print(f"restored {what}", flush=True)
        except Exception as e:  # keep going: the next item may still work
            print(f"could not restore {what}: {e}", flush=True)
    print("rig stopped", flush=True)
    sys.exit(0)


signal.signal(signal.SIGTERM, stop_all)
signal.signal(signal.SIGINT, stop_all)

def main() -> None:
    if a.deployed:
        if subprocess.run(["systemctl", "is-active", "--quiet", "mavlink-router"]).returncode == 0:
            subprocess.run(["systemctl", "stop", "mavlink-router"], check=True)
            restore.append(("mavlink-router.service", lambda: subprocess.run(["systemctl", "start", "mavlink-router"], check=True)))
            print("stopped mavlink-router.service for the run (it holds UDP 14550)", flush=True)
        old_gov = GOV.read_text().strip()
        GOV.write_text("performance")  # as ExecStartPre of ecps-companion.service
        restore.append((f"governor {old_gov}", lambda: GOV.write_text(old_gov)))

    V = R / "ecps295/sitl/variants"
    # BATT_CAPACITY 22000 (test rig only): the SITL quad draws ~20 A, so the drone's 2200 mAh lasted ~3.5 min of flight
    # and the governor's battery LAND (min_battery_pct 30) cut the console tests short
    (W / "sitl/flow.parm").write_text("\n".join([(R / "ecps295/sitl/fhb_delta.parm").read_text(), (V / "companion_fs.parm").read_text(),
                                                 (V / "G_extnav_height.parm").read_text(), (V / "H_terrain_fast.parm").read_text(),
                                                 "BATT_CAPACITY 22000"]))
    for f in (W / "sitl").glob("eeprom.bin"):
        f.unlink()
    binary = R / "build/ap/build/sitl/bin/arducopter"
    defaults = f"{R}/build/ap/Tools/autotest/default_params/copter.parm,{W / 'sitl/flow.parm'}"
    start([str(binary), "--model", "+", "--speedup", "1", "-I3", "--defaults", defaults, "--home", "33.6430,-117.8420,20,0"],
          "sitl.log", cpus="0-3", cwd=W / "sitl")
    comp_port, gcs_port = (14560, 14550) if a.deployed else (14563, 14650)
    (W / "router.conf").write_text(
        "[General]\nTcpServerPort = 0\nReportStats = false\n"
        "[TcpEndpoint sitl]\nAddress = 127.0.0.1\nPort = 5790\nRetryTimeout = 1\n"
        f"[UdpEndpoint companion]\nMode = Normal\nAddress = 127.0.0.1\nPort = {comp_port}\n"
        f"[UdpEndpoint console]\nMode = Server\nAddress = 0.0.0.0\nPort = {gcs_port}\n"
        "[UdpEndpoint pilot]\nMode = Server\nAddress = 0.0.0.0\nPort = 14651\n"
    )
    time.sleep(3)
    start(["mavlink-routerd", "-c", str(W / "router.conf")], "router.log", cpus="4")
    sim_io = ["--camera", "sim", "--tof", f"sitl:{W / a.layout}", "--log-dir", str(W / "logs"), "--set", "safety.max_flight_s=900"]
    if a.deployed:  # as ecps-companion.service (CPUAffinity 6 7, Nice -10, default --hw / --url / console port) but guided
        cmd, env = (["nice", "-n", "-10", "taskset", "-c", "6,7", str(R / "venv/bin/python"), "-u", str(R / "companion/ecps_companion.py"),
                     "--mode", "guided", "--tag", "console_lan_e2e", *sim_io], None)
    else:
        cmd, env = ([str(R / "venv/bin/python"), "-u", str(W / "companion/ecps_companion.py"), "--mode", "guided",
                     "--url", "udpin:127.0.0.1:14563", "--console", str(a.console_port), "--affinity", "6,7",
                     "--hw", str(W / "config/companion.yaml"), "--tag", "console_sitl", *sim_io], dict(os.environ, ECPS_ROOT=str(R)))
    comp = start(cmd, "companion.log", env=env)
    t0 = time.time()
    while time.time() - t0 < a.seconds:
        time.sleep(1.0)
        dead = [p for p in procs if p.poll() is not None]
        if dead:
            print(f"process {dead[0].args[:3]} exited ({dead[0].returncode})", flush=True)
            if comp in dead:  # the companion exits with 1 when no FC heartbeat came in time: start it again
                procs.remove(comp)
                comp = start(cmd, "companion.log", env=env)
            else:
                break


try:
    main()
except Exception:  # still stop everything and put the router / governor back
    traceback.print_exc()
stop_all()
