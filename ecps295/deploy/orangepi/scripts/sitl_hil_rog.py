#!/usr/bin/env python3
"""SITL hand-over test of the companion on the simulation PC (ROG, Ubuntu), 2026-11-25: the companion code of this
tree (ecps_companion.py --mode guided --sitl, simulated camera + ToF) against ArduCopter 4.7.0 SITL, phases T1-T10.

Differences from sitl_hil_test.py (the on-Pi test, unchanged): no mavlink-router and no CPU pinning. SITL serves the test
(the pilot / GCS, sysid 255) on SERIAL0 (TCP 5760 + 10 x instance) and the companion on SERIAL2 (udpclient to
127.0.0.1:14560 + instance, the Pi's UART is SERIAL2 too); ArduPilot's own MAVLink routing forwards the companion's
broadcast messages (heartbeat, VISION_POSITION_ESTIMATE, STATUSTEXT) to the GCS channel instead of the router. Every
stream is requested per channel, so the GCS asks for RANGEFINDER itself (route-B age = RANGEFINDER on the GCS channel to
the forwarded VISO, not the same sample the companion used: indicative only).

Runs per repeat (each one fresh SITL + companion, instance -I7..-I9 only):
  main        sitl_hil_test.py's parameter set (GPS on, copter.parm + companion_fs + G_extnav_height + SITL rangefinder)
              and sequence: T1-T6 exactly as there, the hand-over height from the companion config (mavlink.handover_alt_m),
              T7 = commanded body motion (ticks.csv) vs the FC's echo (POSITION_TARGET_LOCAL_NED velocity projected on
              the body axes; yaw: ATTITUDE_TARGET body yaw rate, because 4.7.0 always echoes yaw_rate 0 in
              POSITION_TARGET_LOCAL_NED, GCS_MAVLink_Copter.cpp send_position_target_local_ned). Console off (PATROL)
              so that the brain moves; --no-fence-turn as in sitl_hil_test.py.
  home_true   flow-only set (copter.parm + mtf01_sitl_rng.parm + companion_fs + G_extnav_height), SITL --home 5 m north
              of the config ekf_origin, mavlink.home_frame true, fence turn on, console on (temporary key, hand-over HOLD,
              no client). If the FC does not boot at x ~ +5 m (no GPS: the flow EKF starts at 0 wherever it is), the
              pilot first flies it ~5 m north in LOITER (FC fence off for that), lands and disarms: the same EKF offset
              as a drone carried to its take-off spot. Then ALT_HOLD take-off to 0.8 m, GUIDED, hold_s of HOLD (T8),
              then T9: the front ToF freezes (SIGUSR1, test switch ECPS_TEST_SIGUSR1_TOF=1): hover, then LAND.
  home_false  as home_true with mavlink.home_frame false, no T9 (T8 "what happens").
  t10_tof_frozen / t10_tof_absent / t10_cam_absent / t10_cam_off
              T10: flow-only set, ALT_HOLD take-off to 0.8 m, GUIDED for 10 s with the front ToF frozen before take-off
              (SIGUSR1) / the rear ToF removed from the config / a camera that never delivers a frame
              (--camera shm:<missing file>, as a UVC camera that is unplugged) / --camera off (no camera object at all).

    python sitl_hil_rog.py --out-dir ~/sim/hil_rog --repeats 3
"""

from __future__ import annotations

import argparse
import ast
import csv
import glob
import gzip
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml
from pymavlink import mavutil

TREE = Path(__file__).resolve().parents[4]  # worktree root (ecps295/deploy/orangepi/scripts/<this>)
OPI = TREE / "ecps295/deploy/orangepi"
HOME = Path.home()
ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--sitl-bin", default=str(HOME / "sim/fw_ecps295_flowgate/arducopter_sitl_gate"))
ap.add_argument("--copter-parm", default=str(HOME / "sim/ardupilot/Tools/autotest/default_params/copter.parm"))
ap.add_argument("--mtf01-parm", default=str(HOME / "sim/mtf01/mtf01_sitl_rng.parm"))
ap.add_argument("--python", default=sys.executable, help="interpreter for the companion")
ap.add_argument("--brain", default=str(HOME / "sim/FlyDrones/ecps295/minifly/minifly_v4.npz"),
                help="connectome (not in git; read only, passed as --set brain.source)")
ap.add_argument("--out-dir", default=str(HOME / "sim/hil_rog"))
ap.add_argument("--instance", type=int, default=7, choices=[7, 8, 9])
ap.add_argument("--repeats", type=int, default=3)
ap.add_argument("--first-repeat", type=int, default=1)
ap.add_argument("--only", default="", help="comma list of runs (default: all)")
ap.add_argument("--merge", action="store_true", help="keep the other runs of an existing report.json (re-run some runs)")
ap.add_argument("--hold-s", type=float, default=20.0, help="T8: hold under companion control")
a = ap.parse_args()
OUT = Path(a.out_dir)
OUT.mkdir(parents=True, exist_ok=True)

HW0 = yaml.safe_load((OPI / "config/companion.yaml").read_text())  # committed config: read only, copies are edited
MV = HW0["mavlink"]
HANDOVER_M = float(MV["handover_alt_m"])
ORIGIN = [float(v) for v in MV["ekf_origin"]]
GUARD = HW0.get("guard", {})
TOF_STALE_S, TOF_LAND_S = float(GUARD.get("tof_stale_s", 0.3)), float(GUARD.get("tof_land_s", 2.0))
M_PER_DEG = 111319.49


def _envs_dict(name: str) -> dict:
    tree = ast.parse((TREE / "ecps295/envs.py").read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(getattr(t, "id", "") == name for t in node.targets):
            return ast.literal_eval(node.value)
    return {}


_model = yaml.safe_load((TREE / "ecps295/minifly" / HW0.get("model", "s7_w2map.yaml")).read_text()) or {}
DYN = {k: _envs_dict("ECPS295_DYN")[k] for k in ("v_max", "vz_max", "yaw_rate_max_dps")}
DYN.update(_model.get("ecps_drone", {}) or {})
FENCE_R = float({**_envs_dict("ECPS295_SAFETY_20FT"), **(_model.get("safety") or {})}.get("geofence_radius_m", 2.0))
V_MAX, YR_MAX = float(DYN["v_max"]), math.radians(float(DYN["yaw_rate_max_dps"]))

SERIAL0 = 5760 + 10 * a.instance
COMP_PORT = 14560 + a.instance
CONSOLE_PORT = 14700 + a.instance
V = TREE / "ecps295/sitl/variants"
HIL_EXTRA = ["RNGFND1_TYPE 100", "RNGFND1_MAX 3", "RNGFND1_MIN 0.05", "RNGFND1_ORIENT 25", "GUID_TIMEOUT 1"]


def parm_main() -> str:  # sitl_hil_test.py: companion_fs + route B + SITL rangefinder (GPS stays on)
    return "\n".join([(V / "companion_fs.parm").read_text(), (V / "G_extnav_height.parm").read_text(), *HIL_EXTRA,
                      "SERIAL2_PROTOCOL 2"]) + "\n"


# mtf01_sitl_rng.parm carries FENCE_ALT_MAX 2.5 + FENCE_MARGIN 2 (AVOID_ENABLE 3): fence avoidance then stops a LOITER /
# ALT_HOLD climb at ~0.55-0.6 m (dev run 2026-11-25), i.e. at the new hand-over height. The flight values instead:
# FENCE_ALT_MAX 1.3 (config/fc_flight.parm), FENCE_MARGIN 0.3 (docs/FLIGHT_CONTROLLER.md) = 1.0 m soft ceiling
FLOW_FENCE = ["FENCE_ALT_MAX 1.3", "FENCE_MARGIN 0.3"]


def parm_flow() -> str:  # the real drone's estimator: flow + rangefinder, no GPS, route-B height, companion failsafe
    return "\n".join([Path(a.mtf01_parm).read_text(), *FLOW_FENCE, (V / "companion_fs.parm").read_text(),
                      (V / "G_extnav_height.parm").read_text(), "GUID_TIMEOUT 1", "SERIAL2_PROTOCOL 2"]) + "\n"


# ====================================================================== one SITL + companion session
class Run:
    def __init__(self, name: str, rep: int, *, flow: bool, home_north_m: float = 0.0):
        self.name, self.rep, self.flow = name, rep, flow
        self.W = OUT / f"rep{rep}" / name
        if self.W.exists():
            shutil.rmtree(self.W)
        self.W.mkdir(parents=True)
        self.procs: list[subprocess.Popen] = []
        self.comp: subprocess.Popen | None = None
        self.obs = {k: [] for k in ("viso", "hb254", "modes", "ptgt", "atgt", "att", "lpn", "rng", "viso_age", "text254",
                                    "fc_text", "truth")}
        self.rc = {"ch": None, "last": 0.0, "hb": 0.0, "silent": False, "req": 0.0}
        self.mark: dict[str, float] = {}
        self.res: dict = {"run": name, "repeat": rep, "work": str(self.W)}
        lat = ORIGIN[0] + home_north_m / M_PER_DEG
        self.home = f"{lat:.7f},{ORIGIN[1]:.7f},{ORIGIN[2]:.1f},0"
        self.res["sitl_home"] = self.home

    # ------------------------------------------------------------------ processes (only PIDs started here are signalled)
    def _start(self, cmd, log, env=None) -> subprocess.Popen:
        p = subprocess.Popen(cmd, stdout=open(self.W / log, "w"), stderr=subprocess.STDOUT, cwd=self.W,
                             start_new_session=True, env=env)
        self.procs.append(p)
        (self.W / "pids.txt").open("a").write(f"{p.pid} {log} {' '.join(map(str, cmd))[:200]}\n")
        return p

    def start_sitl(self) -> None:
        (self.W / "sitl.parm").write_text(parm_flow() if self.flow else parm_main())
        defaults = f"{a.copter_parm},{self.W / 'sitl.parm'}"
        self._start([a.sitl_bin, "--model", "+", "--speedup", "1", f"-I{a.instance}", "--defaults", defaults,
                     "--home", self.home, "--serial2", f"udpclient:127.0.0.1:{COMP_PORT}"], "sitl.log")
        t0 = time.monotonic()
        self.g = None
        while self.g is None and time.monotonic() - t0 < 30:
            time.sleep(0.5)
            try:
                self.g = mavutil.mavlink_connection(f"tcp:127.0.0.1:{SERIAL0}", source_system=255, retries=1)
            except Exception:
                self.g = None
        assert self.g is not None, "no TCP connection to SITL SERIAL0"
        assert self.g.wait_heartbeat(timeout=60) is not None, "no SITL heartbeat"
        self.g.target_system, self.g.target_component = 1, 1
        self.t0 = time.monotonic()

    def start_companion(self, hw: dict, extra: list[str], env_extra: dict | None = None, tag: str = "") -> None:
        hw_yaml = self.W / "companion_hw.yaml"
        hw_yaml.write_text(yaml.safe_dump(hw))
        self.clog = self.W / "clog"
        env = dict(os.environ, ECPS_ROOT=str(TREE), **(env_extra or {}))
        self.res["companion_env"] = env_extra or {}
        self.res["companion_args"] = extra
        self.comp = self._start([a.python, "-u", str(OPI / "companion/ecps_companion.py"), "--mode", "guided", "--sitl",
                                 "--url", f"udpin:127.0.0.1:{COMP_PORT}", "--tag", tag or f"rog_{self.name}",
                                 "--hw", str(hw_yaml), "--log-dir", str(self.clog), "--set", f"brain.source={a.brain}",
                                 *extra], "companion.log", env=env)

    def stop(self) -> None:
        for p in self.procs[::-1]:
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        for p in self.procs[::-1]:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                p.wait(timeout=5)
        try:
            self.g.close()
        except Exception:
            pass
        with gzip.open(self.W / "obs.json.gz", "wt") as fo:  # what the GCS saw (time = CLOCK_MONOTONIC, as ticks.csv mono)
            json.dump({"mark": self.mark, **self.obs}, fo)
        for f in glob.glob(str(self.W / "logs/*")):  # dataflash logs: gzip (disk 89 % full)
            if not f.endswith(".gz") and os.path.isfile(f):
                with open(f, "rb") as fi, gzip.open(f + ".gz", "wb", compresslevel=6) as fo:
                    shutil.copyfileobj(fi, fo)
                os.remove(f)
        for f in ("eeprom.bin",):
            if (self.W / f).exists():
                (self.W / f).unlink()

    # ------------------------------------------------------------------ the pilot / GCS
    def pump(self, sec: float) -> None:
        g, rc, obs = self.g, self.rc, self.obs
        t_end = time.monotonic() + sec
        while time.monotonic() < t_end:
            now = time.monotonic()
            if not rc["silent"] and now - rc["last"] > 0.1:
                g.mav.rc_channels_override_send(1, 1, *(rc["ch"] or (0, 0, 0, 0)), 0, 0, 0, 0)
                rc["last"] = now
            if not rc["silent"] and now - rc["hb"] > 1.0:
                g.mav.heartbeat_send(6, 8, 0, 0, 0)
                rc["hb"] = now
            if not rc["silent"] and now - rc["req"] > 5.0:
                # LOCAL_POSITION_NED, ATTITUDE, EKF_STATUS, POSITION_TARGET_LOCAL_NED, ATTITUDE_TARGET, RANGEFINDER, SIM_STATE
                for mid, hz in ((32, 10), (30, 20), (193, 2), (85, 20), (83, 20), (173, 20), (108, 10)):
                    g.mav.command_long_send(1, 1, 511, 0, mid, 1e6 / hz, 0, 0, 0, 0, 0)
                rc["req"] = now
            m = g.recv_match(blocking=True, timeout=0.05)
            if m is None:
                continue
            now = time.monotonic()
            k, src = m.get_type(), m.get_srcSystem()
            if src == 254:
                if k == "VISION_POSITION_ESTIMATE":
                    obs["viso"].append((now, -m.z))
                    if obs["rng"]:
                        obs["viso_age"].append(now - obs["rng"][-1])
                elif k == "HEARTBEAT":
                    obs["hb254"].append(now)
                elif k == "STATUSTEXT":
                    obs["text254"].append((now, m.text))
            elif src == 1:
                if k == "HEARTBEAT":
                    mode, armed = mavutil.mode_string_v10(m), bool(m.base_mode & 128)
                    if not obs["modes"] or obs["modes"][-1][1:] != (mode, armed):
                        obs["modes"].append((now, mode, armed))
                elif k == "POSITION_TARGET_LOCAL_NED":
                    obs["ptgt"].append((now, m.vx, m.vy, m.vz, m.type_mask))
                elif k == "ATTITUDE_TARGET":
                    obs["atgt"].append((now, m.body_yaw_rate))
                elif k == "ATTITUDE":
                    obs["att"].append((now, m.yaw, m.yawspeed))
                elif k == "RANGEFINDER":
                    obs["rng"].append(now)
                elif k == "LOCAL_POSITION_NED":
                    obs["lpn"].append((now, m.x, m.y, -m.z))
                elif k == "SIM_STATE":
                    obs["truth"].append((now, (m.lat_int * 1e-7 - ORIGIN[0]) * M_PER_DEG,
                                         (m.lon_int * 1e-7 - ORIGIN[1]) * M_PER_DEG * math.cos(math.radians(ORIGIN[0]))))
                elif k == "STATUSTEXT":
                    obs["fc_text"].append((now, m.text))

    def mode_now(self):
        return self.obs["modes"][-1][1] if self.obs["modes"] else "?"

    def armed_now(self):
        return self.obs["modes"][-1][2] if self.obs["modes"] else False

    def alt_now(self):
        return self.obs["lpn"][-1][3] if self.obs["lpn"] else 0.0

    def xy_now(self):
        return self.obs["lpn"][-1][1:3] if self.obs["lpn"] else (None, None)

    def yaw_now(self):
        return self.obs["att"][-1][1] if self.obs["att"] else 0.0

    def mode_time(self, after, mode):
        return next((t for t, m, _ in self.obs["modes"] if t >= after and m == mode), None)

    def set_param(self, name: str, value: float) -> None:
        for _ in range(3):
            self.g.mav.param_set_send(1, 1, name.encode(), value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            self.pump(0.5)

    def wait_ekf_flow(self, timeout=150.0) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.pump(1.0)
            e = self.g.messages.get("EKF_STATUS_REPORT")
            if self.obs["lpn"] and e is not None and (e.flags & 0x1) and (e.flags & (0x8 | 0x100)) and time.monotonic() - t0 > 20:
                break
        self.res["ekf_ready_s"] = round(time.monotonic() - t0, 1)

    def arm(self, mode: str) -> bool:
        self.g.set_mode(mode)
        self.rc["ch"] = (1500, 1500, 1000, 1500)
        self.pump(1.0)
        for _ in range(12):
            if self.armed_now():
                return True
            self.g.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
            self.pump(1.5)
        return self.armed_now()

    def takeoff_althold(self, target: float = 0.8, mode: str = "ALT_HOLD") -> dict:
        """The pilot: ALT_HOLD (or LOITER), arm, throttle up to target, stick centred (as sitl_rtl_test.py R1)."""
        ok = self.arm(mode)
        t_arm = time.monotonic()
        lp = self.obs["lpn"][-1] if self.obs["lpn"] else None
        tr = self.obs["truth"][-1] if self.obs["truth"] else None
        self.rc["ch"] = (1500, 1500, 1650, 1500)
        while ok and time.monotonic() - t_arm < 25 and self.alt_now() < target:
            self.pump(0.1)
        self.rc["ch"] = (1500, 1500, 1500, 1500)
        self.pump(1.0)
        return {"mode": mode, "armed": ok, "t_arm": t_arm, "fc_xy_at_arm": [round(lp[1], 2), round(lp[2], 2)] if lp else None,
                "truth_ne_at_arm": [round(tr[1], 2), round(tr[2], 2)] if tr else None, "alt_m": round(self.alt_now(), 2)}

    def land_and_disarm(self, timeout=45.0) -> bool:
        self.g.set_mode("LAND")
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.pump(0.5)
            if not self.armed_now():
                return True
        return False

    def relocate_north(self, target_x: float = 5.0) -> dict:
        """Fly the vehicle target_x m north in LOITER (FC fence off), land and disarm: the FC position at the next arming
        is then ~target_x (the EKF offset of a drone carried to its take-off spot)."""
        self.set_param("FENCE_ENABLE", 0)
        to = self.takeoff_althold(0.8)
        self.g.set_mode("LOITER")
        self.pump(2.0)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 40:
            x, y = self.xy_now()
            if x is None or x >= target_x - 0.4:
                break
            d = 90 if target_x - x > 1.5 else 40
            psi = self.yaw_now()
            self.rc["ch"] = (int(1500 - d * math.sin(psi)), int(1500 - d * math.cos(psi)), 1500, 1500)
            self.pump(0.2)
        self.rc["ch"] = (1500, 1500, 1500, 1500)
        self.pump(4.0)
        landed = self.land_and_disarm()
        self.rc["ch"] = (1500, 1500, 1000, 1500)
        self.pump(2.0)
        # the FC keeps home (= circle fence centre) at the first arming point: re-arming 5 m away fails "Vehicle breaching
        # Circle fence". A first arming of a carried drone sets home where it stands; DO_SET_HOME (current) does the same
        self.g.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 1, 0, 0, 0, 0, 0, 0)
        self.pump(1.0)
        ack = self.g.messages.get("COMMAND_ACK")
        self.set_param("FENCE_ENABLE", 1)
        x, y = self.xy_now()
        return {"takeoff": to, "landed_disarmed": landed,
                "set_home_ack": ack.result if ack is not None and ack.command == 179 else None, "fc_xy_after": [round(x, 2), round(y, 2)] if x is not None else None,
                "duration_s": round(time.monotonic() - t0, 1)}

    # ------------------------------------------------------------------ companion logs
    def ticks(self) -> list[dict]:
        d = sorted(glob.glob(str(self.clog / "*_guided_*")))
        if not d:
            return []
        self.res["companion_log"] = d[-1]
        rows = list(csv.DictReader(open(Path(d[-1]) / "ticks.csv")))
        tf = 0.0
        for r in rows:  # pilot time t (watchdog events) = sum of the dt of the earlier ticks
            r["t_flight"] = tf
            tf += float(r["dt_ms"]) / 1000.0
        return rows

    def run_json(self) -> dict:
        d = sorted(glob.glob(str(self.clog / "*_guided_*")))
        p = Path(d[-1]) / "run.json" if d else None
        return json.load(open(p)) if p and p.exists() else {}

    def comp_lines(self, sub: str) -> list[str]:
        return [ln for ln in (self.W / "companion.log").read_text(errors="replace").splitlines() if sub in ln]


def f(x):
    return float(x) if x not in ("", None) else None


def hw_copy(**mv) -> dict:
    hw = json.loads(json.dumps(HW0))
    hw["mavlink"].update(mv)
    hw.setdefault("console", {})["port"] = 0
    return hw


# ====================================================================== run main: T1-T7
def run_main(rep: int) -> dict:
    r = Run("main", rep, flow=False)
    ph = {}
    try:
        r.start_sitl()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 120:  # EKF ready on GPS + baro
            r.pump(1.0)
            e = r.g.messages.get("EKF_STATUS_REPORT")
            if e is not None and (e.flags & 0x1F) == 0x1F and time.monotonic() - t0 > 20:
                break
        r.res["ekf_ready_s"] = round(time.monotonic() - t0, 1)
        r.start_companion(hw_copy(), ["--camera", "sim", "--tof", "sim", "--no-fence-turn"])
        r.pump(15.0)
        mark = r.mark
        mark["T1"] = time.monotonic()
        r.pump(5.0)
        mark["T2"] = time.monotonic()
        r.g.set_mode("GUIDED")
        r.pump(1.0)
        r.g.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
        r.pump(3.0)
        mark["T3"] = time.monotonic()
        r.g.mav.command_long_send(1, 1, 22, 0, 0, 0, 0, 0, 0, 0, 0.8)
        r.pump(15.0)
        mark["T4"] = time.monotonic()
        r.rc["ch"] = (1500, 1500, 1500, 1500)
        r.pump(0.5)
        r.g.set_mode("LOITER")
        r.pump(5.0)
        mark["T5"] = time.monotonic()
        r.g.set_mode("GUIDED")
        r.pump(6.0)
        r.rc["ch"] = None
        r.rc["silent"] = True
        mark["T6"] = time.monotonic()
        os.killpg(r.comp.pid, signal.SIGKILL)
        r.pump(10.0)
        mark["end"] = time.monotonic()
    finally:
        r.stop()
    obs, mark = r.obs, r.mark
    T = r.ticks()
    tick = [(float(x["mono"]), int(x["in_control"]), f(x["alt"])) for x in T]

    def ctl(t0, t1):
        return [t for t, c, _ in tick if t0 <= t < t1 and c]

    def n_in(lst, t0, t1):
        return sum(1 for x in lst if t0 <= (x[0] if isinstance(x, tuple) else x) < t1)

    p = ph["T1_disarmed"] = {"control_ticks": len(ctl(mark["T1"], mark["T2"])),
                             "hb254_hz": round(n_in(obs["hb254"], mark["T1"], mark["T2"]) / 5, 2),
                             "viso_hz": round(n_in(obs["viso"], mark["T1"], mark["T2"]) / 5, 1)}
    p["pass"] = p["control_ticks"] == 0 and p["hb254_hz"] >= 0.8 and p["viso_hz"] >= 10
    armed_t = next((t for t, m, ar in obs["modes"] if t >= mark["T2"] and ar), None)
    n2 = len(ctl(mark["T2"], mark["T3"]))
    ph["T2_armed_on_ground"] = {"armed": armed_t is not None, "control_ticks": n2, "pass": armed_t is not None and n2 == 0}
    c3 = ctl(mark["T3"], mark["T4"])
    alt_gcs = min(((abs(t - c3[0]), h) for t, _, _, h in obs["lpn"]), default=(0, None))[1] if c3 else None
    alt_comp = next((h for t, c, h in tick if c3 and t == c3[0]), None)
    gaps = [b - a_ for a_, b in zip(c3, c3[1:])]
    pt = [x for x in obs["ptgt"] if c3 and c3[0] + 0.5 <= x[0] < mark["T4"]]
    p = ph["T3_takeoff_handover"] = {
        "handover_alt_cfg_m": HANDOVER_M,
        "takeover_after_takeoff_s": round(c3[0] - mark["T3"], 2) if c3 else None,
        "alt_at_takeover_m": None if alt_gcs is None else round(alt_gcs, 2),
        "alt_at_takeover_companion_m": None if alt_comp is None else round(alt_comp, 3),
        "setpoint_hz": round(len(c3) / (mark["T4"] - c3[0]), 1) if c3 else 0.0,
        "setpoint_gap_max_ms": round(1000 * max(gaps), 1) if gaps else None,
        "fc_target_echo_hz": round(len(pt) / max(1e-3, mark["T4"] - c3[0] - 0.5), 1) if c3 else 0.0,
        "fc_target_v_nonzero": sum(1 for x in pt if abs(x[1]) + abs(x[2]) + abs(x[3]) > 1e-3),
        "alt_end_m": round(r.alt_now(), 2) if obs["lpn"] else None,
    }
    # the companion's own height at the latch must be >= the config value; the GCS sample (10 Hz) may lag a climb by 0.1 m
    p["pass"] = (bool(c3) and (alt_comp or 0) >= HANDOVER_M and (alt_gcs or 0) >= HANDOVER_M - 0.1
                 and p["setpoint_hz"] > 18)
    t_l = r.mode_time(mark["T4"], "LOITER")
    after = [t for t in ctl(mark["T4"], mark["T5"]) if t_l and t > t_l]
    rel = next((t for t, c, _ in tick if t_l and t > t_l and not c), None)
    ph["T4_loiter_release"] = {"fc_loiter_seen": t_l is not None,
                               "release_latency_ms": round(1000 * (rel - t_l), 1) if rel and t_l else None,
                               "control_ticks_after_switch": len(after),
                               "pass": t_l is not None and rel is not None and rel - t_l < 0.2}
    t_g = r.mode_time(mark["T5"], "GUIDED")
    c5 = ctl(mark["T5"], mark["T6"])
    ph["T5_guided_resume"] = {"resume_latency_ms": round(1000 * (c5[0] - t_g), 1) if c5 and t_g else None,
                              "setpoint_hz": round(len(c5) / (mark["T6"] - c5[0]), 1) if c5 else 0.0,
                              "pass": bool(c5) and t_g is not None and c5[0] - t_g < 0.3}
    land = r.mode_time(mark["T6"], "LAND")
    last_hb = max([t for t in obs["hb254"] if t < mark["T6"] + 0.5], default=None)
    ph["T6_companion_killed"] = {"modes_after": [m for t, m, _ in obs["modes"] if t >= mark["T6"]],
                                 "land_after_kill_s": round(land - mark["T6"], 2) if land else None,
                                 "land_after_last_heartbeat_s": round(land - last_hb, 2) if land and last_hb else None,
                                 "pass": land is not None and 1.5 < land - mark["T6"] < 4.0}
    ph["T7_command_direction"] = t7(T, obs)
    vh = [h for _, h in obs["viso"]]
    r.res["route_b_height_m"] = {"n": len(vh), "min": round(min(vh), 3) if vh else None, "max": round(max(vh), 3) if vh else None}
    ra = sorted(obs["viso_age"])
    if ra:
        r.res["route_b_age_ms_indicative"] = {"p50": round(1e3 * ra[len(ra) // 2], 1), "p95": round(1e3 * ra[int(0.95 * (len(ra) - 1))], 1)}
    r.res["modes"] = [(round(t - r.t0, 2), m, ar) for t, m, ar in obs["modes"]]
    r.res["fc_text"] = [x for _, x in obs["fc_text"]][-30:]
    r.res["phases"] = ph
    return r.res


def t7(T: list[dict], obs: dict) -> dict:
    """Each FC echo is matched to the last control tick whose setpoint had been sent (tick start + compute time) before
    the echo arrived; forward / lateral from POSITION_TARGET_LOCAL_NED (velocity submode only), yaw from ATTITUDE_TARGET
    (the attitude controller's shaped rate: compared only where the command kept its sign for the last 0.25 s)."""
    ctl = [x for x in T if int(x["in_control"]) and not x.get("guard")]
    allt = [(float(x["mono"]) + float(x["compute_ms"]) / 1000.0, x) for x in T]
    out = {"v_max": V_MAX, "yaw_rate_max_dps": round(math.degrees(YR_MAX), 1), "control_ticks": len(ctl)}
    if not ctl:
        out["pass"] = False
        return out

    def tick_before(te):
        lo, hi = 0, len(allt)
        while lo < hi:
            mid = (lo + hi) // 2
            if allt[mid][0] + 0.002 < te:
                lo = mid + 1
            else:
                hi = mid
        # the tick must have been sent within the last tick period (+ margin): ticks.csv is flushed once a second, so a
        # SIGKILLed companion (T6) loses its last rows and later echoes would be matched to a stale command
        # (and an echo after the last logged row may answer a row that was never written)
        return lo - 1 if 0 < lo < len(allt) and te - allt[lo - 1][0] < 0.1 else -1

    axes = {"forward": [], "lateral": [], "yaw": []}
    for te, vx, vy, vz, mask in obs["ptgt"]:
        if mask & 0x7 != 0x7:  # not the velocity submode (take-off / position hold): no comparison
            continue
        i = tick_before(te)
        if i < 0:
            continue
        x = allt[i][1]
        if not int(x["in_control"]) or x.get("guard"):
            continue
        psi = math.radians(float(x["yaw"]))
        fb = vx * math.cos(psi) + vy * math.sin(psi)
        lb = -vx * math.sin(psi) + vy * math.cos(psi)
        for ax, tgt, c in (("forward", fb, float(x["cmd_forward"])), ("lateral", lb, float(x["cmd_lateral"]))):
            if abs(c) > 0.1:
                axes[ax].append((c, tgt, tgt / (c * V_MAX)))
    for te, yr in obs["atgt"]:
        i = tick_before(te)
        if i < 5:
            continue
        x = allt[i][1]
        if not int(x["in_control"]) or x.get("guard"):
            continue
        c = float(x["cmd_yaw"])
        if abs(c) <= 0.1:
            continue
        prev = [float(allt[j][1]["cmd_yaw"]) for j in range(i - 5, i)]
        if not all(math.copysign(1, p) == math.copysign(1, c) and abs(p) > 0.05 for p in prev):
            continue
        axes["yaw"].append((c, yr, yr / (c * YR_MAX)))
    ok = True
    for ax, v in axes.items():
        bad = [1 for c, tgt, _ in v if math.copysign(1, c) != math.copysign(1, tgt) or tgt == 0.0]
        ratios = sorted(q for *_, q in v)
        out[ax] = {"n": len(v), "sign_mismatch": len(bad),
                   "ratio_p05_p50_p95": [round(ratios[int(p * (len(ratios) - 1))], 3) for p in (0.05, 0.5, 0.95)] if ratios else None}
        ok = ok and not bad
    out["n_echo_velocity"] = sum(1 for x in obs["ptgt"] if x[4] & 0x7 == 0x7)
    out["pass"] = ok and (out["forward"]["n"] + out["lateral"]["n"]) > 0
    return out


# ====================================================================== T8 (+ T9)
def run_home(rep: int, home_frame: bool) -> dict:
    """home_frame true: hand-over in console HOLD (no client), so any fence turn could only come from the frame.
    home_frame false: hand-over in PATROL (console off): the brain flies and the fence turn shows in the commands."""
    name = "home_true" if home_frame else "home_false"
    r = Run(name, rep, flow=True, home_north_m=5.0)
    res = r.res
    hw = hw_copy(home_frame=home_frame)
    if home_frame:
        key = r.W / "console.key"
        key.write_bytes(os.urandom(32))
        key.chmod(0o600)
        hw["console"].update(port=CONSOLE_PORT, key_file=str(key), handover="hold")
    res["handover_mode"] = "console HOLD" if home_frame else "PATROL (console off)"
    try:
        r.start_sitl()
        r.start_companion(hw, ["--camera", "sim", "--tof", "sim"],
                          env_extra={"ECPS_TEST_SIGUSR1_TOF": "1"} if home_frame else None)
        r.wait_ekf_flow()
        r.pump(10.0)
        x, y = r.xy_now()
        res["fc_xy_ground_boot"] = [round(x, 2), round(y, 2)] if x is not None else None
        if x is None or math.hypot(x - 5.0, y) > 1.5:
            res["relocation"] = r.relocate_north(5.0)
        x, y = r.xy_now()
        res["fc_xy_before_arm"] = [round(x, 2), round(y, 2)] if x is not None else None
        to = r.takeoff_althold(0.8, "LOITER")
        if not to["armed"]:
            to = r.takeoff_althold(0.8, "ALT_HOLD")
        res["takeoff"] = to
        r.mark["guided"] = time.monotonic()
        r.g.set_mode("GUIDED")
        r.pump(a.hold_s + 6.0)  # the hand-over comes when the FC reports >= handover_alt_m
        r.mark["hold_end"] = time.monotonic()
        if home_frame and r.armed_now() and r.mode_now() == "GUIDED":
            r.mark["fault"] = time.monotonic()
            os.kill(r.comp.pid, signal.SIGUSR1)
            r.pump(12.0)
        else:
            r.pump(1.0)
        r.mark["end"] = time.monotonic()
    finally:
        r.stop()
    T = r.ticks()
    rj = r.run_json()
    c = [x for x in T if int(x["in_control"]) and float(x["mono"]) >= r.mark["guided"]]
    t_ho = float(c[0]["mono"]) if c else None
    first = [(f(x["x"]), f(x["y"])) for x in c[:20] if f(x["x"]) is not None]
    hf_lines = r.comp_lines("[guard] home frame")
    ev = [(t, w) for t, w in (rj.get("summary", {}).get("watchdog_events") or []) if "fence" in str(w)]

    def mono_of(tf):
        return next((float(x["mono"]) for x in T if x["t_flight"] >= tf - 1e-6), None)

    fence = [(mono_of(t), w) for t, w in ev]
    fence_rel = [round(t - t_ho, 2) for t, w in fence if t is not None and t_ho is not None]  # all, also before
    fence_after = [d for d in fence_rel if d >= -0.5]
    hold_ticks = [x for x in c if float(x["mono"]) < min(r.mark["hold_end"], (t_ho or 0) + a.hold_s)]
    turn = [float(x["mono"]) - t_ho for x in hold_ticks if "fence turn" in x["note"]]
    # the brain's own wish to fly forward (raw_forward, before the fence turn), outside escapes / saccades
    wish = [float(x["mono"]) - t_ho for x in hold_ticks if f(x["raw_forward"]) > 0.05 and not int(x["escape"])]
    rs = [math.hypot(f(x["x"]), f(x["y"])) for x in hold_ticks if f(x["x"]) is not None]

    def near(series, t):
        s = min(series, key=lambda q: abs(q[0] - t)) if series and t is not None else None
        return [round(s[1], 2), round(s[2], 2)] if s else None

    fc_ho, tr_ho = near(r.obs["lpn"], t_ho), near(r.obs["truth"], t_ho)
    tr_arm = to["truth_ne_at_arm"]
    t8 = {
        "home_frame": home_frame,
        "takeoff_mode": to["mode"],
        "fc_xy_at_arm": to["fc_xy_at_arm"], "truth_ne_at_arm": tr_arm,
        "fc_xy_at_handover": fc_ho, "truth_ne_at_handover": tr_ho,
        "truth_moved_arm_to_handover_m": round(math.hypot(tr_ho[0] - tr_arm[0], tr_ho[1] - tr_arm[1]), 2) if tr_ho and tr_arm else None,
        "guard_home_frame_line": hf_lines[-1].strip() if hf_lines else None,
        "handover_after_guided_s": round(t_ho - r.mark["guided"], 2) if t_ho else None,
        "pilot_xy_first_tick": [round(v, 3) for v in first[0]] if first else None,
        "pilot_r_first_tick_m": round(math.hypot(*first[0]), 3) if first else None,
        "pilot_r_max_hold_m": round(max(rs), 3) if rs else None,
        "control_s": round(len(hold_ticks) / 20.0, 1),
        "fence_radius_m": FENCE_R,
        "fence_events_rel_handover_s": fence_rel,
        "fence_turn_ticks": len(turn),
        "first_fence_turn_tick_s": round(turn[0], 2) if turn else None,
        "first_forward_wish_s": round(wish[0], 2) if wish else None,
        "handover_note": (c[5]["note"] if len(c) > 5 else None),
        "fc_modes_after_guided": [(round(t - r.mark["guided"], 2), m, ar) for t, m, ar in r.obs["modes"] if t >= r.mark["guided"]],
    }
    meaningful = to["fc_xy_at_arm"] is not None and to["fc_xy_at_arm"][0] >= 3.0
    if home_frame:
        t8["pass"] = bool(meaningful and hf_lines and first and math.hypot(*first[0]) <= 0.3 and not turn
                          and not [d for d in fence_after if d <= a.hold_s] and t8["control_s"] >= a.hold_s - 1)
    else:  # expected: the pilot sees x ~ +5 m, outside the fence radius: fence turn at once
        # the turn-back state is latched while the pilot already sees x ~ +5 m (also before the hand-over); it acts on
        # the first tick in which the brain wants to fly forward
        t8["expected_fence_turn_at_once"] = bool(turn and wish and turn[0] <= wish[0] + 0.06)
        t8["pass"] = bool(meaningful and t8["expected_fence_turn_at_once"] and not hf_lines)
    res["phases"] = {"T8_home_frame_" + ("true" if home_frame else "false"): t8}
    if home_frame:
        tf = r.mark.get("fault")
        land = r.mode_time(tf, "LAND") if tf else None
        hov = [float(x["mono"]) for x in T if tf and float(x["mono"]) >= tf and "ToF" in (x.get("guard") or "")]
        ge = [(t, w) for t, w in (rj.get("summary", {}).get("guard_events") or []) if tf and t >= tf - 0.1]
        res["phases"]["T9_tof_loss_in_flight"] = {
            "fault_injected": tf is not None,
            "first_hover_after_fault_s": round(hov[0] - tf, 2) if hov else None,
            "land_after_fault_s": round(land - tf, 2) if land else None,
            "hover_ticks": len(hov),
            "guard_events": [(round(t - tf, 2), w) for t, w in ge][:6],
            "disarmed_after": not r.armed_now(),
            "expected_s": {"hover": TOF_STALE_S, "land": TOF_STALE_S + TOF_LAND_S},
            "pass": bool(hov) and hov[0] - tf < TOF_STALE_S + 0.3 and land is not None and 1.8 < land - tf < 3.5,
        }
    res["fc_text"] = [x for _, x in r.obs["fc_text"]][-30:]
    return res


# ====================================================================== T10
def run_t10(rep: int, case: str) -> dict:
    r = Run(f"t10_{case}", rep, flow=True)
    hw = hw_copy()
    extra = ["--camera", "sim", "--tof", "sim", "--no-fence-turn"]
    env = None
    if case == "tof_frozen":
        env = {"ECPS_TEST_SIGUSR1_TOF": "1"}
    elif case == "tof_absent":
        hw["tof"]["sensors"].pop("rear")  # required_tof_bearings [0, 180]: the rear one is gone
    elif case == "cam_absent":
        extra[1] = f"shm:{r.W / 'no_camera_frames'}"  # camera object that never delivers a frame (unplugged UVC)
    elif case == "cam_off":
        extra[1] = "off"
    try:
        r.start_sitl()
        r.start_companion(hw, extra, env_extra=env)
        r.wait_ekf_flow()
        r.pump(10.0)
        if case == "tof_frozen":
            r.mark["fault"] = time.monotonic()
            os.kill(r.comp.pid, signal.SIGUSR1)
            r.pump(2.0)
        to = r.takeoff_althold(0.8, "LOITER")
        if not to["armed"]:
            to = r.takeoff_althold(0.8, "ALT_HOLD")
        r.res["takeoff"] = to
        r.mark["guided"] = time.monotonic()
        r.g.set_mode("GUIDED")
        r.pump(10.0)
        r.mark["end"] = time.monotonic()
    finally:
        r.stop()
    T = r.ticks()
    tg = r.mark["guided"]
    win = [x for x in T if tg <= float(x["mono"]) < r.mark["end"]]
    eligible = [x for x in win if x["fc_mode"] == "GUIDED" and int(x["armed"]) and (f(x["alt"]) or 0) >= HANDOVER_M]
    ctl = [x for x in win if int(x["in_control"])]
    txt = [x for t, x in r.obs["text254"] if t >= tg and "not taking control" in x]
    refused = r.comp_lines("hand-over refused")
    t10 = {"case": case, "eligible_s": round(len(eligible) / 20.0, 1), "control_ticks": len(ctl),
           "statustext_not_taking_control": len(txt), "first_text": txt[0] if txt else None,
           "companion_refusal_line": refused[0].strip() if refused else None,
           "alt_m": to["alt_m"],
           "fc_modes_after_guided": [(round(t - tg, 2), m, ar) for t, m, ar in r.obs["modes"] if t >= tg]}
    if case == "cam_off":  # informative: no camera object at all
        t10["taken"] = len(ctl) > 0
        t10["pass"] = None
    else:
        t10["pass"] = bool(len(eligible) >= 60 and not ctl and txt and refused)
    r.res["phases"] = {f"T10_{case}": t10}
    return r.res


# ====================================================================== main
RUNS = {"main": run_main, "home_true": lambda k: run_home(k, True), "home_false": lambda k: run_home(k, False),
        **{f"t10_{c}": (lambda c: lambda k: run_t10(k, c))(c) for c in ("tof_frozen", "tof_absent", "cam_absent", "cam_off")}}
only = [s for s in a.only.split(",") if s] or list(RUNS)
report_path = OUT / "report.json"
allres = json.load(open(report_path)) if report_path.exists() and (a.first_repeat > 1 or a.merge) else {"repeats": {}}
allres["setup"] = {"tree": str(TREE), "sitl_bin": a.sitl_bin, "instance": a.instance, "serial0_tcp": SERIAL0,
                   "companion_udp": COMP_PORT, "handover_alt_m": HANDOVER_M, "ekf_origin": ORIGIN, "v_max": V_MAX,
                   "yaw_rate_max_dps": math.degrees(YR_MAX), "geofence_radius_m": FENCE_R,
                   "tof_stale_s": TOF_STALE_S, "tof_land_s": TOF_LAND_S,
                   "parm_main": "copter.parm + companion_fs + G_extnav_height + " + ", ".join(HIL_EXTRA) + ", SERIAL2_PROTOCOL 2",
                   "parm_flow": f"copter.parm + {a.mtf01_parm} + {', '.join(FLOW_FENCE)} + companion_fs + G_extnav_height + GUID_TIMEOUT 1, SERIAL2_PROTOCOL 2"}
for k in range(a.first_repeat, a.first_repeat + a.repeats):
    for name in only:
        print(f"=== repeat {k} {name}", flush=True)
        t0 = time.monotonic()
        try:
            res = RUNS[name](k)
        except Exception as e:  # a crashed run is a failed run; the other runs go on
            import traceback

            res = {"run": name, "repeat": k, "error": f"{type(e).__name__}: {e}", "trace": traceback.format_exc()[-2000:]}
        res["wall_s"] = round(time.monotonic() - t0, 1)
        allres["repeats"].setdefault(str(k), {})[name] = res
        for pn, p in (res.get("phases") or {}).items():
            print(f"  {pn}: {'PASS' if p.get('pass') else ('INFO' if p.get('pass') is None else 'FAIL')} "
                  f"{json.dumps({kk: v for kk, v in p.items() if kk not in ('modes_after', 'fc_modes_after_guided')})[:600]}",
                  flush=True)
        if "error" in res:
            print("  ERROR", res["error"], flush=True)
        json.dump(allres, open(report_path, "w"), indent=1, default=str)


def key_numbers(name: str, p: dict) -> str:
    g = p.get
    ph = name.split("_")[0]
    if ph == "T1":
        return f"hb254 {g('hb254_hz')} Hz, VISO {g('viso_hz')} Hz, ctl ticks {g('control_ticks')}"
    if ph == "T2":
        return f"armed {g('armed')}, ctl ticks {g('control_ticks')}"
    if ph == "T3":
        return (f"take-over {g('takeover_after_takeoff_s')} s after take-off cmd at {g('alt_at_takeover_companion_m')} m "
                f"(companion) / {g('alt_at_takeover_m')} m (GCS); {g('setpoint_hz')} Hz, gap max {g('setpoint_gap_max_ms')} ms")
    if ph == "T4":
        return f"release {g('release_latency_ms')} ms after LOITER"
    if ph == "T5":
        return f"resume {g('resume_latency_ms')} ms, {g('setpoint_hz')} Hz"
    if ph == "T6":
        return f"LAND {g('land_after_kill_s')} s after kill"
    if ph == "T7":
        fw, la, yw = g("forward") or {}, g("lateral") or {}, g("yaw") or {}
        return (f"fwd n {fw.get('n')} mism {fw.get('sign_mismatch')} ratio p5/50/95 {fw.get('ratio_p05_p50_p95')}; "
                f"lat n {la.get('n')} mism {la.get('sign_mismatch')}; yaw n {yw.get('n')} mism {yw.get('sign_mismatch')} "
                f"ratio {yw.get('ratio_p05_p50_p95')}")
    if ph == "T8" and g("home_frame"):
        return (f"FC xy at arm {g('fc_xy_at_arm')}, pilot xy at hand-over {g('pilot_xy_first_tick')} (r {g('pilot_r_first_tick_m')} m), "
                f"r max {g('pilot_r_max_hold_m')} m in {g('control_s')} s, fence turn ticks {g('fence_turn_ticks')}, "
                f"guard line {'yes' if g('guard_home_frame_line') else 'no'}")
    if ph == "T8":
        modes = [f"{m}@{t}" for t, m, _ in (g("fc_modes_after_guided") or [])][:4]
        return (f"FC xy at arm {g('fc_xy_at_arm')}, pilot xy at hand-over {g('pilot_xy_first_tick')}; fence turn-back latched "
                f"{g('fence_events_rel_handover_s')} s rel. hand-over, first turn tick {g('first_fence_turn_tick_s')} s "
                f"(first forward wish {g('first_forward_wish_s')} s), {g('fence_turn_ticks')} turn ticks; FC {modes}")
    if ph == "T9":
        return f"hover {g('first_hover_after_fault_s')} s, LAND {g('land_after_fault_s')} s after the freeze"
    if ph == "T10":
        modes = [f"{m}@{t}" for t, m, _ in (g("fc_modes_after_guided") or [])][:3]
        return (f"GUIDED+armed>={HANDOVER_M} m {g('eligible_s')} s, ctl ticks {g('control_ticks')}, "
                f"texts {g('statustext_not_taking_control')} '{g('first_text')}'; FC {modes}")
    return ""


reps = sorted(allres["repeats"], key=int)
rows: dict[str, dict] = {}
for k in reps:
    for run, res in allres["repeats"][k].items():
        if "error" in res:
            rows.setdefault(f"{run} (run error)", {})[k] = {"pass": False, "error": res["error"]}
        for pn, p in (res.get("phases") or {}).items():
            rows.setdefault(pn, {})[k] = p
md = ["# ECPS295 SITL hand-over test on ROG (sitl_hil_rog.py)", "",
      f"Generated {time.strftime('%Y-%m-%d %H:%M')}; tree {TREE}; SITL {a.sitl_bin} -I{a.instance}; "
      f"handover_alt_m {HANDOVER_M} (config); v_max {V_MAX} m/s, yaw {math.degrees(YR_MAX):.0f} deg/s; geofence {FENCE_R} m.", "",
      "| phase | " + " | ".join(f"rep {k}" for k in reps) + " | key numbers (per repeat) |",
      "|---|" + "---|" * len(reps) + "---|"]
for pn in sorted(rows, key=lambda s: (int(''.join(c for c in s[1:3] if c.isdigit()) or 99), s)):
    cells, nums = [], []
    for k in reps:
        p = rows[pn].get(k)
        if p is None:
            cells.append("-")
            continue
        cells.append("PASS" if p.get("pass") else ("INFO" if p.get("pass") is None else "FAIL"))
        nums.append(f"r{k}: " + (p.get("error") or key_numbers(pn, p)))
    md.append(f"| {pn} | " + " | ".join(cells) + " | " + "<br>".join(nums).replace("|", "/") + " |")
md += ["", "INFO = informative case without a pass criterion (T10_cam_off: no camera object at all).",
       "Setup: " + json.dumps(allres["setup"]), ""]
(OUT / "report.md").write_text("\n".join(md))
print("->", report_path, OUT / "report.md")
