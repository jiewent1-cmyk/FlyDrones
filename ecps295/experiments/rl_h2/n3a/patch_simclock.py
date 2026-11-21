"""ROG Gazebo tree (2026-10-15): optional simulated-time pacing for run_g4.py, default off (ECPS_SIMCLOCK=1 turns it on).
run_g4 ticks the brain every 50 ms of WALL time and times each phase on the wall clock, while Gazebo runs at ~0.8 x real time
(rtf.log): a "120 s" run flies ~96 s of simulated time and the brain gets 25 ticks per simulated second, i.e. it reacts
faster relative to the physics than on the real drone (and than in rl.twin). With ECPS_SIMCLOCK=1 the companion waits for the
FC's time_boot_ms (SITL runs in lockstep with Gazebo) to advance 50 ms per tick, passes the simulated dt to the brain and runs
each phase for its length in simulated time (wall-clock guard 3 x + 60 s). run.json: simclock, sim_s, wall_s.
Run from ~/sim/rl_h2/ecps295; backup run_g4.py.pre_simclock"""
import shutil
from pathlib import Path

p = Path("run_g4.py")
s = p.read_text()
assert "SIMCLOCK" not in s, "already patched"
R = [
("""def run_phase(phase: str, secs: float, vx=0.0, vz_up=0.0, yaw_dps=0.0):
    \"\"\"Tick the brain at HZ for secs; in probe mode also drive the vehicle with a scripted body velocity.\"\"\"
""", """SIMCLOCK = os.environ.get("ECPS_SIMCLOCK") == "1"  # 2026-10-15: pace the brain by simulated time (see _run_phase_sim)
pace = {"sim_s": 0.0, "wall_s": 0.0}


def _sim_now() -> float | None:
    \"\"\"Simulated time (s) from the FC's time_boot_ms (SITL runs in lockstep with Gazebo). Reading telemetry refreshes the
    cached MAVLink messages; the pilot reads the same cache, so nothing is lost.\"\"\"
    drone.telemetry()
    ts = [getattr(drone.m.messages.get(k), "time_boot_ms", None) for k in ("ATTITUDE", "LOCAL_POSITION_NED")] if drone.m is not None else []
    ts = [x for x in ts if x is not None]
    return max(ts) / 1000.0 if ts else None


def _run_phase_sim(phase: str, secs: float, s0: float) -> bool:
    \"\"\"Tick every DT of simulated time for secs of simulated time; dt passed to the brain is simulated time.\"\"\"
    last, last_wall, w0 = s0, time.monotonic(), time.monotonic()
    while True:
        now = _sim_now()
        if time.monotonic() - w0 > 3 * secs + 60:
            print(f"simclock: wall-clock guard hit in {phase} (sim {0 if now is None else now - s0:.1f} of {secs} s)", flush=True)
            return True
        if now is None or now - last < DT - 1e-6:
            if now is not None and now - s0 >= secs:
                break
            time.sleep(0.002)
            continue
        if now - s0 >= secs:
            break
        dt = min(0.25, now - last)
        w = time.monotonic()
        tick_dt_ms.append((w - last_wall) * 1000)
        last, last_wall = now, w
        tick(clock["t"], dt, phase)
        clock["t"] += dt
        if pilot.safety.land_requested:
            pace["sim_s"] += now - s0; pace["wall_s"] += time.monotonic() - w0
            return False
    pace["sim_s"] += last - s0; pace["wall_s"] += time.monotonic() - w0
    return True


def run_phase(phase: str, secs: float, vx=0.0, vz_up=0.0, yaw_dps=0.0):
    \"\"\"Tick the brain at HZ for secs; in probe mode also drive the vehicle with a scripted body velocity.\"\"\"
    if SIMCLOCK and a.mode != "probe":
        s0 = _sim_now()
        if s0 is not None:
            return _run_phase_sim(phase, secs, s0)
"""),
("""summary["tof_cone3d"] = os.environ.get("ECPS_TOF_CONE3D") == "1\"""", """summary["simclock"] = SIMCLOCK  # 2026-10-15: brain paced by simulated time
summary["sim_s"], summary["wall_s"] = round(pace["sim_s"], 1), round(pace["wall_s"], 1)
summary["tof_cone3d"] = os.environ.get("ECPS_TOF_CONE3D") == "1\""""),
]
for a_, b_ in R:
    assert s.count(a_) == 1, a_[:70]
    s = s.replace(a_, b_)
shutil.copy2(p, str(p) + ".pre_simclock")
p.write_text(s)
print("patched run_g4.py")
