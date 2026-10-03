# ecps295: digital twin of the ECPS 295 course drone

Simulation work for UCI ECPS 295 (Fall 2026), following the course's phase-1 simulation route.
Nothing under `src/flydrones/` is modified; everything here subclasses or drives upstream code.

## Layers

| Layer | What | Files |
|---|---|---|
| L1 | FlyDrones `SimDrone` with SITL-identified per-axis lag (xy 0.60 s, z 0.35 s, yaw 0.12 s) | `envs.py` |
| L2 | ArduPilot SITL Copter-4.7.0 + course params + 295 g / 2S frame model | `make_sitl_params.py`, `sitl/` |
| L2 link | `MavlinkDrone` with fixes F1 (stream requests), F3, F4 (checked takeoff), F7, F9 (heartbeat) | `mavlink_twin.py` |

## Setup

```bash
# ArduPilot (no sudo needed if gcc/g++ are present)
git clone --branch Copter-4.7.0 --recurse-submodules --shallow-submodules --depth 1 https://github.com/ArduPilot/ardupilot.git
conda create -n ardupilot python=3.11 && conda activate ardupilot
pip install "empy==3.3.4" future pexpect pymavlink MAVProxy dronecan lxml numpy pyserial setuptools
cd ardupilot && ./waf configure --board sitl && ./waf copter

# FlyDrones
conda create -n flydrones python=3.11 && conda activate flydrones
pip install -e ".[vision,gestures,mavlink,dev,data]"

# SITL base params from the course .param file
python ecps295/make_sitl_params.py path/to/Params08Aug2026_CRSF.param
```

`scripts/run_sitl.sh` and `scripts/exp.sh` assume `~/sim/ardupilot`, `~/sim/FlyDrones` and `~/sim/runs`; copy them to `~/sim/`.
SITL 4.7 only loads `--model quad:<file>.json` relative to the working directory, so `run_sitl.sh` copies the JSON into the run dir.

## Gazebo Harmonic (S12)

Gazebo Harmonic 8.15 from the OSRF apt repo plus `ArduPilot/ardupilot_gazebo` built in `~/sim/ardupilot_gazebo`
(outside conda). `scripts/verify_gz.sh` runs `iris_runway.sdf` headless on the GPU, connects SITL over JSON,
and flies GUIDED takeoff to 5 m / hover / land (2026-10-03: hover SD 5 mm, RTF 1.00).

## Experiments

```bash
P=~/sim/FlyDrones/ecps295; M=quad:$P/sitl/ecps295.json
# S4 hover + learned MOT_THST_HOVER
MODEL=$M bash ~/sim/smoke.sh twin 60 $P/sitl/course_base.parm
# S4 time constants (then plot_sysid.py)
MODEL=$M bash ~/sim/exp.sh sysid "$P/sysid_tau.py" $P/sitl/course_base.parm
# S5 closed loop, L1 vs L2
python $P/run_s5.py --mode l1 --out l1          # needs PYTHONPATH=$P
MODEL=$M bash ~/sim/exp.sh s5 "$P/run_s5.py --mode l2-fixed --out l2fixed" $P/sitl/course_base.parm
python $P/compare_l1_l2.py l1.csv ~/sim/runs/s5/l2fixed.csv
# S7 / S7c no-GPS (optical flow + 1.2 m rangefinder); variants in sitl/variants/, INST=n runs in parallel
MODEL=$M INST=0 bash ~/sim/exp.sh s7 "$P/s7_flow.py" $P/sitl/course_base.parm $P/sitl/fhb_delta.parm
python $P/s7_compare.py ~/sim/runs/s7
```

`sitl/fhb_delta.parm` reflects the S7c results: rangefinder as primary height (`EK3_SRC1_POSZ 2`),
altitude-only fence (circle fence blocks arming without GPS) and `FENCE_MARGIN 0.3` (soft ceiling 1.0 m, hard fence 1.3 m).

### G1: `ecps295_quad` (Gazebo airframe)

`gazebo/make_quad.py` generates `gazebo/models/ecps295_quad/` and `gazebo/worlds/ecps295_flat.sdf` (edit the generator,
not the SDF). Same structure as Iris (blade LiftDrag + ArduPilotPlugin rotor velocity loop) sized from the twin:
295 g, 0.16 m wheelbase, 4" props, rotor loop tau 16 ms. Run with `scripts/gz_exp.sh` and SITL params
`sitl/course_base.parm gazebo/gz_ecps295.parm`.

| check (2026-10-03) | SITL twin (`ecps295.json`) | Gazebo `ecps295_quad` |
|---|---|---|
| learned MOT_THST_HOVER (target 0.32) | 0.338 | 0.323 |
| hover SD at 0.6 m | 0.013 m | 0.002 m |
| tau x / z / yaw | 0.60 / 0.35 / 0.12 s | 0.53 / 0.44 / 0.11 s |
| RTF | - | 1.000 |

Known gaps: thrust is quadratic in rotor speed (real ESC/prop curve is closer to MOT_THST_EXPO 0.52),
max rotor speed 838 rad/s is the Iris value (real 4" ~2300 rad/s), no battery current model under JSON.

### G2: front camera (ELP OV7725 twin)

`make_quad.py --camera wide` (default) adds a `wideanglecamera` with an equidistant lens to `base_link`:
120 deg hfov, 640x480, 60 Hz, gaussian noise 0.007, 12 deg nose-down, 30 mm ahead of the front motors and
15 mm below the prop plane (§4.2 estimate). Topic `/ecps295/camera`. Checked 2026-10-03:

- Harmonic 8.15 / gz-rendering8 ogre2 supports wideanglecamera + equidistant; straight edges bend as expected.
- 62.7 Hz in sim time with RTF 1.000 on the RTX 3070; hover still learns MOT_THST_HOVER 0.322.
- Props: at this mount they stay out of frame; with the camera level with the motors they fill both upper corners.
- Sensors live on `base_link`: a 10 g camera link on a `fixed` joint made the vehicle hover at 15% less thrust.

Watch it on the ROG desktop: `bash ecps295/gazebo/gz_view.sh` (3D chase view + docked camera, demo flight from
`demo_flight.py`; `--no-demo` to fly yourself). Grab frames headless: `/usr/bin/python3 gazebo/gz_grab.py`.
Only one ArduPilot Gazebo instance at a time for now (plugin port 9002 is fixed).

### G4: MiniFly flies by the Gazebo camera

`gazebo/cam_bridge.py` (system python, has the gz bindings) converts `/ecps295/camera` to 192x144 grayscale and
publishes it in `/dev/shm/ecps295_cam`; `gz_camera_drone.py::GazeboCameraMavlinkDrone` serves it as `frame()`.
`scripts/gz_g4.sh <world> <tag> "<run_g4.py args>" [--gui]` starts Gazebo, the bridge, SITL and `run_g4.py`.
Results 2026-10-03 (MiniFly synthetic, default config, camtest worlds):

- bridge 62 frames/s, frame age p95 8.8 ms, brain >= 2.2x real time
- probe (open loop): yaw right/left, climb, descend all produce an opposing command (optomotor stabilisation OK);
  side effects: yawing also raises throttle (+0.2), flying forward 0.3 m/s commands descent (-0.09, same size as a
  real 0.2 m/s climb), so S8-d is a real concern
- looming needs texture: plain cardboard box peaks DNp03/DNp01 10/20 Hz (no escape); taped box 60/40 Hz,
  DNp03 >= 20 Hz at 0.74 m, escape onset only at 0.23 m (decoder GF filter)
- closed loop: hover 40 s at 0.60-0.88 m with no safety events; approach at ~0.25 m/s brakes from 0.7 m,
  escapes (climb) at ~0.1 m and clears the 0.5 m box by 0.15 m; geofence stops it at 2.16 m
- ROOM panel: `make_quad.py` writes `worlds/<world>.layout.json`; `gz_g4.sh` passes it so the obstacles are drawn

### Monitor window (`monitor.py`)

One window: MiniFly dashboard on the left, live Gazebo view on the right (keys: `c` switch camera, `q` quit).

    bash ~/sim/gz_g4.sh ecps295_camtest_tape.sdf g4_watch "--mode approach --seconds 40 --cruise 0.5 --live --out watch"

- `run_g4.py` publishes every tick on 127.0.0.1:5799 from a forked child (ticks are dropped when no monitor is
  connected); `--live` starts `monitor.py` unless it is already running. The monitor outlives runs and reconnects.
- The right panel is rendered by Gazebo: a chase camera rigid behind the drone (and optionally a fixed overview
  camera). cam_bridge.py publishes it to /dev/shm next to the drone camera.
- Each extra camera sensor costs ~4-6% RTF under lockstep, almost independent of size and rate:
  none 1.000, chase 640x360@30 0.957, chase+overview 640x360@15 0.916, both 960x540@30 0.879
  (a 512 instead of 1024 wide-angle cube map did not help). So the view cameras only exist in the
  `ecps295_quad_monitor` model and `*_monitor` worlds that gz_g4.sh picks with `--live`; plain worlds stay at 1.000.
- Monitor run 2026-10-03: control tick p50/p95/max 50.1/50.1/51.0 ms, frame age p95 8.6 ms, RTF mean 0.946
  (one 0.14 stall sample; gz_g4.sh now also reports the median).

### G4 flight-test suite (`scripts/g4_suite.sh`, 2026-10-03)

Upstream synthetic MiniFly, default config, headless plain worlds (RTF 1.000, control tick p95 50.1 ms). Tall walls
(1.5 m, above the 1.0 m ceiling) 1.7 m ahead. Clearance = prop tip to surface; negative = contact.

| test | setup | contact | brake onset | escape onset | note |
|---|---|---|---|---|---|
| T1_v015 | taped wall, 0.15 m/s | yes | 0.23 m | 0.06 m | slow approach looms late |
| T1_v025 | taped wall, 0.25 m/s | yes | 0.44 m | 0.40 m | |
| T1_v035 | taped wall, 0.35 m/s | yes | 1.01 m | 0.50 m | stops at 0.33 m after escape, then creeps in |
| T2_plain | plain wall, 0.25 m/s | yes | 0.28 m | - | never escapes |
| T3_offset | taped wall 0.45 m right | yes | 0.31 m | 0.15 m | 3 escapes, no turn-away |
| T4_freeze | taped, camera frozen at 3 s | yes | 0.02 m | - | no frame-staleness check anywhere |
| T5_hover | 90 s hover | no | - | - | 0.60-0.98 m, no safety events |

Failure mechanism (T1_v035 trace, `suite_trace.py`): looming only signals expansion. Once the drone has stopped, the
wall fills the view, nothing expands, DNp03 goes silent, the brake memory (x0.93 per tick) fades and the cruise
pushes it into the wall. Needs: brake/escape memory or a no-advance state after a scare, a near-obstacle cue that
does not depend on expansion (S2), and a camera staleness watchdog next to F2 (S6).

### G3: course cages (`ecps295_cage20.sdf`, `ecps295_cage10.sdf`)

Generated by `make_quad.py` (centred on the take-off point, drone facing north):
20' cage 6.1 x 6.1 x 3.05 m and 10' cage 3.05^3 m; 5 cm aluminium posts every <= 3.05 m with top rails; net walls as
real strands (4 mm on a 10 cm grid, with a thin collision box) so the camera sees far buildings and trees 30-60 m
outside; 24" two-tone EVA checker floor with random tape strips; 18-24" cardboard boxes (taped and plain) and in the
20' cage a 1.1 m two-box stack straight ahead (above the 1.0 m ceiling); four lab lamps. The net walls are also in the
layout, so the ROOM panel draws the cage and the clearance metric includes them.

2026-10-03, 20' cage: hover 30 s at 0.60-0.86 m with no safety events, RTF 1.000, tick p95 50.1 ms, compute p95
9.1 ms. Approach toward the stack at 0.25 m/s reproduces the wall-suite failure (escape at 0.24 m, stops, creeps in).
Still open from §6A G3: real EVA mat photo texture and light levels from a lux measurement in the cage.

