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
