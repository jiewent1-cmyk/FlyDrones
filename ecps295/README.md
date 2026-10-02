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
