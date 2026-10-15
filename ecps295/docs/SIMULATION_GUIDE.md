# Simulation guide: set up, run, reproduce

> State of 2026-10-15 · what the simulators are and how they fit together: [ARCHITECTURE.md](ARCHITECTURE.md) §9 ·
> full lab notebook with every command in order: [../README.md](../README.md) · RL details (Chinese):
> [../rl/README.md](../rl/README.md)

There are two simulators. They run the **same** retina, brain, decoder, pilot and shield code.

| | Fast twin (`rl/twin.py`) | Gazebo + ArduPilot SITL (`run_g4.py`) |
|---|---|---|
| Physics | NED kinematics with SITL-identified lags (xy 0.60 s, z 0.35 s, yaw 0.12 s) | ArduPilot Copter-4.7.0 SITL, Gazebo Harmonic airframe `ecps295_quad` |
| Camera | ray-cast 120° fisheye aligned with Gazebo | Gazebo wide-angle camera → `cam_bridge.py` → shared memory |
| Navigation | ground truth or a randomised optical-flow error model (`--nav flow --flow dr`) | real EKF3 on simulated flow + ToF, route B |
| Speed | ≈ 20–30 × real time on 12 cores | real time (lock-step, RTF ≥ 0.95) or slower with the sim clock |
| Used for | CMA-ES, screens over thousands of episodes, control arms | acceptance tests, fault injection, paired comparisons |
| Determinism | bit-identical across machines (`rl.gates`) | stochastic (SITL noise, scheduling) |

## 1. Environments

Python 3.11. The fast twin needs **numpy 2.4.6 and scipy 1.17.1** to reproduce the anchors bit for bit (pinned in
[`../deploy/desktop/requirements_flydrones.txt`](../deploy/desktop/requirements_flydrones.txt)).

```bash
git clone -b ecps295-sim https://github.com/jiewent1-cmyk/FlyDrones.git
cd FlyDrones
conda create -n flydrones python=3.11 && conda activate flydrones
pip install -e ".[vision,gestures,mavlink,dev,data]"
cd ecps295
for v in v2 v3 v4; do python my_minifly.py $v --out minifly/minifly_$v.npz; done   # network weights (not in git)
export PYTHONPATH=../src:.
python -m rl.gates all          # sensory gate, smoke tests, bit-identical anchors; must print 12/12
```

For Gazebo you also need ArduPilot SITL and Gazebo Harmonic:

```bash
git clone --branch Copter-4.7.0 --recurse-submodules --shallow-submodules --depth 1 https://github.com/ArduPilot/ardupilot.git
conda create -n ardupilot python=3.11 && conda activate ardupilot
pip install "empy==3.3.4" future pexpect pymavlink MAVProxy dronecan lxml numpy pyserial setuptools
cd ardupilot && git apply ../FlyDrones/ecps295/sitl/ardupilot_sitl.patch
./waf configure --board sitl && ./waf copter
python ../FlyDrones/ecps295/make_sitl_params.py path/to/Params08Aug2026_CRSF.param   # course_base.parm
```

Gazebo Harmonic comes from the OSRF apt repository, plus `ArduPilot/ardupilot_gazebo` at commit `082a0fe` built
outside conda. On a fresh Ubuntu 24.04 + NVIDIA machine, [`../deploy/desktop/setup_desktop.sh`](../deploy/desktop/README.md)
does all of this in resumable stages and `verify_desktop.sh` checks it (gates, anchors, RTF, a few Gazebo flights).
The launch scripts in [`../scripts/`](../scripts/) assume `~/sim/ardupilot`, `~/sim/FlyDrones` and `~/sim/runs`.

## 2. Fast twin

```bash
cd ecps295 && export PYTHONPATH=../src:.
python -m rl.episode --config rl/configs/v3_2_es3.yaml --seconds 120          # one episode
python -m rl.hw_arms --conds s1_noback s2_mem s3_rng s23 --out rl/out/struct.jsonl   # S1–S3 screen
python -m rl.shield_screen --seed-from rl/out/struct.jsonl --out rl/out/shield.jsonl # S4/S5 shield screen
python -m rl.diag_contacts --conds v4 s3_rng --out rl/out/diag_contacts.jsonl       # contact diagnosis
```

Test sets: `lowbox_s0–49` (low boxes + a 1.1 m stack), `cage20` (the 20 ft course cage), empty field (false alarms).
Training uses procedural worlds only; `cage20` and `lowbox_s0–49` are never trained on.

## 3. CMA-ES

```bash
python -m rl.cmaes_run --run n3s5_s1 --space v4hw_sh --fitness v5 --shield s5 --arm intact \
    --base minifly/v4.yaml --nav flow --flow dr --lowbox-frac 0.5 --gens 20 --k 16 --popsize 14
```

| Option | Meaning |
|---|---|
| `--space` | parameter space: `default`/`wide` (17 decoder params), `v4hw` (22, hardware caps), `v4hw_eff`, `v4hw_sh` (+ cruise, d_safe, α) |
| `--fitness` | v1 … v5 on this branch (v6 with hard safety constraints is in N3 round 2, pending push) |
| `--arm` | `intact`, `bypass`, `shuffle<k>`, … (see `rl/arms.py`) |
| `--shield` | run every candidate with a shield condition from `rl/shield_screen.py` |
| `--nav flow --flow dr` | optical-flow navigation with per-episode randomised errors |

Each run writes `rl/out/<run>/` (`gens.jsonl`, `val.jsonl`, `best_val.json`, `config.json`, `cmaes.log`); the best
hold-out generation is exported as a `minifly/*.yaml` for Gazebo. Archived runs are in
[`../rl/results/twin/`](../rl/results/twin/).

## 4. Gazebo + SITL

```bash
# wall suite (T1–T5) for one variant
VARIANT=v2 DECODER=ecps PILOT=ecps RETINA=ecps bash scripts/g4_suite.sh
# 120 s patrol in the 20 ft cage
bash ~/sim/gz_g4.sh ecps295_cage20.sdf patrol "--config minifly/v3.yaml --decoder ecps --pilot ecps \
     --retina ecps --mode patrol --seconds 120 --fence-turn --out patrol"
# hardware-aligned matrix (flow nav + route B + companion failsafe), 3 parallel instances
V=~/sim/hw/ecps295/sitl/variants
NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm \
  nohup setsid bash run_matrix_par.sh <matrix> ~/sim/runs/matrix_<name>.log 3
```

Useful `run_g4.py` options: `--mode probe|hover|approach|patrol`, `--nav gps|flow`, `--config`,
`--decoder/--pilot/--retina ecps`, `--fence-turn`, fault injection `--freeze-cam-at`, `--ext-drop-at`,
`--ext-drop-for`, `--mute-at`, `--record-frames` (input for `pil_replay.py`), `--live` (monitor window).
Environment switches (all off by default): `ECPS_ARM` (control arm), `ECPS_TOF_CONE3D=1` (27° cone ToF),
`ECPS_SIMCLOCK` (sim-clock mode on slow machines).

Rules that every Gazebo result in the reports follows:

- A run counts only if its whole-run **RTF ≥ 0.95** (lock-step), or if it ran under the sim clock.
- Scores use simulator truth (`SIM_STATE`); the EKF estimate is logged next to it.
- Comparisons are **paired** by world and brain seed (exact McNemar for contact runs, paired bootstrap / Wilcoxon for
  continuous metrics, Holm correction across metrics).
- Each run writes `run.json` (summary, counters, flags) and a per-tick CSV; collect them into
  `rl/results/gazebo/hw_runs.csv`.

## 5. Machines

| Machine | Use | Notes |
|---|---|---|
| ROG laptop | Gazebo, 3 instances | quiet 01:00–07:00 PT (runners pause themselves); check `nvidia-smi` after every reboot |
| Desktop | Gazebo, 2–4 instances | only when the owner is not using it |
| Lab server (shared) | many Gazebo instances with the sim clock | only when needed; keep total load ≲ 90 cores |
| Mac mini M4 | fast twin, CMA-ES | fastest per core |
| Jetson AGX Orin | fast twin, CMA-ES overflow | — |

Never `pkill -f <pattern>` over ssh: the pattern matches the ssh command itself.

## 6. Reproducing the reports

```bash
cd ecps295
python reports/make_figures_hw.py   # figures 9–16 and reports/hw_stats.txt (reports 5–6)
python reports/make_figures_n.py    # figures 17–21 and reports/n_stats.txt (reports 7–8)
```

| Data | File |
|---|---|
| Gazebo runs (hardware-aligned) | `rl/results/gazebo/hw_runs.csv` |
| Gazebo runs (GPS cage, server) | `rl/results/gazebo/gazebo_runs.csv` |
| Twin tests: controls, seeds, OOD | `rl/results/twin/hw/tests.csv.gz` |
| Twin tests: S1–S5, diagnosis | `rl/results/twin/n/tests.csv.gz` |
| Twin tests: stage E | `rl/results/twin/mb/tests.csv.gz` |
| S8 ablation | `results/ablation_2026-10-06/` |

Before pushing changes to the shared code, run `python -m rl.gates all`; new behaviour must be behind an option that
is off by default so older configs keep their anchors.
