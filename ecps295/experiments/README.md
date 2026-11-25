# Experiment working trees

Hand-written scripts, job lists, parameter files, result tables and reports from the experiment
directories on the simulation PC (`~/sim/<name>`), imported 2026-11-21. Until then they lived outside git.

These are working copies, kept as they were run. Paths inside the scripts still point at
`/home/jacky/sim/...`; adjust them before running elsewhere.

| Directory | Source | Content |
|---|---|---|
| `sitl_bench/mtf01` | `~/sim/mtf01` | ArduCopter SITL study of the MTF-01 flow + range sensor (2026-11-19) |
| `sitl_bench/opt_flow`, `opt_flow2` | `~/sim/opt_flow*` | optical-flow parameter studies; the first did not pass acceptance |
| `sitl_bench/gps_flow` | `~/sim/gps_flow` | GPS + flow configuration study |
| `sitl_bench/abs_fix` | `~/sim/abs_fix` | absolute position fix (marker) study |
| `sitl_bench/val1` ... `val8` | `~/sim/val*` | follow-up validation rounds (2026-11-20), incl. the baro-vouches-for-rangefinder monitor (val7, val8) |
| `sitl_bench/abl` | `~/sim/abl` | safety ablation of the new hardware layout (2026-11-21): `report.md`, `report_followup.md` (rounds 3 to 6 and Gazebo), `all_runs.csv` (10,236 SITL runs), tables per round, `gz/` (Gazebo matrices, scripts, result lines `matrix_e*.log`) |
| `sitl_bench/field1119` | `~/sim/field1119` | rehearsal of the 2026-11-19 no-GPS AUTO mission: scripts, the four run logs, `README.md` with figures |
| `gps_hover_20261123` | `~/sim/gps_hover` | GPS + compass hover study (2026-11-23, 2250 SITL runs): scripts, job lists, result tables, `report.md`; basis of `ecps295/ecps_gps_anchor.py` |
| `bench_20261118` | Mac | bench-day tools (`s7_watch.py`, `tof_view.py`, `flow_check.py`) and FC parameters before / after |
| `rl_h2` | `~/sim/rl_h2` | Gazebo matrices, chain scripts and patch scripts of the hover / hold / PIL work (`hv`, `n3a`, `s5gz`-`s7gz`) |
| `rl_trk` | `~/sim/rl_trk` | TargetTrack matrix scripts not already under `ecps295/gazebo` |
| `rl_exp` | `~/sim/rl_exp` | files of the 2026-10-04 RL snapshot that never reached the main tree |
| `gzc` | `~/sim/gzc2` | Gazebo console verification: plans, queue, planner on / off configs, result files `verify/pv_result.json` + `pv_truth.json`, `README.md` with figures |
| `misc` | `~/sim` | `thermal_log.sh`, low-box matrix result lists |

Figures: most directories with result data have a `make_figures.py` that redraws the PNG files in `figures/` from the
result files next to it (`python make_figures.py`, needs matplotlib, numpy, pandas). The reports show them in place;
[`../docs/FIGURES.md`](../docs/FIGURES.md) lists them all. `rl_h2`, `rl_trk` and `misc` hold job lists only (per-run results were not kept): their `README.md` shows how the
scripts chain and where the results are reported.

Firmware build inputs are in `ecps295/firmware`, 3D-print generators in `ecps295/hardware/3d_print`.

## Left out on purpose

- Per-run output: `runs/`, `runs_failed/`, `logs/`, `terrain/`, `*.log`, dataflash logs, `abl/hg/`.
- Generated parameter sets (`gps_flow/parms`, `opt_flow/parm`); the `gen*.py` next to them rebuilds them.
- Large job lists: `abl/jobs_mac.txt`, `abl/jobs_r2.txt`, `val7/jobs_v7_s4.txt`. (The result tables `val7/fp_s1.csv`,
  `gzc/verify/pv_truth.json`, `gzc/verify/pv_result.json` and `abl/all_runs.csv` were added on 2026-11-24: the reports
  and figures need them.)
- Copies of the main tree inside the working directories (`*/ecps295`, `*/companion`, `gzc*/src`); they are older
  than the tracked files.
- Binaries: firmware `.apj` / `.bin`, `es.pkl`, STL / 3MF models.
- `gzc2/config/console.key` (a secret).
