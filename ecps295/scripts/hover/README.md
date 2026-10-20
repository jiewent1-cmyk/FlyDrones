# Hover-precision tests (feat/hover)

`run_g4.py --mode hovertest` runs the plan in `ecps295/hover_test.py`. The brain stays passive and is only logged; the script sends the setpoints itself. The plan is hover 40 s, yaw 90°, hover 20 s, then forward 2 s, then hover, yaw back, hover. Env vars:

- `ECPS_HOLD=vel`: velocity 0, the same as the companion today.
- `ECPS_HOLD=pos`: position target at the hover start (step 1).
- `ECPS_YAW_DPS`: yaw rate in °/s, default 30.

Analysis: `python ecps295/hover_stats.py ~/sim/runs <tagA> [<tagB>]`. It reports truth / EKF / estimator-drift displacement per block and a paired Wilcoxon test.

The runners here are the ROG copies; they call `~/sim/rl_h2/run_matrix_mine.sh` and `gz_g4_mine.sh` and use instances 5–7:

| Runner | Matrix | What it runs |
|---|---|---|
| `par.sh` | `batch.txt` | vel vs pos, 20 lowbox worlds |
| `par2.sh` | `sweep.txt` | pos hold with EK3_FLOW_DELAY 10/40/60/80 ms (`d*.parm`) and yaw 15 °/s |

Results of the 10-18 batch (pos hold vs vel 0) are in the Chinese RL roadmap §3.12. In block A, the median truth rms went from 14.4 to 6.0 cm.

Sync to ROG: `rsync` the changed `ecps295/*.py` into `~/sim/rl_h2/ecps295/` and this folder into `~/sim/rl_h2/hv/`. ROG has no git credentials, so commit here on the Mac.
