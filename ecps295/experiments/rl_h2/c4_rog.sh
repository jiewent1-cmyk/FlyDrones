#!/bin/bash
# C4 under the hardware-aligned setting: bypass arm, same protocol as H2 (v4hw, fitness v3, flow dr, lowbox 0.5), seeds 3 then 2.
source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null || source ~/anaconda3/etc/profile.d/conda.sh; conda activate flydrones
cd ~/sim/rl_h2/ecps295; export PYTHONPATH=~/sim/rl_h2/src:. OMP_NUM_THREADS=1
for s in 3 2; do
  nice -n 19 python -m rl.cmaes_run --run c4_s$s --arm bypass --space v4hw --fitness v3 --base minifly/v4.yaml --x0-yaml minifly/v4.yaml --nav flow --flow dr --lowbox-frac 0.5 --cma-seed $s --gens 20 --k 12 --popsize 12 --seconds 90 --procs 14 --val-every 5 --val-k 24 > rl/out/c4_s$s.log 2>&1
done
echo C4_DONE > rl/out/c4.done
