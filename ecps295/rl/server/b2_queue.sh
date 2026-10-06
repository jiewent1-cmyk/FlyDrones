#!/bin/bash
# B2: coverage-weight sweep (Pareto points), warm-started from es3, same protocol otherwise. Runs next to A1 wave 1.
cd ~/sim; while ! grep -q "all done" gz_fix.log; do sleep 60; done
source ~/sim/miniforge/etc/profile.d/conda.sh; conda activate fly
cd ~/sim/ecps295; export PYTHONPATH=~/sim/src:~/sim/ecps295 OMP_NUM_THREADS=1
for w in 2 4 8; do nice -n 10 python -m rl.cmaes_run --run b2_wcov$w --w-cov $w --gens 20 --k 12 --popsize 12 --seconds 90 --procs 12 --val-every 5 --val-k 24 --space wide --x0-yaml minifly/v3_2_es3.yaml > rl/out/b2_wcov$w.log 2>&1 & sleep 5; done; wait
echo B2_DONE > rl/out/b2.done
