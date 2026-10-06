#!/bin/bash
# A1 control arms optimised in the fast twin with the same CMA-ES protocol as a1_intact (Jetson): from v3_2, wide space,
# 20 generations x 12 candidates x 12 episodes, held-out validation every 5. Starts after the Gazebo A0 batch.
cd ~/sim; while ! grep -q "all done" gz_fix.log; do sleep 60; done
source ~/sim/miniforge/etc/profile.d/conda.sh; conda activate fly
cd ~/sim/ecps295; export PYTHONPATH=~/sim/src:~/sim/ecps295 OMP_NUM_THREADS=1
run() { nice -n 10 python -m rl.cmaes_run --run a1_$1 --arm $1 --gens 20 --k 12 --popsize 12 --seconds 90 --procs 12 --val-every 5 --val-k 24 --space wide > rl/out/a1_$1.log 2>&1; }
mkdir -p rl/out
for wave in "shuffle1 shuffle2 shuffle3" "shuffle4 shuffle5 randread1"; do
  for arm in $wave; do run $arm & sleep 5; done; wait
done
echo A1_SERVER_DONE > rl/out/a1_server.done
