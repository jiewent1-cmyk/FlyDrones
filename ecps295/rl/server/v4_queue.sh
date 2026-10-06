#!/bin/bash
# v4 (user-improved, no RL) on cage20 GPS, 32 seeds, after B2 has finished (to keep my CPU share on the shared server low).
cd ~/sim; while [ ! -f ecps295/rl/out/b2.done ]; do sleep 60; done
source ~/sim/miniforge/etc/profile.d/conda.sh; conda activate fly
rm -rf runs/fix_v4_s*
ECPS_E=$HOME/sim/ecps295_v4 nice -n 19 python gz_batch.py --slots 8 --seeds 0-31 --prefix fix --arm v4=$HOME/sim/ecps295_v4/minifly/v4.yaml > gz_v4.log 2>&1
