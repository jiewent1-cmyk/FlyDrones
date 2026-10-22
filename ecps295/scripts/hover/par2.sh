#!/bin/bash
# 2026-10-19 hover sweep (pos hold): lines "<tag> <world> <parm|-> <yaw dps> <seed>" over 3 workers on instances 5-7
cd ~/sim/rl_h2
echo "start $(date)"
for w in 0 1 2; do
  ( sleep $((w * 20))
    awk -v w=$w "(NR - 1) % 3 == w" hv/sweep.txt | while read -r tag world parm yaw seed; do
      echo "$tag $world s7_w2map@intact $seed --mode hovertest --ext-height" > hv/l2_$w.txt
      bash hv/run2.sh hv/l2_$w.txt ~/sim/runs/hv_sweep.part$w pos $((5 + w)) $parm $yaw < /dev/null
    done ) &
done
wait
cat ~/sim/runs/hv_sweep.part* | grep -v ^DONE > ~/sim/runs/hv_sweep.log; echo DONE >> ~/sim/runs/hv_sweep.log
echo "end $(date)"
