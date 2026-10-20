#!/bin/bash
# 2026-10-18 hover batch: lines "<tag> <world> <vel|pos> <seed>" round-robin over 3 workers on instances 5-7
# (other sessions use 0-2). Starts at >= 09:00 PT once no other Gazebo is running; run_matrix_mine.sh pauses in quiet hours.
cd ~/sim/rl_h2
until [ $((10#$(TZ=America/Los_Angeles date +%H%M))) -ge 900 ] && ! pgrep -f "^gz sim" > /dev/null; do sleep 60; done
echo "start $(date)"
for w in 0 1 2; do
  ( sleep $((w * 20))
    awk -v w=$w "(NR - 1) % 3 == w" hv/batch.txt | while read -r tag world hold seed; do
      echo "$tag $world s7_w2map@intact $seed --mode hovertest --ext-height" > hv/line$w.txt
      bash hv/run.sh hv/line$w.txt ~/sim/runs/hv_batch.part$w $hold $((5 + w)) < /dev/null
    done ) &
done
wait
cat ~/sim/runs/hv_batch.part* | grep -v ^DONE > ~/sim/runs/hv_batch.log; echo DONE >> ~/sim/runs/hv_batch.log
echo "end $(date)"
