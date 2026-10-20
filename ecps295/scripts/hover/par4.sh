#!/bin/bash
# 2026-10-20 patrol check of ECPS_HOLD=pos (mavlink_twin hover hold): lines "<tag> <world> <vel|pos> <hold yaw dps> <seed>",
# hp0 = velocity (baseline), hp1 = pos hold + 15 deg/s turn-in-place cap, hp2 = pos hold, no cap (30 = yaw_rate_max).
# Waits for the trk2 batch (other session) to finish, then 3 workers on instances 5-7.
cd ~/sim/rl_h2
until [ "$(tail -1 ~/sim/runs/matrix_trk2.log 2>/dev/null)" = DONE ] && ! pgrep -f "^gz sim" > /dev/null; do sleep 60; done
echo "start $(date)"
for w in 0 1 2; do
  ( sleep $((w * 20))
    awk -v w=$w "(NR - 1) % 3 == w" hv/hp.txt | while read -r tag world hold yaw seed; do
      echo "$tag $world s7_w2map@intact $seed --mode patrol --seconds 120 --cruise 0.5 --max-forward 0.5 --fence-turn --ext-height" > hv/l4_$w.txt
      ECPS_HOLD_YAW_DPS=$yaw bash hv/run.sh hv/l4_$w.txt ~/sim/runs/hp.part$w $hold $((5 + w)) < /dev/null
    done ) &
done
wait
cat ~/sim/runs/hp.part* | grep -v ^DONE > ~/sim/runs/hp.log; echo DONE >> ~/sim/runs/hp.log
echo "end $(date)"
