#!/bin/bash
# Part 2 matrix (task V3): HOV and SQ, 3 runs per path.
# MAVLink path = instance 40 (FLOW_TYPE=5, RNGFND1_TYPE=10), native drivers = instance 41.
cd /home/jacky/sim/val3
chmod +x run_fly.sh
MAV="FLOW_TYPE=5,RNGFND1_TYPE=10,MAV_GCS_SYSID=200,FENCE_ENABLE=0,VISO_TYPE=0"
NAT="FLOW_TYPE=10,RNGFND1_TYPE=100,MAV_GCS_SYSID=200,FENCE_ENABLE=0,VISO_TYPE=0"

run_pair() {
  local sfx=$1 mis=$2 hold=$3
  shift 3
  (bash run_fly.sh p2_${mis}${sfx}_mav 40 "$MAV" $mis --alt 2.0 --hold $hold --seconds 180 --send 1 "$@" > p2_${mis}${sfx}_mav.out 2>&1 &)
  sleep 2
  (bash run_fly.sh p2_${mis}${sfx}_nat 41 "$NAT" $mis --alt 2.0 --hold $hold --seconds 180 --send 0 "$@" > p2_${mis}${sfx}_nat.out 2>&1 &)
  while pgrep -f "run_fly.sh p2_${mis}${sfx}_" > /dev/null; do sleep 5; done
  sleep 3
  echo "round $mis$sfx done"
}

run_pair 1 HOV 60
run_pair 1 SQ 2 --side 2.0 --laps 1
run_pair 2 HOV 60
run_pair 2 SQ 2 --side 2.0 --laps 1
run_pair 3 HOV 60
run_pair 3 SQ 2 --side 2.0 --laps 1
echo MATRIX_DONE
