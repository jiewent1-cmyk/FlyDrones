#!/bin/bash
cd /home/jacky/sim/val3
chmod +x run_fly.sh
MAV="FLOW_TYPE=5,RNGFND1_TYPE=10,MAV_GCS_SYSID=200,FENCE_ENABLE=0,VISO_TYPE=0"
NAT="FLOW_TYPE=10,RNGFND1_TYPE=100,MAV_GCS_SYSID=200,FENCE_ENABLE=0,VISO_TYPE=0"
(bash run_fly.sh p2_t3_mav 40 "$MAV" HOV --alt 2.0 --hold 15 --seconds 120 --send 1 > p2_t3_mav.out 2>&1 &)
sleep 2
(bash run_fly.sh p2_t3_nat 41 "$NAT" HOV --alt 2.0 --hold 15 --seconds 120 --send 0 > p2_t3_nat.out 2>&1 &)
sleep 3
echo launched
