#!/bin/bash
# part 1 test battery, task V3 (own instances 40 and 41)
D=/home/jacky/sim/val3
cd $D
OV=FLOW_TYPE=5,RNGFND1_TYPE=10
(bash run_feed.sh p1_dist 40 $OV --dist-schedule "0.5:5,1.0:5,2.0:5,7.9:5,9.0:5" --duration 25 > runs_A.log 2>&1 &)
sleep 1
(bash run_feed.sh p1_flow 41 $OV --flow --flow-schedule "0.5:-0.25:8,0.5:-0.25:8" --quality-schedule "100:8,0:8" --stop-flow-at 16 --duration 20 > runs_B.log 2>&1 &)
wait
echo "batch A+B done"
