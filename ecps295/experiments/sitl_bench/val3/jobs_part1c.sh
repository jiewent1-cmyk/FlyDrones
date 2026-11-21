#!/bin/bash
# part 1 test battery, task V3 (own instances 40 and 41), run 3 (with heartbeats)
D=/home/jacky/sim/val3
cd $D
OV=FLOW_TYPE=5,RNGFND1_TYPE=10
SCHED="0.5:4,1.0:4,2.0:4,7.9:4,9.0:4"
FS="0.5:-0.25:10"
(bash run_feed.sh p1_sys200 40 $OV --sysid 200 --dist-schedule "$SCHED" --duration 20 --flow --flow-schedule "$FS" --quality 100 > runs_A.log 2>&1 &)
sleep 1
(bash run_feed.sh p1_sys254 41 $OV --sysid 254 --dist-schedule "$SCHED" --duration 20 --flow --flow-schedule "$FS" --quality 100 > runs_B.log 2>&1 &)
sleep 60
echo launched
